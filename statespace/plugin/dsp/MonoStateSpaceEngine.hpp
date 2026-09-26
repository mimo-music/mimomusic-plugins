// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// MonoStateSpaceEngine.hpp
//
// Single-channel counterpart to StateSpaceEngine.hpp. Same real-time
// rules (no locks, no allocation, no unbounded loops in processBlock()),
// same wait-free double-buffered model hand-off, same coefficient-
// smoothing-on-same-order / duck-on-order-change behavior for click-free
// edits (see StateSpaceEngine.hpp's header comment for the full reasoning
// behind both -- unchanged here, just against a single input/output
// sample per frame instead of a stereo pair). The only real difference
// from the stereo engine is the shape of the math itself: one state
// recursion driven by one input, one output tap, no cross-channel D/C
// terms to compute -- which is the entire point of this file existing
// separately rather than the mono plugin just feeding the same signal
// into both channels of the stereo engine (that would still pay the
// stereo engine's full O(order^2) cost for an always-unused second
// channel's worth of states; see MonoStateSpaceTypes.hpp's header
// comment).

#pragma once

#include "MonoStateSpaceTypes.hpp"

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>

namespace ssfilter {
namespace mono {

class MonoStateSpaceEngine
{
public:
    MonoStateSpaceEngine() = default;

    void prepare(double sampleRate) noexcept
    {
        sampleRate_ = sampleRate;
        resetState();
    }

    void setModel(const MonoStateSpaceMatrices& m)
    {
        std::lock_guard<std::mutex> lock(writeMutex_);
        const int live = activeSlot_.load(std::memory_order_relaxed);
        const int next = 1 - live;
        slots_[next] = m;
        activeSlot_.store(next, std::memory_order_release);
        version_.fetch_add(1, std::memory_order_release);
    }

    MonoStateSpaceMatrices getModel() const
    {
        std::lock_guard<std::mutex> lock(writeMutex_);
        return slots_[activeSlot_.load(std::memory_order_acquire)];
    }

    void resetState() noexcept
    {
        for (double& v : x_) v = 0.0;
        nlEnvLin_ = 1e-9;
        faultFlag_.store(false, std::memory_order_relaxed);
        const int slot = activeSlot_.load(std::memory_order_acquire);
        smoothed_ = slots_[slot];
        smoothedOrder_ = clampOrder(smoothed_.order);
        smoothingActive_ = false;
        lastSeenVersion_ = version_.load(std::memory_order_acquire);
        duckState_ = DuckState::None;
        duckGain_ = 1.0;
    }

    bool consumeFaultFlag() noexcept
    {
        return faultFlag_.exchange(false, std::memory_order_relaxed);
    }

    // RT-safe main processing entry point -- one input channel, one output
    // channel. See StateSpaceEngine::processBlock() for the fully-commented
    // stereo original this mirrors step for step.
    void processBlock(const float* in, float* out, uint32_t numFrames) noexcept
    {
        const int slot = activeSlot_.load(std::memory_order_acquire);
        const MonoStateSpaceMatrices& m = slots_[slot];
        const int targetN = clampOrder(m.order);

        const uint32_t ver = version_.load(std::memory_order_acquire);
        if (ver != lastSeenVersion_)
        {
            lastSeenVersion_ = ver;
            if (targetN == smoothedOrder_)
            {
                smoothingActive_ = true;
            }
            else
            {
                pendingModel_ = m;
                pendingOrder_ = targetN;
                smoothingActive_ = false;
                if (duckState_ == DuckState::None)
                {
                    duckState_ = DuckState::FadingOut;
                    duckStep_ = 1.0 / (kDuckTimeSec * sampleRate_);
                }
            }
        }

        const double smoothCoeff = smoothingActive_
                                        ? (1.0 - std::exp(-1.0 / (kCoeffSmoothTimeConstantSec * sampleRate_)))
                                        : 0.0;

        const bool nlEnabled = m.nonlinear.enabled;
        double nlAttackCoeff = 0.0, nlReleaseCoeff = 0.0;
        if (nlEnabled)
        {
            const double attackMs  = m.nonlinear.attackMs  > 0.05 ? m.nonlinear.attackMs  : 0.05;
            const double releaseMs = m.nonlinear.releaseMs > 1.0  ? m.nonlinear.releaseMs : 1.0;
            nlAttackCoeff  = std::exp(-1.0 / (0.001 * attackMs  * sampleRate_));
            nlReleaseCoeff = std::exp(-1.0 / (0.001 * releaseMs * sampleRate_));
        }

        for (uint32_t f = 0; f < numFrames; ++f)
        {
            double duckMul = 1.0;
            if (duckState_ == DuckState::FadingOut)
            {
                duckGain_ -= duckStep_;
                if (duckGain_ <= 0.0)
                {
                    duckGain_ = 0.0;
                    smoothed_ = pendingModel_;
                    smoothedOrder_ = pendingOrder_;
                    for (double& v : x_) v = 0.0;
                    duckState_ = DuckState::FadingIn;
                }
                duckMul = duckGain_;
            }
            else if (duckState_ == DuckState::FadingIn)
            {
                duckGain_ += duckStep_;
                if (duckGain_ >= 1.0) { duckGain_ = 1.0; duckState_ = DuckState::None; }
                duckMul = duckGain_;
            }

            const int n = smoothedOrder_;

            if (smoothingActive_)
            {
                double maxDiff = 0.0;
                for (int i = 0; i < n; ++i)
                    for (int j = 0; j < n; ++j)
                    {
                        double& s = smoothed_.A[i][j];
                        const double t = m.A[i][j];
                        s += smoothCoeff * (t - s);
                        maxDiff = std::max(maxDiff, std::fabs(t - s));
                    }
                for (int i = 0; i < n; ++i)
                {
                    double& s = smoothed_.B[i];
                    const double t = m.B[i];
                    s += smoothCoeff * (t - s);
                    maxDiff = std::max(maxDiff, std::fabs(t - s));
                }
                for (int j = 0; j < n; ++j)
                {
                    double& s = smoothed_.C[j];
                    const double t = m.C[j];
                    s += smoothCoeff * (t - s);
                    maxDiff = std::max(maxDiff, std::fabs(t - s));
                }
                {
                    double& s = smoothed_.D;
                    const double t = m.D;
                    s += smoothCoeff * (t - s);
                    maxDiff = std::max(maxDiff, std::fabs(t - s));
                }
                if (maxDiff < 1e-9) smoothingActive_ = false;
            }

            const double u = double(in[f]);

            double y = smoothed_.D * u;
            for (int j = 0; j < n; ++j)
                y += smoothed_.C[j] * x_[j];

            for (int i = 0; i < n; ++i)
            {
                double s = smoothed_.B[i] * u;
                for (int j = 0; j < n; ++j)
                    s += smoothed_.A[i][j] * x_[j];
                xNext_[i] = s;
            }
            for (int i = 0; i < n; ++i)
            {
                double v = xNext_[i];
                if (std::fabs(v) < 1e-15) v = 0.0;
                x_[i] = v;
            }

            if (!std::isfinite(y))
            {
                faultFlag_.store(true, std::memory_order_relaxed);
                for (double& v : x_) v = 0.0;
                y = 0.0;
                duckState_ = DuckState::None;
                duckGain_ = 1.0;
            }
            else
            {
                if (nlEnabled)
                {
                    const double levelLin = std::fabs(u);
                    const double coeff = (levelLin > nlEnvLin_) ? nlAttackCoeff : nlReleaseCoeff;
                    nlEnvLin_ = coeff * nlEnvLin_ + (1.0 - coeff) * levelLin;

                    const double envDb = 20.0 * std::log10(std::max(nlEnvLin_, 1e-9));
                    const double gainDb = m.nonlinear.curve.evaluate(envDb);
                    const double gainLin = std::pow(10.0, gainDb / 20.0);
                    y *= gainLin;
                }

                constexpr double kCeiling = 64.0;
                if (y > kCeiling) y = kCeiling; else if (y < -kCeiling) y = -kCeiling;
            }

            out[f] = float(y * duckMul);
        }
    }

private:
    static constexpr double kCoeffSmoothTimeConstantSec = 0.005;
    static constexpr double kDuckTimeSec = 0.008;

    std::array<MonoStateSpaceMatrices, 2> slots_{};
    std::atomic<int> activeSlot_{0};
    std::atomic<uint32_t> version_{0};
    mutable std::mutex writeMutex_;

    double x_[kMaxOrder] = {};
    double xNext_[kMaxOrder] = {};
    double sampleRate_ = 44100.0;
    double nlEnvLin_ = 1e-9;
    std::atomic<bool> faultFlag_{false};

    MonoStateSpaceMatrices smoothed_;
    int smoothedOrder_ = 0;
    bool smoothingActive_ = false;
    uint32_t lastSeenVersion_ = 0;

    enum class DuckState : uint8_t { None, FadingOut, FadingIn };
    DuckState duckState_ = DuckState::None;
    double duckGain_ = 1.0;
    double duckStep_ = 0.0;
    MonoStateSpaceMatrices pendingModel_;
    int pendingOrder_ = 0;
};

} // namespace mono
} // namespace ssfilter
