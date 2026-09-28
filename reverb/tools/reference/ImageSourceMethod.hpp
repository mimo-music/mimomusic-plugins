// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// ImageSourceMethod.hpp
//
// The higher-order generalization of plugin/dsp/ReverbEngine.hpp's own
// firstOrderImageSources() -- the Allen & Berkley (1979) shoebox-room
// image method, which that function's own comment already named as the
// "specular single-bounce half of geometric acoustics" and deliberately
// stopped at first order (docs/REVERB_CONCEPT.md's own "Known
// limitations" section: "this deliberately stops at first order rather
// than growing into full recursive path tracing"). This file is that
// growth, built specifically for the offline reference-sample generator
// (tools/reference/generate_reference_samples.cpp), where computation
// cost doesn't matter -- hours per set of good test samples is fine.
//
// Why THIS as the expensive, more physically correct reference, not
// just a bigger/denser FDN (which is what reverb_dense already is, one
// notch further): reverb/reverb_dense/reverb_plate are all statistical
// approximations to a diffuse sound field -- structures designed to
// SOUND like the aggregate behavior of thousands of real reflections
// without computing any of them individually. The image method computes
// the real, individual, geometrically exact reflection paths of a
// shoebox room directly (position, delay, and amplitude of every wall-
// bounce path up to a given order) -- the actual physics the other three
// engines are only ever approximating statistically. Higher order is
// expensive on purpose: image count grows arguments ~O(order^3) before
// filtering (a real, unavoidable cost of computing genuine ray paths
// instead of approximating their aggregate statistics), which is exactly
// the trade accepted here (long computation) in exchange for a genuinely independent ground truth -- useful
// later both as a fitting target and, separately, as a way to test how
// well the sibling ../statespace project's own state-space/matrix-fitting
// machinery can learn this kind of behavior at all (an explicitly later,
// not-yet-started goal).
//
// Derivation, verified against ReverbEngine.hpp's own firstOrderImageSources()
// before being trusted (not taken from memory alone): a source at sx
// along an axis of length L has images at x = 2*l*L + (-1)^p * sx for
// integer l and p in {0,1}. Hand-deriving the actual reflection order by
// repeatedly mirroring (source -> -sx [order 1] -> 2L-sx [order 1] ->
// 2L+sx [order 2] -> -2L+sx [order 2] -> -2L-sx [order 3] -> 4L-sx
// [order 3] -> ...) and matching against (l,p) gives the closed form
// order = |2*l - p| -- checked against 8 independently hand-derived
// (position, order) pairs, not assumed. Summed across all three axes,
// this reduces to ReverbEngine.hpp's own 6 first-order taps exactly when
// maxOrder=1 (each single-wall mirror is order 1 on one axis, order 0 on
// the other two) -- verified directly in
// tools/reference/generate_reference_samples.cpp's own startup sanity
// check, not just argued here.

#pragma once

#include "../../plugin/dsp/ReverbEngine.hpp" // RoomDims, EarlyReflection, Vec3, kSpeedOfSoundMPerSec

#include <cmath>
#include <cstdint>
#include <vector>

namespace ssfilter {
namespace reverb {
namespace reference {

// Every wall-bounce path up to `maxOrder` reflections, for a shoebox
// room -- NOT bounded to kMaxEarlyReflections (that cap exists for the
// real-time engines' own fixed-size early-reflection arrays; this is an
// offline tool, a std::vector sized however large the true tap count is,
// is the correct choice here). Source/receiver placed at the same
// fractional room-relative offsets ReverbEngine.hpp's own
// firstOrderImageSources() uses, for direct comparability.
//
// `maxOrder` directly controls both fidelity and cost: order 1 reduces
// to the existing function's own 6 taps; each further order roughly
// triples the search volume (before the many high-order, high-distance,
// heavily-attenuated taps are implicitly deprioritized by their own tiny
// gain) -- order 15-25+ is where computation takes hours, which is
// acceptable for an offline reference.
// `rcvOffsetM` shifts the receiver along the room's own width axis from
// its default position -- used to place two distinct "ear" receivers
// (see tools/reference/ReferenceEngine.hpp) for genuine physically-
// derived stereo (two real points in the same room hearing the same
// source via different path geometries), rather than the mid/side trick
// the other three engines use. 0.0 (the default) reproduces the exact
// receiver position firstOrderImageSources() itself uses, which is what
// makes the maxOrder=1 cross-check against it valid.
inline std::vector<EarlyReflection> higherOrderImageSources(
    const RoomDims &room, double wallAbsorption01, int maxOrder, double rcvOffsetM = 0.0) {
    const Vec3 src{room.lengthM * 0.3, room.widthM * 0.5, 1.6};
    const Vec3 rcv{room.lengthM * 0.7, room.widthM * 0.4 + rcvOffsetM, 1.2};
    const double reflCoeff = std::sqrt(std::clamp(1.0 - wallAbsorption01, 0.0, 1.0));

    std::vector<EarlyReflection> out;
    if (maxOrder < 1) return out;

    // order_x = |2*l - p| <= maxOrder (with the other two axes
    // contributing 0) bounds l to roughly [-(maxOrder+1)/2, (maxOrder+1)/2].
    const int bound = maxOrder / 2 + 1;

    for (int lx = -bound; lx <= bound; ++lx)
    for (int ly = -bound; ly <= bound; ++ly)
    for (int lz = -bound; lz <= bound; ++lz)
    for (int px = 0; px <= 1; ++px)
    for (int py = 0; py <= 1; ++py)
    for (int pz = 0; pz <= 1; ++pz) {
        const int ordX = std::abs(2 * lx - px);
        const int ordY = std::abs(2 * ly - py);
        const int ordZ = std::abs(2 * lz - pz);
        const int order = ordX + ordY + ordZ;
        if (order == 0 || order > maxOrder) continue; // order 0 == the direct path, not a reflection

        const double ix = 2.0 * lx * room.lengthM + (px == 0 ? src.x : -src.x);
        const double iy = 2.0 * ly * room.widthM + (py == 0 ? src.y : -src.y);
        const double iz = 2.0 * lz * room.heightM + (pz == 0 ? src.z : -src.z);

        const double dx = ix - rcv.x, dy = iy - rcv.y, dz = iz - rcv.z;
        const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < 0.05) continue; // degenerate (receiver essentially at an image position)

        EarlyReflection r;
        r.delaySec = dist / kSpeedOfSoundMPerSec;
        r.gain = std::pow(reflCoeff, (double)order) / std::max(dist, 0.1);
        out.push_back(r);
    }

    return out;
}

} // namespace reference
} // namespace reverb
} // namespace ssfilter
