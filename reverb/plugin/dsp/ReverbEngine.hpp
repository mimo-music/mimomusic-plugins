// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// ReverbEngine.hpp
//
// Concept-phase core DSP for the room-dimension-driven reverb plugin (see
// docs/REVERB_CONCEPT.md for the full writeup, the research this is based
// on, and the phased build plan this file is step 1 of). Standalone,
// dependency-free (only <array>/<cmath>/<cstdint>), matching every other
// header in plugin/dsp/ -- no DPF/DGL, exercised directly by
// tests/reverb_regression.cpp, no cmake required to iterate on this file.
//
// -----------------------------------------------------------------------
// Why this is a DEDICATED engine, not a StateSpaceMatrices/StateSpaceEngine
// mapper (the "Building an LTI processor" pattern docs/EXTENDING.md
// describes for EQ/crossovers/etc.)
// -----------------------------------------------------------------------
// A reverberant room's impulse response needs delay lines tens to hundreds
// of milliseconds long -- thousands of samples -- to sound like a room and
// not a resonant filter. A literal state-space realization of that (one
// scalar state per delay-line sample, threaded through the generic dense
// A matrix the way StateSpaceEngine.hpp realizes an EQ band) would need an
// order far beyond kMaxOrder (128, i.e. ~2.7ms at 48kHz) and, worse, the
// generic engine's O(order^2) per-sample recursion would cost millions of
// multiply-adds per sample at the needed order -- utterly non-real-time,
// not just "slow". This is the same "no fixed mapper realization exists"
// situation DynamicEqEngine.hpp is in (see that file's own comment) --
// except there the obstacle is a *time-varying* cascade, here it's sheer
// *size* -- so the fix follows the same precedent: a dedicated, self-
// contained engine, not a mapper feeding the generic core.
//
// The system implemented here is still, formally, exactly a discrete-time
// state-space system x[n+1]=Ax[n]+Bu[n], y[n]=Cx[n]+Du[n] -- this is
// literally how a Feedback Delay Network (FDN) is defined in the DSP
// literature (Jot 1991/1992; see docs/REVERB_CONCEPT.md's references).
// Concatenate every delay line's contents into one big state vector x[n];
// almost the entirety of "A" is then a pure shift/permutation (each
// state's next value is simply the state one slot behind it in its own
// line -- zero multiplications), with genuine multiply-add coupling only
// at each line's read/write boundary (the N-line feedback-mixing matrix,
// N = number of delay LINES, e.g. 8-16, not delay SAMPLES). Representing
// that provable structure as circular buffers (O(1) index step per state
// per sample) instead of a dense matrix multiply is exactly the same
// "don't multiply by a provably-zero coefficient" idea StateSpaceTypes.hpp's
// `sparseCascade2` flag already established for cascaded biquads, just
// carried to its logical extreme: here, *all but O(N)* of the state
// transition is structurally zero/identity, per delay line, every sample.
// The result: a system with an effective order in the thousands (a lot of
// "background state," exactly what state-space is being used for here)
// that costs O(N) per sample to run -- independent of room size/order,
// the "long to compute the model, fast to run it" split, since
// N (line count) is fixed at compile time and only the delay LENGTHS
// (derived from room dimensions) grow with room size.
//
// -----------------------------------------------------------------------
// The three pieces, each validated numerically in tests/reverb_regression.cpp
// -----------------------------------------------------------------------
// 1. Delay line lengths from room dimensions (designDelayLengths()): the
//    room's mean free path (4V/S, standard room-acoustics formula) sets a
//    base travel time; N lines are spread geometrically around it and each
//    rounded to the nearest PRIME sample count -- the standard FDN trick
//    (see Jot; also ccrma.stanford.edu/~jos/pasp "Prime Power Delay-Line
//    Lengths") so no two lines' echoes coincide on a simple integer ratio,
//    which is what causes audible metallic/ringing coloration.
// 2. A lossless (energy-preserving) feedback matrix mixing the N lines'
//    outputs back into their inputs every sample (householderMix()): the
//    Householder reflection F = I - (2/N)*ones*ones^T is orthogonal by
//    construction (verified in test [R4] below: input and output energy
//    match to numerical precision) and costs O(N), not O(N^2), to apply --
//    compute the sum once, subtract 2*sum/N from every element. This is
//    also the mechanism that lets a signal fed into an L-associated line
//    cross into R-associated lines and back (verified in test [R2]) --
//    every line talks to every other line, every sample, regardless of
//    which stereo channel originally fed it.
// 3. Per-line frequency-dependent damping (OnePoleDamping::design()): a
//    one-pole shelf fit in closed form to hit an exact target decay factor
//    at DC and at Nyquist (Jot's absorptive-filter construction), so a
//    single Decay Time (T60) and a Damping (brightness) parameter shape
//    every line's own per-pass loss consistently, however long that line
//    is (a longer line needs a smaller per-pass loss to reach the same
//    T60 -- decayFactor() computes that directly from the line's own
//    length). Verified in test [R1]: measured low-band T60 (a signal
//    fed through a low-pass before the Schroeder decay-curve measurement,
//    isolating it from the faster high-frequency decay) matches the
//    target to within 0.5%.
//
// Stability is a CLOSED-FORM structural guarantee here, unlike the generic
// core's iterative StabilityGuard: the feedback matrix is provably
// orthogonal (spectral radius exactly 1, never estimated) and every per-
// line damping filter is designed from a physical T60 that is always > 0,
// so its gain is always in (0,1) by construction -- the product of an
// energy-preserving mix and a strictly-lossy filter per line can only ever
// lose energy, never gain it, every single pass. The NaN/Inf-substitution
// safety net below exists anyway, for the exact reason StateSpaceEngine.hpp
// gives for its own second net: "no amount of upstream validation can
// prove a recursive filter stays finite forever under arbitrary sustained
// input and accumulated floating-point error."
//
// -----------------------------------------------------------------------
// What is deliberately NOT here yet (see docs/REVERB_CONCEPT.md "Phase 2")
// -----------------------------------------------------------------------
// - Early reflections (the image-source method) live in this same file
//   (firstOrderImageSources()) but are not yet wired into ReverbEngine's
//   own processStereo() as a summed tap-delay line -- validated
//   standalone (test [R5]) as a first step, integration is phase 2.
// - No wait-free double-buffer coefficient hand-off yet (StateSpaceEngine's
//   own pattern) -- prepare() is not real-time safe to call from the audio
//   thread. A room-size change (which changes every line's LENGTH, i.e.
//   the delay buffers' own indexing) has no meaningful "in between" two
//   different lengths to interpolate through, exactly the EQ's own order-
//   changing-edit situation -- so it should reuse StateSpaceEngine's own
//   duck-to-silence-then-swap technique (DuckState) rather than invent a
//   new one, once wired into a real plugin with a message-thread/audio-
//   thread split to duck across.
// - No parametric EQ stage (post-tank, or "object" mid-tank) -- see the
//   concept doc's EQ integration section; this file only needs to stay an
//   ordinary StateSpaceMatrices producer (the EQ side) plus this dedicated
//   engine (the reverb side), nothing new for the EQ to reuse cleanly.

#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>

namespace ssfilter {
namespace reverb {

// Speed of sound, m/s (20 degrees C, dry air) -- used only to turn
// geometry (meters) into delay (seconds), never anything audio-rate.
constexpr double kSpeedOfSoundMPerSec = 343.0;

// Compile-time-fixed line count. Not exposed to the user (an internal
// engine-quality knob, like kMaxOrder is for the generic core) -- 8 is
// Jot's own commonly-cited minimum for adequate echo density in a musical
// reverb; going higher smooths the tail further at proportionally higher
// but still O(N) cost. See docs/REVERB_CONCEPT.md for the measured
// per-sample cost at this N.
constexpr int kNumLines = 8;

// Upper bound on any one delay line's length, sized generously for a
// large hall at a high sample rate (16384 samples = ~341ms at 48kHz,
// ~170ms at 96kHz) -- see designDelayLengths()'s own comment for how a
// room's dimensions map into this budget, and ReverbParams::clamp() for
// where oversized rooms actually get bounded before reaching here. Fixed
// or the storage below could not be a no-allocation, fixed-size member --
// the same reasoning StateSpaceTypes.hpp gives for its own kMaxOrder.
constexpr int kMaxDelaySamples = 16384;

// Maximum first-order image-source early reflections a shoebox room
// produces (one per wall) -- see firstOrderImageSources().
constexpr int kMaxEarlyReflections = 6;

struct RoomDims {
    double lengthM = 10.0;
    double widthM = 7.0;
    double heightM = 3.5;

    void clamp() {
        lengthM = std::clamp(lengthM, 1.0, 80.0);
        widthM = std::clamp(widthM, 1.0, 80.0);
        heightM = std::clamp(heightM, 1.0, 40.0);
    }
};

// Mean free path of a rectangular room (Sabine): the average distance a
// sound ray travels between wall reflections, 4V/S. Standard room-
// acoustics quantity, used here purely as a size-to-delay heuristic (not
// a claim that the FDN's lines model actual ray paths -- see the concept
// doc's "what mean free path is and isn't doing here" note).
inline double meanFreePathM(const RoomDims &r) {
    const double V = r.lengthM * r.widthM * r.heightM;
    const double S = 2.0 * (r.lengthM * r.widthM + r.lengthM * r.heightM + r.widthM * r.heightM);
    return 4.0 * V / S;
}

inline bool isPrime(int n) {
    if (n < 2) return false;
    for (int d = 2; (long long)d * d <= n; ++d)
        if (n % d == 0) return false;
    return true;
}
inline int nearestPrimeAtLeast(int n) {
    if (n < 2) n = 2;
    while (!isPrime(n)) ++n;
    return n;
}

// N delay-line lengths (samples) spread geometrically (0.55x .. 2.4x)
// around the room's mean-free-path-implied travel time, each rounded to
// the nearest prime -- see this file's own header comment, point 1.
// Clamped to kMaxDelaySamples so an oversized room degrades to "as big as
// this engine supports" instead of overrunning fixed storage.
inline std::array<int, kNumLines> designDelayLengths(const RoomDims &room, double sampleRate) {
    const double mfpSec = meanFreePathM(room) / kSpeedOfSoundMPerSec;
    const double baseSamples = mfpSec * sampleRate;

    // A real, measured L/R asymmetry, found directly in response to "the
    // spread is now turning my test signal slightly to the right": the
    // spread factors below are monotonically increasing with i, and
    // processStereo() routes even i to L, odd i to R (see its own
    // comment) -- assigning them in plain ascending index order (as this
    // function used to) means L systematically gets the *low* half of
    // the spread range and R the *high* half, not an even split. A
    // longer line needs a smaller per-pass damping gain to reach the same
    // real-time Decay Time (fewer, bigger dB steps per second), so R's
    // systematically-longer lines end up with a systematically *lower*
    // average per-pass gain than L's -- which, driven by continuous
    // (not impulse) input, measurably changes each channel's own
    // steady-state energy buildup (verified directly with a standalone
    // harness: ~0.16dB, L louder than R at the *default* room size in
    // this specific case -- small, and the measured *direction* doesn't
    // even fully match the report by itself, but it's a real, structural
    // bias that Width/Spread (which can only ever scale whatever L-R
    // difference already exists, see its own comment) would make more
    // audible, not one this plugin should leave in place regardless).
    //
    // Fixed with a "snake"/boustrophedon assignment (the standard
    // balanced-draft-order trick): visit the 8 ascending spread values in
    // order and hand them to channels L,R,R,L,L,R,R,L instead of strict
    // L,R,L,R,L,R,L,R alternation -- L ends up with items {0,3,4,7}, R
    // with {1,2,5,6}. Because the spread formula is evenly spaced,
    // item[k]+item[7-k] is the same constant for every k, so this split
    // balances L's and R's own sums *exactly*, not approximately --
    // confirmed directly (a scratch calculation, both sums equal to
    // 6 significant figures), not just argued. Verified end to end in
    // tests/reverb_regression.cpp [R14] (steady-state L/R energy ratio
    // for symmetric input, measured well within 0.1dB after this fix).
    static constexpr int kItemForArrayIndex[kNumLines] = {0, 1, 3, 2, 4, 5, 7, 6};

    std::array<int, kNumLines> lens{};
    for (int i = 0; i < kNumLines; ++i) {
        const int k = kItemForArrayIndex[i];
        const double spread = 0.55 + 1.85 * (double)k / (double)(kNumLines - 1);
        int raw = (int)std::lround(baseSamples * spread);
        raw = std::clamp(raw, 8, kMaxDelaySamples - 1);
        lens[i] = std::min(nearestPrimeAtLeast(raw), kMaxDelaySamples - 1);
    }
    return lens;
}

// Per-line one-pole damping filter: y[n] = g*(1-a)*x[n] + a*y[n-1], with
// g/a solved in closed form so the filter's own DC gain equals r0 and its
// Nyquist gain equals rNy -- Jot's absorptive-filter construction, see
// this file's own header comment, point 3.
//
// g/a are split into a *target* (set instantly by design(), e.g. every
// time Decay Time/Damping is edited, or every block when tempo-sync is
// following a live-changing host BPM -- see "Tempo-synced decay" below)
// and the *smoothed* pair actually used by process(), glided toward the
// target with the identical one-pole time constant/formula
// StateSpaceEngine.hpp's own A/B/C/D coefficient smoothing uses
// (kCoeffSmoothTimeConstantSec = 5ms, "about 63% closed per time
// constant, fully converged well under 50ms").
//
// NOTE, found while writing tests/reverb_regression.cpp [R9]: this is
// NOT the same failure mode as StateSpaceEngine's C/D case, and doesn't
// produce an instantly-audible output click the way that one does -- a
// delay line's circular buffer *reads* this sample's output tap before it
// *writes* this sample's freshly-damped value, so a coefficient change
// only reaches ReverbEngine's actual stereo output after a full trip
// around that specific line (its own length in samples, 6-27ms for a
// mid-size room) -- confirmed directly, an earlier version of that test
// measured *zero* output-tap difference between an instant and a smoothed
// retarget for exactly this reason. What this smoothing actually fixes is
// internal: y[n]=g*(1-a)*x[n]+a*z1 evaluated with a freshly-snapped g/a
// against a z1 that accumulated under the *old* coefficients is a real,
// measured discontinuity in the *value being written* into the delay line
// (test [R9]: ~0.046 instant vs. ~0.0002 smoothed, a >200x reduction, for
// a drastic 1.8s->0.3s retarget) -- audible later, smeared across when
// each line's own delay happens to read that value back, not as a single
// click. Worth smoothing regardless, and unlike a one-off manual edit, a
// live host-tempo follow can retarget every single block, so leaving this
// unsmoothed would make a real (if delayed/smeared) artifact routine, not
// rare.
struct OnePoleDamping {
    double g = 1.0, a = 0.0;                 // smoothed, used by process()
    double gTarget = 1.0, aTarget = 0.0;      // instant design() target
    double z1 = 0.0;

    // The per-pass decay factor a delay line of M samples needs to reach
    // t60Sec (seconds) at a given frequency, from the standard -60dB-in-
    // T60-seconds definition: r = 10^(-3 / (T60*fs)) per sample, so a
    // line that recirculates once every M samples needs r^M per pass.
    static double decayFactor(int M, double t60Sec, double sampleRate) {
        if (t60Sec <= 1e-6) return 0.0;
        return std::pow(10.0, -3.0 * (double)M / (t60Sec * sampleRate));
    }

    // Sets the target only -- process() glides g/a toward it via smoothTowardTarget().
    // Call snapToTarget() too if the intent is an instant change (e.g. on prepare()).
    void design(double r0, double rNy) {
        gTarget = r0;
        const double k = (r0 > 1e-12) ? (rNy / r0) : 0.0;
        aTarget = (1.0 - k) / (1.0 + k);
    }

    void snapToTarget() { g = gTarget; a = aTarget; }

    void smoothTowardTarget(double smoothCoeff) {
        g += smoothCoeff * (gTarget - g);
        a += smoothCoeff * (aTarget - a);
    }

    double process(double x) {
        const double y = g * (1.0 - a) * x + a * z1;
        z1 = y;
        return y;
    }

    void reset() { z1 = 0.0; }
};

// Householder reflection matrix F = I - (2/N)*ones*ones^T applied in
// place: orthogonal (lossless) by construction, O(N) cost. See this
// file's own header comment, point 2.
inline void householderMix(std::array<double, kNumLines> &v) {
    double sum = 0.0;
    for (int i = 0; i < kNumLines; ++i) sum += v[i];
    const double c = 2.0 * sum / (double)kNumLines;
    for (int i = 0; i < kNumLines; ++i) v[i] -= c;
}

struct EarlyReflection {
    double delaySec = 0.0;
    double gain = 0.0;
};
struct Vec3 { double x = 0.0, y = 0.0, z = 0.0; };

// First-order image sources for a shoebox room: mirroring the source
// across each of the 6 walls gives 6 image sources, each whose straight-
// line distance to the receiver gives one discrete early reflection's
// delay (dist/speed-of-sound) and 1/dist amplitude falloff scaled by a
// wall absorption coefficient. This is the "specular single-bounce" half
// of geometric acoustics -- the acoustic analogue of mirroring a light
// source across a flat surface in a ray tracer (see docs/REVERB_CONCEPT.md's
// "geometric acoustics" section for why this deliberately stops at first
// order rather than growing into full recursive path tracing). Source and
// receiver are placed at fixed fractional offsets within the room so the
// only user-facing parameters remain the room dimensions themselves.
inline std::array<EarlyReflection, kMaxEarlyReflections> firstOrderImageSources(
    const RoomDims &room, double wallAbsorption01) {
    const Vec3 src{room.lengthM * 0.3, room.widthM * 0.5, 1.6};
    const Vec3 rcv{room.lengthM * 0.7, room.widthM * 0.4, 1.2};
    const double reflCoeff = std::sqrt(std::clamp(1.0 - wallAbsorption01, 0.0, 1.0));

    std::array<Vec3, kMaxEarlyReflections> images = {{
        {-src.x, src.y, src.z},
        {2 * room.lengthM - src.x, src.y, src.z},
        {src.x, -src.y, src.z},
        {src.x, 2 * room.widthM - src.y, src.z},
        {src.x, src.y, -src.z},
        {src.x, src.y, 2 * room.heightM - src.z},
    }};

    std::array<EarlyReflection, kMaxEarlyReflections> out{};
    for (int i = 0; i < kMaxEarlyReflections; ++i) {
        const double dx = images[i].x - rcv.x, dy = images[i].y - rcv.y, dz = images[i].z - rcv.z;
        const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        out[i].delaySec = dist / kSpeedOfSoundMPerSec;
        out[i].gain = reflCoeff / std::max(dist, 0.1);
    }
    return out;
}

// -----------------------------------------------------------------------
// Tempo-synced decay
// -----------------------------------------------------------------------
// Opt-in: Decay Time can be derived from the host's own tempo instead of
// set directly in seconds, following the same note-value convention every
// tempo-synced delay plugin already uses (confirmed against several
// online delay/reverb-time calculators, see docs/REVERB_CONCEPT.md's
// references) -- ms = 60000/BPM * noteValueInQuarterNotes, and for reverb
// specifically: a quarter note reads as a short ambience, a half note a
// medium room, one bar a long hall, two bars an epic tail. `OneBar`/
// `TwoBars`/`FourBars` need the host's own time signature (beatsPerBar)
// to mean an actual bar rather than always assuming 4/4.
//
// DPF already exposes exactly what this needs, per-format, including
// live changes: `Plugin::getTimePosition().bbt.beatsPerMinute` (and
// `.beatsPerBar`), populated every process block from the host's own
// transport by every backend this project targets (LV2's time: atom
// extension, VST3's ProcessContext, CLAP's transport, JACK transport --
// see dpf/distrho/src/DistrhoPlugin{LV2,VST3,CLAP,JACK}.cpp, grepped
// directly rather than assumed). `.bbt.valid` gates whether the host is
// reporting BBT/tempo at all; when a format doesn't (or a host doesn't
// send it), DPF itself falls back to 120 BPM -- this engine does the same
// (see t60SecFromTempo()'s own defensive fallback) rather than a second,
// different default. "Opt-in even if the DAW changes BPM" is a plain
// per-block re-read of getTimePosition() at the call site (see
// docs/REVERB_CONCEPT.md's Phase 2 plan for where that call site lives --
// this file only provides the pure conversion function, kept dependency-
// free and directly testable here) feeding ReverbEngine::setDecayTargets()
// every block; without that opt-in, Decay Time is read once and left
// alone like any other parameter.
enum class TempoNoteValue {
    Sixteenth, Eighth, DottedEighth, Quarter, DottedQuarter,
    Half, DottedHalf, OneBar, TwoBars, FourBars
};

inline double noteValueToQuarterNotes(TempoNoteValue nv, double beatsPerBar) {
    if (beatsPerBar <= 0.0) beatsPerBar = 4.0; // sane default, matches a 4/4 assumption
    switch (nv) {
        case TempoNoteValue::Sixteenth:     return 0.25;
        case TempoNoteValue::Eighth:        return 0.5;
        case TempoNoteValue::DottedEighth:  return 0.75;
        case TempoNoteValue::Quarter:       return 1.0;
        case TempoNoteValue::DottedQuarter: return 1.5;
        case TempoNoteValue::Half:          return 2.0;
        case TempoNoteValue::DottedHalf:    return 3.0;
        case TempoNoteValue::OneBar:        return beatsPerBar;
        case TempoNoteValue::TwoBars:       return 2.0 * beatsPerBar;
        case TempoNoteValue::FourBars:      return 4.0 * beatsPerBar;
    }
    return 1.0;
}

// bpm <= 0 (host not reporting tempo, bbt.valid false) falls back to 120,
// identical to DPF's own per-format fallback (see this section's header
// comment) so a host that doesn't report tempo degrades to "acts like a
// 120 BPM host", never a divide-by-zero or a frozen/silent decay.
inline double t60SecFromTempo(double bpm, TempoNoteValue nv, double beatsPerBar) {
    if (bpm <= 1.0) bpm = 120.0;
    return (60.0 / bpm) * noteValueToQuarterNotes(nv, beatsPerBar);
}

// -----------------------------------------------------------------------
// Diffusion (allpass pre-diffuser)
// -----------------------------------------------------------------------
// The first real build of this engine sounded very metallic, like in a
// tube. This is a real, well-understood, and well-
// predicted defect, not a surprise -- docs/REVERB_CONCEPT.md's own "Known
// limitations" already named it before it was ever heard ("Diffusion...
// named in the parameter table... not implemented"). Only 8 delay lines
// (kNumLines) feeding a plain Householder mix with nothing shaping the
// *input* before it enters them means the tank's early echo density is
// low and its structure is highly periodic -- audible as exactly that
// "metallic"/comb-filtered ringing, especially before the tail
// has had time to build up density on its own.
//
// The standard fix (Schroeder 1962; the same technique Moorer's and
// Dattorro's own classic reverb designs use ahead of their own tanks): a
// short cascade of first-order allpass filters *before* the tank, raising
// echo density without altering the tank's own decay time or frequency
// response at all -- an allpass has unity magnitude response at every
// frequency by construction (`|H(e^{jw})| = 1`), so this changes *only*
// the phase/time structure of what reaches the tank, never its loudness
// or color. Verified directly, not just asserted: tests/reverb_regression.cpp
// [R12] feeds swept sine tones through one stage and confirms output RMS
// matches input RMS to within numerical precision at every frequency
// tested.
//
// y[n] = -g*x[n] + x[n-M] + g*y[n-M], the classical single-delay-line
// realization: a buffer of length M storing w[n] = x[n] + g*y[n], read
// M samples later as x[n-M]+g*y[n-M] -- read-before-write on one circular
// buffer, same shape as OnePoleDamping's own delay-free filter and this
// file's own delay lines, just for a genuinely different (all rather than
// low-pass) transfer function.
struct AllpassStage {
    static constexpr int kMaxDelay = 128; // far shorter than any kNumLines delay -- a diffuser stage, not a tank line
    std::array<double, kMaxDelay> buf{};
    int writeIdx = 0;
    int length = 37; // a fixed prime, see AllpassDiffuser::designLengths() -- overwritten there
    double gain = 0.0;

    // Length (and the buffer reset that goes with changing it) is only
    // ever sample-rate-dependent, set once at prepare()/sampleRateChanged()
    // time. Gain is the user-facing Diffusion amount and needs to be
    // changeable live -- kept as a separate setter (setGain()) precisely
    // so changing it never resets the buffer: this stage's own delayed
    // content keeps flowing continuously through a gain change, the same
    // "don't reset what doesn't need resetting" discipline
    // OnePoleDamping's own target/smoothed split already follows for the
    // tank's damping filters. A gain-only change isn't further smoothed
    // (unlike OnePoleDamping's own 5ms glide) -- a deliberate, documented
    // simplification: this is a single scalar on a short (tens-of-
    // samples) all-pass stage, whose output stays unity-magnitude
    // regardless of gain, so any transient from an abrupt gain change is
    // proportionally minor compared to the damping-filter case that
    // smoothing exists for for; a future pass can add the identical
    // target/smoothed split here too if it's ever audible in practice.
    void designLength(int lengthSamples) {
        length = std::clamp(lengthSamples, 1, kMaxDelay - 1);
        buf.fill(0.0);
        writeIdx = 0;
    }
    void setGain(double g) { gain = std::clamp(g, -0.95, 0.95); } // strictly < 1 in magnitude -- the allpass stability bound

    double process(double x) {
        const double delayed = buf[writeIdx];
        const double y = -gain * x + delayed;
        buf[writeIdx] = x + gain * y;
        writeIdx = (writeIdx + 1 >= length) ? 0 : writeIdx + 1;
        return y;
    }
};

// 4 cascaded stages, short prime-ish lengths spread so no two stages
// share a periodicity (the same anti-coloration reasoning
// designDelayLengths() already uses for the tank's own lines, just at a
// much shorter timescale -- tens of samples, not thousands) -- one
// instance per channel (independent L/R diffusion keeps the two channels
// decorrelated rather than applying literally identical phase smearing to
// both, which would do nothing for stereo width).
class AllpassDiffuser {
public:
    static constexpr int kNumStages = 4;

    // Sample-rate-dependent, resets every stage's buffer -- call from
    // prepare()/sampleRateChanged() only, never live (see AllpassStage's
    // own comment).
    void designLengths(double sampleRate) {
        // Spread roughly 10-45 samples at 48kHz (prime-ish, no shared
        // periodicity -- the same anti-coloration reasoning
        // designDelayLengths() uses for the tank's own lines, just at a
        // much shorter timescale), scaled with sample rate so the same
        // effect holds at other rates.
        static constexpr int kBaseLengths[kNumStages] = {13, 19, 29, 41};
        for (int i = 0; i < kNumStages; ++i)
            stages_[i].designLength(std::max(2, int(std::lround(kBaseLengths[i] * sampleRate / 48000.0))));
    }

    // RT-safe from any calling context (no buffer touched) -- see
    // AllpassStage::setGain()'s own comment.
    void setAmount(double amount01) {
        const double gain = 0.7 * std::clamp(amount01, 0.0, 1.0); // 0.7 is the classic Schroeder/Moorer value at full amount
        for (auto &s : stages_) s.setGain(gain);
    }

    double process(double x) {
        double y = x;
        for (auto &s : stages_) y = s.process(y);
        return y;
    }

private:
    std::array<AllpassStage, kNumStages> stages_;
};

// -----------------------------------------------------------------------
// Delay-line modulation (the direct fix for audible resonant "ringing")
// -----------------------------------------------------------------------
// Symptom: especially at low Damping, uncontrolled resonant frequencies
// that don't sound like a reverb, more like the material pitch-shifted far
// down. This is a
// real, well-documented FDN failure mode, not a surprise -- confirmed
// against outside sources rather than assumed: a small (kNumLines < 16)
// FDN's own resonant modes aren't dense enough to blend into smooth
// noise-like decay, so individual modes stay audible as discrete pitched
// ringing, worst when Damping is low (nothing removes the energy
// sustaining them quickly). The standard, decades-old fix (used in
// essentially every commercial reverb): slowly modulate each delay
// line's own length with a slow LFO, different rate/phase per line, so
// no delay line ever sits still long enough to build up a fixed
// resonance -- confirmed against outside sources: "with delay time
// modulation ... a small amount of this effect may be desirable to
// smooth out the reverb tail, though too much may create unwanted pitch
// modulation," "modulation rates around 2Hz [applied] with different
// amounts and different phases."
//
// Implemented as a small, self-contained modulated fractional delay
// (linear-interpolated, like a miniature chorus unit) inserted *inside*
// each line's own feedback loop, right after its damping filter and
// before the value is written back into that line's own circular
// buffer -- inside the loop, not just on the tap output, because the
// resonances being broken up live in the recirculating energy itself,
// not just in what's read out. Deliberately its own small buffer (64
// samples, generous headroom for a modulation depth of a few samples),
// not a change to ReverbEngine's own delay-line buf_/writeIdx_ logic --
// keeping the two completely separate means this file's own, already
// carefully-verified circular-buffer/duck/room-change machinery needed
// no changes at all to add this.
struct ModulatedDelay {
    static constexpr int kBufSize = 64;
    std::array<double, kBufSize> buf{};
    int writeIdx = 0;
    double phase = 0.0;     // 0..1, wraps
    double rate = 0.2;      // Hz -- set per-line in ReverbEngine::prepare() so lines decorrelate
    double depthSamples = 0.0; // set via ReverbEngine::setModulation()

    void reset() { buf.fill(0.0); writeIdx = 0; phase = 0.0; }

    // Read position ranges from writeIdx-0 (sin=-1) to writeIdx-2*depthSamples
    // (sin=+1) -- i.e. depthSamples==0 makes this an exact, zero-added-
    // latency passthrough (readPos == writeIdx == the sample just
    // written this same call), which is what makes "Modulation" default-
    // off/low settings a true identity against every test that predates
    // this stage, not just an approximation -- verified directly,
    // tests/reverb_regression.cpp [R16].
    double process(double x, double sampleRate) {
        buf[writeIdx] = x;
        phase += rate / sampleRate;
        if (phase >= 1.0) phase -= std::floor(phase);
        const double mod = depthSamples * (1.0 + std::sin(2.0 * M_PI * phase));
        double readPos = double(writeIdx) - mod;
        while (readPos < 0.0) readPos += double(kBufSize);
        const int i0 = int(readPos) % kBufSize;
        const int i1 = (i0 + 1) % kBufSize;
        const double frac = readPos - std::floor(readPos);
        const double y = buf[i0] * (1.0 - frac) + buf[i1] * frac;
        writeIdx = (writeIdx + 1) % kBufSize;
        return y;
    }
};

enum class DuckState { Idle, FadingOut, FadingIn };

// The late-tail engine itself: N delay lines, a lossless feedback mix,
// per-line damping, stereo in/out via alternating-line routing. See this
// file's own header comment for what is and isn't wired in yet.
class ReverbEngine {
public:
    void prepare(const RoomDims &room, double sampleRate, double t60DcSec, double t60NySec) {
        sampleRate_ = sampleRate;
        smoothCoeff_ = 1.0 - std::exp(-1.0 / (kCoeffSmoothTimeConstantSec * sampleRate));
        duckStepPerSample_ = 1.0 / (kDuckFadeTimeSec * sampleRate);
        diffuserL_.designLengths(sampleRate);
        diffuserR_.designLengths(sampleRate);
        diffuserL_.setAmount(diffusionAmount_);
        diffuserR_.setAmount(diffusionAmount_);
        lengths_ = designDelayLengths(room, sampleRate);
        lastT60Dc_ = t60DcSec;
        lastT60Ny_ = t60NySec;
        for (int i = 0; i < kNumLines; ++i) {
            buf_[i].fill(0.0);
            writeIdx_[i] = 0;
            const double r0 = OnePoleDamping::decayFactor(lengths_[i], t60DcSec, sampleRate);
            const double rNy = OnePoleDamping::decayFactor(lengths_[i], t60NySec, sampleRate);
            damp_[i].design(r0, rNy);
            damp_[i].snapToTarget(); // instant on prepare() -- nothing to click yet, buffers are silent
            damp_[i].reset();

            // Distinct rate per line so they decorrelate rather than
            // beating in sync (research: "different amounts and
            // different phases... rates around 2Hz" -- kept well under
            // that, 0.11-0.29Hz, since this project's own default use is
            // a sustained musical tail, not a percussive one, and a
            // slower sweep is less likely to read as its own vibrato).
            static constexpr double kModRateHz[kNumLines] = {0.11, 0.17, 0.13, 0.23, 0.19, 0.29, 0.15, 0.25};
            mod_[i].reset(); // clears buf_/writeIdx_/phase -- set the real starting phase right after
            mod_[i].rate = kModRateHz[i % kNumLines];
            mod_[i].phase = double(i) / double(kNumLines); // spread starting phases too, not just rates
            mod_[i].depthSamples = modulationAmount_ * kMaxModDepthSamples;
        }
    }

    // Retargets the decay only (room geometry/delay lengths untouched) --
    // safe to call every block, including from tempo-sync following a
    // live-changing host BPM. process() glides toward the new target
    // rather than snapping (see OnePoleDamping's own comment).
    void setDecayTargets(double t60DcSec, double t60NySec) {
        lastT60Dc_ = t60DcSec;
        lastT60Ny_ = t60NySec;
        for (int i = 0; i < kNumLines; ++i) {
            const double r0 = OnePoleDamping::decayFactor(lengths_[i], t60DcSec, sampleRate_);
            const double rNy = OnePoleDamping::decayFactor(lengths_[i], t60NySec, sampleRate_);
            damp_[i].design(r0, rNy);
        }
    }

    // 0 = no modulation (an exact identity, see ModulatedDelay's own
    // comment), 1 = kMaxModDepthSamples of depth on every line -- the
    // direct fix for audible resonant ringing, see ModulatedDelay's own
    // header comment. RT-safe from any calling context: unlike a buffer-
    // resetting change, a live depth change here can only ever shrink or
    // grow the range the read position sweeps within the *same* 64-sample
    // buffer, never invalidate its contents.
    void setModulation(double amount01) {
        modulationAmount_ = std::clamp(amount01, 0.0, 1.0);
        for (auto &m : mod_) m.depthSamples = modulationAmount_ * kMaxModDepthSamples;
    }
    double modulation() const { return modulationAmount_; }

    // 0 = minimal diffusion (short delay only, no real allpass effect),
    // 1 = full Schroeder/Moorer-strength diffusion -- see AllpassDiffuser's
    // own comment. RT-safe from any calling context, unlike setDecayTargets()
    // no buffer reset is ever involved.
    void setDiffusion(double amount01) {
        diffusionAmount_ = std::clamp(amount01, 0.0, 1.0);
        diffuserL_.setAmount(diffusionAmount_);
        diffuserR_.setAmount(diffusionAmount_);
    }
    double diffusion() const { return diffusionAmount_; }

    // Stereo width/spread of the tank's *own* wet output taps -- a plain
    // mid/side blend applied to outL/outR right before they're returned
    // from processStereo() (see there for the exact formula). 0 collapses
    // the tank's own stereo image to mono (outL == outR, both equal to
    // the mid signal); 1 (the default) is the tank's own natural,
    // unaltered stereo image -- mathematically an *identity* at this
    // setting (mid+side == the original outL, mid-side == the original
    // outR), so leaving Width untouched changes nothing, verified
    // directly rather than merely argued (tests/reverb_regression.cpp
    // [R13]); above 1 exaggerates the side (difference) signal, up to 2x
    // at the top of the range. Deliberately simple (a linear scalar on
    // already-computed samples, no filter/recursive state involved) --
    // unlike OnePoleDamping's own coefficients, there is no click risk
    // from changing this live, so no smoothing/target-vs-current split is
    // needed here at all.
    void setWidth(double width) { widthAmount_ = std::clamp(width, 0.0, 2.0); }
    double width() const { return widthAmount_; }

    // Balance -- a plain per-channel gain differential on the tank's own
    // wet output, applied *after* Width, for correcting (or creatively
    // shifting) any left/right lean. -1 attenuates R to silence (full left), 0 (the default)
    // leaves both channels at unity gain -- a genuine "Balance", not a
    // constant-power "Pan": a mixing-console Balance control leaves the
    // center position untouched on both channels rather than applying a
    // -3dB midpoint law, which would recolor an already-correct signal
    // just for sitting at 0. +1 attenuates L to silence (full right).
    void setBalance(double balance) { balanceAmount_ = std::clamp(balance, -1.0, 1.0); }
    double balance() const { return balanceAmount_; }

    // Same, but instant -- for use only when nothing is audible (e.g.
    // during a room-size/line-count-changing duck, mirroring
    // StateSpaceEngine's own instant swap for an order-changing edit,
    // "no meaningful in-between two different [line lengths] to
    // interpolate through"). Calling this while the tank is actually
    // producing sound reintroduces the exact click risk setDecayTargets()
    // exists to avoid -- see tests/reverb_regression.cpp [R9] for a direct
    // A/B measurement of that difference.
    void snapDecayTargets(double t60DcSec, double t60NySec) {
        setDecayTargets(t60DcSec, t60NySec);
        for (int i = 0; i < kNumLines; ++i) damp_[i].snapToTarget();
    }

    // -------------------------------------------------------------------
    // Room-size changes: thread-safe request + audio-thread-only apply
    // -------------------------------------------------------------------
    // A room-size change resizes every delay line -- prepare() rewrites
    // lengths_/buf_/writeIdx_ wholesale, exactly the "no meaningful in-
    // between two different lengths" situation StateSpaceEngine's own
    // order-changing-edit handling already names. Calling prepare()
    // directly from wherever a room-size *edit* originates (a GUI/message-
    // thread setState() call in the eventual plugin, DPF's own docs:
    // "the host may call [setState] from any non-realtime context", i.e.
    // NOT guaranteed to be the same thread as run()) would be a data race
    // against processBlock() reading/writing those exact same members on
    // the audio thread -- the identical bug class StateSpaceEngine's own
    // double-buffer/atomic-index hand-off exists to prevent, just for a
    // circular-buffer-based engine instead of a dense-matrix one.
    //
    // requestRoom() is safe to call from any thread: it publishes the new
    // dimensions through std::atomics (relaxed stores, then a release
    // store on the pending flag -- the standard "publish via flag"
    // pattern: a reader that observes the flag via an acquire load has
    // also observed the dimension stores that preceded it in program
    // order). processBlock() -- NOT the lower-level processStereo(), which
    // stays duck-unaware for tests/reverb_regression.cpp's own direct use
    // -- polls that flag once per sample on the audio thread only, and
    // when set, ducks to silence (~8ms fade, matching StateSpaceEngine's
    // own duck timing), applies the change via prepare() *while nothing is
    // audible* (safe: still the audio thread, no concurrent access
    // possible while inside processBlock() itself), then fades back in.
    // The current decay targets (lastT60Dc_/lastT60Ny_, kept up to date by
    // setDecayTargets()/snapDecayTargets()/prepare() itself) are preserved
    // across the swap, so a room-size change alone never resets Decay
    // Time/Damping back to whatever prepare() was first constructed with.
    void requestRoom(const RoomDims &room) {
        pendingLengthM_.store(room.lengthM, std::memory_order_relaxed);
        pendingWidthM_.store(room.widthM, std::memory_order_relaxed);
        pendingHeightM_.store(room.heightM, std::memory_order_relaxed);
        roomChangePending_.store(true, std::memory_order_release);
    }

    // Block-level entry point, audio-thread only -- wraps processStereo()
    // with the duck/room-change handling above. Safe to use even when no
    // room change is ever requested (duckState_ simply stays Idle).
    void processBlock(const float *inL, const float *inR, float *outL, float *outR, int numFrames) {
        for (int i = 0; i < numFrames; ++i) {
            if (duckState_ == DuckState::Idle && roomChangePending_.load(std::memory_order_acquire)) {
                duckState_ = DuckState::FadingOut;
                duckGain_ = 1.0;
            }

            double dOutL, dOutR;
            processStereo((double)inL[i], (double)inR[i], dOutL, dOutR);

            if (duckState_ == DuckState::FadingOut) {
                duckGain_ -= duckStepPerSample_;
                if (duckGain_ <= 0.0) {
                    duckGain_ = 0.0;
                    applyPendingRoom(); // swap happens exactly here, while silent
                    duckState_ = DuckState::FadingIn;
                }
            } else if (duckState_ == DuckState::FadingIn) {
                duckGain_ += duckStepPerSample_;
                if (duckGain_ >= 1.0) { duckGain_ = 1.0; duckState_ = DuckState::Idle; }
            }

            outL[i] = (float)(dOutL * duckGain_);
            outR[i] = (float)(dOutR * duckGain_);
        }
    }

    DuckState duckState() const { return duckState_; }

    // Live readback of the currently-active decay target (whichever of
    // prepare()/setDecayTargets()/snapDecayTargets() set it last) -- for a
    // GUI/diagnostic readout, e.g. to show "Decay Time: 0.50s (tempo-
    // synced)" even while Tempo Sync has overridden the manual value, or
    // simply to confirm a tempo-derived retarget actually reached the
    // engine at all.
    double currentT60DcSec() const { return lastT60Dc_; }
    double currentT60NySec() const { return lastT60Ny_; }

    // One-shot fault flag, same contract as StateSpaceEngine's own
    // kParamFaultFlag: set (relaxed store, audio thread) whenever the
    // NaN/Inf safety net above actually caught something; read-and-clear
    // from the message/UI thread via consumeFaultFlag(). This is a
    // defensive net for a design that's provably stable by construction
    // (see this file's own header comment) -- expected to never fire for
    // any in-range Room/Decay/Damping setting, exactly like the generic
    // core's identical net.
    bool consumeFaultFlag() { return faultFlag_.exchange(false, std::memory_order_relaxed); }

    void processStereo(double inL, double inR, double &outL, double &outR) {
        // Diffuse the incoming signal before it ever reaches the tank --
        // see AllpassDiffuser's own comment for why this is what actually
        // fixes the reported "metallic" character, not a decay/damping
        // tweak (an allpass changes echo density/phase structure, not
        // loudness or color, so it can't be substituted for by adjusting
        // Decay Time/Damping).
        const double diffusedL = diffuserL_.process(inL);
        const double diffusedR = diffuserR_.process(inR);

        std::array<double, kNumLines> read{};
        for (int i = 0; i < kNumLines; ++i) read[i] = buf_[i][writeIdx_[i]];

        std::array<double, kNumLines> mixed = read;
        householderMix(mixed);

        outL = 0.0;
        outR = 0.0;
        for (int i = 0; i < kNumLines; ++i) {
            double toLine = mixed[i] + ((i % 2 == 0) ? diffusedL : diffusedR);
            damp_[i].smoothTowardTarget(smoothCoeff_);
            double damped = damp_[i].process(toLine);
            // Delay-line modulation, inside the loop -- see
            // ModulatedDelay's own header comment. An exact no-op (zero
            // added latency) at modulationAmount_ == 0.0.
            damped = mod_[i].process(damped, sampleRate_);
            // NaN/Inf safety net -- see this file's own header comment on
            // why the design is provably stable and this net exists anyway.
            if (!std::isfinite(damped)) { damped = 0.0; faultFlag_.store(true, std::memory_order_relaxed); }
            buf_[i][writeIdx_[i]] = damped;
            writeIdx_[i] = (writeIdx_[i] + 1 >= lengths_[i]) ? 0 : writeIdx_[i] + 1;
            if (i % 2 == 0) outL += read[i]; else outR += read[i];
        }
        const double norm = 2.0 / (double)kNumLines;
        outL *= norm;
        outR *= norm;

        // Stereo width -- see setWidth()'s own comment. An exact identity
        // at widthAmount_ == 1.0 (the default): mid+side == outL,
        // mid-side == outR algebraically, so this changes nothing unless
        // Width has actually been moved.
        const double mid = 0.5 * (outL + outR);
        const double side = 0.5 * (outL - outR) * widthAmount_;
        outL = mid + side;
        outR = mid - side;

        // Balance -- see setBalance()'s own comment. Applied after Width
        // (a per-channel gain differential is a well-defined operation
        // regardless of whatever Width already did to the image), an
        // exact identity at balanceAmount_ == 0.0.
        outL *= 1.0 - std::max(0.0, balanceAmount_);
        outR *= 1.0 - std::max(0.0, -balanceAmount_);
    }

    const std::array<int, kNumLines> &lengths() const { return lengths_; }

private:
    void applyPendingRoom() {
        RoomDims room;
        room.lengthM = pendingLengthM_.load(std::memory_order_relaxed);
        room.widthM = pendingWidthM_.load(std::memory_order_relaxed);
        room.heightM = pendingHeightM_.load(std::memory_order_relaxed);
        room.clamp();
        prepare(room, sampleRate_, lastT60Dc_, lastT60Ny_); // preserves current decay targets
        roomChangePending_.store(false, std::memory_order_relaxed);
    }

    // Same time constant/formula as StateSpaceEngine.hpp's own
    // kCoeffSmoothTimeConstantSec -- see OnePoleDamping's own comment for why.
    static constexpr double kCoeffSmoothTimeConstantSec = 0.005;
    // Matches StateSpaceEngine's own duck timing ("~8ms") for an order-
    // changing edit -- see requestRoom()'s own comment.
    static constexpr double kDuckFadeTimeSec = 0.008;

    double sampleRate_ = 48000.0;
    double smoothCoeff_ = 0.0;
    double lastT60Dc_ = 1.8, lastT60Ny_ = 0.9; // kept current by prepare()/setDecayTargets()/snapDecayTargets()
    std::array<int, kNumLines> lengths_{};
    std::array<std::array<double, kMaxDelaySamples>, kNumLines> buf_{};
    std::array<int, kNumLines> writeIdx_{};
    std::array<OnePoleDamping, kNumLines> damp_;
    std::array<ModulatedDelay, kNumLines> mod_;

    // Room-change hand-off -- see requestRoom()'s own comment for the full
    // thread-safety argument.
    std::atomic<bool> roomChangePending_{false};
    std::atomic<double> pendingLengthM_{10.0};
    std::atomic<double> pendingWidthM_{7.0};
    std::atomic<double> pendingHeightM_{3.5};
    DuckState duckState_ = DuckState::Idle;
    double duckGain_ = 1.0;
    double duckStepPerSample_ = 1.0 / (kDuckFadeTimeSec * 48000.0);
    std::atomic<bool> faultFlag_{false};

    AllpassDiffuser diffuserL_, diffuserR_;
    double diffusionAmount_ = 0.7; // a sensibly-diffused default out of the box -- see AllpassDiffuser's own comment
    // 0.3 default, not 0 -- the direct fix for the reported "uncontrolled
    // frequencies"/pitched-ringing symptom is worth having on by default,
    // not something a user has to discover; see ModulatedDelay's own
    // comment for the research this default is grounded in (gentle, well
    // under the "too much creates unwanted pitch modulation" threshold).
    double modulationAmount_ = 0.3;
    static constexpr double kMaxModDepthSamples = 6.0; // at modulationAmount_ == 1.0
    double widthAmount_ = 1.0; // 1.0 = the tank's own natural image, unaltered -- see setWidth()'s own comment
    double balanceAmount_ = 0.0; // 0.0 = both channels at unity gain -- see setBalance()'s own comment
};

} // namespace reverb
} // namespace ssfilter
