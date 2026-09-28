// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "PluginContour.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

USE_NAMESPACE_DISTRHO
using namespace sscomp;

PluginContour::PluginContour()
    : Plugin(kParamCount, 0 /* programs */, kStateCount)
{
    sampleRateChanged(getSampleRate());
    // Primes the Parameter mirror (both banks) to match curve_/curveB_'s
    // own real default (the 2-point identity line) rather than leaving
    // mirrorPoints_/mirrorPointsB_ at their raw, all-zero initial
    // contents -- see mirrorEverSet_'s own comment for why that matters:
    // adopting this primed default later (a genuinely fresh instance,
    // mirrorReceivedFromHost_ still false at activate() time) is then a
    // harmless no-op, identical to what curve_/curveB_ already are.
    syncCurveMirrorFromModel();
    mirrorEverSet_ = true;
}

void PluginContour::syncCurveMirrorFromModel()
{
    const int n = curve_.pointCount(); // always 2..kMaxPoints by TransferCurve's own invariants
    for (int i = 0; i < n; ++i) mirrorPoints_[i] = curve_.point(i);
    for (int i = n; i < int(kNumMirroredPoints); ++i) mirrorPoints_[i] = CurvePoint();
    mirrorPointCount_ = n;
    mirrorTriggerCondition_ = triggerCondition_;
    mirrorTriggerAction_ = triggerAction_;
    mirrorThresholdLink_ = curve_.thresholdLink();

    const int nB = curveB_.pointCount();
    for (int i = 0; i < nB; ++i) mirrorPointsB_[i] = curveB_.point(i);
    for (int i = nB; i < int(kNumMirroredPoints); ++i) mirrorPointsB_[i] = CurvePoint();
    mirrorPointCountB_ = nB;
    mirrorTriggerConditionB_ = triggerConditionB_;
    mirrorTriggerActionB_ = triggerActionB_;
    mirrorThresholdLinkB_ = curveB_.thresholdLink();
}

void PluginContour::adoptMirrorIfCurveNotReceived()
{
    // mirrorEverSet_ alone can't tell "the constructor's own self-primed
    // default" apart from "genuine data, import-restored or otherwise" --
    // see mirrorReceivedFromHost_'s own comment (PluginContour.hpp).
    if (!mirrorReceivedFromHost_) return;
    if (mirrorAdopted_) return;
    mirrorAdopted_ = true;
    applyMirrorToModel();
}

void PluginContour::applyMirrorToModel()
{
    // loadPoints() itself now carries handleInputDb/handleOutputDb (and
    // direction) through as part of each CurvePoint -- see
    // plugin/dsp/CurveDsp.hpp's own header comment on "per-point
    // interpolation" for why there's no separate interp mode to restore
    // here any more.
    curve_.loadPoints(mirrorPoints_, mirrorPointCount_, mirrorThresholdLink_);
    triggerCondition_ = mirrorTriggerCondition_;
    triggerAction_ = mirrorTriggerAction_;

    curveB_.loadPoints(mirrorPointsB_, mirrorPointCountB_, mirrorThresholdLinkB_);
    triggerConditionB_ = mirrorTriggerConditionB_;
    triggerActionB_ = mirrorTriggerActionB_;
}

void PluginContour::initAudioPort(bool input, uint32_t index, AudioPort& port)
{
    if (kContourMono)
    {
        // Mono build: input 0 = main, input 1 = the external sidechain/key, output 0.
        if (input && index == 1)
        {
            port.hints = kAudioPortIsSidechain;
            port.name = "Sidechain";
            port.symbol = "sidechain_in";
            return;
        }
        port.name = input ? "Input" : "Output";
        port.symbol = input ? "audio_in" : "audio_out";
        return;
    }
    // See PluginCompressorStereo::initAudioPort()'s identical comment:
    // indices 0/1 are always the main L/R pair, 2/3 (input only) are the
    // external sidechain/key signal's own L/R pair.
    if (input && index >= 2)
    {
        port.hints = kAudioPortIsSidechain;
        port.name = (index == 2) ? "Sidechain L" : "Sidechain R";
        port.symbol = (index == 2) ? "sidechain_in_l" : "sidechain_in_r";
        return;
    }
    port.groupId = kPortGroupStereo;
    Plugin::initAudioPort(input, index, port);
}

void PluginContour::initParameter(uint32_t index, Parameter& parameter)
{
    switch (index)
    {
    case kParamInputGain:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Input Gain";
        parameter.symbol = "input_gain";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamThreshold:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Threshold";
        parameter.symbol = "threshold";
        parameter.unit = "dB";
        parameter.ranges.def = -18.0f;
        parameter.ranges.min = float(TransferCurve::kMinDb);
        parameter.ranges.max = float(TransferCurve::kMaxDb);
        break;
    case kParamAttack:
        // No kParameterIsLogarithmic here (unlike the sibling Compressor/
        // Limiter plugins' own Attack) -- a log-scale hint is undefined
        // right at min=0 (log(0)), and this plugin's own min really is 0 --
        // 0 means genuinely instant, see CurveDsp.hpp's
        // CurveVoice/EnvelopeFollower. A host's own generic slider for
        // this parameter is linear as a result; this project's own GUI
        // field (ParamStripWidget) still drags/scrolls it in a pseudo-log
        // feel independent of this hint, see refreshStripFields().
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Attack";
        parameter.symbol = "attack";
        parameter.unit = "ms";
        parameter.ranges.def = 10.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 300.0f;
        break;
    case kParamRelease:
        parameter.hints = kParameterIsAutomatable; // see kParamAttack's own comment just above
        parameter.name = "Release";
        parameter.symbol = "release";
        parameter.unit = "ms";
        parameter.ranges.def = 150.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 2000.0f;
        break;
    case kParamOutputGain:
        // Renamed from "Output Gain" to "Makeup Gain" -- symbol
        // and index untouched, only the display name changes -- see
        // ../plugin_stereo/PluginCompressorStereo.cpp's identical comment
        // on its own kParamOutputGain for why that's fully session-
        // compatible.
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Makeup Gain";
        parameter.symbol = "output_gain";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    // Removed -- see kParamMeterCompGainDb's own comment below for why
    // these stay declared instead of being deleted. Names marked
    // "(unused)" so they read as vestigial if anything shows them outside
    // this plugin's own GUI.
    case kParamCompensation:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Volume Compensation (unused)";
        parameter.symbol = "compensation";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamCompensationMeasure:
        parameter.hints = kParameterIsAutomatable | kParameterIsTrigger;
        parameter.name = "Recalibrate Compensation (unused)";
        parameter.symbol = "compensation_measure";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamBypass:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Bypass";
        parameter.symbol = "bypass";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamSidechainEnable:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Trigger Source: External Sidechain";
        parameter.symbol = "sidechain_enable";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamSidechainGain:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Trigger Gain";
        parameter.symbol = "sidechain_gain";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = float(sscomp::kSidechainGainMinDb); parameter.ranges.max = float(sscomp::kSidechainGainMaxDb);
        break;

    case kParamMeterInputRmsL:
    case kParamMeterInputPeakL:
    case kParamMeterInputRmsR:
    case kParamMeterInputPeakR:
    case kParamMeterOutputRmsL:
    case kParamMeterOutputPeakL:
    case kParamMeterOutputRmsR:
    case kParamMeterOutputPeakR:
    {
        static const char* names[] = {
            "Input RMS L", "Input Peak L", "Input RMS R", "Input Peak R",
            "Output RMS L", "Output Peak L", "Output RMS R", "Output Peak R"
        };
        static const char* symbols[] = {
            "meter_in_rms_l", "meter_in_peak_l", "meter_in_rms_r", "meter_in_peak_r",
            "meter_out_rms_l", "meter_out_peak_l", "meter_out_rms_r", "meter_out_peak_r"
        };
        const uint32_t i = index - kParamMeterInputRmsL;
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = names[i];
        parameter.symbol = symbols[i];
        parameter.unit = "dB";
        parameter.ranges.def = -120.0f; parameter.ranges.min = -120.0f; parameter.ranges.max = 12.0f;
        break;
    }
    case kParamMeterDetectorA:
    case kParamMeterDetectorB:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = (index == kParamMeterDetectorA) ? "Detector A Level" : "Detector B Level";
        parameter.symbol = (index == kParamMeterDetectorA) ? "meter_detector_a" : "meter_detector_b";
        parameter.unit = "dB";
        parameter.ranges.def = -120.0f; parameter.ranges.min = -120.0f; parameter.ranges.max = 12.0f;
        break;
    case kParamMeterGainReductionA:
    case kParamMeterGainReductionB:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = (index == kParamMeterGainReductionA) ? "Gain Reduction A" : "Gain Reduction B";
        parameter.symbol = (index == kParamMeterGainReductionA) ? "meter_gr_a" : "meter_gr_b";
        parameter.unit = "dB";
        // Wide enough for both a gate's own deep cut and an upward-shaped
        // curve's boost -- see TransferCurve::kMinDb/kMaxDb.
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = float(TransferCurve::kMinDb);
        parameter.ranges.max = float(TransferCurve::kMaxDb) + 24.0f; // curve's own output can exceed input by up to its own range width
        break;
    // Always report 0 now -- see kParamCompensation's own comment above.
    case kParamMeterCompGainDb:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Compensation Gain (unused)";
        parameter.symbol = "meter_comp_gain";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamMeterCompState:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Compensation State (unused)";
        parameter.symbol = "meter_comp_state";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 2.0f;
        break;
    // The curve/trigger-condition Parameter mirror -- see ParameterIndex's
    // own comment. All hidden/automatable-only, same convention as
    // ../statespace/plugin_eq/PluginEq.cpp's identical bank.
    case kParamCurvePointCount:
    case kParamCurvePointCountB:
        parameter.hints = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
        parameter.name = (index == kParamCurvePointCount) ? "Curve Point Count (internal)" : "Curve Point Count B (internal)";
        parameter.symbol = (index == kParamCurvePointCount) ? "mirror_curve_point_count" : "mirror_curve_point_count_b";
        parameter.ranges.def = 2.0f;
        parameter.ranges.min = 2.0f; // a curve always has at least its 2 fixed endpoints
        parameter.ranges.max = float(kNumMirroredPoints);
        break;
    // Vestigial -- interpolation moved per-point (a tangent handle, see
    // kParamCurvePointHandleInBase's own comment) this session, replacing
    // the old curve-wide Linear/Spline choice this pair used to mirror.
    // Declared at their exact existing positions and always report 0 now
    // -- same "stays declared, renamed (unused), no live effect" as
    // kParamCompensation's own removal earlier this session, not deleted
    // outright (deleting would renumber every Parameter after them).
    case kParamCurveInterp:
    case kParamCurveInterpB:
        parameter.hints = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
        parameter.name = (index == kParamCurveInterp) ? "Curve Interp (unused)" : "Curve Interp B (unused)";
        parameter.symbol = (index == kParamCurveInterp) ? "mirror_curve_interp" : "mirror_curve_interp_b";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 1.0f;
        break;
    case kParamCurveTriggerCondition:
    case kParamCurveTriggerConditionB:
        parameter.hints = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
        parameter.name = (index == kParamCurveTriggerCondition) ? "Curve Trigger Condition (internal)" : "Curve Trigger Condition B (internal)";
        parameter.symbol = (index == kParamCurveTriggerCondition) ? "mirror_curve_trigger" : "mirror_curve_trigger_b";
        parameter.ranges.def = 0.0f; // sscomp::TriggerCondition::Above
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = float(1 + 2 * (sscomp::kMaxLinkGroups + 10)); // condition + 2*(threshold link group + 10*action)
        break;
    case kParamMeterTrailA:
    case kParamMeterTrailB:
        parameter.hints = kParameterIsOutput | kParameterIsInteger | kParameterIsHidden;
        parameter.name = (index == kParamMeterTrailA) ? "Trail Pair A (internal)" : "Trail Pair B (internal)";
        parameter.symbol = (index == kParamMeterTrailA) ? "meter_trail_a" : "meter_trail_b";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -16777215.0f; parameter.ranges.max = 16777215.0f; // sign bit = triggered flag
        break;
    case kParamListenMode:
        parameter.hints = kParameterIsAutomatable | kParameterIsInteger;
        parameter.name = "Listen (M/S)";
        parameter.symbol = "listen_mode";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 2.0f;
        break;
    case kParamThresholdB:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Threshold B";
        parameter.symbol = "threshold_b";
        parameter.unit = "dB";
        parameter.ranges.def = -18.0f;
        parameter.ranges.min = float(TransferCurve::kMinDb);
        parameter.ranges.max = float(TransferCurve::kMaxDb);
        break;
    case kParamAttackB:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Attack B";
        parameter.symbol = "attack_b";
        parameter.unit = "ms";
        parameter.ranges.def = 10.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 300.0f;
        break;
    case kParamReleaseB:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Release B";
        parameter.symbol = "release_b";
        parameter.unit = "ms";
        parameter.ranges.def = 150.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 2000.0f;
        break;
    case kParamMakeupGainB:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Makeup Gain B";
        parameter.symbol = "makeup_gain_b";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamGlobalOutputGain:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Output Gain";
        parameter.symbol = "global_output_gain";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamScFilterEnable:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Trigger Filter";
        parameter.symbol = "sc_filter_enable";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamScFilterHighpassHz:
        parameter.hints = kParameterIsAutomatable | kParameterIsLogarithmic;
        parameter.name = "Trigger Filter Highpass";
        parameter.symbol = "sc_filter_highpass_hz";
        parameter.unit = "Hz";
        parameter.ranges.def = 100.0f; parameter.ranges.min = 20.0f; parameter.ranges.max = 8000.0f;
        break;
    case kParamScFilterLowpassHz:
        parameter.hints = kParameterIsAutomatable | kParameterIsLogarithmic;
        parameter.name = "Trigger Filter Lowpass";
        parameter.symbol = "sc_filter_lowpass_hz";
        parameter.unit = "Hz";
        parameter.ranges.def = 8000.0f; parameter.ranges.min = 100.0f; parameter.ranges.max = 20000.0f;
        break;
    case kParamScFilterLinkLR:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Trigger Filter Link L/R";
        parameter.symbol = "sc_filter_link_lr";
        parameter.ranges.def = 1.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamScFilterHighpassHzR:
        parameter.hints = kParameterIsAutomatable | kParameterIsLogarithmic;
        parameter.name = "Trigger Filter Highpass R";
        parameter.symbol = "sc_filter_highpass_hz_r";
        parameter.unit = "Hz";
        parameter.ranges.def = 100.0f; parameter.ranges.min = 20.0f; parameter.ranges.max = 8000.0f;
        break;
    case kParamScFilterLowpassHzR:
        parameter.hints = kParameterIsAutomatable | kParameterIsLogarithmic;
        parameter.name = "Trigger Filter Lowpass R";
        parameter.symbol = "sc_filter_lowpass_hz_r";
        parameter.unit = "Hz";
        parameter.ranges.def = 8000.0f; parameter.ranges.min = 100.0f; parameter.ranges.max = 20000.0f;
        break;
    case kParamInputGainB:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Input Gain B";
        parameter.symbol = "input_gain_b";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamWetDry:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Wet/Dry";
        parameter.symbol = "wet_dry";
        parameter.unit = "%";
        parameter.ranges.def = 100.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 100.0f;
        break;
    case kParamWetDryB:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Wet/Dry B";
        parameter.symbol = "wet_dry_b";
        parameter.unit = "%";
        parameter.ranges.def = 100.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 100.0f;
        break;
    case kParamGlobalInputGain:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Global Input Gain";
        parameter.symbol = "global_input_gain";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamMeterScLevelDbL:
    case kParamMeterScLevelDbR:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = (index == kParamMeterScLevelDbL) ? "Trigger Monitor Level L" : "Trigger Monitor Level R";
        parameter.symbol = (index == kParamMeterScLevelDbL) ? "meter_sc_level_l" : "meter_sc_level_r";
        parameter.unit = "dB";
        parameter.ranges.def = -120.0f; parameter.ranges.min = -120.0f; parameter.ranges.max = 12.0f;
        break;
    case kParamScListenEnable:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Trigger Listen";
        parameter.symbol = "sc_listen_enable";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamMirrorCommit:
        // NOT kParameterIsTrigger -- see kParamMirrorCommit's own comment
        // (PluginContour.hpp) for why.
        parameter.hints = kParameterIsAutomatable | kParameterIsHidden;
        parameter.name = "Apply Curve (internal)";
        parameter.symbol = "mirror_commit";
        // A counter (increases with every edit), not a 0/1 toggle -- see UIContour::pushMirrorCommit().
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 1000000.0f;
        break;
    case kParamMirrorCommitB:
        // See kParamMirrorCommitB's own comment (PluginContour.hpp).
        parameter.hints = kParameterIsAutomatable | kParameterIsHidden;
        parameter.name = "Apply Curve B (internal)";
        parameter.symbol = "mirror_commit_b";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 1000000.0f; // counter, see kParamMirrorCommit
        break;
    case kParamTrailSource:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean | kParameterIsHidden;
        parameter.name = "Trail Dots: Detector Level (internal)";
        parameter.symbol = "trail_source";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamUiHeartbeat:
        parameter.hints = kParameterIsAutomatable | kParameterIsHidden;
        parameter.name = "Scope Active (internal)";
        parameter.symbol = "ui_heartbeat";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1000000.0f; // counter
        break;
    case kParamLookahead:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Lookahead";
        parameter.symbol = "lookahead";
        parameter.unit = "ms";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = float(kLookaheadMaxMs);
        break;
    default:
        if (index >= kParamMeterTrailBankA && index < kParamMeterTrailBankA + 2 * kTrailBankSize)
        {
            // The live trail's bank of (level, gain) pairs -- see kParamMeterTrailBankA.
            const bool isB = index >= kParamMeterTrailBankB;
            const uint32_t slot = index - (isB ? kParamMeterTrailBankB : kParamMeterTrailBankA);
            char symbol[32];
            std::snprintf(symbol, sizeof(symbol), isB ? "meter_trail_b_%u" : "meter_trail_a_%u", slot);
            parameter.hints = kParameterIsOutput | kParameterIsInteger | kParameterIsHidden;
            parameter.name = isB ? "Trail Bank B (internal)" : "Trail Bank A (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f; parameter.ranges.min = -16777215.0f; parameter.ranges.max = 16777215.0f; // sign bit = triggered flag
        }
        else if (index >= kParamCurvePointInBase && index < kParamCurvePointInBase + kNumMirroredPoints)
        {
            const uint32_t slot = index - kParamCurvePointInBase;
            char symbol[32];
            std::snprintf(symbol, sizeof(symbol), "mirror_curve_%u_in", slot);
            parameter.hints = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name = "Curve Point In (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = float(TransferCurve::kMinDb);
            parameter.ranges.min = float(TransferCurve::kMinDb);
            parameter.ranges.max = float(TransferCurve::kMaxDb);
        }
        else if (index >= kParamCurvePointOutBase && index < kParamCurvePointOutBase + kNumMirroredPoints)
        {
            const uint32_t slot = index - kParamCurvePointOutBase;
            char symbol[32];
            std::snprintf(symbol, sizeof(symbol), "mirror_curve_%u_out", slot);
            parameter.hints = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name = "Curve Point Out (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = float(TransferCurve::kMinDb);
            parameter.ranges.min = float(TransferCurve::kMinDb);
            parameter.ranges.max = float(TransferCurve::kMaxDb);
        }
        else if (index >= kParamCurvePointInBaseB && index < kParamCurvePointInBaseB + kNumMirroredPoints)
        {
            const uint32_t slot = index - kParamCurvePointInBaseB;
            char symbol[32];
            std::snprintf(symbol, sizeof(symbol), "mirror_curve_%u_in_b", slot);
            parameter.hints = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name = "Curve Point In B (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = float(TransferCurve::kMinDb);
            parameter.ranges.min = float(TransferCurve::kMinDb);
            parameter.ranges.max = float(TransferCurve::kMaxDb);
        }
        else if (index >= kParamCurvePointOutBaseB && index < kParamCurvePointOutBaseB + kNumMirroredPoints)
        {
            const uint32_t slot = index - kParamCurvePointOutBaseB;
            char symbol[32];
            std::snprintf(symbol, sizeof(symbol), "mirror_curve_%u_out_b", slot);
            parameter.hints = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name = "Curve Point Out B (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = float(TransferCurve::kMinDb);
            parameter.ranges.min = float(TransferCurve::kMinDb);
            parameter.ranges.max = float(TransferCurve::kMaxDb);
        }
        else if (index >= kParamCurvePointDirBase && index < kParamCurvePointDirBase + kNumMirroredPoints)
        {
            const uint32_t slot = index - kParamCurvePointDirBase;
            char symbol[32];
            std::snprintf(symbol, sizeof(symbol), "mirror_curve_%u_dir", slot);
            parameter.hints = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
            parameter.name = "Curve Point Direction (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = float(2 + 3 * (sscomp::kMaxLinkGroups + 10 * sscomp::kLinkAxisY));
        }
        else if (index >= kParamCurvePointDirBaseB && index < kParamCurvePointDirBaseB + kNumMirroredPoints)
        {
            const uint32_t slot = index - kParamCurvePointDirBaseB;
            char symbol[32];
            std::snprintf(symbol, sizeof(symbol), "mirror_curve_%u_dir_b", slot);
            parameter.hints = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
            parameter.name = "Curve Point Direction B (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = float(2 + 3 * (sscomp::kMaxLinkGroups + 10 * sscomp::kLinkAxisY));
        }
        // Per-point Inkscape-style tangent handle mirror -- see
        // kParamCurvePointHandleInBase's own comment (PluginContour.hpp).
        else if (index >= kParamCurvePointHandleInBase && index < kParamCurvePointHandleInBase + kNumMirroredPoints)
        {
            const uint32_t slot = index - kParamCurvePointHandleInBase;
            char symbol[32];
            std::snprintf(symbol, sizeof(symbol), "mirror_curve_%u_hin", slot);
            parameter.hints = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name = "Curve Point Handle In (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f; parameter.ranges.min = -float(TransferCurve::kMaxDb - TransferCurve::kMinDb); // dx may be negative, see CurvePoint::handleInputDb
            parameter.ranges.max = float(TransferCurve::kMaxDb - TransferCurve::kMinDb);
        }
        else if (index >= kParamCurvePointHandleOutBase && index < kParamCurvePointHandleOutBase + kNumMirroredPoints)
        {
            const uint32_t slot = index - kParamCurvePointHandleOutBase;
            char symbol[32];
            std::snprintf(symbol, sizeof(symbol), "mirror_curve_%u_hout", slot);
            parameter.hints = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name = "Curve Point Handle Out (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = -float(TransferCurve::kMaxDb - TransferCurve::kMinDb);
            parameter.ranges.max = float(TransferCurve::kMaxDb - TransferCurve::kMinDb);
        }
        else if (index >= kParamCurvePointHandleInBaseB && index < kParamCurvePointHandleInBaseB + kNumMirroredPoints)
        {
            const uint32_t slot = index - kParamCurvePointHandleInBaseB;
            char symbol[32];
            std::snprintf(symbol, sizeof(symbol), "mirror_curve_%u_hin_b", slot);
            parameter.hints = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name = "Curve Point Handle In B (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f; parameter.ranges.min = -float(TransferCurve::kMaxDb - TransferCurve::kMinDb); // dx may be negative, see CurvePoint::handleInputDb
            parameter.ranges.max = float(TransferCurve::kMaxDb - TransferCurve::kMinDb);
        }
        else if (index >= kParamCurvePointHandleOutBaseB && index < kParamCurvePointHandleOutBaseB + kNumMirroredPoints)
        {
            const uint32_t slot = index - kParamCurvePointHandleOutBaseB;
            char symbol[32];
            std::snprintf(symbol, sizeof(symbol), "mirror_curve_%u_hout_b", slot);
            parameter.hints = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name = "Curve Point Handle Out B (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = -float(TransferCurve::kMaxDb - TransferCurve::kMinDb);
            parameter.ranges.max = float(TransferCurve::kMaxDb - TransferCurve::kMinDb);
        }
        // The two packed sidechain-monitor spectrum banks -- Out
        // (kParamMeterScSpectrumBase, post-filter/whatever's currently
        // reaching the detector) and In (kParamMeterScSpectrumInBase,
        // always pre-filter) -- see kParamMeterScSpectrumInBase's own
        // comment (PluginContour.hpp) and SpectrumAnalyzer.hpp's own
        // "Packed transport" comment for the full reasoning.
        else if (index >= kParamMeterScSpectrumBase && index < kParamMeterScSpectrumBase + sscomp::SpectrumAnalyzer::kNumPackedBands)
        {
            const uint32_t band = index - kParamMeterScSpectrumBase;
            char name[32], symbol[32];
            std::snprintf(name, sizeof(name), "SC Spectrum Out Packet %u", band);
            std::snprintf(symbol, sizeof(symbol), "meter_sc_spectrum_out_%u", band);
            parameter.hints = kParameterIsAutomatable | kParameterIsOutput | kParameterIsInteger;
            parameter.name = name;
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = sscomp::SpectrumAnalyzer::kMaxPackedValue;
        }
        else if (index >= kParamMeterScSpectrumInBase && index < kParamInputGainB)
        {
            const uint32_t band = index - kParamMeterScSpectrumInBase;
            char name[32], symbol[32];
            std::snprintf(name, sizeof(name), "SC Spectrum In Packet %u", band);
            std::snprintf(symbol, sizeof(symbol), "meter_sc_spectrum_in_%u", band);
            parameter.hints = kParameterIsAutomatable | kParameterIsOutput | kParameterIsInteger;
            parameter.name = name;
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = sscomp::SpectrumAnalyzer::kMaxPackedValue;
        }
        if (index >= kParamCurvePointInBase) { mirrorDirty_ = true; mirrorIdleBlocks_ = 0; } // any mirror data field: see mirrorDirty_
        break;
    }
}

void PluginContour::initState(uint32_t index, State& state)
{
    switch (index)
    {
    case kStateStereoMode:
        state.key = "stereoMode";
        state.label = "Stereo Mode (Linked/DualMono/MidSide)";
        state.hints = 0; // DSP-relevant (see setState()), but a discrete topology choice, not a continuous automatable control
        state.defaultValue = "0";
        break;
    case kStatePresetDir:
        state.key = "presetDir";
        state.label = "Last Preset Directory";
        state.hints = 0; // UI convenience only, the DSP side never reads this
        state.defaultValue = "";
        break;
    // A/B compare, made permanent -- see StateIndex's own comment.
    case kStateAbSlotA:
        state.key = "abSlotA"; state.label = "A/B Slot A"; state.hints = 0; state.defaultValue = "";
        break;
    case kStateAbSlotB:
        state.key = "abSlotB"; state.label = "A/B Slot B"; state.hints = 0; state.defaultValue = "";
        break;
    case kStateAbHasA:
        state.key = "abHasA"; state.label = "A/B Slot A Populated"; state.hints = 0; state.defaultValue = "0";
        break;
    case kStateAbHasB:
        state.key = "abHasB"; state.label = "A/B Slot B Populated"; state.hints = 0; state.defaultValue = "0";
        break;
    case kStateAbActive:
        state.key = "abActive"; state.label = "A/B Active Slot"; state.hints = 0; state.defaultValue = "0";
        break;
    case kStateLinkAB:
        state.key = "linkAB";
        state.label = "Link A/B Dynamics (Dual-Mono only)";
        state.hints = 0; // DSP-relevant (see setState()), but a discrete two-way choice, not a continuous automatable control
        state.defaultValue = "1"; // linked -- matches the sibling compressor's own pre-existing default
        break;
    }
}

float PluginContour::getParameterValue(uint32_t index) const
{
    switch (index)
    {
    case kParamInputGain:  return float(inputGainDb_);
    case kParamThreshold:  return float(thresholdDb_);
    case kParamAttack:     return float(attackMs_);
    case kParamRelease:    return float(releaseMs_);
    case kParamOutputGain: return float(outputGainDb_);
    case kParamCompensation: return compensationEnabled_ ? 1.0f : 0.0f;
    case kParamBypass:       return bypass_ ? 1.0f : 0.0f;
    case kParamSidechainEnable: return sidechainEnabled_ ? 1.0f : 0.0f;
    case kParamSidechainGain:   return float(sidechainGainDb_);

    case kParamMeterInputRmsL:  return meterInputRmsL_;
    case kParamMeterInputPeakL: return meterInputPeakL_;
    case kParamMeterInputRmsR:  return meterInputRmsR_;
    case kParamMeterInputPeakR: return meterInputPeakR_;
    case kParamMeterOutputRmsL:  return meterOutputRmsL_;
    case kParamMeterOutputPeakL: return meterOutputPeakL_;
    case kParamMeterOutputRmsR:  return meterOutputRmsR_;
    case kParamMeterOutputPeakR: return meterOutputPeakR_;
    case kParamMeterDetectorA: return meterDetectorA_;
    case kParamMeterDetectorB: return meterDetectorB_;
    case kParamMeterGainReductionA: return meterGainReductionA_;
    case kParamMeterGainReductionB: return meterGainReductionB_;
    case kParamTrailSource: return trailDetectorMode_ ? 1.0f : 0.0f;
    case kParamLookahead:   return float(lookaheadMs_);
    case kParamUiHeartbeat: return 0.0f; // write-only
    case kParamMeterTrailA: return meterTrailA_;
    case kParamMeterTrailB: return meterTrailB_;
    // Always 0 now -- see kParamCompensation's own comment (initParameter()).
    case kParamMeterCompGainDb: return 0.0f;
    case kParamMeterCompState:  return 0.0f;
    // The Parameter mirror -- see ParameterIndex's own comment.
    case kParamCurvePointCount: return float(mirrorPointCount_);
    case kParamCurveInterp:     return 0.0f; // vestigial, see initParameter()'s own comment
    case kParamCurveTriggerCondition: return sscomp::packConditionAndThresholdLink(int(mirrorTriggerCondition_), mirrorThresholdLink_, int(mirrorTriggerAction_));
    case kParamCurvePointCountB: return float(mirrorPointCountB_);
    case kParamCurveInterpB:     return 0.0f;
    case kParamCurveTriggerConditionB: return sscomp::packConditionAndThresholdLink(int(mirrorTriggerConditionB_), mirrorThresholdLinkB_, int(mirrorTriggerActionB_));
    case kParamMirrorCommit: return 0.0f; // write-only trigger, nothing meaningful to read back
    case kParamMirrorCommitB: return 0.0f; // ditto -- see kParamMirrorCommitB's own comment (PluginContour.hpp)

    case kParamListenMode: return float(listenMode_);
    case kParamThresholdB: return float(thresholdDbB_);
    case kParamAttackB:    return float(attackMsB_);
    case kParamReleaseB:   return float(releaseMsB_);
    case kParamMakeupGainB: return float(makeupGainDbB_);
    case kParamGlobalOutputGain: return float(globalOutputGainDb_);
    case kParamScFilterEnable:     return scFilterEnabled_ ? 1.0f : 0.0f;
    case kParamScFilterHighpassHz: return float(scFilterHighpassHz_);
    case kParamScFilterLowpassHz:  return float(scFilterLowpassHz_);
    case kParamScFilterLinkLR:      return scFilterLinkLR_ ? 1.0f : 0.0f;
    case kParamScFilterHighpassHzR: return float(scFilterHighpassHzR_);
    case kParamScFilterLowpassHzR:  return float(scFilterLowpassHzR_);
    case kParamInputGainB: return float(inputGainDbB_);
    case kParamWetDry:  return float(wetDry_ * 100.0);
    case kParamWetDryB: return float(wetDryB_ * 100.0);
    case kParamGlobalInputGain: return float(globalInputGainDb_);
    case kParamMeterScLevelDbL: return float(scMonitorPeakL_.peakDb());
    case kParamMeterScLevelDbR: return float(scMonitorPeakR_.peakDb());
    case kParamScListenEnable: return scListenEnabled_ ? 1.0f : 0.0f;
    default:
        if (index >= kParamMeterTrailBankA && index < kParamMeterTrailBankA + kTrailBankSize)
            return meterTrailBankA_[index - kParamMeterTrailBankA];
        if (index >= kParamMeterTrailBankB && index < kParamMeterTrailBankB + kTrailBankSize)
            return meterTrailBankB_[index - kParamMeterTrailBankB];
        if (index >= kParamCurvePointInBase && index < kParamCurvePointInBase + kNumMirroredPoints)
            return float(mirrorPoints_[index - kParamCurvePointInBase].inputDb);
        if (index >= kParamCurvePointOutBase && index < kParamCurvePointOutBase + kNumMirroredPoints)
            return float(mirrorPoints_[index - kParamCurvePointOutBase].outputDb);
        if (index >= kParamCurvePointInBaseB && index < kParamCurvePointInBaseB + kNumMirroredPoints)
            return float(mirrorPointsB_[index - kParamCurvePointInBaseB].inputDb);
        if (index >= kParamCurvePointOutBaseB && index < kParamCurvePointOutBaseB + kNumMirroredPoints)
            return float(mirrorPointsB_[index - kParamCurvePointOutBaseB].outputDb);
        if (index >= kParamCurvePointDirBase && index < kParamCurvePointDirBase + kNumMirroredPoints)
            return sscomp::packDirectionAndLink(mirrorPoints_[index - kParamCurvePointDirBase].direction, mirrorPoints_[index - kParamCurvePointDirBase].link, mirrorPoints_[index - kParamCurvePointDirBase].linkAxis, mirrorPoints_[index - kParamCurvePointDirBase].diagonal);
        if (index >= kParamCurvePointDirBaseB && index < kParamCurvePointDirBaseB + kNumMirroredPoints)
            return sscomp::packDirectionAndLink(mirrorPointsB_[index - kParamCurvePointDirBaseB].direction, mirrorPointsB_[index - kParamCurvePointDirBaseB].link, mirrorPointsB_[index - kParamCurvePointDirBaseB].linkAxis, mirrorPointsB_[index - kParamCurvePointDirBaseB].diagonal);
        if (index >= kParamCurvePointHandleInBase && index < kParamCurvePointHandleInBase + kNumMirroredPoints)
            return float(mirrorPoints_[index - kParamCurvePointHandleInBase].handleInputDb);
        if (index >= kParamCurvePointHandleOutBase && index < kParamCurvePointHandleOutBase + kNumMirroredPoints)
            return float(mirrorPoints_[index - kParamCurvePointHandleOutBase].handleOutputDb);
        if (index >= kParamCurvePointHandleInBaseB && index < kParamCurvePointHandleInBaseB + kNumMirroredPoints)
            return float(mirrorPointsB_[index - kParamCurvePointHandleInBaseB].handleInputDb);
        if (index >= kParamCurvePointHandleOutBaseB && index < kParamCurvePointHandleOutBaseB + kNumMirroredPoints)
            return float(mirrorPointsB_[index - kParamCurvePointHandleOutBaseB].handleOutputDb);
        if (index >= kParamMeterScSpectrumBase && index < kParamMeterScSpectrumBase + sscomp::SpectrumAnalyzer::kNumPackedBands)
            return scMonitorSpectrumOut_.getPackedTriple(int(index - kParamMeterScSpectrumBase));
        if (index >= kParamMeterScSpectrumInBase && index < kParamInputGainB)
            return scMonitorSpectrumIn_.getPackedTriple(int(index - kParamMeterScSpectrumInBase));
        return 0.0f;
    }
}

void PluginContour::setParameterValue(uint32_t index, float value)
{
    switch (index)
    {
    case kParamInputGain:  inputGainDb_ = value; break;
    case kParamThreshold:  thresholdDb_ = value; break;
    case kParamAttack:     attackMs_ = value; break;
    case kParamRelease:    releaseMs_ = value; break;
    case kParamOutputGain: outputGainDb_ = value; break;
    // Inert -- see kParamCompensation's own comment (initParameter()).
    case kParamCompensation: compensationEnabled_ = value >= 0.5f; break;
    case kParamCompensationMeasure: break;
    case kParamBypass: bypass_ = value >= 0.5f; break;
    case kParamSidechainEnable: sidechainEnabled_ = value >= 0.5f; break;
    case kParamSidechainGain:   sidechainGainDb_ = value; break;
    // The Parameter mirror -- see PluginEq::setParameterValue()'s
    // identical block (../statespace/plugin_eq/PluginEq.cpp) for the full
    // reasoning: order-independent fixed slots, no engine rebuild here
    // (that only ever happens once, from activate()'s own
    // adoptMirrorIfCurveNotReceived()).
    case kParamCurvePointCount:
        mirrorPointCount_ = std::max(2, std::min(int(kNumMirroredPoints), int(value + 0.5f)));
        mirrorEverSet_ = true;
        mirrorDirty_ = true; mirrorIdleBlocks_ = 0;
        mirrorReceivedFromHost_ = true;
        break;
    case kParamCurveInterp:  break; // vestigial, see initParameter()'s own comment
    case kParamCurveTriggerCondition:
        mirrorTriggerCondition_ = clampTriggerCondition(sscomp::unpackConditionInt(value));
        mirrorThresholdLink_ = sscomp::unpackThresholdLink(value);
        mirrorTriggerAction_ = sscomp::clampTriggerAction(sscomp::unpackTriggerActionInt(value));
        mirrorEverSet_ = true;
        mirrorDirty_ = true; mirrorIdleBlocks_ = 0;
        break;
    case kParamCurvePointCountB:
        mirrorPointCountB_ = std::max(2, std::min(int(kNumMirroredPoints), int(value + 0.5f)));
        mirrorEverSet_ = true;
        mirrorDirty_ = true; mirrorIdleBlocks_ = 0;
        mirrorReceivedFromHost_ = true;
        break;
    case kParamCurveInterpB: break;
    case kParamCurveTriggerConditionB:
        mirrorTriggerConditionB_ = clampTriggerCondition(sscomp::unpackConditionInt(value));
        mirrorThresholdLinkB_ = sscomp::unpackThresholdLink(value);
        mirrorTriggerActionB_ = sscomp::clampTriggerAction(sscomp::unpackTriggerActionInt(value));
        mirrorEverSet_ = true;
        mirrorDirty_ = true; mirrorIdleBlocks_ = 0;
        break;
    case kParamMirrorCommit:
        // See PluginEq::setParameterValue()'s identical case
        // (../statespace/plugin_eq/PluginEq.cpp) for the full reasoning --
        // any delivery at all means "apply now," unconditional, applies
        // whatever both mirrors currently hold.
        applyMirrorToModel();
        mirrorDirty_ = false;
        break;
    case kParamMirrorCommitB:
        // Same handler as kParamMirrorCommit -- see kParamMirrorCommitB's
        // own comment (PluginContour.hpp) for why a second, later-indexed
        // trigger is needed at all.
        applyMirrorToModel();
        mirrorDirty_ = false;
        break;
    case kParamListenMode:      listenMode_ = std::max(0, std::min(2, int(value + 0.5f))); break;
    case kParamThresholdB: thresholdDbB_ = value; break;
    case kParamAttackB:    attackMsB_ = value; break;
    case kParamReleaseB:   releaseMsB_ = value; break;
    case kParamMakeupGainB:      makeupGainDbB_ = value; break;
    case kParamGlobalOutputGain: globalOutputGainDb_ = value; break;
    case kParamScFilterEnable:     scFilterEnabled_ = value >= 0.5f; break;
    case kParamScFilterHighpassHz: scFilterHighpassHz_ = value; updateScFilterFrequencies(); break;
    case kParamScFilterLowpassHz:  scFilterLowpassHz_ = value; updateScFilterFrequencies(); break;
    case kParamScFilterLinkLR:
        scFilterLinkLR_ = value >= 0.5f;
        // "start linked, split on edit" -- re-linking snaps R's own
        // stored frequencies back to L's, same convention as
        // PluginCompressorStereo's own identical case.
        if (scFilterLinkLR_) { scFilterHighpassHzR_ = scFilterHighpassHz_; scFilterLowpassHzR_ = scFilterLowpassHz_; }
        updateScFilterFrequencies();
        break;
    case kParamScFilterHighpassHzR: scFilterHighpassHzR_ = value; updateScFilterFrequencies(); break;
    case kParamScFilterLowpassHzR:  scFilterLowpassHzR_ = value; updateScFilterFrequencies(); break;
    case kParamScListenEnable: scListenEnabled_ = value >= 0.5f; break;
    case kParamTrailSource: trailDetectorMode_ = value >= 0.5f; break;
    case kParamLookahead:   lookaheadMs_ = std::max(0.0, std::min(kLookaheadMaxMs, double(value))); break;
    case kParamUiHeartbeat: uiHeartbeatSamplesSinceLast_ = 0; break;
    case kParamInputGainB: inputGainDbB_ = value; break;
    case kParamWetDry:  wetDry_ = double(value) * 0.01; break;
    case kParamWetDryB: wetDryB_ = double(value) * 0.01; break;
    case kParamGlobalInputGain: globalInputGainDb_ = value; break;
    default:
        if (index >= kParamCurvePointInBase && index < kParamCurvePointInBase + kNumMirroredPoints)
        {
            mirrorPoints_[index - kParamCurvePointInBase].inputDb = double(value);
            mirrorEverSet_ = true;
        }
        else if (index >= kParamCurvePointOutBase && index < kParamCurvePointOutBase + kNumMirroredPoints)
        {
            mirrorPoints_[index - kParamCurvePointOutBase].outputDb = double(value);
            mirrorEverSet_ = true;
        }
        else if (index >= kParamCurvePointInBaseB && index < kParamCurvePointInBaseB + kNumMirroredPoints)
        {
            mirrorPointsB_[index - kParamCurvePointInBaseB].inputDb = double(value);
            mirrorEverSet_ = true;
        }
        else if (index >= kParamCurvePointOutBaseB && index < kParamCurvePointOutBaseB + kNumMirroredPoints)
        {
            mirrorPointsB_[index - kParamCurvePointOutBaseB].outputDb = double(value);
            mirrorEverSet_ = true;
        }
        else if (index >= kParamCurvePointDirBase && index < kParamCurvePointDirBase + kNumMirroredPoints)
        {
            mirrorPoints_[index - kParamCurvePointDirBase].direction = sscomp::unpackDirection(value);
            mirrorPoints_[index - kParamCurvePointDirBase].link = sscomp::unpackLink(value);
            mirrorPoints_[index - kParamCurvePointDirBase].linkAxis = sscomp::unpackLinkAxis(value);
            mirrorPoints_[index - kParamCurvePointDirBase].diagonal = sscomp::unpackPointDiagonal(value);
            mirrorEverSet_ = true;
        }
        else if (index >= kParamCurvePointDirBaseB && index < kParamCurvePointDirBaseB + kNumMirroredPoints)
        {
            mirrorPointsB_[index - kParamCurvePointDirBaseB].direction = sscomp::unpackDirection(value);
            mirrorPointsB_[index - kParamCurvePointDirBaseB].link = sscomp::unpackLink(value);
            mirrorPointsB_[index - kParamCurvePointDirBaseB].linkAxis = sscomp::unpackLinkAxis(value);
            mirrorPointsB_[index - kParamCurvePointDirBaseB].diagonal = sscomp::unpackPointDiagonal(value);
            mirrorEverSet_ = true;
        }
        else if (index >= kParamCurvePointHandleInBase && index < kParamCurvePointHandleInBase + kNumMirroredPoints)
        {
            mirrorPoints_[index - kParamCurvePointHandleInBase].handleInputDb = double(value);
            mirrorEverSet_ = true;
        }
        else if (index >= kParamCurvePointHandleOutBase && index < kParamCurvePointHandleOutBase + kNumMirroredPoints)
        {
            mirrorPoints_[index - kParamCurvePointHandleOutBase].handleOutputDb = double(value);
            mirrorEverSet_ = true;
        }
        else if (index >= kParamCurvePointHandleInBaseB && index < kParamCurvePointHandleInBaseB + kNumMirroredPoints)
        {
            mirrorPointsB_[index - kParamCurvePointHandleInBaseB].handleInputDb = double(value);
            mirrorEverSet_ = true;
        }
        else if (index >= kParamCurvePointHandleOutBaseB && index < kParamCurvePointHandleOutBaseB + kNumMirroredPoints)
        {
            mirrorPointsB_[index - kParamCurvePointHandleOutBaseB].handleOutputDb = double(value);
            mirrorEverSet_ = true;
        }
        break;
    }
}

void PluginContour::setState(const char* key, const char* value)
{
    if (std::strcmp(key, "stereoMode") == 0)
    {
        // The mono build has no stereo modes: always Linked, whatever a (stereo) session/preset says.
        if (!kContourMono) stereoMode_ = clampStereoMode(std::atoi(value));
        return;
    }
    if (std::strcmp(key, "linkAB") == 0)
    {
        linkAB_ = std::atoi(value) != 0;
        return;
    }
    // "presetDir" is a pure UI preference -- see PluginCompressorMono's
    // identical comment on why nothing further is needed for it here. The
    // curve itself (shape/interp/trigger condition, both A's and B's) no
    // longer round-trips through a State at all -- see kParamMirrorCommit.
}

void PluginContour::activate()
{
    // Called after both State and Parameter restore are complete -- see
    // ../statespace/plugin_eq/PluginEq.cpp's PluginEq::activate() own
    // comment for why that makes this exactly the right, and only, place
    // adoptMirrorIfCurveNotReceived() is called from.
    adoptMirrorIfCurveNotReceived();
    voiceA_.reset();
    voiceB_.reset();
    loudnessInL_.reset(); loudnessInR_.reset(); loudnessOutL_.reset(); loudnessOutR_.reset();
    peakInL_.reset(); peakInR_.reset(); peakOutL_.reset(); peakOutR_.reset();
    scFilterL_.reset(); scFilterR_.reset();
    scMonitorPeakL_.reset(); scMonitorPeakR_.reset();
    std::fill(lookaheadRing_.begin(), lookaheadRing_.end(), 0.0);
}

void PluginContour::run(const float** inputs, float** outputs, uint32_t frames)
{
    // See the sibling statespace plugins' identical call
    // (../../statespace/plugin_eq/PluginEq.cpp's PluginEq::run()) for why
    // this has to be retried here, not just from activate() -- DPF only
    // delivers a restored input Parameter to our own setParameterValue()
    // from inside run() itself, never synchronously during restore, so
    // activate() alone is provably too early on a cold import-recovered
    // instance (confirmed via diagnostic logging on the stereo EQ).
    adoptMirrorIfCurveNotReceived();

    // Safety net for a lost commit (see mirrorDirty_): mirror data arrived but
    // no commit followed for kMirrorSettleBlocks blocks -> apply it anyway, so
    // the DSP can never keep serving an old curve. In the normal case the
    // commit arrives in the same block as the data and clears the flag first.
    if (mirrorDirty_ && ++mirrorIdleBlocks_ >= kMirrorSettleBlocks)
    {
        applyMirrorToModel();
        mirrorDirty_ = false;
    }

    // Is anything reading the spectrum? See kParamUiHeartbeat. (~2 s at 48 kHz: longer than any normal idle-tick
    // gap or a stalled UI thread catching up, same figure the EQs use.)
    constexpr uint64_t kUiHeartbeatTimeoutSamples = 96000;
    if (uiHeartbeatSamplesSinceLast_ <= UINT64_MAX - frames) uiHeartbeatSamplesSinceLast_ += frames;
    else uiHeartbeatSamplesSinceLast_ = UINT64_MAX;
    const bool scopeShowing = uiHeartbeatSamplesSinceLast_ < kUiHeartbeatTimeoutSamples;

    // Mono build: one main input + one sidechain input, one output. The single channel is simply fed to the
    // stereo code as L = R (Linked mode, so the detector's max(|L|,|R|) is just |L|); the R output does not exist.
    const float* inL = inputs[0];
    const float* inR = kContourMono ? inputs[0] : inputs[1];
    const float* scL = kContourMono ? inputs[1] : inputs[2]; // external key input, L/R -- see PluginCompressorStereo::run()'s identical comment
    const float* scR = kContourMono ? inputs[1] : inputs[3];
    float* outL = outputs[0];
    float* outR = kContourMono ? nullptr : outputs[1];

    if (bypass_)
    {
        if (outL != inL) std::copy(inL, inL + frames, outL);
        if (outR != nullptr && outR != inR) std::copy(inR, inR + frames, outR);
        const double scGainLinBypass = dbToLinear(sidechainGainDb_);
        for (uint32_t i = 0; i < frames; ++i)
        {
            const double l = double(inL[i]), r = double(inR[i]);
            // The input meters show M/S in Mid/Side mode -- same
            // Mid/Side re-derivation the main (non-bypass) path below
            // uses, ported from PluginCompressorStereo::run()'s identical
            // bypass-branch fix.
            double meterL = l, meterR = r;
            if (stereoMode_ == StereoMode::MidSide) { meterL = (l + r) * 0.5; meterR = (l - r) * 0.5; }
            peakInL_.process(std::fabs(meterL)); peakInR_.process(std::fabs(meterR));
            peakOutL_.process(std::fabs(meterL)); peakOutR_.process(std::fabs(meterR));
            loudnessInL_.process(meterL); loudnessInR_.process(meterR);
            loudnessOutL_.process(meterL); loudnessOutR_.process(meterR);

            // Sidechain monitor -- kept running during Bypass too, same
            // reasoning as the I/O meters right above.
            // Trigger source: the external Sidechain input while that's
            // selected, the plain main input otherwise -- gain/filter/
            // monitor apply to whichever one is feeding the detector.
            const double scGLRaw = (sidechainEnabled_ ? double(scL[i]) : l) * scGainLinBypass;
            const double scGRRaw = (sidechainEnabled_ ? double(scR[i]) : r) * scGainLinBypass;
            double scGL = scGLRaw, scGR = scGRRaw;
            if (scFilterEnabled_)
            {
                scGL = scFilterL_.process(scGL);
                scGR = scFilterR_.process(scGR);
            }
            scMonitorPeakL_.process(std::fabs(scGL));
            scMonitorPeakR_.process(std::fabs(scGR));
            if (i < kScMonitorScratchFrames)
            {
                scMonitorScratchIn_[i] = float((scGLRaw + scGRRaw) * 0.5);
                scMonitorScratchOut_[i] = float((scGL + scGR) * 0.5);
            }
        }
        if (scopeShowing)
        {
            scMonitorSpectrumIn_.process(scMonitorScratchIn_, std::min(frames, kScMonitorScratchFrames));
            scMonitorSpectrumOut_.process(scMonitorScratchOut_, std::min(frames, kScMonitorScratchFrames));
        }
        meterInputRmsL_ = float(loudnessInL_.rmsDb());  meterInputPeakL_ = float(peakInL_.peakDb());
        meterInputRmsR_ = float(loudnessInR_.rmsDb());  meterInputPeakR_ = float(peakInR_.peakDb());
        meterOutputRmsL_ = float(loudnessOutL_.rmsDb()); meterOutputPeakL_ = float(peakOutL_.peakDb());
        meterOutputRmsR_ = float(loudnessOutR_.rmsDb()); meterOutputPeakR_ = float(peakOutR_.peakDb());
        meterGainReductionA_ = meterGainReductionB_ = 0.0f;
        meterTrailA_ = meterTrailB_ = sscomp::packTrailPair(-120.0, 0.0);
        for (uint32_t k = 0; k < kTrailBankSize; ++k) meterTrailBankA_[k] = meterTrailBankB_[k] = meterTrailA_;
        return;
    }

    voiceA_.setParams(attackMs_, releaseMs_, thresholdDb_, triggerCondition_, triggerAction_);

    // B's own bank (including its own curve) only actually applies while
    // independent -- see isChannelBIndependent()'s own comment; while it
    // isn't, B simply mirrors A's own values/curve, same as this
    // plugin's pre-existing (shared-parameter) behavior for its scalars.
    const bool bIndependent = isChannelBIndependent();
    const double attackB = bIndependent ? attackMsB_ : attackMs_;
    const double releaseB = bIndependent ? releaseMsB_ : releaseMs_;
    const double thresholdB = bIndependent ? thresholdDbB_ : thresholdDb_;
    const TriggerCondition condB = bIndependent ? triggerConditionB_ : triggerCondition_;
    voiceB_.setParams(attackB, releaseB, thresholdB, condB, bIndependent ? triggerActionB_ : triggerAction_);
    const TransferCurve& curveForB = bIndependent ? curveB_ : curve_;

    // Lookahead: whole samples, capped to the ring; reported to the host as latency (only when it changes).
    const uint32_t laSamples = std::min<uint32_t>(kLookaheadRing - 1, uint32_t(lookaheadMs_ * 0.001 * getSampleRate() + 0.5));
    lookaheadSamples_ = laSamples;
    voiceA_.setLookaheadSamples(int(laSamples));
    voiceB_.setLookaheadSamples(int(laSamples));
    if (laSamples != reportedLatency_) { setLatency(laSamples); reportedLatency_ = laSamples; }

    const double inGainLinA = dbToLinear(inputGainDb_);
    const double inGainLinB = dbToLinear(bIndependent ? inputGainDbB_ : inputGainDb_);
    const double scGainLin = dbToLinear(sidechainGainDb_);
    const double makeupALin = dbToLinear(outputGainDb_);
    const double makeupBLin = dbToLinear(bIndependent ? makeupGainDbB_ : outputGainDb_);
    const double globalOutGainLin = dbToLinear(globalOutputGainDb_);
    const double wetA = wetDry_;
    const double wetB = bIndependent ? wetDryB_ : wetDry_;
    const double globalInGainLin = (stereoMode_ != StereoMode::Linked) ? dbToLinear(globalInputGainDb_) : 1.0;
    uint32_t trailSlot = 0;
    double lastVoiceInA = 0.0, lastVoiceInB = 0.0;
    // One trail point for a voice: x = the real input level (default) or the detector level,
    // y = x + the gain the voice is applying at this sample, flagged when the trigger
    // condition is met.
    auto trailPair = [this](const CurveVoice& v, double inLin) {
        // x = the audio's own level, y = x + the gain applied: the curve maps the audio, so a dot sits on the drawn curve
        // while the trigger says "curve", on the diagonal while it says "diagonal", and in between while it switches.
        // (The old "detector" view -- x = the trigger's level -- has no meaning now; the Parameter stays for old sessions.)
        (void)inLin;
        const double xDb = v.audioLevelDb(); // the level the curve was read at
        return sscomp::packTrailPair(xDb, v.gainDb(), v.triggered());
    };
    auto trailPos = [frames](uint32_t k) -> uint32_t {
        const uint64_t p = (uint64_t(k) * frames + frames / 2) / kTrailBankSize;
        return uint32_t(p < frames ? p : frames - 1);
    };

    for (uint32_t i = 0; i < frames; ++i)
    {
        // Gain-staged main signal for this stereo mode, computed up front
        // (before the trigger signal below) because the trigger can now be
        // derived from it: amplification and filtering also work on the
        // internal signal, e.g. for gates that open and close at certain
        // frequencies. xgL/xgR: Linked/Dual-Mono's own L/R
        // pair; mid/side: Mid/Side's own decoded pair.
        double xgL = 0.0, xgR = 0.0, mid = 0.0, side = 0.0;
        double intL, intR; // the main signal as an L/R pair, whatever the mode -- the internal trigger source
        switch (stereoMode_)
        {
        case StereoMode::Linked:
            xgL = double(inL[i]) * inGainLinA;
            xgR = double(inR[i]) * inGainLinA;
            intL = xgL; intR = xgR;
            break;
        case StereoMode::DualMono:
            xgL = double(inL[i]) * globalInGainLin * inGainLinA;
            xgR = double(inR[i]) * globalInGainLin * inGainLinB;
            intL = xgL; intR = xgR;
            break;
        case StereoMode::MidSide:
        default:
        {
            const double gL = double(inL[i]) * globalInGainLin;
            const double gR = double(inR[i]) * globalInGainLin;
            mid = ((gL + gR) * 0.5) * inGainLinA;
            side = ((gL - gR) * 0.5) * inGainLinB;
            intL = mid + side; intR = mid - side; // exact inverse of the decode below
            break;
        }
        }

        // Lookahead: the audio path runs on a copy delayed by lookaheadSamples_ (dxgL/dxgR/dmid/dside); the
        // trigger below keeps using the un-delayed signal, so the gain is in place before the audio arrives.
        double dxgL = xgL, dxgR = xgR, dmid = mid, dside = side;
        {
            constexpr uint32_t kMask = kLookaheadRing - 1;
            lookaheadRing_[0 * kLookaheadRing + lookaheadPos_] = xgL;
            lookaheadRing_[1 * kLookaheadRing + lookaheadPos_] = xgR;
            lookaheadRing_[2 * kLookaheadRing + lookaheadPos_] = mid;
            lookaheadRing_[3 * kLookaheadRing + lookaheadPos_] = side;
            const uint32_t rd = (lookaheadPos_ - laSamples) & kMask;
            dxgL = lookaheadRing_[0 * kLookaheadRing + rd];
            dxgR = lookaheadRing_[1 * kLookaheadRing + rd];
            dmid = lookaheadRing_[2 * kLookaheadRing + rd];
            dside = lookaheadRing_[3 * kLookaheadRing + rd];
            lookaheadPos_ = (lookaheadPos_ + 1) & kMask;
        }

        // The trigger signal: external Sidechain input while selected,
        // the (already gain-staged) main signal otherwise -- then Trigger
        // Gain, then the optional Highpass/Lowpass band filter, identically
        // for both sources. This is what the detector, the monitor scope
        // and Trigger Listen all see.
        const double trigL = sidechainEnabled_ ? double(scL[i]) : intL;
        const double trigR = sidechainEnabled_ ? double(scR[i]) : intR;
        const double scGLRaw = trigL * scGainLin;
        const double scGRRaw = trigR * scGainLin;
        double scGL = scGLRaw, scGR = scGRRaw;
        if (scFilterEnabled_)
        {
            scGL = scFilterL_.process(scGL);
            scGR = scFilterR_.process(scGR);
        }
        scMonitorPeakL_.process(std::fabs(scGL));
        scMonitorPeakR_.process(std::fabs(scGR));
        if (i < kScMonitorScratchFrames)
        {
            scMonitorScratchIn_[i] = float((scGLRaw + scGRRaw) * 0.5);
            scMonitorScratchOut_[i] = float((scGL + scGR) * 0.5);
        }

        double yL, yR;
        double yMid = 0.0, ySide = 0.0; // only meaningful when stereoMode_ == MidSide -- see the Listen override below
        double meterInRawL, meterInRawR, meterInGainedL, meterInGainedR;
        // The REAL audio level entering each voice (after Input Gain, before any trigger
        // processing) -- what the trail shows on x in its default "input -> output" mode.
        double voiceInA = 0.0, voiceInB = 0.0;
        switch (stereoMode_)
        {
        case StereoMode::Linked:
        {
            voiceInA = voiceInB = std::max(std::fabs(dxgL), std::fabs(dxgR));
            const double detector = std::max(std::fabs(scGL), std::fabs(scGR));
            const double grLin = voiceA_.process(detector, voiceInA, curve_);
            voiceB_.process(detector, voiceInB, curve_); // kept in lock-step purely so its readout mirrors A's, see header comment
            const double wetSigL = dxgL * grLin * makeupALin;
            const double wetSigR = dxgR * grLin * makeupALin;
            yL = dxgL * (1.0 - wetA) + wetSigL * wetA;
            yR = dxgR * (1.0 - wetA) + wetSigR * wetA;
            meterInRawL = double(inL[i]); meterInRawR = double(inR[i]);
            meterInGainedL = xgL; meterInGainedR = xgR;
            break;
        }
        case StereoMode::DualMono:
        {
            voiceInA = std::fabs(dxgL); voiceInB = std::fabs(dxgR);
            const double grL = voiceA_.process(std::fabs(scGL), voiceInA, curve_);
            const double grR = voiceB_.process(std::fabs(scGR), voiceInB, curveForB);
            const double wetSigL = dxgL * grL * makeupALin;
            const double wetSigR = dxgR * grR * makeupBLin;
            yL = dxgL * (1.0 - wetA) + wetSigL * wetA;
            yR = dxgR * (1.0 - wetB) + wetSigR * wetB;
            meterInRawL = double(inL[i]); meterInRawR = double(inR[i]);
            meterInGainedL = xgL; meterInGainedR = xgR;
            break;
        }
        case StereoMode::MidSide:
        default:
        {
            const double trueRawMid = (double(inL[i]) + double(inR[i])) * 0.5;
            const double trueRawSide = (double(inL[i]) - double(inR[i])) * 0.5;
            const double scMid = (scGL + scGR) * 0.5;
            const double scSide = (scGL - scGR) * 0.5;
            voiceInA = std::fabs(dmid); voiceInB = std::fabs(dside);
            const double grMid = voiceA_.process(std::fabs(scMid), voiceInA, curve_);
            const double grSide = voiceB_.process(std::fabs(scSide), voiceInB, curveForB);
            const double wetMid = dmid * grMid * makeupALin;
            const double wetSide = dside * grSide * makeupBLin;
            yMid = dmid * (1.0 - wetA) + wetMid * wetA;
            ySide = dside * (1.0 - wetB) + wetSide * wetB;
            yL = yMid + ySide;
            yR = yMid - ySide;
            meterInRawL = trueRawMid; meterInRawR = trueRawSide;
            meterInGainedL = mid; meterInGainedR = side;
            break;
        }
        }

        // Trail bank: kTrailBankSize (level, gain) pairs, evenly spaced through
        // this block, each read from the voices right after they processed
        // THIS sample -- see kParamMeterTrailBankA. (Integer positions; with a
        // block shorter than the bank, neighbouring slots simply share a sample.)
        while (trailSlot < kTrailBankSize && trailPos(trailSlot) <= i)
        {
            meterTrailBankA_[trailSlot] = trailPair(voiceA_, voiceInA);
            meterTrailBankB_[trailSlot] = (stereoMode_ == StereoMode::Linked) ? meterTrailBankA_[trailSlot] : trailPair(voiceB_, voiceInB);
            ++trailSlot;
        }
        lastVoiceInA = voiceInA; lastVoiceInB = voiceInB;

        peakInL_.process(std::fabs(meterInRawL));
        peakInR_.process(std::fabs(meterInRawR));
        loudnessInL_.process(meterInGainedL); loudnessInR_.process(meterInGainedR);

        if (stereoMode_ == StereoMode::MidSide && listenMode_ != 0)
        {
            const double m = (listenMode_ == 1) ? yMid : 0.0;
            const double s = (listenMode_ == 2) ? ySide : 0.0;
            yL = m + s;
            yR = m - s;
        }

        if (scListenEnabled_)
        {
            // Soft-limited: with up to +60dB of trigger gain the raw trigger signal could be
            // enormous -- see sscomp::softLimitListen().
            yL = softLimitListen(scGL);
            yR = softLimitListen(scGR);
        }

        yL *= globalOutGainLin;
        yR *= globalOutGainLin;
        if (stereoMode_ == StereoMode::MidSide)
        {
            const double mOut = (yL + yR) * 0.5;
            const double sOut = (yL - yR) * 0.5;
            loudnessOutL_.process(mOut); loudnessOutR_.process(sOut);
            peakOutL_.process(std::fabs(mOut)); peakOutR_.process(std::fabs(sOut));
        }
        else
        {
            loudnessOutL_.process(yL); loudnessOutR_.process(yR);
            peakOutL_.process(std::fabs(yL));
            peakOutR_.process(std::fabs(yR));
        }

        outL[i] = float(yL);
        if (outR != nullptr) outR[i] = float(yR);
    }

    if (scopeShowing)
    {
        scMonitorSpectrumIn_.process(scMonitorScratchIn_, std::min(frames, kScMonitorScratchFrames));
        scMonitorSpectrumOut_.process(scMonitorScratchOut_, std::min(frames, kScMonitorScratchFrames));
    }

    meterInputRmsL_ = float(loudnessInL_.rmsDb());  meterInputPeakL_ = float(peakInL_.peakDb());
    meterInputRmsR_ = float(loudnessInR_.rmsDb());  meterInputPeakR_ = float(peakInR_.peakDb());
    meterOutputRmsL_ = float(loudnessOutL_.rmsDb()); meterOutputPeakL_ = float(peakOutL_.peakDb());
    meterOutputRmsR_ = float(loudnessOutR_.rmsDb()); meterOutputPeakR_ = float(peakOutR_.peakDb());
    meterDetectorA_ = float(voiceA_.detectorLevelDb());
    meterDetectorB_ = float(voiceB_.detectorLevelDb());
    meterGainReductionA_ = float(voiceA_.gainDb());
    meterGainReductionB_ = (stereoMode_ == StereoMode::Linked) ? meterGainReductionA_ : float(voiceB_.gainDb());
    // Both numbers of each pair from the SAME sample, in one value -- see
    // kParamMeterTrailA.
    meterTrailA_ = trailPair(voiceA_, lastVoiceInA);
    meterTrailB_ = (stereoMode_ == StereoMode::Linked) ? meterTrailA_ : trailPair(voiceB_, lastVoiceInB);
}

void PluginContour::sampleRateChanged(double newSampleRate)
{
    voiceA_.prepare(newSampleRate);
    voiceB_.prepare(newSampleRate);
    loudnessInL_.prepare(newSampleRate); loudnessInR_.prepare(newSampleRate);
    loudnessOutL_.prepare(newSampleRate); loudnessOutR_.prepare(newSampleRate);
    peakInL_.prepare(newSampleRate); peakInR_.prepare(newSampleRate);
    peakOutL_.prepare(newSampleRate); peakOutR_.prepare(newSampleRate);
    scFilterL_.prepare(newSampleRate); scFilterR_.prepare(newSampleRate);
    updateScFilterFrequencies();
    scMonitorPeakL_.prepare(newSampleRate); scMonitorPeakR_.prepare(newSampleRate);
    scMonitorSpectrumIn_.prepare(newSampleRate); scMonitorSpectrumOut_.prepare(newSampleRate);
}

START_NAMESPACE_DISTRHO
Plugin* createPlugin() { return new PluginContour(); }
END_NAMESPACE_DISTRHO
