// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// Wav.hpp
//
// Minimal, dependency-free stereo WAV file I/O for the developer tools
// (tools/reference/generate_reference_samples.cpp, and any future
// tools/trainer/ fitting tool that consumes its output) -- copied
// verbatim from ../../statespace/tools/testbench/Wav.hpp, matching this
// codebase's own established per-project-duplication convention (see
// e.g. plugin/dsp/ReverbEngineDense.hpp's own header comment on
// duplication over cross-project sharing) rather than reaching across a
// project boundary with a relative include.
// Not part of the plugin itself; kept dependency-free like the rest of the
// project rather than pulling in libsndfile or similar. Writing is always
// 32-bit float PCM; reading accepts 16/24/32-bit integer PCM and 32-bit
// float, mono or stereo (mono is duplicated to both channels), at
// whatever sample rate the file itself specifies.

#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace wavio {

inline void writeU32(std::ofstream& f, uint32_t v) { f.write(reinterpret_cast<const char*>(&v), 4); }
inline void writeU16(std::ofstream& f, uint16_t v) { f.write(reinterpret_cast<const char*>(&v), 2); }

// Writes an interleaved stereo, 32-bit float PCM WAV file.
inline bool writeWavStereoFloat(const std::string& path, double sampleRate,
                                 const std::vector<float>& left, const std::vector<float>& right)
{
    const uint32_t numFrames = uint32_t(std::min(left.size(), right.size()));
    const uint16_t numChannels = 2;
    const uint16_t bitsPerSample = 32;
    const uint32_t byteRate = uint32_t(sampleRate) * numChannels * (bitsPerSample / 8);
    const uint16_t blockAlign = numChannels * (bitsPerSample / 8);
    const uint32_t dataSize = numFrames * blockAlign;

    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f.good()) return false;

    f.write("RIFF", 4);
    writeU32(f, 36 + dataSize);
    f.write("WAVE", 4);

    f.write("fmt ", 4);
    writeU32(f, 16);
    writeU16(f, 3); // 3 = IEEE float
    writeU16(f, numChannels);
    writeU32(f, uint32_t(sampleRate));
    writeU32(f, byteRate);
    writeU16(f, blockAlign);
    writeU16(f, bitsPerSample);

    f.write("data", 4);
    writeU32(f, dataSize);
    for (uint32_t i = 0; i < numFrames; ++i)
    {
        f.write(reinterpret_cast<const char*>(&left[i]), 4);
        f.write(reinterpret_cast<const char*>(&right[i]), 4);
    }
    return f.good();
}

inline uint32_t readU32(const uint8_t* p) { uint32_t v; std::memcpy(&v, p, 4); return v; }
inline uint16_t readU16(const uint8_t* p) { uint16_t v; std::memcpy(&v, p, 2); return v; }

// Reads any of: 16/24/32-bit integer PCM, 32-bit IEEE float, mono or
// stereo (mono duplicated to both output channels). Returns false (and
// leaves outputs empty) if the file can't be parsed as WAV or uses a
// format this reader doesn't handle. `sampleRateOut` is set to whatever
// the file itself specifies.
inline bool readWavStereoFloat(const std::string& path, double& sampleRateOut,
                                std::vector<float>& left, std::vector<float>& right)
{
    left.clear();
    right.clear();

    std::ifstream f(path, std::ios::binary);
    if (!f.good()) return false;
    std::vector<uint8_t> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (buf.size() < 44) return false;
    if (std::memcmp(buf.data(), "RIFF", 4) != 0 || std::memcmp(buf.data() + 8, "WAVE", 4) != 0)
        return false;

    uint16_t formatTag = 0, numChannels = 0, bitsPerSample = 0;
    uint32_t sampleRate = 0;
    const uint8_t* dataPtr = nullptr;
    uint32_t dataSize = 0;

    size_t pos = 12;
    while (pos + 8 <= buf.size())
    {
        char id[5] = {};
        std::memcpy(id, &buf[pos], 4);
        const uint32_t chunkSize = readU32(&buf[pos + 4]);
        const size_t bodyStart = pos + 8;
        if (bodyStart + chunkSize > buf.size()) break;

        if (std::memcmp(id, "fmt ", 4) == 0 && chunkSize >= 16)
        {
            formatTag     = readU16(&buf[bodyStart + 0]);
            numChannels   = readU16(&buf[bodyStart + 2]);
            sampleRate    = readU32(&buf[bodyStart + 4]);
            bitsPerSample = readU16(&buf[bodyStart + 14]);
            if (formatTag == 0xFFFE && chunkSize >= 40) // WAVE_FORMAT_EXTENSIBLE
                formatTag = readU16(&buf[bodyStart + 24]); // subformat's first two bytes (PCM=1 or FLOAT=3)
        }
        else if (std::memcmp(id, "data", 4) == 0)
        {
            dataPtr = &buf[bodyStart];
            dataSize = chunkSize;
        }

        pos = bodyStart + chunkSize + (chunkSize & 1); // chunks are word-aligned
    }

    if (dataPtr == nullptr || numChannels == 0 || bitsPerSample == 0 || sampleRate == 0)
        return false;
    if (formatTag != 1 && formatTag != 3) // 1 = integer PCM, 3 = IEEE float
        return false;

    const uint32_t bytesPerSample = bitsPerSample / 8;
    const uint32_t frameBytes = bytesPerSample * numChannels;
    if (frameBytes == 0) return false;
    const uint32_t numFrames = dataSize / frameBytes;

    left.resize(numFrames);
    right.resize(numFrames);

    auto readSample = [&](const uint8_t* p) -> float {
        if (formatTag == 3 && bitsPerSample == 32)
        {
            float v; std::memcpy(&v, p, 4); return v;
        }
        if (formatTag == 1 && bitsPerSample == 16)
        {
            int16_t v; std::memcpy(&v, p, 2); return float(v) / 32768.0f;
        }
        if (formatTag == 1 && bitsPerSample == 24)
        {
            int32_t v = (int32_t(p[0]) << 8) | (int32_t(p[1]) << 16) | (int32_t(p[2]) << 24);
            return float(v >> 8) / 8388608.0f;
        }
        if (formatTag == 1 && bitsPerSample == 32)
        {
            int32_t v; std::memcpy(&v, p, 4); return float(v) / 2147483648.0f;
        }
        return 0.0f;
    };

    for (uint32_t i = 0; i < numFrames; ++i)
    {
        const uint8_t* frame = dataPtr + size_t(i) * frameBytes;
        const float l = readSample(frame);
        const float r = numChannels >= 2 ? readSample(frame + bytesPerSample) : l;
        left[i] = l;
        right[i] = r;
    }

    sampleRateOut = double(sampleRate);
    return true;
}

} // namespace wavio
