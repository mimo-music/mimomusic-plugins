// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// limiter_bypass_test.cpp
//
// Drives the REAL PluginLimiter::run(): an impulse must come out exactly
// the reported latency later, with Bypass off and on, with separate and
// in-place (output == input) buffers. Bypass used to copy the input
// straight through while the Lookahead latency stayed reported, so
// toggling it shifted the audio by the lookahead.
//
// Not part of the CMake build (it needs DPF's Plugin base class plus
// -fno-access-control); build and run it by hand from the project root:
//
//   D=$PWD; g++ -std=gnu++17 -O2 -fno-access-control -DDGL_USE_FILE_BROWSER \
//     -DDISTRHO_PLUGIN_TARGET_LV2 -I$D/dpf/distrho -I$D/dpf/distrho/src -I$D/plugin_limiter \
//     -I$D/plugin/dsp -o /tmp/limiter_bypass_test \
//     tests/limiter_bypass_test.cpp plugin_limiter/PluginLimiter.cpp dpf/distrho/src/DistrhoPlugin.cpp \
//   && /tmp/limiter_bypass_test

#include "DistrhoPlugin.hpp"
#include "DistrhoPluginInternal.hpp"
#include "PluginLimiter.hpp"
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>
START_NAMESPACE_DISTRHO
Plugin* createPlugin();
END_NAMESPACE_DISTRHO
using namespace DISTRHO;
namespace DISTRHO { extern uint32_t d_nextBufferSize; extern double d_nextSampleRate; }
int main(){ int fails=0;
 for (int mode=0; mode<4; ++mode){ const bool bypass=mode&1; const bool inPlace=mode&2;
  d_nextBufferSize=256; d_nextSampleRate=48000; std::unique_ptr<Plugin> p(createPlugin());
  p->setParameterValue(kParamBypass, bypass?1:0); p->activate();
  std::vector<float> l(4096,0), r(4096,0), ol(4096), orr(4096); l[1000]=r[1000]=0.01f; l[3000]=0.02f;
  for(size_t off=0;off<l.size();off+=256){ const float* in[2]={l.data()+off,r.data()+off}; float* out[2]; if(inPlace){out[0]=l.data()+off; out[1]=r.data()+off;} else {out[0]=ol.data()+off; out[1]=orr.data()+off;} p->run(in,out,256);}
  const std::vector<float>& o = inPlace ? l : ol; long at=-1; for(size_t i=0;i<o.size();++i) if(std::fabs(o[i])>0.005f){at=long(i);break;}
  const long lat=long(p->pData->latency); const bool ok = at-1000==lat;
  printf("bypass %s, %s: latency %ld, impulse delayed %ld: %s\n", bypass?"on ":"off", inPlace?"in-place  ":"separate  ", lat, at-1000, ok?"OK":"FAIL"); if(!ok) ++fails; }
 printf("%s\n", fails?"FAIL":"PASS"); return fails; }
