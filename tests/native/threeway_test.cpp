#include "threeway_filter.h"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

int main() {
    using namespace ThreewayCoefficients;
    ThreewayFilter f;
    int16_t out[3];
    for(int c=0;c<3;c++) {
        int64_t dc=0,l1=0;
        for(int k=0;k<kTaps;k++) {
            dc+=kFir[c][k]; l1+=std::abs((int64_t)kFir[c][k]);
            if(c==0) assert(kFir[c][k]==0); // LFi must never receive bipolar FIR output
            assert(kFir[c][k]==kFir[c][kTaps-1-k]);
        }
        assert(dc==0);
        assert(l1 < (1LL<<30)); // bounded accumulator, including full-range input
    }
    // Impulse matches coefficients, including startup and channel order.
    for(int n=0;n<kTaps+5;n++) {
        f.process(n==0?32767:0,out);
        for(int c=0;c<3;c++)
            assert(out[c]==(n<kTaps?(int64_t)32767*kFir[c][n]/(1LL<<28):0));
    }
    // Arbitrary full-range input cannot exceed peak limit; reset clears tails.
    uint32_t rng=42;
    for(int n=0;n<100000;n++) {
        rng=rng*1664525u+1013904223u;
        f.process((int16_t)(rng>>16),out);
        for(auto y:out) assert(std::abs((int)y)<=kPeakQ15);
    }
    f.reset();f.process(0,out);for(auto y:out) assert(y==0);
    for(int n=0;n<kTaps+5;n++) f.process(32767,out);
    for(auto y:out) assert(y==0);
    puts("Threeway: impulse, symmetry, DC rejection, bounded output and reset passed");
}
