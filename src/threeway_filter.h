#pragma once
#include <stdint.h>
#include <string.h>
#include "threeway_coefficients.h"

// 2 kHz mono input, Q15. Outputs LFi/HF/LF, routed to slots 0/2/3.
// All FIRs share 16 ms delay. Coefficients contain crossover AND enclosure EQ.
// A shared peak limiter preserves the inter-channel mix during overload.
class ThreewayFilter {
    int16_t history_[ThreewayCoefficients::kTaps] = {};
    int head_ = 0;
public:
    void reset() { memset(history_,0,sizeof(history_)); head_=0; }
    void process(int32_t input, int16_t output[3]) {
        using namespace ThreewayCoefficients;
        input=input>32767?32767:input < -32767?-32767:input;
        history_[head_]=(int16_t)input;
        int32_t y[3]={};
        int32_t peak=0;
        // LFi's FIR is intentionally zero: its asymmetric pulse bypasses EQ.
        // Avoid spending a third of the convolution time on that silent path.
        for(int ch=1;ch<3;++ch) {
            int64_t sum=0;
            // Symmetric FIR: pair equal coefficients, halving multiplies.
            int left=head_,right=head_+1;
            if(right==kTaps)right=0;
            for(int k=0;k<kTaps/2;++k) {
                sum+=(int64_t)((int32_t)history_[left]+history_[right])*kFir[ch][k];
                if(--left<0)left=kTaps-1;
                if(++right==kTaps)right=0;
            }
            sum+=(int64_t)history_[left]*kFir[ch][kTaps/2];
            y[ch]=(int32_t)(sum/(1LL<<28));
            int32_t mag=y[ch]<0?-y[ch]:y[ch];
            if(mag>peak) peak=mag;
        }
        for(int ch=0;ch<3;++ch)
            output[ch]=(int16_t)(peak>kPeakQ15 ? (int64_t)y[ch]*kPeakQ15/peak : y[ch]);
        if(++head_==kTaps) head_=0;
    }
};
