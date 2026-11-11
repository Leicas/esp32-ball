#pragma once
#include <math.h>
#include <stdint.h>

// Four-millisecond contact voices overlap instead of restarting one long
// impact envelope. Dense collisions add weight without increasing the cap.
class RattlePulse {
    float peak_[8] = {};
    uint8_t age_[8] = {};
public:
    void reset() { for (int i=0;i<8;++i) { peak_[i]=0; age_[i]=8; } }
    float step(float hit, unsigned rate) {
        if (!isfinite(hit)) hit=0;
        if (hit>0) {
            int slot=0;
            for (int i=0;i<8;++i) {
                if (peak_[i]==0) { slot=i; break; }
                if (age_[i]>age_[slot]) slot=i;
            }
            peak_[slot]=fminf(hit,1.0f); age_[slot]=0;
        }
        constexpr float shape[8]={.25f,.75f,1,1,.75f,.5f,.25f,0};
        float out=0;
        for (int i=0;i<8;++i) if (peak_[i]>0) {
            if (age_[i]>=8) { peak_[i]=0; continue; }
            out+=peak_[i]*shape[age_[i]];
            age_[i]+=(rate>=2000 ? 1 : 2);
        }
        return fminf(out,1.0f);
    }
};
