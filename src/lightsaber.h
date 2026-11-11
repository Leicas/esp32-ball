#pragma once
#include <cmath>

// Clean tactile carriers; LFi is reserved for positive clashes.
class Lightsaber {
    float table_[256]={},phase_=0,phase2_=0,phase3_=0,phaseMf_=0,phasePower_=0,phaseNeg_=0,motion_=0,fade_=0,acc_=0,cool_=0;
    int clash_age_=100;
    bool armed_=true;
public:
    struct Output { float hum,impact,motion,lf,mf,hf,lfi; };
    void init() { for(int i=0;i<256;i++) table_[i]=std::sin(6.28318530718f*i/256); reset(); }
    void reset() { phase_=phase2_=phase3_=phaseMf_=phasePower_=phaseNeg_=motion_=fade_=acc_=cool_=0;clash_age_=100;armed_=true; }
    Output step(float angular,float acceleration,float rate=2000) {
        if(!std::isfinite(angular)) angular=0;
        if(!std::isfinite(acceleration)) acceleration=0;
        angular=std::fmin(std::fabs(angular),20.f);
        acceleration=std::fmin(std::fabs(acceleration),100.f);
        float target=std::fmin(1.f,angular/6.f+acceleration/80.f);
        motion_+=(target-motion_)*(target>motion_?35.f:8.f)/rate;
        acc_+=(acceleration-acc_)*35.f/rate;
        fade_=std::fmin(1.f,fade_+5.f/rate);
        cool_=std::fmax(0.f,cool_-1.f/rate);
        if(acc_<7) armed_=true;
        if(armed_ && acc_>22 && cool_==0) { clash_age_=0;cool_=.4f;armed_=false; }
        phase_+=(88.f+8.f*motion_)/rate;
        phase2_+=(176.f+16.f*motion_)/rate;
        phase3_+=(179.f+16.f*motion_)/rate;
        if(phase_>=1)phase_-=1;
        if(phase2_>=1)phase2_-=1;
        if(phase3_>=1)phase3_-=1;
        float hum=fade_*(.32f+.5f*motion_)*(.65f*table_[(int)(phase_*256)]+.22f*table_[(int)(phase2_*256)]+.13f*table_[(int)(phase3_*256)]);
        float impact=0;
        const int duration=(int)(rate*.008f);
        if(clash_age_<duration) {
            float age=clash_age_++;
            impact=std::fmin(1.f,std::fmin((age+1)/(rate*.001f),(duration-age)/(rate*.002f)));
        }
        phaseMf_+=130.f/rate;phasePower_+=500.f/rate;
        if(phaseMf_>=1)phaseMf_-=1;if(phasePower_>=1)phasePower_-=1;
        float lf=fade_*(.50f+.15f*motion_)*table_[(int)(phase_*256)];
        // MF is a modest harmonically related layer, not an unrelated 130 Hz tone.
        float mf=fade_*(.12f+.08f*motion_)*table_[(int)(phase2_*256)];
        float hf=0; // audible reference has the HF channel to itself
        float lfi=.85f*impact; // continuous negative pulses disabled after rough-sound feedback
        return {hum,impact,motion_,lf,mf,hf,lfi};
    }
};
