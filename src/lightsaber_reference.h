#pragma once
#include <cmath>
#include <stdint.h>
// Recorded samples are local-only; fall back to a silent stub when absent.
#if __has_include("lightsaber_samples.h")
#include "lightsaber_samples.h"
#else
#include "lightsaber_samples_stub.h"
#endif

// Linear overlapping excerpts: motion supplies the continuous whoosh envelope.
class LightsaberReference {
    struct Grain { uint32_t phase=0,step=65536; int age=0,variant=0; };
    Grain hum_[2],swing_[2];
    uint32_t start_=0,rng_=0x927ac51u;
    float motion_=0;
    int32_t gain_=0;
    uint32_t pitch_=65536;
    bool enabled_=false;
    int sub_=0;
    int32_t previous_=0,current_=0;
    uint32_t random() { rng_^=rng_<<13;rng_^=rng_>>17;rng_^=rng_<<5;return rng_; }
    static int32_t read(const int16_t* p,uint32_t n,uint32_t phase) {
        uint32_t i=phase>>16;if(i+1>=n)return 0;
        return p[i]+((int32_t)p[i+1]-p[i])*(int32_t)((phase&65535)>>1)/32768;
    }
    void seed(Grain &g,bool hum,int age=0) {
        using namespace LightsaberSamples;
        g.variant=random()%2;g.age=age;
        g.step=hum?64225+random()%2622:pitch_;
        uint32_t n=hum?sizeof(idle)/sizeof(int16_t):swings[g.variant].count;
        // Reserve source for maximum pitch even if speed rises mid-grain.
        uint32_t extent=hum?16330:7042;
        uint32_t offset=hum?random()%(n-extent-2):480+random()%(n-extent-482);
        g.phase=(offset<<16)+age*g.step;
    }
    int32_t grains(Grain (&pair)[2],bool hum) {
        using namespace LightsaberSamples;
        const int length=hum?16000:6400,half=length/2;
        int32_t result=0;
        for(auto &g:pair) {
            int weight=g.age<half?g.age:length-g.age;
            const int16_t* p=hum?idle:swings[g.variant].data;
            uint32_t n=hum?sizeof(idle)/sizeof(int16_t):swings[g.variant].count;
            result+=read(p,n,g.phase)*weight/half;
            g.phase+=hum?g.step:pitch_;
            if(++g.age>=length)seed(g,hum);
        }
        return result;
    }
public:
    void diagnosticReset() {rng_=0x927ac51u;reset();start_=(sizeof(LightsaberSamples::ignition)/sizeof(int16_t))<<16;enabled_=true;}
    void reset() {
        start_=0;motion_=0;gain_=0;pitch_=65536;enabled_=false;
        previous_=current_=0;sub_=0;
        // Preserve random state across mode changes.
        seed(hum_[0],true);seed(hum_[1],true,8000);
        seed(swing_[0],false);seed(swing_[1],false,3200);
    }
    void control(float motion,float rate=2000) {
        if(!std::isfinite(motion)||!std::isfinite(rate)||rate<=0){reset();return;}
        if(!enabled_) { reset();enabled_=true; }
        motion=std::fmax(0.f,std::fmin(1.f,motion));
        float target=std::fmax(0.f,(motion-.025f)/.975f);
        float tau=target>motion_?.012f:.055f;
        motion_+=(target-motion_)/(1.f+rate*tau);
        gain_=(int32_t)(.90f*std::sqrt(motion_)*32767);
        pitch_=(uint32_t)((.55f+.55f*motion_)*65536);
    }
    int32_t next() {
        if(sub_==0) { previous_=current_;current_=source(); }
        int32_t y=(previous_+((int32_t)current_-previous_)*(sub_+1)/3);
        sub_=(sub_+1)%3;return y;
    }
private:
    int32_t source() {
        if(!enabled_)return 0;
        using namespace LightsaberSamples;
        constexpr uint32_t ns=sizeof(ignition)/sizeof(int16_t);
        uint32_t i=start_>>16;
        int32_t fade=i>ns-3200?(int32_t)((i-(ns-3200))*32767/3200):0;
        if(fade>32767)fade=32767;
        int32_t ignite=read(ignition,ns,start_);
        if(i<ns)start_+=65536;
        int32_t bed=grains(hum_,true),whoosh=grains(swing_,false);
        // Leave headroom rather than distorting sample peaks.
        int32_t bed_gain=6553-gain_/6;
        int32_t y=ignite*(32767-fade)/32768*(32767-gain_)/32768;
        y+=bed*fade/32768*bed_gain/32768;
        y+=whoosh*gain_/32768;
        return y;
    }
};
