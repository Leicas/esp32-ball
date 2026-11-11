#pragma once
#include <cmath>
#include <cstdint>

// 48 kHz sound layer, separate from the 2 kHz tactile EQ. Per-sample work is
// integer-only: band-limited noise + swept tones + a decaying metallic ring.
class LightsaberAudio {
    int16_t sine_[256]={};
    uint32_t phase_=0,ring_phase_=0,metal_phase_=0,rng_=0x6ac31u;
    uint32_t inc_=0,ring_inc_=0,metal_inc_=0;
    int32_t fast_=0,slow_=0,level_=0,target_=0,ring_level_=0;
    float ring_=0,attack_=0,cool_=0;
    bool hit_=false;
    static int32_t bound(int32_t x) { return x<-32767?-32767:x>32767?32767:x; }
public:
    void init() { for(int i=0;i<256;++i)sine_[i]=(int16_t)(32767*std::sin(6.28318530718f*i/256));reset(); }
    void reset() {
        phase_=ring_phase_=metal_phase_=inc_=ring_inc_=metal_inc_=0;
        fast_=slow_=level_=target_=ring_level_=0;ring_=attack_=cool_=0;
        rng_=0x6ac31u;hit_=false;
    }
    void control(float motion,bool clash,float rate=2000) {
        if(!std::isfinite(motion)) {reset();return;}
        motion=std::fmax(0.f,std::fmin(1.f,motion));
        cool_=std::fmax(0.f,cool_-1/rate);
        // Ordinary swings stay a deep whoosh. High metallic sound is a clash,
        // not an automatic whistle at every swing onset.
        if((clash&&!hit_)&&cool_==0) {
            ring_=1;attack_=0;cool_=.22f;
        }
        hit_=clash;
        attack_=std::fmin(1.f,attack_+250/rate);
        ring_*=1-3.5f/rate;if(ring_<.0001f)ring_=0;
        float whoosh=std::fmax(0.f,(motion-.045f)/.955f);
        // Linear motion envelope: ordinary swings no longer lose most of
        // their level to squaring a small motion value.
        target_=(int32_t)(whoosh*32767);
        ring_level_=(int32_t)(ring_*attack_*26000);
        inc_=(uint32_t)((100+260*motion)*(4294967296.0/48000));
        ring_inc_=(uint32_t)((900+1800*ring_)*(4294967296.0/48000));
        metal_inc_=(uint32_t)((1420+2450*ring_)*(4294967296.0/48000));
    }
    int32_t next() {
        if(target_==0 && level_==0 && ring_level_==0)return 0;
        // 2 ms attack, 5 ms release; exact-zero tail for silence after reset.
        int32_t delta=target_-level_;
        level_+=delta/(delta>0?96:240);
        if(std::abs(delta)<240)level_=target_;
        rng_^=rng_<<13;rng_^=rng_>>17;rng_^=rng_<<5;
        int32_t noise=(int32_t)(rng_&65535)-32768;
        fast_+=(noise-fast_)/40;slow_+=(noise-slow_)/240;
        phase_+=inc_;ring_phase_+=ring_inc_;metal_phase_+=metal_inc_;
        int32_t wind=bound((fast_-slow_)*6);
        int32_t swoosh=(wind+sine_[phase_>>24]*3)/4;
        int32_t metal=(sine_[ring_phase_>>24]*3+sine_[metal_phase_>>24])/4;
        int32_t y=(swoosh*level_)/32768+(metal*ring_level_)/32768;
        return y*3;
    }
};
