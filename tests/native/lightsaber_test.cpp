#include "lightsaber.h"
#include "lightsaber_audio.h"
#include "lightsaber_reference.h"
#include <cassert>
#include <cmath>
#include <limits>
#include <cstdio>
int main() {
    Lightsaber s;s.init();double idle=0,moving=0;
    for(int i=0;i<4000;++i) { auto v=s.step(0,0);assert(v.impact==0);if(i>2000)idle+=v.hum*v.hum; }
    assert(idle>1);
    for(int i=0;i<4000;++i) { auto v=s.step(8,0);assert(v.impact==0);assert(std::fabs(v.hum)<=.83);if(i>2000)moving+=v.hum*v.hum; }
    assert(moving>idle*3);
    s.reset();int pulses=0,count=0;bool last=false;
    for(int i=0;i<3000;++i) {
        auto v=s.step(0,70);assert(v.impact>=0 && v.impact<=1);
        bool on=v.impact>0;if(on&&!last)++pulses;if(on)++count;last=on;
    }
    assert(pulses==1 && count<=16);
    s.reset();auto v=s.step(std::numeric_limits<float>::quiet_NaN(),0);
    assert(std::isfinite(v.hum)&&v.impact==0);
    LightsaberAudio audio;audio.init();
    for(int n=0;n<4800;++n) { if(n%24==0)audio.control(0,false);assert(audio.next()==0); }
    double energy=0;int crossings=0;int16_t previous=0;
    for(int n=0;n<24000;++n) {
        if(n%24==0)audio.control(.9f,false);
        int16_t sample=audio.next();energy+=(double)sample*sample;
        if(sample>0&&previous<=0)++crossings;previous=sample;
        assert(sample>=-32767&&sample<=32767);
    }
    assert(energy/24000>1000000 && crossings>80 && crossings<230); // low whoosh, no automatic high-pitched ring
    audio.reset();energy=0;
    for(int n=0;n<24000;++n) {
        if(n%24==0)audio.control(.2f,false);
        double sample=audio.next();if(n>2400)energy+=sample*sample;
    }
    assert(std::sqrt(energy/21600)>2000); // normal swings must be audible too
    audio.reset();for(int n=0;n<100;++n)assert(audio.next()==0);
    audio.control(std::numeric_limits<float>::quiet_NaN(),false);assert(audio.next()==0);
    LightsaberReference reference;reference.reset();assert(reference.next()==0);
    double ignition_energy=0,loop_energy=0;
    for(int n=0;n<48000*5;++n) {
        if(n%24==0)reference.control(0);
        int sample=reference.next();assert(sample>=-32767&&sample<=32767);
        if(n<48000)ignition_energy+=(double)sample*sample;
        if(n>48000*3)loop_energy+=(double)sample*sample;
    }
    assert(ignition_energy>1e10 && loop_energy>1e10);
    reference.reset();assert(reference.next()==0);
    // A steady slow swing must remain audible well beyond a one-shot clip.
    LightsaberReference moving_ref,idle_ref;
    moving_ref.reset();idle_ref.reset();
    double sustained[4]={},stopped=0;
    for(int n=0;n<48000*9;++n) {
        if(n%24==0) {
            moving_ref.control(n>=48000*3&&n<48000*7?.25f:0.f);
            idle_ref.control(0);
        }
        int a=moving_ref.next(),b=idle_ref.next();
        assert(a>-32767&&a<32767); // linear mix keeps clean headroom
        double d=(double)a-b;
        if(n>=48000*3&&n<48000*7)sustained[n/48000-3]+=d*d;
        if(n>=48000*8)stopped+=d*d;
    }
    for(double e:sustained)assert(e/48000>100000);
    assert(stopped/48000<100); // swing envelope really stops
    puts("Lightsaber: idle hum, motion response, bounded positive clash, cooldown/reset passed");
}
