#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <algorithm>
// 48 kHz, 24-frame lookahead. Gain endpoints never exceed the pending
// block's peak requirement; interpolation is therefore bounded without clipping.
class LightsaberMix {
 int32_t hp_=0,top_=0,lo_=0,lo2_=0,mid_=0,mid2_=0,sub_=0,sub2_=0,negativeSmooth_=0,lfiHp1_=0,lfiHp2_=0;
 int32_t pending_[24][4]={};
 int32_t required_[4]={32768,32768,32768,32768},gain_[4]={32768,32768,32768,32768};
 static int32_t low(int32_t x,int32_t &state,int a) {
  state+=(int32_t)((int64_t)(x-state)*a/32768);return state;
 }
public:
 void reset(){hp_=top_=lo_=lo2_=mid_=mid2_=sub_=sub2_=0,negativeSmooth_=0,lfiHp1_=0,lfiHp2_=0;std::memset(pending_,0,sizeof(pending_));for(int c=0;c<4;c++)required_[c]=gain_[c]=32768;}
 int32_t minGain[4]={32768,32768,32768,32768},maxOutput[4]={};
 void clearStats(){for(int c=0;c<4;c++){minGain[c]=32768;maxOutput[c]=0;}}
 // Complementary residual crossover: LF + MF + HF reconstruct the
 // common band exactly before channel gain trims (electrical sum).
 int32_t negativeBass=0;
 void voice(int32_t x,int32_t out[3],bool boost=false) {
  int32_t band=low(x-low(x,hp_,150),top_,18760); // 35 Hz / 6.5 kHz
  int32_t bass=low(low(band,lo_,511),lo2_,511); // two poles, 120 Hz
  int32_t rest=band-bass;
  int32_t middle=low(low(rest,mid_,4763),mid2_,4763); // 1.2 kHz
  int32_t treble=rest-middle;
  out[0]=middle*4/5;out[1]=treble*6/5;out[2]=bass*7/5;
  int32_t sub=low(low(band,sub_,341),sub2_,341); // 80 Hz, negative side only
  negativeBass=std::max<int32_t>(-11468,std::min<int32_t>(0,sub));
  if(boost)for(int c=0;c<3;c++)out[c]=out[c]*5/4;
 }
 // Sub-bass only (the negative-side LFi feed), for voices that route their own bands.
 void subBass(int32_t x) {
  int32_t band=low(x-low(x,hp_,150),top_,18760);
  int32_t sub=low(low(band,sub_,341),sub2_,341);
  negativeBass=std::max<int32_t>(-11468,std::min<int32_t>(0,sub));
 }
 void process(const int32_t input[24][4],int16_t *out,int slots,float master) {
  const int32_t q=(int32_t)(master*32768);const int32_t all[4]={q,q,q,q};
  process(input,out,slots,all);
 }
 // masterQ: Q15 output gain per channel (intensity x live trim), applied with
 // the limiter gain in one multiply per sample.
 void process(const int32_t input[24][4],int16_t *out,int slots,const int32_t masterQ[4]) {
  for(int c=0;c<4;c++) {
   int32_t peak=1;for(int i=0;i<24;i++)peak=std::max(peak,std::abs(input[i][c]));
   int32_t next=std::min<int32_t>(32768,986316800/peak);
   int32_t target=std::min(required_[c],next);
   target=std::min(target,gain_[c]+164); // 100 ms-scale release
   const int32_t from=(int32_t)((int64_t)gain_[c]*masterQ[c]/32768);
   const int32_t to=(int32_t)((int64_t)target*masterQ[c]/32768);
   for(int i=0;i<24;i++) {
    const int32_t g=from+(to-from)*(i+1)/24;
    const int32_t y=(int32_t)((int64_t)pending_[i][c]*g/32768);
    out[i*slots+c]=(int16_t)y;
    maxOutput[c]=std::max<int32_t>(maxOutput[c],std::abs(y));
    pending_[i][c]=input[i][c];
   }
   minGain[c]=std::min(minGain[c],target);
   gain_[c]=target;required_[c]=next;
  }
 }
};

