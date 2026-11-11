#pragma once
#include <cmath>
struct BatteryEstimate { int percent; int remaining_s; };
inline BatteryEstimate estimateBattery(float mv) {
 static constexpr int table[][3]={
{3300,0,0},
{3530,5,2621},
{3643,10,5242},
{3702,15,7863},
{3738,20,10485},
{3761,25,13106},
{3774,30,15727},
{3787,35,18348},
{3795,40,20970},
{3819,45,23591},
{3836,50,26212},
{3873,55,28834},
{3911,60,31455},
{3937,65,34076},
{3952,70,36697},
{3959,75,39319},
{3961,80,41940},
{3970,85,44561},
{3978,90,47183},
{4005,95,49804},
{4071,100,52425},
};
 if(!std::isfinite(mv))return {-1,-1};
 if(mv<=table[0][0])return {0,0};
 for(int i=1;i<21;++i)if(mv<=table[i][0]){float f=(mv-table[i-1][0])/(table[i][0]-table[i-1][0]);return {(int)(table[i-1][1]+f*(table[i][1]-table[i-1][1])+.5f),(int)(table[i-1][2]+f*(table[i][2]-table[i-1][2]))};}
 return {100,table[20][2]};
}
