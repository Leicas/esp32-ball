#pragma once
#include <cstdint>
#include <cstdio>
#include <cstddef>
#define INPUT 0
#define LOW 0
inline uint32_t fake_ms=0;
inline bool fake_usb=true;
inline uint32_t millis(){return fake_ms;}
inline void pinMode(int,int){}
inline int digitalRead(int){return fake_usb?0:1;}
inline uint32_t analogReadMilliVolts(int){return 2000;}
struct FakeSerial { operator bool()const{return true;} template<class... T> void printf(const char*,T...){} };
inline FakeSerial Serial;
