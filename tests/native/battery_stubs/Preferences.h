#pragma once
#include <vector>
#include <cstring>
inline std::vector<unsigned char> nv;
inline bool fail_save=false;
struct Preferences {
 bool begin(const char*,bool){return true;}
 void end(){}
 size_t getBytesLength(const char*){return nv.size();}
 size_t getBytes(const char*,void* p,size_t n){if(n!=nv.size())return 0;memcpy(p,nv.data(),n);return n;}
 size_t putBytes(const char*,const void* p,size_t n){if(fail_save)return 0;nv.assign((const unsigned char*)p,(const unsigned char*)p+n);return n;}
};
