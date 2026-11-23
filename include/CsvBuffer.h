#pragma once
#include <cstddef>
#include <algorithm>
namespace bite {
// Bounded single-producer loop buffer. Whole records are dropped on overflow.
template<size_t Capacity> class CsvBuffer {
    char data_[Capacity];size_t head_=0,tail_=0,used_=0;
public:
    void clear(){head_=tail_=used_=0;}
    size_t size()const{return used_;}
    bool append(const char* data,size_t length){if(length>Capacity-used_)return false;for(size_t i=0;i<length;++i){data_[head_]=data[i];head_=(head_+1)%Capacity;}used_+=length;return true;}
    size_t peek(char* out,size_t maximum)const{size_t n=std::min(used_,maximum);for(size_t i=0;i<n;++i)out[i]=data_[(tail_+i)%Capacity];return n;}
    void consume(size_t n){n=std::min(n,used_);tail_=(tail_+n)%Capacity;used_-=n;}
};
}
