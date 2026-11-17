#pragma once
#include <array>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>
namespace startup {
constexpr uint32_t Magic=0x31534246; // FBS1, little endian on ESP32.
constexpr uint8_t Version=1,BatteryValid=1;
enum class Kind:uint8_t {Discover=1,Announce=2,Ack=3};
struct __attribute__((packed)) Packet {
 uint32_t magic=Magic;uint8_t version=Version;Kind kind=Kind::Discover;uint8_t flags=0,reserved=0;
 uint32_t hubSession=0,sensorSession=0;char name[16]{};uint8_t battery=0,node=0;uint16_t reserved2=0;
};
static_assert(sizeof(Packet)==36,"Startup control format");
using Mac=std::array<uint8_t,6>;
inline bool unicast(const uint8_t* mac){if(!mac||(mac[0]&1))return false;uint8_t any=0;for(int i=0;i<6;++i)any|=mac[i];return any!=0;}
inline bool batteryKnown(float voltage){return std::isfinite(voltage)&&voltage>=2.5f&&voltage<=4.5f;}
inline bool nameValid(const char* s){
 size_t n=0;while(n<16&&s[n])++n;
 for(size_t i=0;i<n;){uint8_t b=uint8_t(s[i]);if(b<32||b==127)return false;if(b<128){++i;continue;}
  size_t k=b>=0xc2&&b<=0xdf?2:b>=0xe0&&b<=0xef?3:b>=0xf0&&b<=0xf4?4:0;if(!k||i+k>n)return false;
  uint32_t cp=b&((1u<<(7-k))-1);for(size_t j=1;j<k;++j){uint8_t c=uint8_t(s[i+j]);if((c&0xc0)!=0x80)return false;cp=(cp<<6)|(c&63);}
  if((k==2&&cp<0x80)||(k==3&&cp<0x800)||(k==4&&cp<0x10000)||cp>0x10ffff||(cp>=0xd800&&cp<=0xdfff))return false;i+=k;
 }return true;
}
inline void copyName(char out[16],const char* in){
 std::memset(out,0,16);size_t n=std::strlen(in);if(n>15)n=15;
 // Never cut a UTF-8 continuation sequence in a transmitted name.
 while(n&&uint8_t(in[n])>=0x80&&(uint8_t(in[n])&0xc0)==0x80)--n;
 std::memcpy(out,in,n);if(!nameValid(out))std::memset(out,0,16);
}
inline bool decode(const uint8_t* bytes,size_t length,const uint8_t* mac,Packet& out){
 if(!bytes||length!=sizeof(Packet)||!unicast(mac))return false;Packet p;std::memcpy(&p,bytes,sizeof(p));
 if(p.magic!=Magic||p.version!=Version||p.reserved||p.reserved2||(p.flags&~BatteryValid)||(!p.hubSession&&p.kind!=Kind::Announce))return false;
 char zero[16]{};
 if(p.kind==Kind::Discover){if(p.sensorSession||p.flags||p.node||p.battery||std::memcmp(p.name,zero,16))return false;}
 else if(p.kind==Kind::Announce){if(!p.sensorSession||p.node||p.battery>100||(!(p.flags&BatteryValid)&&p.battery)||!nameValid(p.name))return false;}
 else if(p.kind==Kind::Ack){if(!p.sensorSession||p.node<1||p.node>8||p.flags||p.battery||std::memcmp(p.name,zero,16))return false;}
 else return false;out=p;return true;
}
inline uint32_t elapsed(uint32_t now,uint32_t then){return now-then;}
struct Message {Mac mac{};Packet packet{};};
template<class T,size_t N> class Ring {
 std::array<T,N> items_{};size_t head_=0,count_=0;
 public:bool push(const T& value){if(count_==N)return false;items_[(head_+count_)%N]=value;++count_;return true;}
 bool pop(T& value){if(!count_)return false;value=items_[head_];head_=(head_+1)%N;--count_;return true;}
 void clear(){head_=count_=0;}size_t size()const{return count_;}
};
class Sensor {
 Mac peer_{};uint32_t boot_=0,target_=0,lastDiscover_=0,sentAt_=0;uint8_t attempts_=0,node_=0;
 bool enabled_=false,started_=false,confirmed_=false;
 std::array<uint32_t,4> seen_{};uint8_t seenHead_=0;
 bool seen(uint32_t session)const{for(auto s:seen_)if(s==session)return true;return false;}
 void remember(uint32_t session){if(!seen(session)){seen_[seenHead_]=session;seenHead_=(seenHead_+1)%seen_.size();}}
 public:
 void begin(const uint8_t* peer,uint32_t boot){peer_={};if(peer)std::memcpy(peer_.data(),peer,6);boot_=boot?boot:1;target_=lastDiscover_=sentAt_=0;attempts_=node_=0;started_=confirmed_=false;seen_={};seenHead_=0;enabled_=unicast(peer);}
 void disable(){enabled_=false;confirmed_=false;}
 bool receive(const uint8_t* mac,const Packet& p){
  if(!enabled_||!mac||std::memcmp(peer_.data(),mac,6))return false;
  if(p.kind==Kind::Discover){if(!p.hubSession||seen(p.hubSession))return false;remember(p.hubSession);lastDiscover_=target_=p.hubSession;attempts_=0;started_=confirmed_=false;return true;}
  if(p.kind==Kind::Ack&&started_&&p.sensorSession==boot_&&p.hubSession&&(!target_||target_==p.hubSession)&&p.node>=1&&p.node<=8){confirmed_=true;remember(p.hubSession);target_=p.hubSession;node_=p.node;return true;}return false;
 }
 bool due(uint32_t now)const{return enabled_&&!confirmed_&&attempts_<3&&(!started_||elapsed(now,sentAt_)>=700);}
 Packet announce(uint32_t now,const char* name,bool valid,uint8_t battery){Packet p;p.kind=Kind::Announce;p.hubSession=target_;p.sensorSession=boot_;copyName(p.name,name);p.flags=valid?BatteryValid:0;p.battery=valid?battery:0;++attempts_;started_=true;sentAt_=now;return p;}
 bool confirmed()const{return confirmed_;}uint8_t attempts()const{return attempts_;}uint8_t node()const{return node_;}
};
class Discovery {
 uint32_t session_=1,startedAt_=0;uint8_t attempts_=0;bool begun_=false;
 public:void begin(uint32_t session,uint32_t now){session_=session?session:1;startedAt_=now;attempts_=0;begun_=true;}
 bool due(uint32_t now)const{return begun_&&attempts_<3&&elapsed(now,startedAt_)<8000&&elapsed(now,startedAt_)>=uint32_t(attempts_)*1200;}
 Packet take(){Packet p;p.hubSession=session_;++attempts_;return p;}
 bool accepts(const Packet& p)const{return begun_&&p.kind==Kind::Announce&&p.sensorSession&&(!p.hubSession||p.hubSession==session_);}
 Packet ack(const Packet& hello,uint8_t node)const{Packet p;p.kind=Kind::Ack;p.hubSession=session_;p.sensorSession=hello.sensorSession;p.node=node;return p;}
 uint32_t session()const{return session_;}
};
}
