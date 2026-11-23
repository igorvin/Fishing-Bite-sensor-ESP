#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include "../../shared/StartupRadio.h"

namespace hub {
constexpr size_t MaxRods=8, HistorySize=12, RxDepth=24;
// Sensor permits repeatMs=2000; fresh-sample scheduling can exceed that by
// one sample. Allow bounded scheduling/transport jitter, not unlimited gaps.
constexpr uint32_t AlertMs=3500, EpisodeGapMs=2500, PageMs=8000;
inline uint32_t elapsed(uint32_t now,uint32_t then){return now-then;}
struct __attribute__((packed)) WirePacket {char rodName[16];uint8_t eventType,batteryPct;float deltaG;};
static_assert(sizeof(WirePacket)==22,"Existing sensor wire format must remain 22 bytes");
struct Event {std::array<uint8_t,6> mac{};char name[17]{};uint8_t type=0,battery=0;float delta=0;uint32_t at=0;};
// Strict UTF-8, including rejection of overlong, surrogate and >U+10ffff sequences.
inline size_t utf8(const char* s,size_t n,uint32_t& cp){
 if(!n)return 0;uint8_t b=uint8_t(s[0]);if(b<0x80){cp=b;return 1;}
 size_t k=(b>=0xc2&&b<=0xdf)?2:(b>=0xe0&&b<=0xef)?3:(b>=0xf0&&b<=0xf4)?4:0;
 if(!k||n<k)return 0;cp=b&((1u<<(7-k))-1);
 for(size_t i=1;i<k;++i){uint8_t c=uint8_t(s[i]);if((c&0xc0)!=0x80)return 0;cp=(cp<<6)|(c&63);}
 if((k==2&&cp<0x80)||(k==3&&cp<0x800)||(k==4&&cp<0x10000)||cp>0x10ffff||(cp>=0xd800&&cp<=0xdfff))return 0;
 return k;
}
inline void cleanName(const char in[16],char out[17]){
 size_t n=0;while(n<16&&in[n])++n;size_t i=0,j=0;
 while(i<n){uint32_t cp=0;size_t k=utf8(in+i,n-i,cp);if(!k){out[j++]='?';++i;continue;}
  if(cp<32||cp==127){out[j++]=' ';i+=k;continue;}
  if(j+k>16)break;for(size_t x=0;x<k;++x)out[j++]=in[i++];
 }
 while(j&&out[j-1]==' ')--j;size_t start=0;while(start<j&&out[start]==' ')++start;
 if(start){std::memmove(out,out+start,j-start);j-=start;}out[j]=0;
 if(!j)std::strcpy(out,"ДАТЧИК");
}
inline bool decode(const uint8_t* data,size_t len,const uint8_t* mac,uint32_t now,Event& out){
 if(!data||!mac||len!=sizeof(WirePacket))return false;WirePacket p;std::memcpy(&p,data,sizeof(p));
 if(p.eventType<1||p.eventType>3||p.batteryPct>100||!std::isfinite(p.deltaG)||p.deltaG<0)return false;
 uint8_t any=0;for(size_t i=0;i<6;++i)any|=mac[i];if(!any||(mac[0]&1))return false;
 Event e;std::copy(mac,mac+6,e.mac.begin());cleanName(p.rodName,e.name);e.type=p.eventType;e.battery=p.batteryPct;e.delta=p.deltaG;e.at=now;out=e;return true;
}
// Queue policy shared by the firmware adapter and host tests: drop newest when full.
// Firmware protects this storage with a FreeRTOS portMUX (never noInterrupts()).
template<typename T,size_t N> class Ring {
 std::array<T,N> a_{};size_t head_=0,count_=0;
 public:bool push(const T& t){if(count_==N)return false;a_[(head_+count_)%N]=t;++count_;return true;}
 bool pop(T& t){if(!count_)return false;t=a_[head_];head_=(head_+1)%N;--count_;return true;}
 size_t size()const{return count_;}
};
struct Rod {bool used=false,batteryKnown=true;std::array<uint8_t,6> mac{};char name[17]{};uint8_t battery=0,type=0;float delta=0;uint32_t lastAt=0,motionAt=0,lowAt=0;bool motionSeen=false,lowSeen=false;uint32_t episode=0;};
struct Entry {uint32_t episode=0;uint8_t rod=0,type=0,battery=0;uint32_t at=0,lastAt=0;float peak=0;};
struct Alert {bool valid=false;uint8_t rod=0,type=0,battery=0;uint32_t at=0;};
enum class Screen {Summary,List,History,Service,Alert};
enum class Power {On,Dim,Off};
struct View {Screen screen=Screen::Summary;size_t page=0;};
struct Ingest {bool accepted=false,newEpisode=false,motor=false;uint8_t type=0;};
class Model {
 std::array<Rod,MaxRods> rods_{};std::array<Entry,HistorySize> history_{};size_t count_=0,hcount_=0,hhead_=0;
 std::array<Alert,MaxRods> pending_{};Alert current_{};uint32_t shownAt_=0,activity_=0,navAt_=0,manualAt_=0,seq_=0;
 bool manual_=false;Screen manualScreen_=Screen::List;size_t manualPage_=0;
 static unsigned priority(uint8_t t){return t==3?1:2;}
 void next(uint32_t now){
  size_t choice=MaxRods;for(size_t i=0;i<MaxRods;++i)if(pending_[i].valid&&(choice==MaxRods||priority(pending_[i].type)>priority(pending_[choice].type)||(priority(pending_[i].type)==priority(pending_[choice].type)&&elapsed(now,pending_[i].at)>elapsed(now,pending_[choice].at))))choice=i;
  if(choice==MaxRods){current_={};return;}current_=pending_[choice];pending_[choice]={};shownAt_=now;
 }
 void queue(uint8_t rod,uint8_t type,uint8_t battery,uint32_t now){
  Alert incoming{true,rod,type,battery,now};
  // A newer motion event replaces this rod's stale warning, not another rod's.
  if(type!=3&&pending_[rod].valid&&pending_[rod].type==3)pending_[rod]={};
  if(current_.valid&&current_.rod==rod&&priority(current_.type)>=priority(type))return;
  if(current_.valid&&priority(type)>priority(current_.type)){
   if(current_.rod!=rod&&!pending_[current_.rod].valid)pending_[current_.rod]=current_;current_=incoming;shownAt_=now;return;
  }
  auto& p=pending_[rod];if(!p.valid||priority(type)>priority(p.type))p=incoming;
  if(!current_.valid)next(now);
 }
 public:
 uint32_t registryDrops=0,historyOverwrites=0;
 void begin(uint32_t now){activity_=navAt_=now;}
 int registerRod(const uint8_t* mac,const char name[16],bool batteryValid,uint8_t battery,uint32_t now){
  size_t id=0;for(;id<count_;++id)if(!std::memcmp(rods_[id].mac.data(),mac,6))break;
  if(id==count_){if(count_==MaxRods){++registryDrops;return -1;}rods_[id].used=true;std::memcpy(rods_[id].mac.data(),mac,6);++count_;}
  cleanName(name,rods_[id].name);rods_[id].battery=battery;rods_[id].batteryKnown=batteryValid;rods_[id].lastAt=now;return int(id);
 }
 Ingest ingest(const Event& e,uint32_t now){
  size_t id=0;for(;id<count_;++id)if(rods_[id].mac==e.mac)break;
  if(id==count_){if(count_==MaxRods){++registryDrops;return {};}
   rods_[id].used=true;rods_[id].mac=e.mac;++count_;
  }
  Rod& r=rods_[id];std::memcpy(r.name,e.name,sizeof(r.name));r.battery=e.battery;r.type=e.type;r.delta=e.delta;r.lastAt=e.at;
  bool continuation=e.type==2&&r.motionSeen&&elapsed(e.at,r.motionAt)<=EpisodeGapMs;
  bool repeatedLow=e.type==3&&r.lowSeen&&elapsed(e.at,r.lowAt)<60000;
  bool fresh=!(continuation||repeatedLow);
  if(e.type!=3){r.motionSeen=true;r.motionAt=e.at;}else{r.lowSeen=true;r.lowAt=e.at;}
  if(fresh){
   activity_=now;
   Entry x{++seq_,uint8_t(id),e.type,e.battery,e.at,e.at,e.delta};if(e.type!=3)r.episode=x.episode;
   history_[hhead_]=x;hhead_=(hhead_+1)%HistorySize;if(hcount_<HistorySize)++hcount_;else ++historyOverwrites;
   queue(uint8_t(id),e.type,e.battery,now);
  }else if(continuation){
   for(size_t i=0;i<hcount_;++i){Entry& x=history_[(hhead_+HistorySize-1-i)%HistorySize];if(x.episode==r.episode){x.type=2;x.lastAt=e.at;x.battery=e.battery;x.peak=std::max(x.peak,e.delta);break;}}
   if(current_.valid&&current_.rod==id&&current_.type!=3){current_.type=2;current_.battery=e.battery;}
   if(pending_[id].valid&&pending_[id].type!=3){pending_[id].type=2;pending_[id].battery=e.battery;}
  }
  return {true,fresh,fresh,e.type};
 }
 void tick(uint32_t now){if(current_.valid&&elapsed(now,shownAt_)>=AlertMs){current_={};next(now);if(!current_.valid)activity_=now-8000u;}if(manual_&&elapsed(now,manualAt_)>=30000){manual_=false;navAt_=now;}}
 void select(Screen s,size_t page,uint32_t now){manual_=true;manualScreen_=s;manualPage_=page;manualAt_=activity_=now;}
 View view(uint32_t)const{
  if(current_.valid)return {Screen::Alert,0};
  if(manual_)return {manualScreen_,manualPage_};
  return {Screen::Summary,0};
 }
 Power power(uint32_t now)const{if(current_.valid||manual_)return Power::On;return elapsed(now,activity_)<8000?Power::On:Power::Off;}
 size_t rodCount()const{return count_;}const Rod& rod(size_t id)const{return rods_[id];}
 size_t historyCount()const{return hcount_;}const Entry& history(size_t newest)const{return history_[(hhead_+HistorySize-1-newest)%HistorySize];}
 const Alert& alert()const{return current_;}uint32_t alertRemaining(uint32_t now)const{return !current_.valid?0:AlertMs-std::min(AlertMs,elapsed(now,shownAt_));}
};
class Motor {
 uint32_t at_=0,stoppedAt_=0;uint8_t pattern_=0;bool everStopped_=false;
 public:
 bool busy()const{return pattern_!=0;}
 bool start(uint8_t type,uint32_t now){// A bite may replace a low-battery pattern, never the reverse. The
 // preceding low pulse is at most150ms, so promotion to800ms stays<1s.
 if(pattern_){if(pattern_!=3||type==3)return false;}else if(everStopped_&&elapsed(now,stoppedAt_)<200)return false;
 pattern_=type;at_=now;return true;}
 bool tick(uint32_t now){if(!pattern_)return false;uint32_t age=elapsed(now,at_),end=pattern_==2?800:pattern_==3?420:300;
  if(age>=end){pattern_=0;stoppedAt_=now;everStopped_=true;return false;}
  return pattern_!=3||age<150||age>=270;
 }
};
inline int batteryPercent(float v){if(!std::isfinite(v)||v<2.5f||v>4.5f)return -1;
 const float volts[]={3.45f,3.55f,3.63f,3.70f,3.75f,3.80f,3.85f,3.98f,4.10f,4.18f};
 int p=0;for(int i=0;i<10;++i)if(v>=volts[i])p=(i+1)*10;return p;
}
}
