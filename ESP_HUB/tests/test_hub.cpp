#include "HubCore.h"
#include "HubRender.h"
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
using namespace hub;
static int count=0;
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
#define TEST(name,...) do {__VA_ARGS__; ++count;std::printf("PASS: %s\n",name);}while(0)
Event event(uint8_t rod,uint8_t type,uint32_t at,const char* name="ПРУД"){Event e;e.mac={2,0,0,0,0,rod};std::strncpy(e.name,name,16);e.type=type;e.battery=64;e.delta=.12f;e.at=at;return e;}
WirePacket packet(){WirePacket p{};std::strcpy(p.rodName,"ПРУД");p.eventType=1;p.batteryPct=64;p.deltaG=.1f;return p;}
int main(){
 TEST("wire-size-and-valid-decode",{
  auto p=packet();Event e;uint8_t mac[]={2,0,0,0,0,1};CHECK(sizeof(p)==22);CHECK(decode((uint8_t*)&p,22,mac,123,e));CHECK(e.type==1&&e.battery==64&&e.at==123);CHECK(!std::strcmp(e.name,"ПРУД"));
 });
 TEST("reject-invalid-packets",{
  auto p=packet();Event e;uint8_t mac[]={2,0,0,0,0,1};CHECK(!decode((uint8_t*)&p,21,mac,0,e));CHECK(!decode(nullptr,22,mac,0,e));
  p.eventType=0;CHECK(!decode((uint8_t*)&p,22,mac,0,e));p.eventType=4;CHECK(!decode((uint8_t*)&p,22,mac,0,e));p.eventType=1;
  p.batteryPct=101;CHECK(!decode((uint8_t*)&p,22,mac,0,e));p.batteryPct=64;p.deltaG=-1;CHECK(!decode((uint8_t*)&p,22,mac,0,e));p.deltaG=std::numeric_limits<float>::quiet_NaN();CHECK(!decode((uint8_t*)&p,22,mac,0,e));p.deltaG=std::numeric_limits<float>::infinity();CHECK(!decode((uint8_t*)&p,22,mac,0,e));
 });
 TEST("bounded-nonterminated-name",{
  auto p=packet();std::memset(p.rodName,'A',16);Event e;uint8_t mac[]={2,0,0,0,0,1};CHECK(decode((uint8_t*)&p,22,mac,0,e));CHECK(std::strlen(e.name)==16);CHECK(e.name[16]==0);
 });
 TEST("utf8-name-safety-and-fallback",{
  char raw[16]{};char name[17];std::memcpy(raw,"  ЁЖ  ",8);cleanName(raw,name);CHECK(!std::strcmp(name,"ЁЖ"));
  std::memset(raw,0,16);cleanName(raw,name);CHECK(!std::strcmp(name,"ДАТЧИК"));
  std::memset(raw,'A',16);raw[15]=char(0xd0);cleanName(raw,name);CHECK(name[15]=='?'&&name[16]==0);
  uint32_t cp;CHECK(utf8("Ё",2,cp)==2&&cp==0x401);CHECK(utf8("\xc0\x80",2,cp)==0);CHECK(utf8("\xed\xa0\x80",3,cp)==0);CHECK(utf8("\xf4\x90\x80\x80",4,cp)==0);
 });
 TEST("rx-ring-fifo-and-overflow",{
  Ring<int,3> q;CHECK(q.push(1)&&q.push(2)&&q.push(3));CHECK(!q.push(4));int x;CHECK(q.pop(x)&&x==1);CHECK(q.push(5));CHECK(q.pop(x)&&x==2);CHECK(q.pop(x)&&x==3);CHECK(q.pop(x)&&x==5);CHECK(!q.pop(x));
 });
 TEST("mac-identity-not-name",{
  Model m;m.begin(0);m.ingest(event(1,1,10,"SAME"),10);m.ingest(event(2,1,20,"SAME"),20);CHECK(m.rodCount()==2);CHECK(m.rod(0).mac!=m.rod(1).mac);
 });
 TEST("rod-capacity-retains-stable-ids",{
  Model m;m.begin(0);for(uint8_t i=1;i<=8;++i)CHECK(m.ingest(event(i,1,i*10),i*10).accepted);CHECK(!m.ingest(event(9,1,100),100).accepted);CHECK(m.rodCount()==8&&m.registryDrops==1);CHECK(m.rod(0).mac[5]==1);
 });
 TEST("continuation-coalesces-without-extending-alert",{
  Model m;m.begin(0);CHECK(m.ingest(event(1,1,100),100).newEpisode);for(uint32_t t=300;t<=3900;t+=200){CHECK(!m.ingest(event(1,2,t),t).newEpisode);m.tick(t);}CHECK(m.historyCount()==1);CHECK(m.history(0).type==2);CHECK(!m.alert().valid);
 });
 TEST("continuation-after-gap-becomes-new-episode",{
  Model m;m.begin(0);m.ingest(event(1,1,100),100);CHECK(m.ingest(event(1,2,2201),2201).newEpisode);CHECK(m.historyCount()==2);
 });
 TEST("multiple-rods-display-fifo",{
  Model m;m.begin(0);m.ingest(event(1,1,100),100);m.ingest(event(2,1,200),200);m.ingest(event(3,1,300),300);CHECK(m.alert().rod==0);m.tick(3600);CHECK(m.alert().rod==1);m.tick(7100);CHECK(m.alert().rod==2);CHECK(m.historyCount()==3);
 });
 TEST("bite-preempts-low-battery-and-retains-warning",{
  Model m;m.begin(0);m.ingest(event(1,3,100),100);m.ingest(event(2,1,200),200);CHECK(m.alert().rod==1&&m.alert().type==1);m.tick(3700);CHECK(m.alert().rod==0&&m.alert().type==3);
 });
 TEST("newer-bite-replaces-stale-warning-for-same-rod",{
  Model m;m.begin(0);auto low=event(1,3,100);low.battery=10;m.ingest(low,100);m.ingest(event(1,1,200),200);CHECK(m.alert().type==1);m.tick(3700);CHECK(!m.alert().valid);CHECK(m.historyCount()==2);
 });
 TEST("low-battery-does-not-steal-motion-history",{
  Model m;m.begin(0);m.ingest(event(1,1,100),100);m.ingest(event(1,3,200),200);m.ingest(event(1,2,300),300);CHECK(m.historyCount()==2);CHECK(m.history(0).type==3);CHECK(m.history(1).type==2);CHECK(!m.ingest(event(1,3,400),400).newEpisode);
 });
 TEST("bounded-journal-overwrites-oldest",{
  Model m;m.begin(0);for(int i=0;i<15;++i)m.ingest(event(1,1,uint32_t(10+i*10)),uint32_t(10+i*10));CHECK(m.historyCount()==12&&m.historyOverwrites==3);CHECK(m.history(0).at==150&&m.history(11).at==40);
 });
 TEST("power-off-not-reset-by-auto-pages",{
  Model m;m.begin(100);CHECK(m.power(60099)==Power::On);CHECK(m.power(60100)==Power::Dim);for(uint32_t t=100;t<=120100;t+=1000){m.tick(t);m.view(t);}CHECK(m.power(120100)==Power::Off);m.ingest(event(1,1,120101),120101);CHECK(m.power(120101)==Power::On);
 });
 TEST("rollover-alert-and-power",{
  Model m;uint32_t start=0xfffffff0u;m.begin(start);m.ingest(event(1,1,start),start);m.tick(start+3500u);CHECK(!m.alert().valid);CHECK(m.power(start+120000u)==Power::Off);
 });
 TEST("manual-navigation-expires-and-alert-wins",{
  Model m;m.begin(0);m.select(Screen::Service,0,100);CHECK(m.view(101).screen==Screen::Service);m.ingest(event(1,1,200),200);CHECK(m.view(201).screen==Screen::Alert);m.tick(3700);CHECK(m.view(3700).screen==Screen::Service);m.tick(30100);CHECK(m.view(30100).screen==Screen::List);
 });
 TEST("buttonless-list-and-history-pagination",{
  Model m;m.begin(0);for(int i=1;i<=4;++i)m.ingest(event(uint8_t(i),1,uint32_t(i)),uint32_t(i));for(uint32_t t=4000;t<20000;t+=4000)m.tick(t);
  CHECK(m.view(24000).screen==Screen::History);CHECK(m.view(24000).page==1);CHECK(m.view(32000).screen==Screen::List);
 });
 TEST("motor-patterns-are-bounded-and-no-retrigger",{
  Motor m;CHECK(m.start(2,100));CHECK(!m.start(2,200));CHECK(m.tick(899));CHECK(!m.tick(900));CHECK(!m.start(1,901));CHECK(m.start(3,1100));CHECK(m.tick(1249));CHECK(!m.tick(1250));CHECK(m.busy());CHECK(m.tick(1370));CHECK(!m.tick(1520));CHECK(!m.busy());
 });
 TEST("bite-motor-preempts-low-battery-pattern",{
  Motor m;CHECK(m.start(3,100));CHECK(m.tick(200));CHECK(m.start(2,240));CHECK(!m.start(3,241));CHECK(m.tick(1039));CHECK(!m.tick(1040));
 });
 TEST("motor-rollover",{
  Motor m;uint32_t t=0xfffffff0u;CHECK(m.start(1,t));CHECK(m.tick(t+299));CHECK(!m.tick(t+300));
 });
 TEST("battery-invalid-is-not-zero-percent",{
  CHECK(batteryPercent(.62f)==-1);CHECK(batteryPercent(5.f)==-1);CHECK(batteryPercent(std::numeric_limits<float>::quiet_NaN())==-1);CHECK(batteryPercent(4.2f)==100);CHECK(batteryPercent(3.7f)==40);
 });
 TEST("renderer-empty-and-utf8-screens",{
  Model m;m.begin(0);Frame empty=render(m,0,-1);bool lit=false;for(uint8_t b:empty.bytes)lit|=b!=0;CHECK(lit);
  auto e=event(1,1,100,"ЁЖИК-ДЛИННЫЙ");m.ingest(e,100);Frame alert=render(m,100,78);CHECK(alert.bytes!=empty.bytes);m.tick(3600);m.select(Screen::History,0,3600);CHECK(render(m,3600,78).bytes!=alert.bytes);
  CHECK(glyph(0x401)!=glyph('?'));CHECK(glyph(0x451)==glyph(0x401));CHECK(glyph(0x1f600)==glyph('?'));
 });
 TEST("renderer-respects-text-region",{
  Frame f;f.text("ДЛИННОЕ ИМЯ С Ё",10,10,35,1);const Frame& cf=f;for(int y=0;y<64;++y)for(int x=0;x<128;++x)if(cf.pixel(x,y))CHECK(x>=10&&x<45&&y>=10&&y<17);
 });
 std::printf("%d tests, 0 failed\n",count);
}
