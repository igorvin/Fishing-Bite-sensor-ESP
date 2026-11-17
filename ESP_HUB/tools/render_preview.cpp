#include "HubRender.h"
#include <cstdio>
#include <cstring>
#include <string>
using namespace hub;
Event fixture(uint8_t id,uint8_t type,uint32_t now,const char* name,uint8_t battery=64){
 WirePacket p{};std::memcpy(p.rodName,name,std::min<size_t>(16,std::strlen(name)));p.eventType=type;p.batteryPct=battery;p.deltaG=.02f;
 uint8_t mac[]={2,0,0,0,0,id};Event e;if(!decode(reinterpret_cast<uint8_t*>(&p),sizeof(p),mac,now,e))std::abort();return e;
}
void save(const std::string& folder,const char* name,const Model& m,uint32_t now,int battery,const Diagnostics& diagnostics={}){
 Frame f=render(m,now,battery,diagnostics);auto path=folder+"/"+name+".pbm";FILE* file=std::fopen(path.c_str(),"wb");if(!file)std::abort();std::fprintf(file,"P4\n128 64\n");
 // PBM 1 means black; our framebuffer1 means illuminated OLED pixel.
 for(uint8_t b:f.bytes)std::fputc(uint8_t(~b),file);std::fclose(file);
}
int main(int argc,char** argv){if(argc!=2)return 1;std::string path=argv[1];Model empty;empty.begin(0);save(path,"empty",empty,0,-1);
 Model m;m.begin(0);m.ingest(fixture(1,1,100,"КАРП"),100);m.ingest(fixture(2,1,200,"ФИДЕР"),200);m.ingest(fixture(3,1,300,"ДАЛЬНИЙ"),300);m.ingest(fixture(4,1,400,"FLOATER"),400);
 for(uint32_t t=4000;t<=20000;t+=4000)m.tick(t);
 m.select(Screen::List,0,21000);save(path,"rods",m,21000,78);m.select(Screen::List,1,21000);save(path,"rods-page2",m,21000,78);
 m.select(Screen::History,0,22000);save(path,"history",m,22000,78);
 Model alert;alert.begin(0);alert.ingest(fixture(1,1,100,"КАРП"),100);alert.ingest(fixture(2,1,200,"ФИДЕР"),200);alert.tick(3600);save(path,"bite",alert,3700,78);
 Model low;low.begin(0);low.ingest(fixture(1,3,100,"ПРИЧАЛ-ЛЕВЫЙ",10),100);save(path,"low-long-name",low,200,-1);
 Diagnostics d;d.radio=true;std::strcpy(d.mac,"02:00:00:00:00:01");d.rx=128;d.invalid=2;d.dropped=3;m.select(Screen::Service,0,23000);save(path,"service",m,23000,78,d);
 return 0;
}
