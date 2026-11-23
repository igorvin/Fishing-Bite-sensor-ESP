#pragma once
#include "HubCore.h"
#include <cstdio>

namespace hub {
// Original, hand-authored 5x7 raster alphabet. Uppercase is intentional on a
// 128x64 one-bit panel. No OS fonts, generated proprietary glyphs or host-only
// replacement fonts are used: preview and firmware execute this same renderer.
struct Glyph {uint32_t cp;uint8_t rows[7];};
inline const uint8_t* glyph(uint32_t cp){
 if(cp>='a'&&cp<='z')cp-=32;if(cp>=0x430&&cp<=0x44f)cp-=32;if(cp==0x451)cp=0x401;
 switch(cp){case 0x410:cp='A';break;case 0x412:cp='B';break;case 0x415:cp='E';break;case 0x41a:cp='K';break;case 0x41c:cp='M';break;case 0x41d:cp='H';break;case 0x41e:cp='O';break;case 0x420:cp='P';break;case 0x421:cp='C';break;case 0x422:cp='T';break;case 0x425:cp='X';break;}
 static const Glyph data[]={
 {' ',{0,0,0,0,0,0,0}},{'?',{14,17,1,2,4,0,4}},{'.',{0,0,0,0,0,6,6}},{':',{0,6,6,0,6,6,0}},{'-',{0,0,0,31,0,0,0}},{'/',{1,2,2,4,8,8,16}},{'%',{25,26,2,4,8,11,19}},{'!',{4,4,4,4,4,0,4}},{'+',{0,4,4,31,4,4,0}},
 {'0',{14,17,19,21,25,17,14}},{'1',{4,12,4,4,4,4,14}},{'2',{14,17,1,2,4,8,31}},{'3',{30,1,1,14,1,1,30}},{'4',{2,6,10,18,31,2,2}},{'5',{31,16,16,30,1,1,30}},{'6',{14,16,16,30,17,17,14}},{'7',{31,1,2,4,8,8,8}},{'8',{14,17,17,14,17,17,14}},{'9',{14,17,17,15,1,1,14}},
 {'A',{14,17,17,31,17,17,17}},{'B',{30,17,17,30,17,17,30}},{'C',{14,17,16,16,16,17,14}},{'D',{30,17,17,17,17,17,30}},{'E',{31,16,16,30,16,16,31}},{'F',{31,16,16,30,16,16,16}},{'G',{14,17,16,23,17,17,15}},{'H',{17,17,17,31,17,17,17}},{'I',{14,4,4,4,4,4,14}},{'J',{7,2,2,2,18,18,12}},{'K',{17,18,20,24,20,18,17}},{'L',{16,16,16,16,16,16,31}},{'M',{17,27,21,21,17,17,17}},{'N',{17,25,21,19,17,17,17}},{'O',{14,17,17,17,17,17,14}},{'P',{30,17,17,30,16,16,16}},{'Q',{14,17,17,17,21,18,13}},{'R',{30,17,17,30,20,18,17}},{'S',{15,16,16,14,1,1,30}},{'T',{31,4,4,4,4,4,4}},{'U',{17,17,17,17,17,17,14}},{'V',{17,17,17,17,17,10,4}},{'W',{17,17,17,21,21,21,10}},{'X',{17,17,10,4,10,17,17}},{'Y',{17,17,10,4,4,4,4}},{'Z',{31,1,2,4,8,16,31}},
 {0x401,{10,0,31,16,30,16,31}}, // Ё
 {0x411,{31,16,16,30,17,17,30}}, // Б
 {0x413,{31,16,16,16,16,16,16}}, // Г
 {0x414,{6,10,10,10,10,31,17}}, // Д
 {0x416,{21,21,14,4,14,21,21}}, // Ж
 {0x417,{14,17,1,6,1,17,14}}, // З
 {0x418,{17,17,19,21,25,17,17}}, // И
 {0x419,{10,4,17,19,21,25,17}}, // Й
 {0x41b,{7,9,9,9,9,9,17}}, // Л
 {0x41f,{31,17,17,17,17,17,17}}, // П
 {0x423,{17,17,17,15,1,17,14}}, // У
 {0x424,{4,14,21,21,21,14,4}}, // Ф
 {0x426,{18,18,18,18,18,31,1}}, // Ц
 {0x427,{17,17,17,15,1,1,1}}, // Ч
 {0x428,{21,21,21,21,21,21,31}}, // Ш
 {0x429,{21,21,21,21,21,31,1}}, // Щ
 {0x42a,{24,8,8,14,9,9,14}}, // Ъ
 {0x42b,{17,17,17,25,21,21,25}}, // Ы
 {0x42c,{16,16,16,30,17,17,30}}, // Ь
 {0x42d,{14,17,1,7,1,17,14}}, // Э
 {0x42e,{18,21,21,29,21,21,18}}, // Ю
 {0x42f,{15,17,17,15,5,9,17}}, // Я
 };
 for(const auto& g:data)if(g.cp==cp)return g.rows;return data[1].rows;
}
struct Frame {
 static constexpr int Width=128,Height=64;
 std::array<uint8_t,Width*Height/8> bytes{};
 void pixel(int x,int y,bool on){if(x<0||y<0||x>=Width||y>=Height)return;uint8_t mask=uint8_t(0x80>>(x&7));auto& b=bytes[y*16+x/8];if(on)b|=mask;else b&=uint8_t(~mask);}
 bool pixel(int x,int y)const{return (bytes[y*16+x/8]&(0x80>>(x&7)))!=0;}
 void fill(int x,int y,int w,int h,bool on=true){for(int j=y;j<y+h;++j)for(int i=x;i<x+w;++i)pixel(i,j,on);}
 void text(const char* s,int x,int y,int maxWidth=128,int scale=1,bool on=true){
  uint32_t cps[64];size_t n=0;size_t bytesLeft=std::strlen(s);while(bytesLeft&&n<64){uint32_t cp=0;size_t k=utf8(s,bytesLeft,cp);if(!k){k=1;cp='?';}cps[n++]=cp;s+=k;bytesLeft-=k;}
  size_t capacity=size_t(std::max(0,maxWidth+scale)/(6*scale));bool clipped=n>capacity;if(n>capacity)n=capacity;
  for(size_t i=0;i<n;++i){const uint8_t* rows=glyph(clipped&&i+1==n?'.':cps[i]);for(int row=0;row<7;++row)for(int col=0;col<5;++col)if(rows[row]&(16>>col))fill(x+int(i)*6*scale+col*scale,y+row*scale,scale,scale,on);}
 }
};
inline void ageText(uint32_t age,char* out,size_t n){if(age<60000)std::snprintf(out,n,"%uс",unsigned(age/1000));else if(age<3600000)std::snprintf(out,n,"%uм",unsigned(age/60000));else if(age<86400000)std::snprintf(out,n,"%uч",unsigned(age/3600000));else std::snprintf(out,n,"%uд",unsigned(age/86400000));}
inline const char* shortEvent(uint8_t type){return type==3?"БАТ":type==2?"ДВИЖ":"КЛЁВ";}
struct Diagnostics {bool radio=false,oled=false;uint32_t rx=0,invalid=0,dropped=0;char mac[18]{};float voltage=0;};
inline Frame render(const Model& model,uint32_t now,int hubBattery,const Diagnostics& d={}){
 Frame f;View v=model.view(now);char buf[40];
 // Preserve the original clear inverted HUB header and graphical battery.
 f.fill(0,0,128,18);f.text("HUB",3,5,60,1,false);
 f.fill(104,3,20,12,false);f.fill(106,5,16,8);f.fill(124,7,3,4,false);
 if(hubBattery>=0){int bars=std::max(0,std::min(5,hubBattery*5/100));for(int i=0;i<bars;++i)f.fill(107+i*3,5,2,8,false);}
 else f.text("?",111,5,6,1,false);
 if(v.screen==Screen::Alert){
  const Alert& a=model.alert();const Rod& r=model.rod(a.rod);
  if(r.batteryKnown)std::snprintf(buf,sizeof(buf),"%02u %u%%",unsigned(a.rod+1),unsigned(r.battery));else std::snprintf(buf,sizeof(buf),"%02u --%%",unsigned(a.rod+1));f.text(buf,45,5,54,1,false);
  f.text(a.type==3?"БАТАРЕЯ ДАТЧИКА":a.type==2?"ДВИЖЕНИЕ":"ПОКЛЁВКА",2,21,124);
  // The active rod name occupies the main display, two rows if necessary.
  size_t count=0;const char* ptr=r.name;size_t left=std::strlen(ptr);
  while(left&&count<16){uint32_t cp;size_t n=utf8(ptr,left,cp);if(!n)break;++count;ptr+=n;left-=n;}
  if(count<=10){int width=int(count)*12-2;f.text(r.name,std::max(2,(128-width)/2),34,124,2);}
  else {char first[17]{},second[17]{};size_t cut=0,k=0;while(k<8&&cut<std::strlen(r.name)){uint32_t cp;size_t n=utf8(r.name+cut,std::strlen(r.name)-cut,cp);if(!n)break;cut+=n;++k;}
   std::memcpy(first,r.name,cut);std::strcpy(second,r.name+cut);f.text(first,2,32,124,2);f.text(second,2,48,124,2);}
  if(count<=10){std::snprintf(buf,sizeof(buf),"%02u  %s",unsigned(a.rod+1),r.batteryKnown?"":"БАТ ?");f.text(buf,2,55,124);}
  f.fill(0,63,int(model.alertRemaining(now)*128/AlertMs),1);
 }else if(v.screen==Screen::Service){
  f.text(d.radio?"РАДИО: ПРИЁМ":"РАДИО: ОШИБКА",2,21,124);
  f.text(d.mac[0]?d.mac:"MAC НЕИЗВЕСТЕН",2,32,124);
  std::snprintf(buf,sizeof(buf),"RX %lu / BAD %lu",(unsigned long)d.rx,(unsigned long)d.invalid);f.text(buf,2,43,124);
  std::snprintf(buf,sizeof(buf),"DROP %lu / REG %lu",(unsigned long)d.dropped,(unsigned long)model.registryDrops);f.text(buf,2,54,124);
 }else if(v.screen==Screen::Mac){
  // Large MAC for pairing: "AA:BB:CC" / "DD:EE:FF" at double size (8 x 12 px = 96 px wide).
  f.text("MAC ХАБА",2,21,124);
  if(std::strlen(d.mac)==17){char half[9]{};std::memcpy(half,d.mac,8);f.text(half,16,32,120,2);std::memcpy(half,d.mac+9,8);f.text(half,16,48,120,2);}
  else f.text("НЕИЗВЕСТЕН",2,40,124,2);
 }else if(v.screen==Screen::Summary){
  // The startup screen carries the hub MAC so a sensor can be paired without USB.
  f.text(d.radio?"ГОТОВ К ПРИЁМУ":"РАДИО: ОШИБКА",2,21,124);
  std::snprintf(buf,sizeof(buf),"УДИЛИЩ: %u",unsigned(model.rodCount()));f.text(buf,4,31,120,2);
  f.text("УЧТЕНО С ЗАПУСКА",2,47,124);
  f.text(std::strlen(d.mac)==17?d.mac:"MAC НЕИЗВЕСТЕН",2,57,124);
 }else if(v.screen==Screen::List){
  const size_t pages=std::max<size_t>(1,(model.rodCount()+2)/3),page=v.page%pages;
  std::snprintf(buf,sizeof(buf),"УДИЛИЩА %u/%u",unsigned(page+1),unsigned(pages));f.text(buf,2,21,124);
  if(!model.rodCount()){f.text("ОЖИДАНИЕ",16,31,112,2);f.text("НЕТ СОБЫТИЙ",30,54,96);}
  else for(size_t row=0;row<3;++row){size_t i=page*3+row;if(i>=model.rodCount())break;const Rod& r=model.rod(i);int y=33+int(row)*12;
   std::snprintf(buf,sizeof(buf),"%02u",unsigned(i+1));f.text(buf,2,y,12);f.text(r.name,20,y,65);
   ageText(elapsed(now,r.lastAt),buf,sizeof(buf));f.text(buf,92,y,34);
  }
 }else{
  const size_t pages=std::max<size_t>(1,(model.historyCount()+2)/3),page=v.page%pages;
  std::snprintf(buf,sizeof(buf),"ЖУРНАЛ %u/%u",unsigned(page+1),unsigned(pages));f.text(buf,2,21,124);
  if(!model.historyCount())f.text("НЕТ СОБЫТИЙ",30,38,96);
  else for(size_t row=0;row<3;++row){size_t i=page*3+row;if(i>=model.historyCount())break;const Entry& e=model.history(i);int y=33+int(row)*12;
   std::snprintf(buf,sizeof(buf),"%02u",unsigned(e.rod+1));f.text(buf,2,y,12);f.text(shortEvent(e.type),21,y,44);
   ageText(elapsed(now,e.lastAt),buf,sizeof(buf));f.text(buf,92,y,34);
  }
 }
 return f;
}
}
