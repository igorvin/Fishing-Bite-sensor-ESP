#include <cstdint>
#include <cstring>
#include <string>
struct __attribute__((packed)) BitePacket {
  char    rodName[16];
  uint8_t eventType;     // 1=BITE, 2=RUN, 3=LOW BAT
  uint8_t batteryPct;
  float   deltaG;
};
int main(){BitePacket pkt; std::memset(&pkt, 0x41, sizeof(pkt)); std::string rod(pkt.rodName); return rod.size()==0;}
