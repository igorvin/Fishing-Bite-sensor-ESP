#include "SensorCore.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
int main(int argc,char** argv){
    if(argc!=5){std::cerr<<"usage: replay input.csv dynamic|legacy threshold_g arm_delay_ms\n";return 2;}
    bite::Config cfg;cfg.legacyMode=std::string(argv[2])=="legacy";
    float threshold=std::stof(argv[3]);if(cfg.legacyMode)cfg.legacyThresholdG=threshold;else cfg.thresholdG=threshold;
    bite::SensorCore core;if(!core.configure(cfg)){std::cerr<<"invalid detector configuration\n";return 2;}
    uint32_t armDelay=uint32_t(std::stoul(argv[4]));std::ifstream in(argv[1]);if(!in){std::cerr<<"cannot open input\n";return 2;}
    bool started=false,armRequested=false;uint32_t first=0;size_t rows=0,events=0;std::string line;
    std::cout<<"ms,dynamic_g,legacy_g,noise_g,threshold_g,state,event\n"<<std::fixed<<std::setprecision(6);
    while(std::getline(in,line)){
        std::stringstream record(line);std::string field;std::vector<std::string> fields;while(std::getline(record,field,','))fields.push_back(field);
        if(fields.size()<4||fields[0].empty()||fields[0][0]<'0'||fields[0][0]>'9')continue;
        try{
            uint32_t now=uint32_t(std::stoul(fields[0]));bite::Vec3 a{std::stof(fields[1]),std::stof(fields[2]),std::stof(fields[3])};
            if(!started){core.update(a,now);core.beginCalibration(now);first=now;started=true;}
            if(!armRequested&&core.calibrated()&&bite::elapsed(now,first)>=armDelay){core.setArmed(true,now);armRequested=true;}
            auto event=core.update(a,now);const auto& t=core.telemetry();++rows;if(event)++events;
            std::cout<<now<<','<<t.signal.dynamic<<','<<t.signal.legacy<<','<<t.noiseG<<','<<t.effectiveG<<','<<bite::stateName(core.state())<<','<<unsigned(event)<<'\n';
        }catch(const std::exception&){std::cerr<<"invalid numeric CSV record at row "<<rows+1<<'\n';return 2;}
    }
    std::cerr<<"Replay: "<<rows<<" samples, "<<events<<" emitted pulses; mode="<<argv[2]<<" threshold="<<threshold<<"g; calibration="<<bite::calibrationName(core.calibration())<<".\n";
    if(!rows){std::cerr<<"No usable samples\n";return 2;}return 0;
}
