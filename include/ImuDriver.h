#pragma once
#include <Wire.h>
#include <Adafruit_LSM6DS3.h>
#include "ImuRead.h"
class ImuDriver {
    Adafruit_LSM6DS3 imu_;uint8_t address_=0x6b;
public:
    bool read(uint8_t reg,uint8_t* output,size_t length){
        Wire.beginTransmission(address_);Wire.write(reg);
        if(Wire.endTransmission(false)!=0)return false;
        if(Wire.requestFrom(address_,uint8_t(length))!=length){while(Wire.available())Wire.read();return false;}
        for(size_t i=0;i<length;++i){if(!Wire.available())return false;output[i]=uint8_t(Wire.read());}
        return true;
    }
    bool write(uint8_t reg,uint8_t value){Wire.beginTransmission(address_);Wire.write(reg);Wire.write(value);return Wire.endTransmission()==0;}
    bool begin(){
        uint8_t who=0;bool found=false;
        for(uint8_t addr:{uint8_t(0x6b),uint8_t(0x6a)}){address_=addr;if(read(0x0f,&who,1)&&who==0x69){found=true;break;}}
        if(!found||!imu_.begin_I2C(address_))return false;
        imu_.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);imu_.setAccelDataRate(LSM6DS_RATE_104_HZ);
        imu_.setGyroDataRate(LSM6DS_RATE_SHUTDOWN);
        // BDU prevents torn axis words; auto-increment enables one XYZ burst.
        uint8_t ctrl=0;if(!read(0x12,&ctrl,1)||!write(0x12,ctrl|0x44))return false;
        uint8_t accel=0,gyro=0;
        return read(0x10,&accel,1)&&read(0x11,&gyro,1)&&(accel&0xfc)==0x40&&(gyro&0xf0)==0;
    }
    bite::ReadStatus sample(bite::Vec3& a){return bite::readAccel(*this,a);}
    uint8_t address()const{return address_;}
};
