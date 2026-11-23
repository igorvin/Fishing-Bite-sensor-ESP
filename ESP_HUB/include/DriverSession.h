#pragma once
#include <memory>
#include <utility>

namespace hub {
// Own the display driver across initialization and reconnect attempts.
template<typename Driver> class DriverSession {
 std::unique_ptr<Driver> driver_;
 public:
 template<typename Factory,typename Initialise>
 bool begin(Factory factory,Initialise initialise){
  // GrayOLED::begin allocates a new bus object each time. Destroy the old
  // driver before retrying so its destructor releases that object and buffer.
  // A partly initialized candidate is also destroyed immediately on failure.
  driver_.reset();
  std::unique_ptr<Driver> candidate(factory());
  if(!candidate||!initialise(*candidate))return false;
  driver_=std::move(candidate);
  return true;
 }
 Driver* operator->(){return driver_.get();}
};
}
