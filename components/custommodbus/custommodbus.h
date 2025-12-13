#pragma once
#include "esphome.h"

namespace custommodbus {

class CustomModbus : public esphome::Component, public esphome::UARTDevice {
 public:
  esphome::Sensor *sensor_ = new esphome::Sensor();

  CustomModbus(esphome::UARTComponent *parent);

  void set_name(const std::string &name) { name_ = name; }
  void setup() override;
  void loop() override;

 protected:
  std::string name_;
  uint16_t crc16(uint8_t *buf, uint8_t len);
};

}  // namespace custommodbus
