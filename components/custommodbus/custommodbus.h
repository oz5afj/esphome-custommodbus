#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace custommodbus {

class CustomModbus : public esphome::Component, public esphome::uart::UARTDevice {
 public:
  // Sensoren oprettes af Python-delen og injiceres via setter
  void set_sensor(esphome::sensor::Sensor *sensor) { sensor_ = sensor; }
  void set_name(const std::string &name) { name_ = name; }

  void setup() override;
  void loop() override;

 protected:
  std::string name_;
  esphome::sensor::Sensor *sensor_{nullptr};

  uint16_t crc16(uint8_t *buf, uint8_t len);
};

}  // namespace custommodbus
