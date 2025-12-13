#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace custommodbus {

enum DataType {
  TYPE_UINT16 = 0,
  TYPE_INT16 = 1,
};

class CustomModbus : public esphome::Component, public esphome::uart::UARTDevice {
 public:
  void set_slave_id(uint8_t id) { slave_id_ = id; }
  void set_register(uint16_t reg) { register_ = reg; }
  void set_count(uint8_t c) { count_ = c; }

  // Accept int from ESPHome and cast to enum
  void set_data_type(int t) { data_type_ = static_cast<DataType>(t); }

  void set_scale(float s) { scale_ = s; }
  void set_sensor(esphome::sensor::Sensor *s) { sensor_ = s; }

  void setup() override {}
  void loop() override;

 protected:
  uint8_t slave_id_{1};
  uint16_t register_{0};
  uint8_t count_{1};
  DataType data_type_{TYPE_UINT16};
  float scale_{1.0f};
  esphome::sensor::Sensor *sensor_{nullptr};

  uint16_t crc16(uint8_t *buf, uint8_t len);
};

}  // namespace custommodbus
