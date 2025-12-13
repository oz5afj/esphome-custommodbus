#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>

namespace custommodbus {

enum DataType {
  TYPE_UINT16 = 0,
  TYPE_INT16 = 1,
};

struct BlockSensorConfig {
  esphome::sensor::Sensor *sensor;
  uint8_t offset;
  DataType type;
  float scale;
};

class CustomModbusBlock : public esphome::Component, public esphome::uart::UARTDevice {
 public:
  void set_slave_id(uint8_t id) { slave_id_ = id; }
  void set_start_address(uint16_t addr) { start_address_ = addr; }
  void set_count(uint8_t c) { count_ = c; }
  void set_update_interval(uint32_t ms) { poll_interval_ = ms; }

  void add_sensor(esphome::sensor::Sensor *s, uint8_t offset, DataType type, float scale) {
    BlockSensorConfig cfg{s, offset, type, scale};
    sensors_.push_back(cfg);
  }

  void setup() override { last_poll_ = 0; }

  void loop() override;

 protected:
  uint8_t slave_id_{1};
  uint16_t start_address_{0};
  uint8_t count_{0};

  uint32_t last_poll_{0};
  uint32_t poll_interval_{1000};  // default 1s

  std::vector<BlockSensorConfig> sensors_;

  bool read_block_(std::vector<uint16_t> &regs);
  uint16_t crc16_(uint8_t *buf, uint8_t len);
};

}  // namespace custommodbus
