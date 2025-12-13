#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace sunmodbus {

enum DataType {
  TYPE_UINT16,
  TYPE_INT16,
};

class SunModbus : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void update() override;

  void set_uart(uart::UARTComponent *uart) { this->uart_ = uart; }
  void set_slave_id(uint8_t id) { this->slave_id_ = id; }
  void set_start_address(uint16_t addr) { this->start_address_ = addr; }
  void set_count(uint16_t c) { this->count_ = c; }
  void set_update_interval(uint32_t ms) { this->update_interval_ = ms; }
  void set_offset(uint16_t offset) { offset_ = offset; }
  void set_type(DataType type) { type_ = type; }
  void set_scale(float scale) { scale_ = scale; }

  void add_sensor(sensor::Sensor *sens, uint16_t offset, DataType type, float scale);

 protected:
  bool read_registers(uint16_t start, uint16_t count, uint16_t *buffer);

  uart::UARTComponent *uart_;
  sensor::Sensor *sensor_;
  uint8_t slave_id_;
  uint16_t start_address_;
  uint16_t count_;
  uint16_t offset_;
  DataType type_;
  float scale_;
  uint32_t update_interval_;
};

}  // namespace sunmodbus
}  // namespace esphome

