#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace sunmodbus {

class SunModbus : public PollingComponent, public uart::UARTDevice {
 public:
  SunModbus() : PollingComponent(1000) {}  // default 1s

  void setup() override;
  void update() override;

  void set_uart(uart::UARTComponent *uart) { this->parent_ = uart; }
  void set_slave_id(uint8_t id) { this->slave_id_ = id; }
  void set_start_address(uint16_t addr) { this->start_address_ = addr; }
  void set_count(uint16_t count) { this->count_ = count; }
  void set_update_interval(uint32_t ms) { this->update_interval_ = ms; }

  void set_offset(uint16_t offset) { this->offset_ = offset; }
  void set_type(DataType type) { this->type_ = type; }
  void set_scale(float scale) { this->scale_ = scale; }

 protected:
  uart::UARTComponent *parent_;
  sensor::Sensor *sensor_{nullptr};

  uint8_t slave_id_;
  uint16_t start_address_;
  uint16_t count_;
  uint16_t offset_;
  DataType type_;
  float scale_;
};

}  // namespace sunmodbus
}  // namespace esphome
