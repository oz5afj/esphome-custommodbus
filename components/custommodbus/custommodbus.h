#pragma once
#include "esphome.h"

namespace esphome {
namespace custommodbus {

class CustomModbus : public Component, public UARTDevice {
 public:
  void set_uart_parent(UARTComponent *parent) { this->uart_parent_ = parent; }
  void set_slave_id(uint8_t id) { this->slave_id_ = id; }

  void add_read_sensor(uint16_t reg, uint8_t count, uint8_t dtype, float scale, esphome::sensor::Sensor *sensor);

 protected:
  UARTComponent *uart_parent_{nullptr};
  uint8_t slave_id_{0};
};

}  // namespace custommodbus
}  // namespace esphome
