#pragma once

#include "esphome/components/number/number.h"

namespace esphome {
namespace custommodbus {

class CustomModbus;

class CustomModbusNumber : public number::Number {
 public:
  void set_parent(CustomModbus *parent) { parent_ = parent; }
  void set_register(uint16_t reg) { register_ = reg; }
  void set_slave_id(uint8_t id) { slave_id_ = id; }
  void set_bitmask(uint16_t mask) { bitmask_ = mask; }

 protected:
  void control(float value) override;

  CustomModbus *parent_{nullptr};
  uint16_t register_{0};
  uint16_t bitmask_{0};
  uint8_t slave_id_{1};
};

}  // namespace custommodbus
}  // namespace esphome
