#pragma once

#include "esphome/components/switch/switch.h"

namespace esphome {
namespace custommodbus {

class CustomModbus;

class CustomModbusSwitch : public switch_::Switch {
 public:
  void set_parent(CustomModbus *parent) { parent_ = parent; }
  void set_register(uint16_t reg) { register_ = reg; }
  void set_bitmask(uint16_t mask) { bitmask_ = mask; }
  void set_slave_id(uint8_t id) { slave_id_ = id; }

  uint16_t get_bitmask() const { return bitmask_; }
  uint8_t get_slave_id() const { return slave_id_; }

  void write_state(bool state) override;

 protected:
  CustomModbus *parent_{nullptr};
  uint16_t register_{0};
  uint16_t bitmask_{0};
  uint8_t slave_id_{1};
};

}  // namespace custommodbus
}  // namespace esphome
