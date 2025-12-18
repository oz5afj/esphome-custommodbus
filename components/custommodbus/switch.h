#pragma once

#include "esphome/components/switch/switch.h"

namespace esphome {
namespace custommodbus {

// Forward declaration for correct include order
class CustomModbus;

class CustomModbusSwitch : public switch_::Switch {
 public:
  void set_parent(CustomModbus *parent) { parent_ = parent; }
  void set_register(uint16_t reg) { register_ = reg; }
  void set_bitmask(uint16_t mask) { bitmask_ = mask; }
  void set_slave_id(uint8_t id) { slave_id_ = id; }

 protected:
  void write_state(bool state) override;

  CustomModbus *parent_{nullptr};
  uint16_t register_{0};
  uint16_t bitmask_{0};
  uint8_t slave_id_{1};
};

}  // namespace custommodbus
}  // namespace esphome
