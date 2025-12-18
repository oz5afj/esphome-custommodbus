#include "switch.h"
#include "custommodbus.h"

namespace esphome {
namespace custommodbus {

void CustomModbusSwitch::write_state(bool state) {
  if (bitmask_ != 0) {
    parent_->write_bitmask(register_, bitmask_, state);
  } else {
    parent_->write_single(register_, state ? 1 : 0);
  }
  publish_state(state);
}

}  // namespace custommodbus
}  // namespace esphome
