#include "number.h"
#include "custommodbus.h"

namespace esphome {
namespace custommodbus {

void CustomModbusNumber::control(float value) {
  uint16_t val = static_cast<uint16_t>(value);

  if (bitmask_ != 0) {
    parent_->write_bitmask(register_, bitmask_, val);
  } else {
    parent_->write_single(register_, val);
  }

  publish_state(value);
}

}  // namespace custommodbus
}  // namespace esphome
