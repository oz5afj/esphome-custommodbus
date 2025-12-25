#include "number.h"
#include "custommodbus.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"

namespace esphome {
namespace custommodbus {

static const char *TAG = "custommodbus.number";

void CustomModbusNumber::control(float value) {
  uint16_t val = static_cast<uint16_t>(value);

  if (bitmask_ != 0) {
    parent_->write_bitmask(register_, bitmask_, val != 0);
  } else {
    parent_->write_single(register_, val);
  }

  publish_state(value);
}

// *** KORREKT REGISTRERING ***
REGISTER_NUMBER(CustomModbusNumber, number::Number, "custommodbus");

}  // namespace custommodbus
}  // namespace esphome
