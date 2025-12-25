#include "switch.h"
#include "custommodbus.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"

namespace esphome {
namespace custommodbus {

static const char *TAG = "custommodbus.switch";

void CustomModbusSwitch::write_state(bool state) {
  if (bitmask_ != 0) {
    parent_->write_bitmask(register_, bitmask_, state);
  } else {
    parent_->write_single(register_, state ? 1 : 0);
  }

  // Update HA immediately
  publish_state(state);
}

// *** DENNE LINJE ER AFGØRENDE ***
REGISTER_SWITCH(CustomModbusSwitch, switch_::Switch, "custommodbus");

}  // namespace custommodbus
}  // namespace esphome
