#include "switch.h"
#include "custommodbus.h"
#include "esphome/core/log.h"

namespace esphome {
namespace custommodbus {

static const char *TAG = "custommodbus.switch";

void CustomModbusSwitch::write_state(bool state) {
  if (!parent_) {
    ESP_LOGW(TAG, "No parent assigned");
    return;
  }

  if (bitmask_ != 0) {
    parent_->write_bitmask(register_, bitmask_, state);
  } else {
    parent_->write_single(register_, state ? 1 : 0);
  }

  publish_state(state);
}

}  // namespace custommodbus
}  // namespace esphome
