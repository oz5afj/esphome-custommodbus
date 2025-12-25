#include "switch.h"
#include "custommodbus.h"
#include "esphome/core/log.h"

namespace esphome {
namespace custommodbus {

static const char *TAG = "custommodbus.switch";

void CustomModbusSwitch::write_state(bool state) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "No parent assigned for switch component");
    return;
  }

  if (this->bitmask_ != 0) {
    this->parent_->write_bitmask(this->register_, this->bitmask_, state);
  } else {
    this->parent_->write_single(this->register_, state ? 1 : 0);
  }

  this->publish_state(state);
}

}  // namespace custommodbus
}  // namespace esphome
