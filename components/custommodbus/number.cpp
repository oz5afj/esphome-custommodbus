#include "number.h"
#include "custommodbus.h"
#include "esphome/core/log.h"

namespace esphome {
namespace custommodbus {

static const char *TAG = "custommodbus.number";

void CustomModbusNumber::control(float value) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "No parent assigned for number component");
    return;
  }

  uint16_t raw = static_cast<uint16_t>(value);

  if (this->bitmask_ != 0) {
    // Bitmask write (read-modify-write)
    this->parent_->write_bitmask(this->register_, this->bitmask_, raw != 0);
  } else {
    // Normal single register write
    this->parent_->write_single(this->register_, raw);
  }

  // Update HA immediately
  this->publish_state(value);
}

}  // namespace custommodbus
}  // namespace esphome
