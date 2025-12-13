#include "sunmodbus.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sunmodbus {

static const char *const TAG = "sunmodbus";

void SunModbus::setup() {
  ESP_LOGI(TAG, "SunModbus setup: slave_id=%u start_address=%u count=%u offset=%u scale=%f type=%d",
           this->slave_id_, this->start_address_, this->count_, this->offset_, this->scale_, this->type_);
}

void SunModbus::update() {
  if (this->uart_ == nullptr || this->sensor_ == nullptr) {
    ESP_LOGW(TAG, "UART or sensor not set");
    return;
  }

  uint8_t buffer[256] = {0};

  if (!this->read_holding_registers_(this->slave_id_, this->start_address_, this->count_, buffer, sizeof(buffer))) {
    ESP_LOGW(TAG, "Failed to read holding registers");
    return;
  }

  uint16_t raw = (buffer[0] << 8) | buffer[1];
  float value = 0.0f;

  if (this->type_ == TYPE_UINT16) {
    value = static_cast<uint16_t>(raw);
  } else {
    int16_t s = static_cast<int16_t>(raw);
    value = static_cast<float>(s);
  }

  value = (value + this->offset_) * this->scale_;
  this->sensor_->publish_state(value);
}

bool SunModbus::read_holding_registers_(uint8_t slave, uint16_t start, uint16_t count, uint8_t *buffer, uint16_t len) {
  (void) slave;
  (void) start;
  (void) count;
  (void) buffer;
  (void) len;
  // Stub - implementer rigtig Modbus senere. For nu returnerer vi false for at undgå crash.
  ESP_LOGW(TAG, "read_holding_registers_ stub called - not implemented");
  return false;
}

}  // namespace sunmodbus
}  // namespace esphome
