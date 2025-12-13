#include "sunmodbus.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sunmodbus {

static const char *const TAG = "sunmodbus";

void SunModbus::setup() {
  ESP_LOGI(TAG, "SunModbus setup");
}

void SunModbus::update() {
  uint16_t buffer[125];

  // Læs Modbus-registre
  if (!this->read_registers(this->start_address_, this->count_, buffer)) {
    ESP_LOGW(TAG, "Failed to read registers");
    return;
  }

  // Hent rå værdi
  uint16_t raw = buffer[this->offset_];

  // Skalering
  float value = raw * this->scale_;

  // Publicér til ESPHome
  if (this->sensor_ != nullptr) {
    this->sensor_->publish_state(value);
  }
}

bool SunModbus::read_registers(uint16_t start, uint16_t count, uint16_t *buffer) {
  // Dummy-implementering — du kan senere erstatte med rigtig Modbus-læsning
  for (int i = 0; i < count; i++) {
    buffer[i] = 100 + i;
  }
  return true;
}

}  // namespace sunmodbus
}  // namespace esphome
