#include "sunmodbus.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sunmodbus {

static const char *const TAG = "sunmodbus";

void SunModbus::setup() {
  ESP_LOGI(TAG, "SunModbus setup");
}

void SunModbus::loop() {
  // Nothing here yet
}

void SunModbus::add_sensor(sensor::Sensor *sens, uint16_t offset, DataType type, float scale) {
  this->sensor_ = sens;
  this->offset_ = offset;
  this->type_ = type;
  this->scale_ = scale;
}

void SunModbus::update() {
  uint16_t buffer[125];
  if (!this->read_registers(this->start_address_, this->count_, buffer)) {
    ESP_LOGW(TAG, "Failed to read registers");
    return;
  }

  uint16_t raw = buffer[this->offset_];
  float value = raw * this->scale_;
  this->sensor_->publish_state(value);
}

bool SunModbus::read_registers(uint16_t start, uint16_t count, uint16_t *buffer) {
  // Dummy implementation for now
  for (int i = 0; i < count; i++) buffer[i] = 100 + i;
  return true;
}

}  // namespace sunmodbus
}  // namespace esphome
