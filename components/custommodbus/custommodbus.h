#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace custommodbus {

enum DataType {
  TYPE_UINT16 = 0,
  TYPE_INT16 = 1,
  TYPE_UINT32 = 2,
  TYPE_UINT32_R = 3,
};

struct ReadItem {
  uint16_t reg;
  uint8_t count;
  DataType type;
  float scale;
  esphome::sensor::Sensor *sensor;
  esphome::binary_sensor::BinarySensor *binary_sensor;
  uint16_t bitmask;
  esphome::text_sensor::TextSensor *text_sensor;
};

struct WriteItem {
  uint16_t reg;
  uint16_t value;
  uint16_t mask;
  bool use_mask;
};

class CustomModbus : public esphome::Component, public esphome::uart::UARTDevice {
 public:
  void set_slave_id(uint8_t id) { slave_id_ = id; }

  // READ
  void add_read_sensor(uint16_t reg, uint8_t count, DataType type, float scale,
                       esphome::sensor::Sensor *s);

  void add_binary_sensor(uint16_t reg, uint16_t mask,
                         esphome::binary_sensor::BinarySensor *bs);

  void add_text_sensor(uint16_t reg, esphome::text_sensor::TextSensor *ts);

  // WRITE
  void write_single(uint16_t reg, uint16_t value);
  void write_bitmask(uint16_t reg, uint16_t mask, bool state);

  void setup() override {}
  void loop() override;

 protected:
  uint8_t slave_id_{1};

  std::vector<ReadItem> reads_;
  std::vector<WriteItem> writes_;

  void process_reads();
  void process_writes();

  bool read_registers(uint16_t reg, uint8_t count, uint8_t *resp, uint8_t &resp_len);
  uint16_t crc16(uint8_t *buf, uint8_t len);
};

}  // namespace custommodbus
