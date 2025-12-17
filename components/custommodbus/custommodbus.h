#pragma once

#include "esphome.h"
#include "esphome/components/uart/uart.h"
#include <vector>
#include <string>

namespace esphome {
namespace custommodbus {

struct ReadDef {
  uint16_t reg;
  uint8_t count;
  esphome::Sensor *sensor;
  float smoothing_alpha;

  // runtime state
  float last_val;
  uint32_t last_ts;
  int consecutive_failures;
};

struct Group {
  std::string name;
  uint32_t interval_ms;
  uint32_t next_due_ms;
  size_t index;
  std::vector<ReadDef> reads;
  bool force_now;
};

class CustomModbus : public Component {
 public:
  void setup() override;
  void loop() override;

  void set_uart_parent(esphome::uart::UARTComponent *parent) { this->uart_parent_ = parent; }
  void set_slave_id(uint8_t id) { this->slave_id_ = id; }

  // Python bindings
  void add_group(const std::string &name, uint32_t interval_ms);
  void add_read_to_group(const std::string &group_name, uint16_t reg, uint8_t count,
                         esphome::Sensor *sensor, float smoothing_alpha);

  // services
  bool write_single_register(uint16_t reg, uint16_t value);
  void flush_group_now(const std::string &group_name);

 protected:
  void start_read(uint16_t reg, uint8_t count);
  void handle_read_state();
  void process_reads();
  bool validate_and_publish(ReadDef &rd, uint16_t raw_val);

  esphome::uart::UARTComponent *uart_parent_{nullptr};
  uint8_t slave_id_{1};

  std::vector<Group> groups_;

  // read state
  enum ReadState { IDLE = 0, WAITING = 1 };
  ReadState read_state_{IDLE};
  int read_group_idx_{-1};
  int read_index_in_group_{-1};
  uint16_t read_reg_{0};
  uint8_t read_count_{0};
  int read_expected_{0};
  int read_got_{0};
  uint8_t read_buf_[512];
  uint32_t read_start_ms_{0};
  uint32_t last_byte_ms_{0};
  uint32_t read_timeout_ms_{800};

  uint16_t modbus_crc16(const uint8_t *buf, size_t len);
};

}  // namespace custommodbus
}  // namespace esphome
