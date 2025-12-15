#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace custommodbus {

using esphome::sensor::Sensor;
using esphome::binary_sensor::BinarySensor;
using esphome::text_sensor::TextSensor;

//
// ============================================================================
//  Datatyper for Modbus læsninger
// ============================================================================
//  Disse bruges til at beskrive hvordan rå Modbus-data skal fortolkes.
// ============================================================================
enum DataType {
  TYPE_UINT16,
  TYPE_INT16,
  TYPE_UINT32,
  TYPE_UINT32_R,  // 32-bit, reverse word order
};

//
// ============================================================================
//  ReadItem
// ============================================================================
//  En konfigureret "read operation":
//    - reg: Modbus-registeradresse
//    - count: antal 16-bit registre (1 eller 2)
//    - type: hvordan data skal fortolkes (DataType)
//    - scale: skaleringsfaktor til float-sensorer
//    - sensor: pointer til float-sensor (valgfri)
//    - binary_sensor: pointer til binær sensor (valgfri)
//    - text_sensor: pointer til tekstsensor (valgfri)
//    - bitmask: bitmaske til binary_sensor
// ============================================================================
struct ReadItem {
  uint16_t reg;
  uint8_t count;
  DataType type;
  float scale;
  Sensor *sensor;
  BinarySensor *binary_sensor;
  TextSensor *text_sensor;
  uint16_t bitmask;
};

//
// ============================================================================
//  WriteItem
// ============================================================================
//  En pending write-operation:
//    - reg: registeradresse
//    - value: værdi til at skrive
//    - mask: bitmaske (hvis use_mask = true)
//    - use_mask: hvis true, er denne write en "bitmask write"
// ============================================================================
struct WriteItem {
  uint16_t reg;
  uint16_t value;
  uint16_t mask;
  bool use_mask;
};

//
// ============================================================================
//  CustomModbus
// ============================================================================
//  Hovedklassen for Modbus-driveren. Den:
//    - Arver fra uart::UARTDevice for at bruge ESPHome UART
//    - Arver fra Component for at integrere i ESPHome-livscyklus
//
//  Den håndterer selv:
//    - kø af reads_ (ReadItem)
//    - kø af writes_ (WriteItem)
//    - polling i loop() (kun lidt arbejde per iteration)
// ============================================================================
class CustomModbus : public uart::UARTDevice, public Component {
 public:
  // Konfiguration af slave-ID (settes fra Python/YAML)
  void set_slave_id(uint8_t id) { this->slave_id_ = id; }

  // ESPHome livscyklus
  void setup() override;  // kaldt én gang ved boot
  void loop() override;   // kaldt gentagne gange

  // Registrering af læsninger (kaldt fra Python-platform under konfiguration)
  void add_read_sensor(uint16_t reg, uint8_t count, DataType type,
                       float scale, Sensor *s);
  void add_binary_sensor(uint16_t reg, uint16_t mask, BinarySensor *bs);
  void add_text_sensor(uint16_t reg, TextSensor *ts);

  // Queue write-operationer (kaldt fra fx custommodbus-switch/number)
  void write_single(uint16_t reg, uint16_t value);
  void write_bitmask(uint16_t reg, uint16_t mask, bool state);

 protected:
  // Intern polling af reads og writes (kaldt fra loop())
  void process_reads();
  void process_writes();

  // Lav én Modbus "Read Holding Registers" forespørgsel
  // Non-blocking style: venter max 200 ms, med delay(1) inde i loop.
  bool read_registers(uint16_t reg, uint8_t count, uint8_t *resp, uint8_t &resp_len);

  // CRC16 for Modbus-rammer
  uint16_t crc16(uint8_t *buf, uint8_t len);

  // Konfigureret slave-id
  uint8_t slave_id_{1};

  // Liste over alle læsninger og writes
  std::vector<ReadItem> reads_;
  std::vector<WriteItem> writes_;
};

}  // namespace custommodbus
}  // namespace esphome
