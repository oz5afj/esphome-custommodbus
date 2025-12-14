import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, binary_sensor
from esphome.const import CONF_ID

# Namespace for custommodbus
custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

# Schema for platformen
PLATFORM_SCHEMA = binary_sensor.BINARY_SENSOR_PLATFORM_SCHEMA.extend(
    {
        cv.Required(CONF_ID): cv.use_id(CustomModbus),  # refererer til eksisterende komponent
        cv.Required("slave_id"): cv.int_range(min=1, max=247),
        cv.Required("register"): cv.hex_uint16_t,
        cv.Required("bitmask"): cv.hex_uint16_t,
    }
).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    # Hent eksisterende CustomModbus komponent
    var = cg.get_variable(config[CONF_ID])

    # Registrer UART på komponenten
    await uart.register_uart_device(var, config)

    # Opret binary_sensor objekt
    bs = await binary_sensor.new_binary_sensor(config)

    # Sæt slave_id og tilføj sensor til CustomModbus
    cg.add(var.set_slave_id(config["slave_id"]))
    cg.add(var.add_binary_sensor(config["register"], config["bitmask"], bs))
