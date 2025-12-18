import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, number
from esphome.const import CONF_ID

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

# Robust schema defineret direkte med cv.Schema
PLATFORM_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(number.Number),
        cv.Optional("name"): cv.string,
        cv.Required(CONF_ID): cv.use_id(CustomModbus),
        cv.Required("slave_id"): cv.int_range(min=1, max=247),
        cv.Required("register"): cv.hex_uint16_t,
        cv.Optional("min_value"): cv.float_,
        cv.Optional("max_value"): cv.float_,
        cv.Optional("step"): cv.float_,
        cv.Optional("unit_of_measurement"): cv.string,
    }
).extend(uart.UART_DEVICE_SCHEMA)

# ESPHome loader forventer CONFIG_SCHEMA
CONFIG_SCHEMA = PLATFORM_SCHEMA


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])
    await uart.register_uart_device(parent, config)
    num = await number.new_number(config)

    cg.add(parent.set_slave_id(config["slave_id"]))

    # Tilknyt callback: skriv register ved ændring af talværdien.
    # Bemærk: parent.write_single forventer et tal; cg.float_(num) bruges som wrapper til codegen.
    cg.add(num.add_on_state_callback(
        parent.write_single(config["register"], cg.float_(num))
    ))
