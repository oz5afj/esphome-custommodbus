import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, switch
from esphome.const import CONF_ID

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

# Robust schema defineret direkte med cv.Schema
PLATFORM_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(switch.Switch),
        cv.Optional("name"): cv.string,
        cv.Required(CONF_ID): cv.use_id(CustomModbus),
        cv.Required("slave_id"): cv.int_range(min=1, max=247),
        cv.Required("register"): cv.hex_uint16_t,
        cv.Optional("bitmask"): cv.hex_uint16_t,
        cv.Optional("icon"): cv.icon,
        cv.Optional("entity_category"): cv.string,
    }
).extend(uart.UART_DEVICE_SCHEMA)

# ESPHome loader forventer CONFIG_SCHEMA
CONFIG_SCHEMA = PLATFORM_SCHEMA


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])
    await uart.register_uart_device(parent, config)

    # Opret switch objektet (navn/icon håndteres af switch.new_switch)
    sw = await switch.new_switch(config)

    cg.add(parent.set_slave_id(config["slave_id"]))

    if "bitmask" in config:
        # Tilknyt on/off til bitmask skriv
        cg.add(sw.add_on_turn_on(parent.write_bitmask(config["register"], config["bitmask"], True)))
        cg.add(sw.add_on_turn_off(parent.write_bitmask(config["register"], config["bitmask"], False)))
    else:
        # Tilknyt on/off til simple register skriv
        cg.add(sw.add_on_turn_on(parent.write_single(config["register"], 1)))
        cg.add(sw.add_on_turn_off(parent.write_single(config["register"], 0)))
