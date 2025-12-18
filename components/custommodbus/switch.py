import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, switch
from esphome.const import (
    CONF_DISABLED_BY_DEFAULT,
    CONF_ICON,
    CONF_ENTITY_CATEGORY,
    CONF_DEVICE_CLASS,
)

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

PLATFORM_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(switch.Switch),
        cv.Optional("name"): cv.string,
        cv.Required("custommodbus_id"): cv.use_id(CustomModbus),
        cv.Required("slave_id"): cv.int_range(min=1, max=247),
        cv.Required("register"): cv.hex_uint16_t,
        cv.Optional("bitmask"): cv.hex_uint16_t,
        cv.Optional(CONF_ICON): cv.icon,
        cv.Optional(CONF_ENTITY_CATEGORY): cv.string,
        cv.Optional(CONF_DEVICE_CLASS): cv.string,
        cv.Optional(CONF_DISABLED_BY_DEFAULT, default=False): cv.boolean,
    }
).extend(uart.UART_DEVICE_SCHEMA)

CONFIG_SCHEMA = PLATFORM_SCHEMA

async def to_code(config):
    parent = await cg.get_variable(config["custommodbus_id"])
    await uart.register_uart_device(parent, config)

    config.setdefault(CONF_DISABLED_BY_DEFAULT, False)

    sw = await switch.new_switch(config)

    cg.add(parent.set_slave_id(config["slave_id"]))

    if "bitmask" in config:
        cg.add(sw.add_on_turn_on(parent.write_bitmask(config["register"], config["bitmask"], True)))
        cg.add(sw.add_on_turn_off(parent.write_bitmask(config["register"], config["bitmask"], False)))
    else:
        cg.add(sw.add_on_turn_on(parent.write_single(config["register"], 1)))
        cg.add(sw.add_on_turn_off(parent.write_single(config["register"], 0)))
