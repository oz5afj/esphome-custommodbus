import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, switch
from esphome.const import (
    CONF_DISABLED_BY_DEFAULT,
    CONF_RESTORE_MODE,
    CONF_ICON,
    CONF_ENTITY_CATEGORY,
    CONF_DEVICE_CLASS,
)

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

# Map string names to the SwitchRestoreMode enum values so cv.enum gets a dict
_RESTORE_MODE_MAPPING = {
    "RESTORE_DEFAULT": switch.SwitchRestoreMode.RESTORE_DEFAULT,
    "RESTORE_ALWAYS_OFF": switch.SwitchRestoreMode.RESTORE_ALWAYS_OFF,
    "RESTORE_ALWAYS_ON": switch.SwitchRestoreMode.RESTORE_ALWAYS_ON,
    # Tilføj flere hvis din ESPHome‑version har andre navne
}

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
        # Brug mapping så cv.enum får et dict og codegen kan generere en C++ enum konstant
        cv.Optional(CONF_RESTORE_MODE, default="RESTORE_DEFAULT"): cv.enum(_RESTORE_MODE_MAPPING),
    }
).extend(uart.UART_DEVICE_SCHEMA)

CONFIG_SCHEMA = PLATFORM_SCHEMA


async def to_code(config):
    parent = await cg.get_variable(config["custommodbus_id"])
    await uart.register_uart_device(parent, config)

    # Sikre defaults så setup_switch_core_ ikke kaster KeyError
    config.setdefault(CONF_DISABLED_BY_DEFAULT, False)
    # config[CONF_RESTORE_MODE] er allerede mappet til enum‑værdi af cv.enum

    sw = await switch.new_switch(config)

    cg.add(parent.set_slave_id(config["slave_id"]))

    if "bitmask" in config:
        cg.add(sw.add_on_turn_on(parent.write_bitmask(config["register"], config["bitmask"], True)))
        cg.add(sw.add_on_turn_off(parent.write_bitmask(config["register"], config["bitmask"], False)))
    else:
        cg.add(sw.add_on_turn_on(parent.write_single(config["register"], 1)))
        cg.add(sw.add_on_turn_off(parent.write_single(config["register"], 0)))
