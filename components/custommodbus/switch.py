import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, switch
from esphome.components.switch import SwitchRestoreMode, restore_mode
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_ICON,
    CONF_ENTITY_CATEGORY,
    CONF_DEVICE_CLASS,
    CONF_RESTORE_MODE,
    CONF_DISABLED_BY_DEFAULT,
)

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)
CustomModbusSwitch = custommodbus_ns.class_("CustomModbusSwitch", switch.Switch)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CustomModbusSwitch),
        cv.Required("custommodbus_id"): cv.use_id(CustomModbus),
        cv.Required("slave_id"): cv.int_range(min=1, max=247),
        cv.Required("register"): cv.hex_uint16_t,
        cv.Optional("bitmask", default=0): cv.hex_uint16_t,

        cv.Optional(CONF_NAME): cv.string,
        cv.Optional(CONF_ICON): cv.icon,
        cv.Optional(CONF_ENTITY_CATEGORY): cv.string,
        cv.Optional(CONF_DEVICE_CLASS): cv.string,
        cv.Optional(CONF_DISABLED_BY_DEFAULT, default=False): cv.boolean,

        # Brug ESPHome's egen restore_mode-validator og enum-type
        cv.Optional(
            CONF_RESTORE_MODE,
            default=SwitchRestoreMode.SWITCH_RESTORE_DEFAULT_OFF,
        ): restore_mode,
    }
).extend(uart.UART_DEVICE_SCHEMA)


async def to_code(config):
    parent = await cg.get_variable(config["custommodbus_id"])
    await uart.register_uart_device(parent, config)

    sw = cg.new_Pvariable(config[CONF_ID])

    # Nu får register_switch en rigtig enum, ikke en streng
    await switch.register_switch(sw, config)

    cg.add(sw.set_parent(parent))
    cg.add(sw.set_slave_id(config["slave_id"]))
    cg.add(sw.set_register(config["register"]))
    cg.add(sw.set_bitmask(config["bitmask"]))
