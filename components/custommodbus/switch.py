import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, switch
from esphome.const import CONF_ID

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

PLATFORM_SCHEMA = switch.switch_schema().extend(
    {
        cv.GenerateID(): cv.declare_id(switch.Switch),
        cv.Required(CONF_ID): cv.use_id(CustomModbus),
        cv.Required("slave_id"): cv.int_range(min=1, max=247),
        cv.Required("register"): cv.hex_uint16_t,
        cv.Optional("bitmask"): cv.hex_uint16_t,
    }
).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])
    await uart.register_uart_device(parent, config)
    sw = await switch.new_switch(config)

    cg.add(parent.set_slave_id(config["slave_id"]))

    if "bitmask" in config:
        cg.add(sw.add_on_turn_on(parent.write_bitmask(config["register"], config["bitmask"], True)))
        cg.add(sw.add_on_turn_off(parent.write_bitmask(config["register"], config["bitmask"], False)))
    else:
        cg.add(sw.add_on_turn_on(parent.write_single(config["register"], 1)))
        cg.add(sw.add_on_turn_off(parent.write_single(config["register"], 0)))
