import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, select
from esphome.const import CONF_ID

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

PLATFORM_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(select.Select),
        cv.Optional("name"): cv.string,
        cv.Required("custommodbus_id"): cv.use_id(CustomModbus),
        cv.Required("slave_id"): cv.int_range(min=1, max=247),
        cv.Required("register"): cv.hex_uint16_t,
        cv.Required("options"): cv.Schema({cv.string: cv.positive_int}),
        cv.Optional("icon"): cv.icon,
    }
).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

CONFIG_SCHEMA = PLATFORM_SCHEMA


async def to_code(config):
    parent = await cg.get_variable(config["custommodbus_id"])
    await uart.register_uart_device(parent, config)
    sel = await select.new_select(config)

    cg.add(parent.set_slave_id(config["slave_id"]))

    for name, value in config["options"].items():
        cg.add(sel.add_option(name, value))

    cg.add(sel.add_on_state_callback(
        parent.write_single(config["register"], cg.uint16_t(sel))
    ))
