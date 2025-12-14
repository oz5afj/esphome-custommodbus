import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, number
from esphome.const import CONF_ID

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

PLATFORM_SCHEMA = number.number_schema().extend(
    {
        cv.GenerateID(): cv.declare_id(number.Number),
        cv.Required(CONF_ID): cv.use_id(CustomModbus),
        cv.Required("slave_id"): cv.int_range(min=1, max=247),
        cv.Required("register"): cv.hex_uint16_t,
    }
).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])
    await uart.register_uart_device(parent, config)
    num = await number.new_number(config)

    cg.add(parent.set_slave_id(config["slave_id"]))
    cg.add(num.add_on_state_callback(
        parent.write_single(config["register"], cg.float_(num))
    ))
