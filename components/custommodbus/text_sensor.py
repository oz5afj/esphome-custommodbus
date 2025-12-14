iimport esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, text_sensor
from esphome.const import CONF_ID

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

PLATFORM_SCHEMA = text_sensor.TEXT_SENSOR_PLATFORM_SCHEMA.extend(
    {
        cv.Required(CONF_ID): cv.use_id(CustomModbus),
        cv.Required("slave_id"): cv.int_range(min=1, max=247),
        cv.Required("register"): cv.hex_uint16_t,
    }
).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.get_variable(config[CONF_ID])
    await uart.register_uart_device(var, config)
    ts = await text_sensor.new_text_sensor(config)
    cg.add(var.set_slave_id(config["slave_id"]))
    cg.add(var.add_text_sensor(config["register"], ts))
