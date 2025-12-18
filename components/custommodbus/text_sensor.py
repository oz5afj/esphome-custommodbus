import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, text_sensor
from esphome.const import CONF_ID

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

PLATFORM_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(text_sensor.TextSensor),
        cv.Optional("name"): cv.string,
        cv.Required("custommodbus_id"): cv.use_id(CustomModbus),
        cv.Required("slave_id"): cv.int_range(min=1, max=247),
        cv.Required("register"): cv.hex_uint16_t,
        cv.Optional("data_type"): cv.string,
        cv.Optional("raw_encode"): cv.string,
        cv.Optional("icon"): cv.icon,
    }
).extend(uart.UART_DEVICE_SCHEMA)

CONFIG_SCHEMA = PLATFORM_SCHEMA


async def to_code(config):
    parent = await cg.get_variable(config["custommodbus_id"])
    await uart.register_uart_device(parent, config)
    ts = await text_sensor.new_text_sensor(config)

    cg.add(parent.set_slave_id(config["slave_id"]))
    cg.add(parent.add_text_sensor(
        config["register"],
        ts
    ))
