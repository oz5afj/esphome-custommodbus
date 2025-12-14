import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, text_sensor
from esphome.const import CONF_ID

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema(CustomModbus)
    .extend(
        {
            cv.GenerateID(): cv.declare_id(CustomModbus),
            cv.Required("slave_id"): cv.int_range(min=1, max=247),
            cv.Required("register"): cv.hex_uint16_t,
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    ts = await text_sensor.new_text_sensor(config)

    cg.add(var.set_slave_id(config["slave_id"]))
    cg.add(var.add_text_sensor(config["register"], ts))
