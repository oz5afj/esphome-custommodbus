import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor
from esphome.const import CONF_ID

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

PLATFORM_SCHEMA = sensor.sensor_schema(CustomModbus).extend(
    {
        cv.Required(CONF_ID): cv.use_id(CustomModbus),
        cv.Required("slave_id"): cv.int_range(min=1, max=247),
        cv.Required("register"): cv.hex_uint16_t,
        cv.Optional("count", default=1): cv.int_range(min=1, max=2),
        cv.Optional("scale", default=1.0): cv.float_,
        cv.Optional("data_type", default="uint16"): cv.string,
    }
).extend(uart.UART_DEVICE_SCHEMA)

DATA_TYPE_MAP = {
    "uint16": 0,
    "int16": 1,
    "uint32": 2,
    "uint32_r": 3,
}

async def to_code(config):
    var = cg.get_variable(config[CONF_ID])
    await uart.register_uart_device(var, config)
    sens = await sensor.new_sensor(config)
    data_type = DATA_TYPE_MAP.get(config["data_type"], 0)
    cg.add(var.set_slave_id(config["slave_id"]))
    cg.add(var.add_read_sensor(config["register"], config["count"], data_type, config["scale"], sens))
