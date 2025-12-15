import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor
from esphome.const import CONF_ID

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

PLATFORM_SCHEMA = sensor.sensor_schema().extend(
    {
        cv.Required("custommodbus_id"): cv.use_id(CustomModbus),
        cv.Required("slave_id"): cv.int_range(min=1, max=247),
        cv.Required("register"): cv.hex_uint16_t,
        cv.Optional("count", default=1): cv.int_range(min=1, max=2),
        cv.Optional("scale", default=1.0): cv.float_,
        cv.Optional("data_type", default="uint16"): cv.string,
    }
).extend(uart.UART_DEVICE_SCHEMA)

DATA_TYPE_MAP = {
    "uint16": "TYPE_UINT16",
    "int16": "TYPE_INT16",
    "uint32": "TYPE_UINT32",
    "uint32_r": "TYPE_UINT32_R",
}


async def to_code(config):
    parent = await cg.get_variable(config["custommodbus_id"])
    sens = await sensor.new_sensor(config)

    # Map string → enum symbol
    data_type = DATA_TYPE_MAP.get(config["data_type"], "TYPE_UINT16")

    cg.add(parent.add_read_sensor(
        config["register"],
        config["count"],
        custommodbus_ns.enum(data_type),
        config["scale"],
        sens
    ))


CONFIG_SCHEMA = PLATFORM_SCHEMA
