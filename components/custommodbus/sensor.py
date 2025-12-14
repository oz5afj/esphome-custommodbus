import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor
from esphome.const import CONF_ID

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

DATA_TYPES = {
    "uint16": cg.uint16,
    "int16": cg.int16,
}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(CustomModbus),

            cv.Required("slave_id"): cv.int_range(min=1, max=247),
            cv.Required("register"): cv.hex_uint16_t,
            cv.Optional("count", default=1): cv.int_range(min=1, max=2),
            cv.Optional("data_type", default="uint16"): cv.one_of(*DATA_TYPES.keys(), lower=True),
            cv.Optional("scale", default=1.0): cv.float_,

            cv.Required("sensor"): sensor.sensor_schema(),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_slave_id(config["slave_id"]))
    cg.add(var.set_register(config["register"]))
    cg.add(var.set_count(config["count"]))
    cg.add(var.set_data_type(DATA_TYPES[config["data_type"]]))
    cg.add(var.set_scale(config["scale"]))

    sens = await sensor.new_sensor(config["sensor"])
    cg.add(var.set_sensor(sens))
