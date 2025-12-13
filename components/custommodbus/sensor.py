import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor
from esphome import core

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CustomModbus),
            cv.Optional("name", default="Modbus Test Value"): cv.string,
            cv.Optional("sensor"): sensor.sensor_schema(unit_of_measurement=None),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config["id"])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if "name" in config:
        cg.add(var.set_name(config["name"]))

    if "sensor" in config:
        sens = await sensor.new_sensor(config["sensor"])
        cg.add(var.set_sensor(sens))

