import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_NAME,
    CONF_UNIT_OF_MEASUREMENT,
    CONF_ACCURACY_DECIMALS,
)

from . import CustomModbus

DEPENDENCIES = ["uart"]

# Dette er en simpel sensor-klasse
custommodbus_sensor_ns = cg.esphome_ns.namespace("custommodbus_sensor")
CustomModbusSensor = custommodbus_sensor_ns.class_("CustomModbusSensor", cg.PollingComponent, cg.Sensor)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(CustomModbusSensor),
    cv.Required("uart_id"): cv.use_id(esphome.components.uart.UART),
    cv.Required("slave_id"): cv.int_,
    cv.Required("register"): cv.int_,
    cv.Optional(CONF_UNIT_OF_MEASUREMENT): cv.string,
    cv.Optional(CONF_ACCURACY_DECIMALS): cv.int_,
}).extend(cg.POLLING_COMPONENT_SCHEMA)

async def to_code(config):
    sensor = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(sensor, config)
    await cg.add(sensor.set_uart_id(config["uart_id"]))
    await cg.add(sensor.set_slave_id(config["slave_id"]))
    await cg.add(sensor.set_register(config["register"]))

    if CONF_UNIT_OF_MEASUREMENT in config:
        cg.add(sensor.set_unit_of_measurement(config[CONF_UNIT_OF_MEASUREMENT]))
    if CONF_ACCURACY_DECIMALS in config:
        cg.add(sensor.set_accuracy_decimals(config[CONF_ACCURACY_DECIMALS]))
