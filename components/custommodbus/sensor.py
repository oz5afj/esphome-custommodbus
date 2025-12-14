import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, uart
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_UNIT_OF_MEASUREMENT,
    CONF_ACCURACY_DECIMALS,
    CONF_UPDATE_INTERVAL,
)

DEPENDENCIES = ["uart"]

# Namespace for vores component
custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbusSensor = custommodbus_ns.class_(
    "CustomModbusSensor",
    cg.PollingComponent,
    sensor.Sensor,
)

# Konfigurationsnøgler
CONF_UART_ID = "uart_id"
CONF_SLAVE_ID = "slave_id"
CONF_REGISTER = "register"
CONF_SCALE = "scale"

# Schema for sensoren
CONFIG_SCHEMA = (
    sensor.sensor_schema(CustomModbusSensor)
    .extend(
        {
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Required(CONF_SLAVE_ID): cv.uint8_t,
            cv.Required(CONF_REGISTER): cv.uint16_t,
            cv.Optional(CONF_SCALE, default=1.0): cv.float_,
        }
    )
    .extend(cv.polling_component_schema("10s"))
)

async def to_code(config):
    # Opret sensoren
    var = await sensor.new_sensor(config)
    
    # Registrer componenten
    await cg.register_component(var, config)

    # Sæt parametre
    cg.add(var.set_uart_id(config[CONF_UART_ID]))
    cg.add(var.set_slave_id(config[CONF_SLAVE_ID]))
    cg.add(var.set_register(config[CONF_REGISTER]))
    cg.add(var.set_scale(config[CONF_SCALE]))

    # Sæt unit og decimaler hvis de er defineret
    if CONF_UNIT_OF_MEASUREMENT in config:
        cg.add(var.set_unit_of_measurement(config[CONF_UNIT_OF_MEASUREMENT]))
    if CONF_ACCURACY_DECIMALS in config:
        cg.add(var.set_accuracy_decimals(config[CONF_ACCURACY_DECIMALS]))
