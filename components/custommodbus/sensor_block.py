import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor
from esphome.const import CONF_ID, CONF_NAME, CONF_UART_ID

# Namespace for C++ klasserne
custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbusBlock = custommodbus_ns.class_("CustomModbusBlock", cg.Component, uart.UARTDevice)

DataType = custommodbus_ns.enum("DataType")

# Platformnavn
PLATFORM = "custommodbus_block"

# Dependencies
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["custommodbus"]

# Konfigurationsfelter
CONF_SLAVE_ID = "slave_id"
CONF_START_ADDRESS = "start_address"
CONF_COUNT = "count"
CONF_OFFSET = "offset"
CONF_SCALE = "scale"
CONF_TYPE = "type"
CONF_UPDATE_INTERVAL = "update_interval"

DATA_TYPE_ENUM = cv.enum({
    "uint16": DataType.TYPE_UINT16,
    "int16": DataType.TYPE_INT16,
}, upper=False)

# Sensor-platform schema
CONFIG_SCHEMA = sensor.sensor_schema(platform=PLATFORM).extend({
    cv.GenerateID(): cv.declare_id(CustomModbusBlock),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Required(CONF_SLAVE_ID): cv.int_range(min=1, max=247),
    cv.Required(CONF_START_ADDRESS): cv.int_range(min=0, max=65535),
    cv.Required(CONF_COUNT): cv.int_range(min=1, max=125),
    cv.Required(CONF_OFFSET): cv.int_range(min=0, max=255),
    cv.Required(CONF_SCALE): cv.float_,
    cv.Required(CONF_TYPE): DATA_TYPE_ENUM,
    cv.Optional(CONF_UPDATE_INTERVAL, default="1s"): cv.positive_time_period_milliseconds,
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    uart_comp = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart(uart_comp))
    cg.add(var.set_slave_id(config[CONF_SLAVE_ID]))
    cg.add(var.set_start_address(config[CONF_START_ADDRESS]))
    cg.add(var.set_count(config[CONF_COUNT]))
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))

    sens = await sensor.new_sensor(config)
    cg.add(var.add_sensor(
        sens,
        config[CONF_OFFSET],
        config[CONF_TYPE],
        config[CONF_SCALE],
    ))


CODEOWNERS = ["@oz5afj"]
