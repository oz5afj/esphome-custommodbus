import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")

CustomModbusBlock = custommodbus_ns.class_("CustomModbusBlock", cg.Component, uart.UARTDevice)

DataType = custommodbus_ns.enum("DataType")

CONF_SLAVE_ID = "slave_id"
CONF_START_ADDRESS = "start_address"
CONF_COUNT = "count"
CONF_SENSORS = "sensors"
CONF_OFFSET = "offset"
CONF_SCALE = "scale"
CONF_TYPE = "type"
CONF_UPDATE_INTERVAL = "update_interval"

DATA_TYPE_ENUM = cv.enum({
    "uint16": DataType.TYPE_UINT16,
    "int16": DataType.TYPE_INT16,
}, upper=False)


BLOCK_SENSOR_SCHEMA = sensor.sensor_schema().extend({
    cv.Required(CONF_OFFSET): cv.int_range(min=0, max=255),
    cv.Required(CONF_SCALE): cv.float_,
    cv.Required(CONF_TYPE): DATA_TYPE_ENUM,
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(CustomModbusBlock),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Required(CONF_SLAVE_ID): cv.int_range(min=1, max=247),
    cv.Required(CONF_START_ADDRESS): cv.int_range(min=0, max=65535),
    cv.Required(CONF_COUNT): cv.int_range(min=1, max=125),
    cv.Optional(CONF_UPDATE_INTERVAL, default="1s"): cv.positive_time_period_milliseconds,
    cv.Required(CONF_SENSORS): cv.ensure_list(BLOCK_SENSOR_SCHEMA),
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

    for s_conf in config[CONF_SENSORS]:
        s = await sensor.new_sensor(s_conf)
        cg.add(var.add_sensor(
            s,
            s_conf[CONF_OFFSET],
            s_conf[CONF_TYPE],
            s_conf[CONF_SCALE],
        ))
