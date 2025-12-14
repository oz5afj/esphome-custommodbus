import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import uart, sensor
from esphome.const import CONF_ID, CONF_UART_ID

sunmodbus_ns = cg.esphome_ns.namespace("sunmodbus")
SunModbus = sunmodbus_ns.class_("SunModbus", cg.PollingComponent, uart.UARTDevice)

DEPENDENCIES = ["uart"]

CONF_SLAVE_ID = "slave_id"
CONF_START_ADDRESS = "start_address"
CONF_UPDATE_INTERVAL = "update_interval"

CONF_REG0 = "reg0"
CONF_REG1 = "reg1"
CONF_REG2 = "reg2"
CONF_REG3 = "reg3"
CONF_REG4 = "reg4"
CONF_REG5 = "reg5"
CONF_REG6 = "reg6"
CONF_REG7 = "reg7"
CONF_REG8 = "reg8"
CONF_REG9 = "reg9"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SunModbus),

        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_SLAVE_ID): cv.int_range(min=1, max=247),
        cv.Required(CONF_START_ADDRESS): cv.int_range(min=0, max=65535),

        cv.Optional(CONF_UPDATE_INTERVAL, default="5s"): cv.positive_time_period_milliseconds,

        cv.Required(CONF_REG0): sensor.sensor_schema(),
        cv.Required(CONF_REG1): sensor.sensor_schema(),
        cv.Required(CONF_REG2): sensor.sensor_schema(),
        cv.Required(CONF_REG3): sensor.sensor_schema(),
        cv.Required(CONF_REG4): sensor.sensor_schema(),
        cv.Required(CONF_REG5): sensor.sensor_schema(),
        cv.Required(CONF_REG6): sensor.sensor_schema(),
        cv.Required(CONF_REG7): sensor.sensor_schema(),
        cv.Required(CONF_REG8): sensor.sensor_schema(),
        cv.Required(CONF_REG9): sensor.sensor_schema(),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))

    uart_comp = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart(uart_comp))

    cg.add(var.set_slave_id(config[CONF_SLAVE_ID]))
    cg.add(var.set_start_address(config[CONF_START_ADDRESS]))

    for idx, key in enumerate(
        [CONF_REG0, CONF_REG1, CONF_REG2, CONF_REG3, CONF_REG4,
         CONF_REG5, CONF_REG6, CONF_REG7, CONF_REG8, CONF_REG9]
    ):
        sens = await sensor.new_sensor(config[key])
        cg.add(var.set_sensor(idx, sens))
