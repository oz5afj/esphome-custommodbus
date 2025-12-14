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

REG_KEYS = [
    "reg0","reg1","reg2","reg3","reg4",
    "reg5","reg6","reg7","reg8","reg9"
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SunModbus),

        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_SLAVE_ID): cv.int_range(min=1, max=247),
        cv.Required(CONF_START_ADDRESS): cv.int_range(min=0, max=65535),

        cv.Optional(CONF_UPDATE_INTERVAL, default="5s"): cv.positive_time_period_milliseconds,

        **{cv.Required(k): sensor.sensor_schema() for k in REG_KEYS},
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

    for idx, key in enumerate(REG_KEYS):
        sens = await sensor.new_sensor(config[key])
        cg.add(var.set_sensor(idx, sens))
