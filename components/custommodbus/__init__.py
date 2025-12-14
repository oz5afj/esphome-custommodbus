import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID, CONF_NAME

# Navn på vores komponent
DOMAIN = "custommodbus"

# Klasse
custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component)

# Konfig-nøgle
CONF_UART_ID = "uart_id"
CONF_SLAVE_ID = "slave_id"
CONF_REGISTER = "register"
CONF_DATA_TYPE = "data_type"
CONF_SCALE = "scale"

# Schema for sensorer
SENSOR_SCHEMA = sensor.SENSOR_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(CustomModbus),
    cv.Required(CONF_UART_ID): cv.use_id(esphome.components.uart.UART),
    cv.Required(CONF_SLAVE_ID): cv.uint8_t,
    cv.Required(CONF_REGISTER): cv.uint16_t,
    cv.Optional(CONF_DATA_TYPE, default="uint16"): cv.string,
    cv.Optional(CONF_SCALE, default=1.0): cv.float_,
})

def validate_config(config):
    # Brug en unik id her, hvis du ønsker
    return config

CONFIG_SCHEMA = cv.Schema({
    cv.Required("sensor"): [SENSOR_SCHEMA],
})

async def to_code(config):
    for conf in config["sensor"]:
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)
