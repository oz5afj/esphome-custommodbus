import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

CONF_REGISTER = "register"
CONF_COUNT = "count"
CONF_DATA_TYPE = "data_type"
CONF_SCALE = "scale"
CONF_CUSTOMMODBUS_ID = "custommodbus_id"

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component)

DATA_TYPE_MAP = {
    "uint16": 0,
    "int16": 1,
    "uint32": 2,
    "uint32_r": 3,
}

CONFIG_SCHEMA = sensor.sensor_schema().extend(
    {
        cv.GenerateID(): cv.declare_id(sensor.Sensor),
        cv.GenerateID(CONF_CUSTOMMODBUS_ID): cv.use_id(CustomModbus),

        cv.Required(CONF_REGISTER): cv.uint16_t,
        cv.Optional(CONF_COUNT, default=1): cv.one_of(1, 2, int=True),
        cv.Optional(CONF_DATA_TYPE, default="uint16"): cv.one_of(
            "uint16", "int16", "uint32", "uint32_r", lower=True
        ),
        cv.Optional(CONF_SCALE, default=1.0): cv.float_,
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_CUSTOMMODBUS_ID])
    sens = await sensor.new_sensor(config)

    data_type = DATA_TYPE_MAP[config[CONF_DATA_TYPE]]

    cg.add(parent.add_read_sensor(
        config[CONF_REGISTER],
        config[CONF_COUNT],
        data_type,
        config[CONF_SCALE],
        sens
    ))
