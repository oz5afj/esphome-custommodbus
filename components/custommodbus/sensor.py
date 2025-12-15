import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_REGISTER,
    CONF_COUNT,
    CONF_DATA_TYPE,
    CONF_SCALE,
)

# Namespace matcher C++: esphome::custommodbus
custommodbus_ns = cg.esphome_ns.namespace("custommodbus")

# C++-klasser
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, cg.UARTDevice)
DataType = custommodbus_ns.enum("DataType")

# Map mellem YAML string og C++ DataType enum
DATA_TYPE_MAP = {
    "uint16": DataType.TYPE_UINT16,
    "int16": DataType.TYPE_INT16,
    "uint32": DataType.TYPE_UINT32,
    "uint32_r": DataType.TYPE_UINT32_R,
}

# Konfigurationsskema for en custommodbus-sensor
CONFIG_SCHEMA = sensor.sensor_schema().extend(
    {
        cv.GenerateID(): cv.declare_id(sensor.Sensor),
        cv.GenerateID("custommodbus_id"): cv.use_id(CustomModbus),
        cv.Required(CONF_REGISTER): cv.uint16_t,
        cv.Optional(CONF_COUNT, default=1): cv.one_of(1, 2, int=True),
        cv.Optional(CONF_DATA_TYPE, default="uint16"): cv.one_of(
            "uint16", "int16", "uint32", "uint32_r", lower=True
        ),
        cv.Optional(CONF_SCALE, default=1.0): cv.float_,
    }
)

async def to_code(config):
    # Hent reference til CustomModbus-objektet (parent)
    parent = await cg.get_variable(config["custommodbus_id"])

    # Opret selve sensor-objektet
    sens = await sensor.new_sensor(config)

    # Find C++ enum-værdien til data_type
    data_type = DATA_TYPE_MAP[config[CONF_DATA_TYPE]]

    # Tilføj læsning til CustomModbus
    # Dette registrerer kun læsningen (ReadItem)
    # Den reelle Modbus-trafik sker inde i C++ i loop().
    cg.add(
        parent.add_read_sensor(
            config[CONF_REGISTER],
            config[CONF_COUNT],
            data_type,
            config[CONF_SCALE],
            sens,
        )
    )
