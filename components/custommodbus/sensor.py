#
#  CustomModbus – Sensor platform
#
#  Denne fil definerer YAML‑integration for:
#
#      sensor:
#        - platform: custommodbus
#          register: 0
#          count: 1
#          data_type: uint16
#          scale: 0.1
#
#  ESPHome 2025 fjernede flere konstanter fra esphome.const,
#  så vi definerer dem selv her.
#

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor

# ---------------------------------------------------------------------------
#  DEFINÉR EGNE KONSTANTER (ESPHome har fjernet dem fra esphome.const)
# ---------------------------------------------------------------------------
CONF_REGISTER = "register"
CONF_COUNT = "count"
CONF_DATA_TYPE = "data_type"
CONF_SCALE = "scale"

# ---------------------------------------------------------------------------
#  C++ namespace og klasser
# ---------------------------------------------------------------------------
custommodbus_ns = cg.esphome_ns.namespace("custommodbus")

CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, cg.UARTDevice)
DataType = custommodbus_ns.enum("DataType")

# ---------------------------------------------------------------------------
#  Mapping fra YAML string → C++ enum
# ---------------------------------------------------------------------------
DATA_TYPE_MAP = {
    "uint16": DataType.TYPE_UINT16,
    "int16": DataType.TYPE_INT16,
    "uint32": DataType.TYPE_UINT32,
    "uint32_r": DataType.TYPE_UINT32_R,
}

# ---------------------------------------------------------------------------
#  YAML‑schema for en custommodbus sensor
# ---------------------------------------------------------------------------
CONFIG_SCHEMA = sensor.sensor_schema().extend(
    {
        cv.GenerateID(): cv.declare_id(sensor.Sensor),

        # Reference til CustomModbus‑instansen
        cv.GenerateID("custommodbus_id"): cv.use_id(CustomModbus),

        # Modbus‑registeradresse
        cv.Required(CONF_REGISTER): cv.uint16_t,

        # Antal registre (1 = 16‑bit, 2 = 32‑bit)
        cv.Optional(CONF_COUNT, default=1): cv.one_of(1, 2, int=True),

        # Datatype (fortolkning)
        cv.Optional(CONF_DATA_TYPE, default="uint16"): cv.one_of(
            "uint16", "int16", "uint32", "uint32_r", lower=True
        ),

        # Skalering (fx 0.1 for spænding)
        cv.Optional(CONF_SCALE, default=1.0): cv.float_,
    }
)

# ---------------------------------------------------------------------------
#  GENERÉR C++ KODE
# ---------------------------------------------------------------------------
async def to_code(config):
    # Hent C++ objektet for CustomModbus
    parent = await cg.get_variable(config["custommodbus_id"])

    # Opret ESPHome sensor‑objektet
    sens = await sensor.new_sensor(config)

    # Find C++ enum for datatype
    data_type = DATA_TYPE_MAP[config[CONF_DATA_TYPE]]

    # Registrér læsningen i CustomModbus
    # (ingen Modbus‑trafik sker her – kun registrering)
    cg.add(
        parent.add_read_sensor(
            config[CONF_REGISTER],
            config[CONF_COUNT],
            data_type,
            config[CONF_SCALE],
            sens,
        )
    )
