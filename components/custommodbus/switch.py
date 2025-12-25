import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_ICON,
    CONF_ENTITY_CATEGORY,
    CONF_DEVICE_CLASS,
    CONF_DISABLED_BY_DEFAULT,
)

from . import custommodbus_ns, CustomModbus

# C++ klasse der arver fra esphome::switch_::Switch
CustomModbusSwitch = custommodbus_ns.class_("CustomModbusSwitch", switch.Switch)

CONF_REGISTER = "register"
CONF_BITMASK = "bitmask"
CONF_SLAVE_ID = "slave_id"

# Selvstændigt schema — undgår afhængighed af switch.SWITCH_SCHEMA
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CustomModbusSwitch),
        cv.Required("custommodbus_id"): cv.use_id(CustomModbus),
        cv.Required(CONF_SLAVE_ID): cv.int_range(min=1, max=247),
        cv.Required(CONF_REGISTER): cv.hex_uint16_t,
        cv.Optional(CONF_BITMASK, default=0): cv.hex_uint16_t,

        # Almindelige, valgfrie switch‑felter
        cv.Optional(CONF_NAME): cv.string,
        cv.Optional(CONF_ICON): cv.icon,
        cv.Optional(CONF_ENTITY_CATEGORY): cv.string,
        cv.Optional(CONF_DEVICE_CLASS): cv.string,
        cv.Optional(CONF_DISABLED_BY_DEFAULT, default=False): cv.boolean,
    }
)


async def to_code(config):
    # Hent parent (CustomModbus C++ objekt)
    parent = await cg.get_variable(config["custommodbus_id"])

    # Opret Python/C++ wrapper for switch entiteten
    sw = cg.new_Pvariable(config[CONF_ID])

    # Registrer switch hos ESPHome (ESPHome håndterer standardfelter internt)
    await switch.register_switch(sw, config)

    # Bind til din C++ parent
    cg.add(sw.set_parent(parent))
    cg.add(sw.set_slave_id(config[CONF_SLAVE_ID]))
    cg.add(sw.set_register(config[CONF_REGISTER]))
    cg.add(sw.set_bitmask(config[CONF_BITMASK]))

    # Registrer entiteten i din CustomModbus motor
    # Forudsætter at din C++ CustomModbus klasse implementerer:
    #   void add_switch(uint16_t reg, CustomModbusSwitch *sw);
    cg.add(parent.add_switch(config[CONF_REGISTER], sw))
