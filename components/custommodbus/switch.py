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

CustomModbusSwitch = custommodbus_ns.class_("CustomModbusSwitch", switch.Switch)

CONF_REGISTER = "register"
CONF_BITMASK = "bitmask"
CONF_SLAVE_ID = "slave_id"

CONFIG_SCHEMA = switch.SWITCH_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(CustomModbusSwitch),
        cv.Required("custommodbus_id"): cv.use_id(CustomModbus),
        cv.Required(CONF_SLAVE_ID): cv.int_range(min=1, max=247),
        cv.Required(CONF_REGISTER): cv.hex_uint16_t,
        cv.Optional(CONF_BITMASK, default=0): cv.hex_uint16_t,
    }
)

async def to_code(config):
    parent = await cg.get_variable(config["custommodbus_id"])
    sw = cg.new_Pvariable(config[CONF_ID])

    await switch.register_switch(sw, config)

    cg.add(sw.set_parent(parent))
    cg.add(sw.set_slave_id(config[CONF_SLAVE_ID]))
    cg.add(sw.set_register(config[CONF_REGISTER]))
    cg.add(sw.set_bitmask(config[CONF_BITMASK]))

    cg.add(parent.add_switch(config[CONF_REGISTER], sw))
