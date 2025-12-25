import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_ID

from . import custommodbus_ns, CustomModbus

CustomModbusNumber = custommodbus_ns.class_("CustomModbusNumber", number.Number)

CONF_REGISTER = "register"
CONF_BITMASK = "bitmask"
CONF_SLAVE_ID = "slave_id"

CONFIG_SCHEMA = number.NUMBER_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(CustomModbusNumber),
        cv.Required(CONF_REGISTER): cv.uint16_t,
        cv.Optional(CONF_BITMASK, default=0): cv.uint16_t,
        cv.Optional(CONF_SLAVE_ID, default=1): cv.uint8_t,
        cv.GenerateID("custommodbus_id"): cv.use_id(CustomModbus),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config["custommodbus_id"])
    var = cg.new_Pvariable(config[CONF_ID])

    await number.register_number(var, config)

    cg.add(var.set_parent(parent))
    cg.add(var.set_register(config[CONF_REGISTER]))
    cg.add(var.set_bitmask(config[CONF_BITMASK]))
    cg.add(var.set_slave_id(config[CONF_SLAVE_ID]))

    cg.add(parent.add_number(config[CONF_REGISTER], var))
