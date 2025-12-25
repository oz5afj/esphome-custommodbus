import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_ICON,
    CONF_ENTITY_CATEGORY,
    CONF_DEVICE_CLASS,
    CONF_DISABLED_BY_DEFAULT,
    CONF_MODE,
)

from . import custommodbus_ns, CustomModbus

CustomModbusNumber = custommodbus_ns.class_("CustomModbusNumber", number.Number)

CONF_REGISTER = "register"
CONF_BITMASK = "bitmask"
CONF_SLAVE_ID = "slave_id"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CustomModbusNumber),
        cv.Required("custommodbus_id"): cv.use_id(CustomModbus),
        cv.Required(CONF_SLAVE_ID): cv.int_range(min=1, max=247),
        cv.Required(CONF_REGISTER): cv.hex_uint16_t,
        cv.Optional(CONF_BITMASK, default=0): cv.hex_uint16_t,

        cv.Required("min_value"): cv.float_,
        cv.Required("max_value"): cv.float_,
        cv.Required("step"): cv.float_,

        cv.Optional(CONF_NAME): cv.string,
        cv.Optional(CONF_ICON): cv.icon,
        cv.Optional(CONF_ENTITY_CATEGORY): cv.string,
        cv.Optional(CONF_DEVICE_CLASS): cv.string,
        cv.Optional(CONF_DISABLED_BY_DEFAULT, default=False): cv.boolean,
        cv.Optional(CONF_MODE, default="AUTO"): cv.enum(number.NUMBER_MODES),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config["custommodbus_id"])

    num = cg.new_Pvariable(config[CONF_ID])

    await number.register_number(
        num,
        config,
        min_value=config["min_value"],
        max_value=config["max_value"],
        step=config["step"],
    )

    cg.add(num.set_parent(parent))
    cg.add(num.set_slave_id(config[CONF_SLAVE_ID]))
    cg.add(num.set_register(config[CONF_REGISTER]))
    cg.add(num.set_bitmask(config[CONF_BITMASK]))

    cg.add(parent.add_number(config[CONF_REGISTER], num))
