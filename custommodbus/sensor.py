import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_NAME

DEPENDENCIES = ["uart"]

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(CustomModbus),
    cv.Required("uart_id"): cv.use_id(uart.UARTComponent),
    cv.Required(CONF_NAME): cv.string,
})

async def to_code(config):
    uart_component = await cg.get_variable(config["uart_id"])
    var = cg.new_Pvariable(config[CONF_ID], uart_component)

    cg.add(var.set_name(config[CONF_NAME]))

    await cg.register_component(var)
