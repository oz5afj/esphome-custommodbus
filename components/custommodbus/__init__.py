import esphome.codegen as cg
import esphome.config_validation as cv

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component)

AUTO_LOAD = ["sensor", "binary_sensor", "switch", "number", "select", "text_sensor"]

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(CustomModbus),
})

async def to_code(config):
    var = cg.new_Pvariable(config[cv.CONF_ID])
    await cg.register_component(var, config)
