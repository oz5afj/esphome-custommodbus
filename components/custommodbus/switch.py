import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, switch
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

# Namespace
custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbusSwitch = custommodbus_ns.class_(
    "CustomModbusSwitch", cg.Component, switch.Switch, uart.UARTDevice
)

# Konfigurationsnøgler
CONF_SLAVE_ID = "slave_id"
CONF_REGISTER = "register"
CONF_BITMASK = "bitmask"

# Schema for switch-platformen
CONFIG_SCHEMA = (
    switch.switch_schema(CustomModbusSwitch)
    .extend(
        {
            cv.GenerateID(): cv.declare_id(CustomModbusSwitch),
            cv.Required(CONF_SLAVE_ID): cv.int_range(min=1, max=247),
            cv.Required(CONF_REGISTER): cv.hex_uint16_t,
            cv.Optional(CONF_BITMASK): cv.hex_uint16_t,
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

# to_code-funktion med korrekt async/variable-håndtering
async def to_code(config):
    # Opret switch-componenten
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Hent UART-objektet korrekt
    uart_obj = await cg.get_variable(config["uart_id"])
    cg.add(var.set_uart(uart_obj))

    # Sæt slave_id og register
    cg.add(var.set_slave_id(config[CONF_SLAVE_ID]))
    cg.add(var.set_register(config[CONF_REGISTER]))

    # Opret selve switchen
    sw = await switch.new_switch(config)

    # Tilføj on/off-handling
    if CONF_BITMASK in config:
        cg.add(sw.add_on_turn_on(var.write_bitmask(config[CONF_REGISTER], config[CONF_BITMASK], True)))
        cg.add(sw.add_on_turn_off(var.write_bitmask(config[CONF_REGISTER], config[CONF_BITMASK], False)))
    else:
        cg.add(sw.add_on_turn_on(var.write_single(config[CONF_REGISTER], 1)))
        cg.add(sw.add_on_turn_off(var.write_single(config[CONF_REGISTER], 0)))
