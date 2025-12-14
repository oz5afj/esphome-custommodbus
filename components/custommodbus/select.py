import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, select
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

# Namespace
custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbusSelect = custommodbus_ns.class_(
    "CustomModbusSelect", cg.Component, select.Select, uart.UARTDevice
)

# Konfigurationsnøgler
CONF_SLAVE_ID = "slave_id"
CONF_REGISTER = "register"
CONF_OPTIONS = "options"

# Schema for select-platformen
CONFIG_SCHEMA = (
    select.select_schema(CustomModbusSelect)
    .extend(
        {
            cv.GenerateID(): cv.declare_id(CustomModbusSelect),
            cv.Required(CONF_SLAVE_ID): cv.int_range(min=1, max=247),
            cv.Required(CONF_REGISTER): cv.hex_uint16_t,
            cv.Required(CONF_OPTIONS): cv.Schema({cv.string: cv.uint16_t}),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

# to_code-funktion med korrekt UART-håndtering
async def to_code(config):
    # Opret komponenten
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Hent UART-objektet korrekt
    uart_obj = await cg.get_variable(config["uart_id"])
    cg.add(var.set_uart(uart_obj))

    # Sæt slave_id
    cg.add(var.set_slave_id(config[CONF_SLAVE_ID]))

    # Opret select
    sel = await select.new_select(config)

    # Tilføj options
    for name, value in config[CONF_OPTIONS].items():
        cg.add(sel.add_option(name, value))

    # Tilføj callback når state ændres
    cg.add(sel.add_on_state_callback(var.write_single(config[CONF_REGISTER], sel)))
