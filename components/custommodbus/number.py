import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, number
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

# Namespace
custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbusNumber = custommodbus_ns.class_(
    "CustomModbusNumber", cg.Component, number.Number, uart.UARTDevice
)

# Konfigurationsnøgler
CONF_SLAVE_ID = "slave_id"
CONF_REGISTER = "register"

# Schema for number-platformen
CONFIG_SCHEMA = (
    number.number_schema(CustomModbusNumber)
    .extend(
        {
            cv.GenerateID(): cv.declare_id(CustomModbusNumber),
            cv.Required(CONF_SLAVE_ID): cv.int_range(min=1, max=247),
            cv.Required(CONF_REGISTER): cv.hex_uint16_t,
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

    # Opret number
    num = await number.new_number(config)

    # Tilføj callback til registeret
    cg.add(
        num.add_on_state_callback(
            lambda v: var.write_single(config[CONF_REGISTER], cg.float_(v))
        )
    )
