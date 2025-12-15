#
#  CustomModbus – hovedkomponent
#
#  Denne fil definerer YAML‑platformen:
#
#      custommodbus:
#        id: custommodbus1
#        uart_id: uart_modbus
#        slave_id: 1
#

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart

# C++ namespace
custommodbus_ns = cg.esphome_ns.namespace("custommodbus")

# C++ klasse
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

# YAML‑felter
CONF_SLAVE_ID = "slave_id"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CustomModbus),

        # UART device reference
        cv.Required("uart_id"): cv.use_id(uart.UARTComponent),

        # Modbus slave ID
        cv.Required(CONF_SLAVE_ID): cv.uint8_t,
    }
)

async def to_code(config):
    # Opret C++ objekt
    var = cg.new_Pvariable(config[cv.GenerateID()],)

    # Bind UART
    uart_component = await cg.get_variable(config["uart_id"])
    cg.add(var.set_parent(uart_component))

    # Sæt slave ID
    cg.add(var.set_slave_id(config[CONF_SLAVE_ID]))

    # Registrér komponenten i ESPHome
    await cg.register_component(var, config)
