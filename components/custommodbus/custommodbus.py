#
#  CustomModbus – hovedkomponent (ESPHome 2025+ kompatibel)
#

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart

# C++ namespace
custommodbus_ns = cg.esphome_ns.namespace("custommodbus")

# C++ klasse
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

# YAML felter
CONF_SLAVE_ID = "slave_id"
CONF_UART_ID = "uart_id"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CustomModbus),

        # Reference til UART komponent
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),

        # Modbus slave ID
        cv.Required(CONF_SLAVE_ID): cv.uint8_t,
    }
)

async def to_code(config):
    # Opret C++ objekt
    var = cg.new_Pvariable(config[cv.GenerateID()])

    # Bind UART
    uart_component = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_parent(uart_component))

    # Sæt slave ID
    cg.add(var.set_slave_id(config[CONF_SLAVE_ID]))

    # Registrér komponenten i ESPHome
    await cg.register_component(var, config)
