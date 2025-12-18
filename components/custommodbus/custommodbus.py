import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_UART_ID

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

CONF_SLAVE_ID = "slave_id"
CONF_USE_GROUPED_READS = "use_grouped_reads"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(CustomModbus),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_SLAVE_ID): cv.int_range(min=1, max=247),
        cv.Optional(CONF_USE_GROUPED_READS, default=False): cv.boolean,
    }
)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.register_component(var, config)

    uart_component = yield cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart_parent(uart_component))

    cg.add(var.set_slave_id(config[CONF_SLAVE_ID]))

    cg.add(var.set_use_grouped_reads(config[CONF_USE_GROUPED_READS]))

