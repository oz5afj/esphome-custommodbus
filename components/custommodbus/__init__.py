import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

# Namespace og binding til C++ klassen
custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

# Konfigurationsnøgler
CONF_UART_ID = "uart_id"
CONF_SLAVE_ID = "slave_id"
CONF_USE_GROUPED_READS = "use_grouped_reads"
CONF_READ_INTERVAL_MS = "read_interval_ms"
CONF_PUBLISH_COOLDOWN = "publish_cooldown_ms"
CONF_PUBLISH_THRESHOLD = "publish_threshold"

# Schema
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(CustomModbus),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_SLAVE_ID): cv.int_range(min=1, max=247),
        cv.Optional(CONF_USE_GROUPED_READS, default=False): cv.boolean,
        cv.Optional(CONF_READ_INTERVAL_MS, default=1000): cv.uint32_t,
        cv.Optional(CONF_PUBLISH_COOLDOWN, default=1000): cv.uint32_t,
        cv.Optional(CONF_PUBLISH_THRESHOLD, default=0.1): cv.float_,
    }
)

# to_code: opret og konfigurer C++-instansen
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    # UART parent
    uart_component = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart_parent(uart_component))

    # slave id
    cg.add(var.set_slave_id(config[CONF_SLAVE_ID]))

    # optional: use_grouped_reads
    if config.get(CONF_USE_GROUPED_READS, False):
        cg.add(var.set_use_grouped_reads(config[CONF_USE_GROUPED_READS]))
    cg.add(var.set_read_interval(config.get(CONF_READ_INTERVAL_MS, 1000)))
    cg.add(var.set_publish_cooldown(config.get(CONF_PUBLISH_COOLDOWN, 1000)))
    cg.add(var.set_publish_threshold(config.get(CONF_PUBLISH_THRESHOLD, 0.1)))
    await cg.register_component(var, config)
