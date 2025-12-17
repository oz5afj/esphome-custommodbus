import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor
from esphome.const import CONF_ID, CONF_NAME

DEPENDENCIES = ['uart']

custommodbus_ns = cg.esphome_ns.namespace('custommodbus')
CustomModbus = custommodbus_ns.class_('CustomModbus', cg.Component, uart.UARTDevice)

CONF_SLAVE_ID = 'slave_id'
CONF_UART_ID = 'uart_id'
CONF_GROUPS = 'groups'

READ_SCHEMA = cv.Schema({
    cv.Required('reg'): cv.uint16_t,
    cv.Required('count'): cv.uint8_t,
    cv.Required(CONF_NAME): cv.string,
    cv.Optional('smoothing_alpha', default=0.0): cv.float_,
})

GROUP_SCHEMA = cv.Schema({
    cv.Required('name'): cv.string,
    cv.Required('interval'): cv.positive_time_period_milliseconds,
    cv.Required('reads'): cv.ensure_list(READ_SCHEMA),
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_ID): cv.declare_id(CustomModbus),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Required(CONF_SLAVE_ID): cv.uint8_t,
    cv.Required(CONF_GROUPS): cv.ensure_list(GROUP_SCHEMA),
}, extra=cv.ALLOW_EXTRA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    uart_comp = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart_parent(uart_comp))
    cg.add(var.set_slave_id(config[CONF_SLAVE_ID]))
    await cg.register_component(var, config)

    # Opret grupper og reads
    for g in config[CONF_GROUPS]:
        group_name = g['name']
        # hent millisekunder fra TimePeriodMilliseconds objektet
        # (TimePeriodMilliseconds har attributten total_milliseconds)
        interval_ms = int(g['interval'].total_milliseconds)
        cg.add(var.add_group(group_name, interval_ms))

        for r in g['reads']:
            # Opret en ESPHome sensor og registrer den
            sens = sensor.new_sensor(r[CONF_NAME])
            await sensor.register_sensor(sens, r[CONF_NAME])
            alpha = float(r.get('smoothing_alpha', 0.0))
            # Tilføj read til gruppen
            cg.add(var.add_read_to_group(group_name, r['reg'], r['count'], sens, alpha))
