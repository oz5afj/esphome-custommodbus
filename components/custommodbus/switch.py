import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, switch
from esphome.const import CONF_ID

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

# Brug PLATFORM_SCHEMA i stedet for switch.switch_schema()
PLATFORM_SCHEMA = switch.PLATFORM_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(switch.Switch),
        cv.Required(CONF_ID): cv.use_id(CustomModbus),
        cv.Required("slave_id"): cv.int_range(min=1, max=247),
        cv.Required("register"): cv.hex_uint16_t,
        cv.Optional("bitmask"): cv.hex_uint16_t,
    }
).extend(uart.UART_DEVICE_SCHEMA)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])
    await uart.register_uart_device(parent, config)
    sw = await switch.new_switch(config)

    cg.add(parent.set_slave_id(config["slave_id"]))

    if "bitmask" in config:
        # Bind on/off actions to bitmask writes
        cg.add(sw.add_on_turn_on(parent.write_bitmask(config["register"], config["bitmask"], True)))
        cg.add(sw.add_on_turn_off(parent.write_bitmask(config["register"], config["bitmask"], False)))
    else:
        # Bind on/off actions to single register writes
        cg.add(sw.add_on_turn_on(parent.write_single(config["register"], 1)))
        cg.add(sw.add_on_turn_off(parent.write_single(config["register"], 0)))
