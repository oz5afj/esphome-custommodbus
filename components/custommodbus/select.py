import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, select
from esphome.const import CONF_ID

custommodbus_ns = cg.esphome_ns.namespace("custommodbus")
CustomModbus = custommodbus_ns.class_("CustomModbus", cg.Component, uart.UARTDevice)

# Robust schema defineret direkte med cv.Schema
PLATFORM_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(select.Select),
        cv.Optional("name"): cv.string,
        cv.Required(CONF_ID): cv.use_id(CustomModbus),
        cv.Required("slave_id"): cv.int_range(min=1, max=247),
        cv.Required("register"): cv.hex_uint16_t,
        # options: mapping fra label -> integer value
        cv.Required("options"): cv.Schema({cv.string: cv.positive_int}),
        cv.Optional("icon"): cv.icon,
    }
).extend(uart.UART_DEVICE_SCHEMA)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])
    await uart.register_uart_device(parent, config)
    sel = await select.new_select(config)

    cg.add(parent.set_slave_id(config["slave_id"]))

    # Tilføj alle options
    for name, value in config["options"].items():
        cg.add(sel.add_option(name, value))

    # Når select ændres, skriv den valgte værdi til registeret
    # Bemærk: cg.uint16_t(sel) bruges som wrapper for select objektet, lignende mønster som cg.float_(num)
    cg.add(sel.add_on_state_callback(
        parent.write_single(config["register"], cg.uint16_t(sel))
    ))
