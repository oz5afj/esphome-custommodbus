PLATFORM_SCHEMA = sensor.sensor_schema().extend(
    {
        cv.GenerateID(): cv.declare_id(sensor.Sensor),
        cv.Required(CONF_ID): cv.use_id(CustomModbus),
        cv.Required("slave_id"): cv.int_range(min=1, max=247),
        cv.Required("register"): cv.hex_uint16_t,
        cv.Optional("count", default=1): cv.int_range(min=1, max=2),
        cv.Optional("scale", default=1.0): cv.float_,
        cv.Optional("data_type", default="uint16"): cv.string,
    }
).extend(uart.UART_DEVICE_SCHEMA)
