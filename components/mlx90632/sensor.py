import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

DEPENDENCIES = ["i2c"]

mlx90632_ns = cg.esphome_ns.namespace("mlx90632")
MLX90632Sensor = mlx90632_ns.class_(
    "MLX90632Sensor", sensor.Sensor, cg.PollingComponent, i2c.I2CDevice
)

# Enums
MeasurementMode = mlx90632_ns.enum("MeasurementMode")
MEASUREMENT_MODES = {
    "medical": MeasurementMode.MEASUREMENT_MODE_MEDICAL,
    "extended_range": MeasurementMode.MEASUREMENT_MODE_EXTENDED,
}

RefreshRate = mlx90632_ns.enum("RefreshRate")
REFRESH_RATES = {
    "0.5hz": RefreshRate.REFRESH_RATE_0_5HZ,
    "1hz": RefreshRate.REFRESH_RATE_1HZ,
    "2hz": RefreshRate.REFRESH_RATE_2HZ,
    "4hz": RefreshRate.REFRESH_RATE_4HZ,
    "8hz": RefreshRate.REFRESH_RATE_8HZ,
    "16hz": RefreshRate.REFRESH_RATE_16HZ,
    "32hz": RefreshRate.REFRESH_RATE_32HZ,
    "64hz": RefreshRate.REFRESH_RATE_64HZ,
}

CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(
        MLX90632Sensor,
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(cv.polling_component_schema("2s"))
    .extend(
        {
            cv.Optional("measurement_select", default="medical"): cv.enum(
                MEASUREMENT_MODES, lower=True
            ),
            cv.Optional("refresh_rate", default="2hz"): cv.enum(
                REFRESH_RATES, lower=True
            ),
            cv.Optional("emissivity", default=1.0): cv.float_range(min=0.1, max=1.0),
        }
    )
    .extend(i2c.i2c_device_schema(0x3A))
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_measurement_mode(config["measurement_select"]))
    cg.add(var.set_refresh_rate(config["refresh_rate"]))
    cg.add(var.set_emissivity(config["emissivity"]))
