import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import ICON_THERMOMETER, UNIT_CELSIUS, CONF_ID

DEPENDENCIES = ["i2c"]

mlx90632_ns = cg.esphome_ns.namespace("mlx90632")
MLX90632Component = mlx90632_ns.class_(
    "MLX90632Component", cg.PollingComponent, i2c.I2CDevice
)

# Native enums
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

CONF_OBJECT_TEMPERATURE = "object_temperature"
CONF_AMBIENT_TEMPERATURE = "ambient_temperature"
CONF_MEASUREMENT_SELECT = "measurement_select"
CONF_REFRESH_RATE = "refresh_rate"
CONF_EMISSIVITY = "emissivity"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MLX90632Component),
            cv.Optional(CONF_OBJECT_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=2,
            ),
            cv.Optional(CONF_AMBIENT_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=2,
            ),
            cv.Optional(CONF_MEASUREMENT_SELECT, default="medical"): cv.enum(
                MEASUREMENT_MODES, lower=True
            ),
            cv.Optional(CONF_REFRESH_RATE, default="2hz"): cv.enum(
                REFRESH_RATES, lower=True
            ),
            cv.Optional(CONF_EMISSIVITY, default=1.0): cv.All(
                cv.float_range(min=0.0, max=1.0)
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x3A)),
    cv.has_at_least_one_key(CONF_OBJECT_TEMPERATURE, CONF_AMBIENT_TEMPERATURE),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_measurement_mode(config[CONF_MEASUREMENT_SELECT]))
    cg.add(var.set_refresh_rate(config[CONF_REFRESH_RATE]))
    cg.add(var.set_emissivity(config[CONF_EMISSIVITY]))

    if CONF_OBJECT_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_OBJECT_TEMPERATURE])
        cg.add(var.set_object_temperature_sensor(sens))

    if CONF_AMBIENT_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_AMBIENT_TEMPERATURE])
        cg.add(var.set_ambient_temperature_sensor(sens))
    
    await i2c.register_i2c_device(var, config)
