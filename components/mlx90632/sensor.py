import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import ICON_THERMOMETER, UNIT_CELSIUS, CONF_ID

DEPENDENCIES = ["i2c"]

mlx90632_ns = cg.esphome_ns.namespace("mlx90632")
MLX90632Component = mlx90632_ns.class_(
    "MLX90632Component", cg.PollingComponent, i2c.I2CDevice
)

# Use Adafruit enum from C++
MeasurementSelect = mlx90632_ns.enum("mlx90632_meas_select_t")
MEASUREMENT_SELECTS = {
    "medical": MeasurementSelect.MLX90632_MEAS_MEDICAL,
    "extended_range": MeasurementSelect.MLX90632_MEAS_EXTENDED_RANGE,
}

RefreshRate = mlx90632_ns.enum("mlx90632_refresh_rate_t")
REFRESH_RATES = {
    "0.5hz": RefreshRate.MLX90632_REFRESH_0_5HZ,
    "1hz": RefreshRate.MLX90632_REFRESH_1HZ,
    "2hz": RefreshRate.MLX90632_REFRESH_2HZ,
    "4hz": RefreshRate.MLX90632_REFRESH_4HZ,
    "8hz": RefreshRate.MLX90632_REFRESH_8HZ,
    "16hz": RefreshRate.MLX90632_REFRESH_16HZ,
    "32hz": RefreshRate.MLX90632_REFRESH_32HZ,
    "64hz": RefreshRate.MLX90632_REFRESH_64HZ,
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
                MEASUREMENT_SELECTS, lower=True
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
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_measurement_select(config[CONF_MEASUREMENT_SELECT]))
    cg.add(var.set_refresh_rate(config[CONF_REFRESH_RATE]))
    cg.add(var.set_emissivity(config[CONF_EMISSIVITY]))

    if CONF_OBJECT_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_OBJECT_TEMPERATURE])
        cg.add(var.set_object_temperature_sensor(sens))

    if CONF_AMBIENT_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_AMBIENT_TEMPERATURE])
        cg.add(var.set_ambient_temperature_sensor(sens))
