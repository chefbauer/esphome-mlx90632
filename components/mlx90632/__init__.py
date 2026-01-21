import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_ADDRESS, CONF_UPDATE_INTERVAL
from esphome.components import i2c

DEPENDENCIES = ["i2c"]

mlx90632_ns = cg.esphome_ns.namespace("mlx90632")
MLX90632Sensor = mlx90632_ns.class_("MLX90632Sensor", cg.PollingComponent, i2c.I2CDevice)

CONF_MEASUREMENT_SELECT = "measurement_select"
CONF_REFRESH_RATE = "refresh_rate"
CONF_EMISSIVITY = "emissivity"

MEASUREMENT_SELECT_OPTIONS = {
    "medical": "MEASUREMENT_MODE_MEDICAL",
    "extended_range": "MEASUREMENT_MODE_EXTENDED",
}

REFRESH_RATE_OPTIONS = {
    "0.5hz": "REFRESH_RATE_0_5HZ",
    "1hz": "REFRESH_RATE_1HZ",
    "2hz": "REFRESH_RATE_2HZ",
    "4hz": "REFRESH_RATE_4HZ",
    "8hz": "REFRESH_RATE_8HZ",
    "16hz": "REFRESH_RATE_16HZ",
    "32hz": "REFRESH_RATE_32HZ",
    "64hz": "REFRESH_RATE_64HZ",
}

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MLX90632Sensor),
            cv.Optional(CONF_ADDRESS, default=0x3A): cv.i2c_address,
            cv.Optional(CONF_MEASUREMENT_SELECT, default="medical"): cv.enum(MEASUREMENT_SELECT_OPTIONS),
            cv.Optional(CONF_REFRESH_RATE, default="2hz"): cv.enum(REFRESH_RATE_OPTIONS),
            cv.Optional(CONF_EMISSIVITY, default=1.0): cv.float_range(min=0.0, max=1.0),
            cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.update_interval,
        }
    ).extend(cv.polling_component_schema("2s")).extend(i2c.i2c_device_schema(0x3A))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    cg.add(var.set_measurement_mode(getattr(mlx90632_ns, config[CONF_MEASUREMENT_SELECT])))
    cg.add(var.set_refresh_rate(getattr(mlx90632_ns, config[CONF_REFRESH_RATE])))
    cg.add(var.set_emissivity(config[CONF_EMISSIVITY]))
