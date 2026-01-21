import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import ICON_EMPTY, UNIT_TEMPERATURE

DEPENDENCIES = ["i2c"]

mlx90632_ns = cg.esphome_ns.namespace("mlx90632")
MLX90632Sensor = mlx90632_ns.class_(
    "MLX90632Sensor", cg.PollingComponent, i2c.I2CDevice, sensor.Sensor
)

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        MLX90632Sensor,
        unit_of_measurement=UNIT_TEMPERATURE,
        icon=ICON_EMPTY,
        accuracy_decimals=2,
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x3A))
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
