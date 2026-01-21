import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["i2c"]

mlx90632_ns = cg.esphome_ns.namespace("mlx90632")
