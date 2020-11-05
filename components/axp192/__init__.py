import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import CONF_ID, CONF_BRIGHTNESS

AUTO_LOAD = ['binary_sensor', 'sensor', 'light']
MULTI_CONF = True

CONF_AXP192_ID = 'axp192_id'
DEPENDENCIES = ['i2c']
CONF_CAPACITY = 'battery_capacity'

axp192_ns = cg.esphome_ns.namespace('axp192')

AXP192Component = axp192_ns.class_('AXP192Component', cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(AXP192Component),
    cv.Optional(CONF_BRIGHTNESS, default=1.0): cv.percentage,
    cv.Optional(CONF_CAPACITY, default=80.0): cv.positive_float,
}).extend(cv.polling_component_schema('60s')).extend(i2c.i2c_device_schema(0x77))


def to_code(config):
    cg.add_global(axp192_ns.using)
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield i2c.register_i2c_device(var, config)

    if CONF_BRIGHTNESS in config:
        conf = config[CONF_BRIGHTNESS]
        cg.add(var.set_brightness(conf))

    if CONF_CAPACITY in config:
        conf = config[CONF_CAPACITY]
        cg.add(var.set_battery_capacity(conf))
