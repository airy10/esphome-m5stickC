import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID, CONF_BATTERY_LEVEL, UNIT_PERCENT, ICON_BATTERY
from . import axp192_ns, AXP192Component, CONF_AXP192_ID


DEPENDENCIES = ['axp192']
AXP192Sensor = axp192_ns.class_('AXP192Sensor', sensor.Sensor, cg.Component)

CONFIG_SCHEMA = sensor.sensor_schema(UNIT_PERCENT, ICON_BATTERY, 1).extend({
    cv.GenerateID(): cv.declare_id(AXP192Sensor),
    cv.Required(CONF_AXP192_ID): cv.use_id(AXP192Component)
}).extend(cv.COMPONENT_SCHEMA)


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield sensor.register_sensor(var, config)

    axp = yield cg.get_variable(config[CONF_AXP192_ID])
    cg.add(axp.set_batterylevel_sensor(var))

