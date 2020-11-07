from esphome.components import light
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_OUTPUT_ID
from . import axp192_ns, AXP192Component, CONF_AXP192_ID

DEPENDENCIES = ['axp192']

AXP192Backlight = axp192_ns.class_('AXP192Backlight', light.LightOutput, cg.Component)

CONFIG_SCHEMA = cv.All(light.BRIGHTNESS_ONLY_LIGHT_SCHEMA.extend({
    cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(AXP192Backlight),
    cv.Required(CONF_AXP192_ID): cv.use_id(AXP192Component)
}).extend(cv.COMPONENT_SCHEMA))

def to_code(config):
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    yield cg.register_component(var, config)
    yield light.register_light(var, config)
    axp = yield cg.get_variable(config[CONF_AXP192_ID])
    cg.add(var.set_axp_parent(axp))
