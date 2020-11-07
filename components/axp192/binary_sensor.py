import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_UID, CONF_ID, CONF_TYPE
from . import axp192_ns, AXP192Component, CONF_AXP192_ID

DEPENDENCIES = ['axp192']

AXP192BinarySensor = axp192_ns.class_('AXP192BinarySensor', binary_sensor.BinarySensor, cg.Component)

MonitorType = axp192_ns.enum('Monitor')
MONITOR_TYPE = {
    'PLUGGED': MonitorType.MONITOR_PLUGGED,
    'CHARGING': MonitorType.MONITOR_CHARGING,
    'OVERTEMP': MonitorType.MONITOR_OVERTEMP,
    'LOW_BATTERY': MonitorType.MONITOR_LOWBAT,
    'CRITICAL_BATTERY': MonitorType.MONITOR_CRITBAT,
    'CHARGED': MonitorType.MONITOR_CHARGED,
}

CONFIG_SCHEMA = binary_sensor.BINARY_SENSOR_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(AXP192BinarySensor),
    cv.Required(CONF_AXP192_ID): cv.use_id(AXP192Component),
    cv.Required(CONF_TYPE): cv.enum(MONITOR_TYPE, upper=True, space='_')
}).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield binary_sensor.register_binary_sensor(var, config)

    axp = yield cg.get_variable(config[CONF_AXP192_ID])
    cg.add(axp.register_monitor(var))
    cg.add(var.set_type(config[CONF_TYPE]))
