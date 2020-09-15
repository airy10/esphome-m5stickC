import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import display, spi
from esphome.const import CONF_ID, CONF_LAMBDA
from esphome.const import CONF_DC_PIN, CONF_CS_PIN, CONF_ID, CONF_LAMBDA, CONF_PAGES
from esphome.const import CONF_EXTERNAL_VCC, CONF_LAMBDA, CONF_MODEL, CONF_RESET_PIN, \
    CONF_BRIGHTNESS
from . import st7735_ns

DEPENDENCIES = ['spi']

ST7735 = st7735_ns.class_('ST7735', cg.PollingComponent, spi.SPIDevice, display.DisplayBuffer)
ST7735Ref = ST7735.operator('ref')

CONFIG_SCHEMA = display.FULL_DISPLAY_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(ST7735),
    cv.Required(CONF_RESET_PIN): pins.gpio_output_pin_schema,
    cv.Required(CONF_DC_PIN): pins.gpio_output_pin_schema,
    cv.Required(CONF_CS_PIN): pins.gpio_output_pin_schema,
    cv.Optional(CONF_BRIGHTNESS, default=1.0): cv.percentage,
}).extend(cv.polling_component_schema('1s')).extend(spi.spi_device_schema())


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield spi.register_spi_device(var, config)

    dc = yield cg.gpio_pin_expression(config[CONF_DC_PIN])
    cg.add(var.set_dc_pin(dc))

    reset = yield cg.gpio_pin_expression(config[CONF_RESET_PIN])
    cg.add(var.set_reset_pin(reset))

    if CONF_LAMBDA in config:
        lambda_ = yield cg.process_lambda(
            config[CONF_LAMBDA], [(display.DisplayBufferRef, 'it')], return_type=cg.void)
        cg.add(var.set_writer(lambda_))

    yield display.register_display(var, config)
