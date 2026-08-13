**NOTE**:
These component are now quite obsolete as a st7735 screen component is now part of esphome and **m5stack has published** a axp192 one (some axp192 sensors might not work - to be tested - just showing the exposed ones as a starting point - check m5stack full doc)
Sample yaml to use the **m5stack** component (only some m5stickC  specific stuff) :
```yaml
substitutions:
  devicename: sample-device

esphome:
  name: $devicename
  on_boot:
     then:
       # read the RTC time once when the system boots
       pcf8563.read_time:

external_components:
  - source: github://m5stack/esphome-yaml/components
    components: [axp192]
    refresh: 0s

i2c:
   - id: bus_a
     sda: GPIO21
     scl: GPIO22
     scan: True

axp192:
  id: axp
  address: 0x34
  i2c_id: bus_a

binary_sensor:
  - platform: gpio
    pin:
      number: GPIO39
      inverted: true
    name: ${devicename} Button B

sensor:
  - platform: axp192
    update_interval: 60s
    id: axp192_sensor
    # Optional calibration settings for battery level calculation, suggest chart your own device battery voltage and charge current to configure these
    battery_discharge_100pc_voltage: 3800 #Battery voltage level at which 100% charge is assumed (when not charging, or charging finished) in mV
    #battery_discharge_100pc_voltage: 3670 #Battery voltage level at which 100% charge is assumed (when not charging, or charging finished) in mV
    battery_discharge_0pc_voltage: 3000 #Battery voltage level at which 0% charge is assumed (when not charging, or charging finished) in mV
    battery_charge_100pc_current: 60 #Battery charge current at which 100% charge is assumed (when charging) in mA
    battery_charge_0pc_current: 420 #Battery charge current at which 0% charge is assumed (when charging) in mA
    battery_voltage:
      name: "Battery Voltage"
    usb_voltage:
      name: "USB Voltage"
    usb_current:
      name: "USB Current"
    vbus_voltage:
      name: "VBus Voltage"
    vbus_current:
      name: "VBus Current"
    internal_temperature:
      name: "Int Temp"
    battery_power:
      name: "Battery Power"
    battery_charge_current:
      name: "Batt Charge I"
    battery_discharge_current:
      name: "Batt Discharge I"
    aps_voltage:
      name: "APS Voltage"
    battery_charging:
      id: charger_connected
      name: "Battery Charging"
    battery_level:
      name: "Battery Level"
      id: battery_percent
    usb_present:
      name: "USB Supply Present"
    usb_valid:
      name: "USB Supply Valid"
    vbus_present:
      name: "VBus Supply Present"
    vbus_valid:
      name: "VBus Supply Valid"
    battery_discharging:
      name: "Battery Discharging"
    over_temperature:
      name: "Over Temperature"
    battery_connected:
      name: "Battery Connected"
    battery_in_active_mode:
      name: "Battery in Active Mode"
    low_charge_current:
      name: "Charge current less than expected"
      
# internal LED
light:
  - platform: monochromatic
    id: display_backlight
    name: "LCD Backlight"
    icon: "mdi:television"
    entity_category: config
    output: lcd_backlight_output
    restore_mode: ALWAYS_ON #Always want to have a backlight on when we boot up
    default_transition_length: 250ms
  - platform: binary
    name: "LCD Power"
    output: lcd_power
    restore_mode: ALWAYS_ON #Always want to have a display on when we boot up

remote_transmitter:
  - pin: GPIO9
    carrier_duty_percent: 50%
spi:
  clk_pin: GPIO13
  mosi_pin: GPIO15

output:
  - platform: ledc
    pin: GPIO10
    inverted: true
    id: builtin_led    
  - platform: axp192
    type: range
    channel: DCDC3
    id: lcd_backlight_output
    min_voltage: 2100
    max_voltage: 3500
  - platform: axp192
    channel: DLDO2
    id: lcd_power
    voltage: 3000

switch:
  - platform: axp192
    led_pwr:
      name: "Green LED"

time:
  - platform: pcf8563
    address: 0x51
    # repeated synchronization is not necessary unless the external RTC
    # is much more accurate than the internal clock
    update_interval: never
    # Every morning on weekdays
    on_time:
      - seconds: 0
        minutes: 0
        hours: 2
        then:
          - button.press: reboot

  - platform: homeassistant
    id: homeassistant_time
    on_time_sync:
      then:
        # ... and update the RTC when the synchronization was successful
        pcf8563.write_time:
```

Another axp192 component can be found here :
  https://gitlab.com/geiseri/esphome_extras.git
  https://gitlab.com/geiseri/esphome_extras/-/tree/main/components/axp192

------------
**Original READEME:**
Additional components to use esphome with a M5StickC.  
Mostly a work in progress.  
Copy the components to a `custom_components` directory next to your .yaml configuration file.

- axp192 : power management - configuration hard coded for the M5StickC - should be made more generic.
  Must be present to initialize the screen power.  
  Can be also used to read some of the power info - most sensors aren't published yet.  
  Mostly the AXP192 from M5Stack sample code adapted for esphome.
- st7735 : TFT screen  - configuration hard coded for the M5StickC - should be made more generic.  
  Adapted from https://github.com/musk95/esphome-1/tree/dev/esphome/components/st7789v + M5StickC sample code for the init
