#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/light/light_output.h"
#include "esphome/components/i2c/i2c.h"
#include "AXP192.h"

namespace esphome
{
  namespace axp192
  {
    enum Monitor : uint8_t
    {
      MONITOR_PLUGGED,
      MONITOR_CHARGING,
      MONITOR_OVERTEMP,
      MONITOR_LOWBAT,
      MONITOR_CRITBAT,
      MONITOR_CHARGED,
    };
    class AXP192BinarySensor : public binary_sensor::BinarySensor, public Component
    {
    public:
      void set_type(const Monitor &monitor) { monitor_ = monitor; }
      void update(uint8_t input, uint8_t power, uint32_t irq);
      std::string device_class() override;
      bool is_status_binary_sensor() const override { return true; };
      float get_setup_priority() const override { return setup_priority::DATA; };

    private:
      Monitor monitor_ = Monitor::MONITOR_PLUGGED;
      bool last_state_ = false;
    };

    class AXP192Sensor : public sensor::Sensor, public Component
    {
      float get_setup_priority() const override { return setup_priority::DATA; };
    };

    class AXP192Component;
    class AXP192Backlight : public Component, public light::LightOutput
    {
    public:
      void set_axp_parent(AXP192Component *parent) { parent_ = parent; }
      float get_setup_priority() const override { return setup_priority::HARDWARE; };
      light::LightTraits get_traits() override;
      void write_state(light::LightState *state) override;

    private:
      AXP192Component *parent_{nullptr};
    };

    class AXP192Component : public PollingComponent, public i2c::I2CDevice
    {
    public:
      void set_batterylevel_sensor(AXP192Sensor *batterylevel_sensor) { batterylevel_sensor_ = batterylevel_sensor; }
      void set_battery_capacity(float battery_capacity) { battery_capacity_ = battery_capacity; }
      void set_brightness(float brightness) { brightness_ = brightness; }
      void register_monitor(AXP192BinarySensor *monitor) { monitors_.push_back(monitor); }

      // ========== INTERNAL METHODS ==========
      // (In most use cases you won't need these)
      void setup() override;
      void dump_config() override;
      float get_setup_priority() const override { return setup_priority::HARDWARE; };
      void update() override;
      void loop() override;

    protected:
      AXP192Sensor *batterylevel_sensor_ = nullptr;
      std::vector<AXP192BinarySensor *> monitors_;
      float brightness_{1.0f};
      float curr_brightness_{-1.0f};
      float battery_capacity_{80.0f};

    private:
      AXP192 *axp_ = nullptr;
    };

  } // namespace axp192
} // namespace esphome
