#ifndef __AXP192_H__
#define __AXP192_H__

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/light/light_output.h"
#include "esphome/components/i2c/i2c.h"

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
      // from:
      /**
     * LDO2: Display backlight
     * LDO3: Display Control
     * RTC: Always ON, Switch RTC charging.
     * DCDC1: Main rail. When not set the controller shuts down.
     * DCDC3: Use unknown
     * LDO0: MIC
     */
      void begin(bool disableLDO2 = false, bool disableLDO3 = false, bool disableRTC = false, bool disableDCDC1 = false, bool disableDCDC3 = false, bool disableLDO0 = false);
      void ScreenBreath(uint8_t brightness);
      bool GetBatState();

      uint8_t GetInputPowerStatus();
      uint8_t GetBatteryChargingStatus();

      void DisableAllIRQ(void);
      void ClearAllIRQ(void);
      void EnablePressIRQ(bool short_press, bool long_press);
      void GetPressIRQ(bool *short_press, bool *long_press);
      void ClearPressIRQ(bool short_press, bool long_press);

      void EnableCoulombcounter(void);
      void DisableCoulombcounter(void);
      void StopCoulombcounter(void);
      void ClearCoulombcounter(void);
      uint32_t GetCoulombchargeData(void);    // Raw Data for Charge
      uint32_t GetCoulombdischargeData(void); // Raw Data for Discharge
      float GetCoulombData(void);             // total in - total out and calc

      uint16_t GetVbatData(void) __attribute__((deprecated));
      uint16_t GetIchargeData(void) __attribute__((deprecated));
      uint16_t GetIdischargeData(void) __attribute__((deprecated));
      uint16_t GetTempData(void) __attribute__((deprecated));
      uint32_t GetPowerbatData(void) __attribute__((deprecated));
      uint16_t GetVinData(void) __attribute__((deprecated));
      uint16_t GetIinData(void) __attribute__((deprecated));
      uint16_t GetVusbinData(void) __attribute__((deprecated));
      uint16_t GetIusbinData(void) __attribute__((deprecated));
      uint16_t GetVapsData(void) __attribute__((deprecated));
      uint8_t GetBtnPress(void);

      // -- sleep
      void SetSleep(void);
      void DeepSleep(uint64_t time_in_us = 0);
      void LightSleep(uint64_t time_in_us = 0);
      uint8_t GetWarningLeve(void) __attribute__((deprecated));

    public:
      void SetChargeVoltage(uint8_t);
      void SetChargeCurrent(uint8_t);
      void SetVOff(uint8_t voltage);
      float GetBatVoltage();
      float GetBatCurrent();
      float GetVinVoltage();
      float GetVinCurrent();
      float GetVBusVoltage();
      float GetVBusCurrent();
      float GetTempInAXP192();
      float GetBatPower();
      float GetBatChargeCurrent();
      float GetAPSVoltage();
      float GetBatCoulombInput();
      float GetBatCoulombOut();
      uint8_t GetWarningLevel(void);
      void SetCoulombClear() __attribute__((deprecated)); // use ClearCoulombcounter instead
      void SetLDO2(bool State);                           // Can turn LCD Backlight OFF for power saving
      void SetLDO3(bool State);
      void SetGPIO0(bool State);
      void SetAdcState(bool State);
      void SetAdcRate(uint8_t rate);

      // -- Power Off
      void PowerOff();

      // Power Maintained Storage
      void Read6BytesStorage(uint8_t *bufPtr);
      void Write6BytesStorage(uint8_t *bufPtr);

      void Write1Byte(uint8_t Addr, uint8_t Data);
      uint8_t Read8bit(uint8_t Addr);
      uint16_t Read12Bit(uint8_t Addr);
      uint16_t Read13Bit(uint8_t Addr);
      uint16_t Read16bit(uint8_t Addr);
      uint32_t Read24bit(uint8_t Addr);
      uint32_t Read32bit(uint8_t Addr);
      void ReadBuff(uint8_t Addr, uint8_t Size, uint8_t *Buff);
    };

  } // namespace axp192
} // namespace esphome

#endif
