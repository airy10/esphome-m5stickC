#include "axp192.h"
#include "esphome/core/log.h"

namespace esphome
{
    namespace axp192
    {
        static const char *TAG = "axp192";
        static const char *SENSOR_TAG = "axp192.sensor";
        static const char *BINARY_TAG = "axp192.binary_sensor";
        static const char *LIGHT_TAG = "axp192.light";

        std::string AXP192BinarySensor::device_class()
        {
            if (monitor_ == Monitor::MONITOR_PLUGGED)
            {
                return "plug";
            }
            if (monitor_ == Monitor::MONITOR_CHARGING)
            {
                return "battery_charging";
            }
            if (monitor_ == Monitor::MONITOR_OVERTEMP)
            {
                return "heat";
            }
            if (monitor_ == Monitor::MONITOR_LOWBAT || monitor_ == Monitor::MONITOR_CRITBAT)
            {
                return "battery";
            }
            return "None";
        }

        void AXP192BinarySensor::update(uint8_t input_status, uint8_t power_status, uint32_t irq_status)
        {
            bool should_fire = false;
            if (monitor_ == Monitor::MONITOR_PLUGGED && (input_status & 0b10000000))
            {
                should_fire = true;
            }
            if (monitor_ == Monitor::MONITOR_PLUGGED && (input_status & 0b00100000))
            {
                should_fire = true;
            }
            if (monitor_ == Monitor::MONITOR_CHARGING && (input_status & 0b00000100))
            {
                should_fire = true;
            }
            if (monitor_ == Monitor::MONITOR_CHARGING && (power_status & 0b01000000))
            {
                should_fire = true;
            }
            if (monitor_ == Monitor::MONITOR_OVERTEMP && (power_status & 0b10000000))
            {
                should_fire = true;
            }
            //44:7 - overvoltage 0b10000000 00000000 00000000 00000000
            //44:4 - overvoltage  0b00010000 00000000 00000000 00000000
            //44:1 - undervoltage  0b00000010 00000000 00000000 00000000
            //45:3 - charging  0b00000000 00001000 00000000 00000000
            //45:0 - undertemp  0b00000000 00000001 00000000 00000000
            //47:7 - boot  0b00000000 00000000 00000000 10000000
            //47:6 - shutdown  0b00000000 00000000 00000000 01000000

            //45:1 - overtemp 0b00000000 00000010 00000000 00000000
            if (monitor_ == Monitor::MONITOR_OVERTEMP && (irq_status & 0x00020000))
            {
                should_fire = true;
            }
            //46:7 - overtemp 0b00000000 00000000 10000000 00000000
            if (monitor_ == Monitor::MONITOR_OVERTEMP && (irq_status & 0x00008000))
            {
                should_fire = true;
            }
            //46:4 - bat low  0b00000000 00000000 00010000 00000000
            if (monitor_ == Monitor::MONITOR_LOWBAT && (irq_status & 0x00001000))
            {
                should_fire = true;
            }
            //47:0 - bat critical low 0b00000000 00000000 00000000 00000001
            if (monitor_ == Monitor::MONITOR_CRITBAT && (irq_status & 0x00000001))
            {
                should_fire = true;
            }
            //45:2 - finished charging 0b00000000 00000100 00000000 00000000
            if (monitor_ == Monitor::MONITOR_CHARGED && (irq_status & 0x00040000))
            {
                should_fire = true;
            }
            if (last_state_ == should_fire)
            {
                return;
            }

            ESP_LOGD(BINARY_TAG, "type: %d, input: %s, power: %s, irq: %s",
                     monitor_,
                     uint32_to_string(input_status).c_str(),
                     uint32_to_string(power_status).c_str(),
                     uint32_to_string(irq_status).c_str());

            publish_state(should_fire);
            last_state_ = should_fire;
        }

        light::LightTraits AXP192Backlight::get_traits()
        {
            auto traits = light::LightTraits();
            traits.set_supports_brightness(true);
            return traits;
        }

        void AXP192Backlight::write_state(light::LightState *state)
        {
            float bright = 0.0;
            state->current_values_as_brightness(&bright);
            if (parent_ != nullptr)
            {
                parent_->set_brightness(bright);
            }
        }

        void AXP192Component::setup()
        {
            begin(false, false, false, false, false);
        }

        void AXP192Component::dump_config()
        {
            ESP_LOGCONFIG(TAG, "AXP192:");
            LOG_I2C_DEVICE(this);
            LOG_SENSOR("  ", "Battery Level", this->batterylevel_sensor_);
            for (auto monitor : monitors_)
            {
                LOG_BINARY_SENSOR("  ", "Monitor", monitor);
            }
        }

        void AXP192Component::update()
        {

            if (this->batterylevel_sensor_ != nullptr)
            {
                // To be fixed
                // This is not giving the right value - mostly there to have some sample sensor...
                float vbat = GetBatVoltage();
                //float batterylevel = 100.0 * ((vbat - 3.0) / (4.1 - 3.0));
                float batterylevel = 100 * (GetCoulombData() / this->battery_capacity_);
                ESP_LOGD(TAG, "Got Battery Level=%f (%f)", batterylevel, vbat);
                if (batterylevel > 100.)
                {
                    batterylevel = 100;
                }
                this->batterylevel_sensor_->publish_state(batterylevel);
            }

            auto input_status = GetInputPowerStatus();
            auto power_status = GetBatteryChargingStatus();
            bool ac_in = input_status & 0b10000000;
            bool vbus_in = input_status & 0b00100000;
            bool bat_charge = input_status & 0b00000100;
            bool axp_overtemp = power_status & 0b10000000;
            bool charge_req = power_status & 0b01000000;
            bool bat_active = power_status & 0b00001000;

            ESP_LOGD(TAG, "input: %x, power: %x, ac_in: %d, vbus_in: %d, bat_charge: %d, axp_overtemp: %d, charge_req: %d, bat_active: %d",
                     input_status,
                     power_status,
                     ac_in,
                     vbus_in,
                     bat_charge,
                     axp_overtemp,
                     charge_req,
                     bat_active);

            ESP_LOGD(TAG, "Col in: %u Col out: %u Charge: %fmAh cin: %f batc: %f",
                     GetCoulombchargeData(),
                     GetCoulombdischargeData(),
                     GetCoulombData(),
                     GetBatChargeCurrent(),
                     GetBatCurrent());
        }

        void AXP192Component::loop()
        {

            auto input_status = GetInputPowerStatus();
            auto power_status = GetBatteryChargingStatus();
            auto irq_status = Read32bit(0x44);
            for (auto monitor : monitors_)
            {
                monitor->update(input_status, power_status, irq_status);
            }
            //ESP_LOGD(TAG, "Brightness=%f (Curr: %f)", brightness_, curr_brightness_);
            if (brightness_ == curr_brightness_)
            {
                return;
            }
            curr_brightness_ = brightness_;
            ScreenBreath(curr_brightness_ * 12);
        }

        void AXP192Component::begin(bool disableLDO2, bool disableLDO3, bool disableRTC, bool disableDCDC1, bool disableDCDC3, bool disableLDO0)
        {
            // Set LDO2 & LDO3(TFT_LED & TFT) 3.0V
            Write1Byte(0x28, 0xcc);

            // Set ADC sample rate to 200hz
            Write1Byte(0x84, 0b11110010);

            // Set ADC to All Enable
            Write1Byte(0x82, 0xff);

            // Bat charge voltage to 4.2, Current 100MA
            Write1Byte(0x33, 0xc0);

            // Depending on configuration enable LDO2, LDO3, DCDC1, DCDC3.
            byte buf = (Read8bit(0x12) & 0xef) | 0x4D;
            if (disableLDO3)
                buf &= ~(1 << 3);
            if (disableLDO2)
                buf &= ~(1 << 2);
            if (disableDCDC3)
                buf &= ~(1 << 1);
            if (disableDCDC1)
                buf &= ~(1 << 0);
            Write1Byte(0x12, buf);

            // 128ms power on, 4s power off
            Write1Byte(0x36, 0x0C);

            if (!disableLDO0)
            {
                // Set MIC voltage to 2.8V
                Write1Byte(0x91, 0xA0);

                // Set GPIO0 to LDO
                Write1Byte(0x90, 0x02);
            }
            else
            {
                Write1Byte(0x90, 0x07); // GPIO0 floating
            }

            // Disable vbus hold limit
            Write1Byte(0x30, 0x80);

            // Set temperature protection
            Write1Byte(0x39, 0xfc);

            // Enable RTC BAT charge
            Write1Byte(0x35, 0xa2 & (disableRTC ? 0x7F : 0xFF));

            // Enable bat detection
            Write1Byte(0x32, 0x46);

            // Set Power off voltage 3.0v
            Write1Byte(0x31, (Read8bit(0x31) & 0xf8) | (1 << 2));

            EnableCoulombcounter();
        }

        void AXP192Component::Write1Byte(uint8_t Addr, uint8_t Data)
        {
            write_byte(Addr, Data);
        }

        uint8_t AXP192Component::Read8bit(uint8_t Addr)
        {
            auto data = read_byte(Addr);
            if (data.has_value())
            {
                return data.value();
            }
            return 0;
        }

        uint16_t AXP192Component::Read12Bit(uint8_t Addr)
        {
            auto buf = read_bytes<2>(Addr);
            uint16_t data = 0;
            if (buf.has_value())
            {
                data = (buf.value()[0] << 4) | buf.value()[1];
            }
            return data;
        }

        uint16_t AXP192Component::Read13Bit(uint8_t Addr)
        {
            auto buf = read_bytes<2>(Addr);
            uint16_t data = 0;
            if (buf.has_value())
            {
                data = (buf.value()[0] << 5) | buf.value()[1];
            }
            return data;
        }

        uint16_t AXP192Component::Read16bit(uint8_t Addr)
        {
            auto buf = read_bytes<2>(Addr);
            uint16_t data = 0;
            if (buf.has_value())
            {
                for (auto d : buf.value())
                {
                    data <<= 8;
                    data |= d;
                }
            }
            return data;
        }

        uint32_t AXP192Component::Read24bit(uint8_t Addr)
        {
            auto buf = read_bytes<3>(Addr);
            uint32_t data = 0;
            if (buf.has_value())
            {
                for (auto d : buf.value())
                {
                    data <<= 8;
                    data |= d;
                }
            }
            return data;
        }

        uint32_t AXP192Component::Read32bit(uint8_t Addr)
        {
            auto buf = read_bytes<4>(Addr);
            uint32_t data = 0;
            if (buf.has_value())
            {
                for (auto d : buf.value())
                {
                    data <<= 8;
                    data |= d;
                }
            }
            return data;
        }

        void AXP192Component::ReadBuff(uint8_t Addr, uint8_t Size, uint8_t *Buff)
        {
            read_bytes(Addr, Buff, Size);
        }

        void AXP192Component::ScreenBreath(uint8_t brightness)
        {
            if (brightness > 12)
            {
                brightness = 12;
            }
            uint8_t buf = Read8bit(0x28);
            Write1Byte(0x28, ((buf & 0x0f) | (brightness << 4)));
        }

        // Return True = Battery Exist
        bool AXP192Component::GetBatState()
        {
            if (Read8bit(0x01) | 0x20)
                return true;
            else
                return false;
        }

        // Input Power Status
        uint8_t AXP192Component::GetInputPowerStatus()
        {
            return Read8bit(0x00);
        }

        // Battery Charging Status
        uint8_t AXP192Component::GetBatteryChargingStatus()
        {
            return Read8bit(0x01);
        }

        //---------coulombcounter_from_here---------
        //enable: void EnableCoulombcounter(void);
        //disable: void DisableCOulombcounter(void);
        //stop: void StopCoulombcounter(void);
        //clear: void ClearCoulombcounter(void);
        //get charge data: uint32_t GetCoulombchargeData(void);
        //get discharge data: uint32_t GetCoulombdischargeData(void);
        //get coulomb val affter calculation: float GetCoulombData(void);
        //------------------------------------------
        void AXP192Component::EnableCoulombcounter(void)
        {
            Write1Byte(0xB8, 0x80);
        }

        void AXP192Component::DisableCoulombcounter(void)
        {
            Write1Byte(0xB8, 0x00);
        }

        void AXP192Component::StopCoulombcounter(void)
        {
            Write1Byte(0xB8, 0xC0);
        }

        void AXP192Component::ClearCoulombcounter(void)
        {
            Write1Byte(0xB8, Read8bit(0xB8) | 0x20); // Only set the Clear Flag
        }

        uint32_t AXP192Component::GetCoulombchargeData(void)
        {
            return Read32bit(0xB0);
        }

        uint32_t AXP192Component::GetCoulombdischargeData(void)
        {
            return Read32bit(0xB4);
        }

        float AXP192Component::GetCoulombData(void)
        {
            uint32_t coin = GetCoulombchargeData();
            uint32_t coout = GetCoulombdischargeData();
            uint32_t valueDifferent = 0;
            bool bIsNegative = false;

            if (coin > coout)
            { // Expected, in always more then out
                valueDifferent = coin - coout;
            }
            else
            { // Warning: Out is more than In, the battery is not started at 0%
                // just Flip the output sign later
                bIsNegative = true;
                valueDifferent = coout - coin;
            }
            //c = 65536 * current_LSB * (coin - coout) / 3600 / ADC rate
            //Adc rate can be read from 84H, change this variable if you change the ADC reate
            float ccc = (65536 * 0.5 * valueDifferent) / 3600.0 / 200.0; // Note the ADC has defaulted to be 200 Hz

            if (bIsNegative)
                ccc = 0.0 - ccc; // Flip it back to negative
            return ccc;
        }
        //----------coulomb_end_at_here----------

        uint16_t AXP192Component::GetVbatData(void)
        {

            uint16_t vbat = 0;
            uint8_t buf[2];
            ReadBuff(0x78, 2, buf);
            vbat = ((buf[0] << 4) + buf[1]); // V
            return vbat;
        }

        uint16_t AXP192Component::GetVinData(void)
        {
            uint16_t vin = 0;
            uint8_t buf[2];
            ReadBuff(0x56, 2, buf);
            vin = ((buf[0] << 4) + buf[1]); // V
            return vin;
        }

        uint16_t AXP192Component::GetIinData(void)
        {
            uint16_t iin = 0;
            uint8_t buf[2];
            ReadBuff(0x58, 2, buf);
            iin = ((buf[0] << 4) + buf[1]);
            return iin;
        }

        uint16_t AXP192Component::GetVusbinData(void)
        {
            uint16_t vin = 0;
            uint8_t buf[2];
            ReadBuff(0x5a, 2, buf);
            vin = ((buf[0] << 4) + buf[1]); // V
            return vin;
        }

        uint16_t AXP192Component::GetIusbinData(void)
        {
            uint16_t iin = 0;
            uint8_t buf[2];
            ReadBuff(0x5C, 2, buf);
            iin = ((buf[0] << 4) + buf[1]);
            return iin;
        }

        uint16_t AXP192Component::GetIchargeData(void)
        {
            uint16_t icharge = 0;
            uint8_t buf[2];
            ReadBuff(0x7A, 2, buf);
            icharge = (buf[0] << 5) + buf[1];
            return icharge;
        }

        uint16_t AXP192Component::GetIdischargeData(void)
        {
            uint16_t idischarge = 0;
            uint8_t buf[2];
            ReadBuff(0x7C, 2, buf);
            idischarge = (buf[0] << 5) + buf[1];
            return idischarge;
        }

        uint16_t AXP192Component::GetTempData(void)
        {
            uint16_t temp = 0;
            uint8_t buf[2];
            ReadBuff(0x5e, 2, buf);
            temp = ((buf[0] << 4) + buf[1]);
            return temp;
        }

        uint32_t AXP192Component::GetPowerbatData(void)
        {
            uint32_t power = 0;
            uint8_t buf[3];
            ReadBuff(0x70, 2, buf);
            power = (buf[0] << 16) + (buf[1] << 8) + buf[2];
            return power;
        }

        uint16_t AXP192Component::GetVapsData(void)
        {
            uint16_t vaps = 0;
            uint8_t buf[2];
            ReadBuff(0x7e, 2, buf);
            vaps = ((buf[0] << 4) + buf[1]);
            return vaps;
        }

        void AXP192Component::SetSleep(void)
        {
            Write1Byte(0x31, Read8bit(0x31) | (1 << 3)); // Turn on short press to wake up
            Write1Byte(0x90, Read8bit(0x90) | 0x07);     // GPIO0 floating
            Write1Byte(0x82, 0x00);                      // Disable ADCs
            Write1Byte(0x12, Read8bit(0x12) & 0xA1);     // Disable all outputs but DCDC1
        }

        // -- sleep
        void AXP192Component::DeepSleep(uint64_t time_in_us)
        {
            SetSleep();
            esp_sleep_enable_ext0_wakeup((gpio_num_t)37, LOW);
            if (time_in_us > 0)
            {
                esp_sleep_enable_timer_wakeup(time_in_us);
            }
            else
            {
                esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
            }
            (time_in_us == 0) ? esp_deep_sleep_start() : esp_deep_sleep(time_in_us);
        }

        void AXP192Component::LightSleep(uint64_t time_in_us)
        {
            if (time_in_us > 0)
            {
                esp_sleep_enable_timer_wakeup(time_in_us);
            }
            else
            {
                esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
            }
            esp_light_sleep_start();
        }

        // Return 0 = not press, 0x01 = long press(1.5s), 0x02 = short press
        uint8_t AXP192Component::GetBtnPress()
        {
            uint8_t state = Read8bit(0x46); // IRQ 3 status.
            if (state)
            {
                Write1Byte(0x46, 0x03); // Write 1 back to clear IRQ
            }
            return state;
        }

        // Low Volt Level 1, when APS Volt Output < 3.4496 V
        // Low Volt Level 2, when APS Volt Output < 3.3992 V, then this flag is SET (0x01)
        // Flag will reset once battery volt is charged above Low Volt Level 1
        // Note: now AXP192 have the Shutdown Voltage of 3.0V (B100) Def in REG 31H
        uint8_t AXP192Component::GetWarningLevel(void)
        {
            return Read8bit(0x47) & 0x01;
        }

        float AXP192Component::GetBatVoltage()
        {
            float ADCLSB = 1.1 / 1000.0;
            uint16_t ReData = Read12Bit(0x78);
            return ReData * ADCLSB;
        }

        float AXP192Component::GetBatCurrent()
        {
            float ADCLSB = 0.5;
            uint16_t CurrentIn = Read13Bit(0x7A);
            uint16_t CurrentOut = Read13Bit(0x7C);
            return (CurrentIn - CurrentOut) * ADCLSB;
        }

        float AXP192Component::GetVinVoltage()
        {
            float ADCLSB = 1.7 / 1000.0;
            uint16_t ReData = Read12Bit(0x56);
            return ReData * ADCLSB;
        }

        float AXP192Component::GetVinCurrent()
        {
            float ADCLSB = 0.625;
            uint16_t ReData = Read12Bit(0x58);
            return ReData * ADCLSB;
        }

        float AXP192Component::GetVBusVoltage()
        {
            float ADCLSB = 1.7 / 1000.0;
            uint16_t ReData = Read12Bit(0x5A);
            return ReData * ADCLSB;
        }

        float AXP192Component::GetVBusCurrent()
        {
            float ADCLSB = 0.375;
            uint16_t ReData = Read12Bit(0x5C);
            return ReData * ADCLSB;
        }

        float AXP192Component::GetTempInAXP192()
        {
            float ADCLSB = 0.1;
            const float OFFSET_DEG_C = -144.7;
            uint16_t ReData = Read12Bit(0x5E);
            return OFFSET_DEG_C + ReData * ADCLSB;
        }

        float AXP192Component::GetBatPower()
        {
            float VoltageLSB = 1.1;
            float CurrentLCS = 0.5;
            uint32_t ReData = Read24bit(0x70);
            return VoltageLSB * CurrentLCS * ReData / 1000.0;
        }

        float AXP192Component::GetBatChargeCurrent()
        {
            float ADCLSB = 0.5;
            uint16_t ReData = Read13Bit(0x7A);
            return ReData * ADCLSB;
        }

        float AXP192Component::GetAPSVoltage()
        {
            float ADCLSB = 1.4 / 1000.0;
            uint16_t ReData = Read12Bit(0x7E);
            return ReData * ADCLSB;
        }

        float AXP192Component::GetBatCoulombInput()
        {
            uint32_t ReData = Read32bit(0xB0);
            return ReData * 65536 * 0.5 / 3600 / 25.0;
        }

        float AXP192Component::GetBatCoulombOut()
        {
            uint32_t ReData = Read32bit(0xB4);
            return ReData * 65536 * 0.5 / 3600 / 25.0;
        }

        void AXP192Component::SetCoulombClear()
        {
            Write1Byte(0xB8, 0x20);
        }

        // Can turn LCD Backlight OFF for power saving
        void AXP192Component::SetLDO2(bool State)
        {
            uint8_t buf = Read8bit(0x12);
            if (State == true)
            {
                buf = (1 << 2) | buf;
            }
            else
            {
                buf = ~(1 << 2) & buf;
            }
            Write1Byte(0x12, buf);
        }

        void AXP192Component::SetLDO3(bool State)
        {
            uint8_t buf = Read8bit(0x12);
            if (State == true)
            {
                buf = (1 << 3) | buf;
            }
            else
            {
                buf = ~(1 << 3) & buf;
            }
            Write1Byte(0x12, buf);
        }

        void AXP192Component::SetGPIO0(bool State)
        {
            uint8_t buf = Read8bit(0x90);
            if (State == true)
            {
                buf &= ~(0x07); // clear last 3 bits
                buf |= 0x02;    // set as LDO
            }
            else
            {
                buf |= 0x07; // set as floating
            }
            Write1Byte(0x90, buf);
        }

        // Default is VOLTAGE_4200MV
        void AXP192Component::SetChargeVoltage(uint8_t voltage)
        {
            uint8_t buf = Read8bit(0x33);
            buf = (buf & ~(0x60)) | (voltage & 0x60);
            Write1Byte(0x33, buf);
        }

        // Not recommend to set charge current > 100mA, since Battery is only 80mAh.
        // more then 1C charge-rate may shorten battery life-span.
        void AXP192Component::SetChargeCurrent(uint8_t current)
        {
            uint8_t buf = Read8bit(0x33);
            buf = (buf & 0xf0) | (current & 0x07);
            Write1Byte(0x33, buf);
        }

        // Set power off voltage
        void AXP192Component::SetVOff(uint8_t voltage)
        {
            Write1Byte(0x31, (Read8bit(0x31) & 0xf8) | voltage);
        }

        // Cut all power, except for LDO1 (RTC)
        void AXP192Component::PowerOff()
        {
            Write1Byte(0x32, Read8bit(0x32) | 0x80); // MSB for Power Off
        }

        void AXP192Component::SetAdcState(bool state)
        {
            Write1Byte(0x82, state ? 0xff : 0x00); // Enable / Disable all ADCs
        }

        void AXP192Component::DisableAllIRQ()
        {
            Write1Byte(0x40, 0x00);
            Write1Byte(0x41, 0x00);
            Write1Byte(0x42, 0x00);
            Write1Byte(0x43, 0x00);
            Write1Byte(0x4a, 0x00);
        }

        void AXP192Component::EnablePressIRQ(bool short_press, bool long_press)
        {
            uint8_t value = Read8bit(0x42);
            value &= 0xfc;
            value |= short_press ? (0x01 << 1) : 0x00;
            value |= long_press ? (0x01 << 0) : 0x00;
            Write1Byte(0x42, value);
        }

        void AXP192Component::ClearAllIRQ()
        {
            Write1Byte(0x44, 0xff);
            Write1Byte(0x45, 0xff);
            Write1Byte(0x46, 0xff);
            Write1Byte(0x47, 0xff);
            Write1Byte(0x4D, 0xff);
        }

        void AXP192Component::GetPressIRQ(bool *short_press, bool *long_press)
        {
            uint8_t status = 0x00;
            status = Read8bit(0x46);
            *short_press = (status & (0x01 << 1)) ? true : false;
            *long_press = (status & (0x01 << 0)) ? true : false;
        }

        void AXP192Component::ClearPressIRQ(bool short_press, bool long_press)
        {
            uint8_t value = 0x00;
            value |= short_press ? (0x01 << 1) : 0x00;
            value |= long_press ? (0x01 << 0) : 0x00;
            Write1Byte(0x46, value);
        }

        void AXP192Component::SetAdcRate(uint8_t rate)
        {
            uint8_t buf = Read8bit(0x84);
            buf = (buf & ~(0xc0)) | (rate & 0xc0);
            Write1Byte(0x84, buf);
        }

        // AXP192 have a 6 byte storage, when the power is still valid, the data will not be lost
        void AXP192Component::Read6BytesStorage(uint8_t *bufPtr)
        {
            // Address from 0x06 - 0x0B
            read_bytes(0x06, bufPtr, 6);
        }

        // AXP192 have a 6 byte storage, when the power is still valid, the data will not be lost
        void AXP192Component::Write6BytesStorage(uint8_t *bufPtr)
        {
            // Address from 0x06 - 0x0B
            write_bytes(0x06, bufPtr, 6);
        }

    } // namespace axp192
} // namespace esphome
