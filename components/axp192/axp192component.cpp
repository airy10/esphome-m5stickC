#include "axp192component.h"
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
            axp_ = new AXP192(this);
            axp_->begin(false, false, false, false, false);
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
                float vbat = axp_->GetBatVoltage();
                //float batterylevel = 100.0 * ((vbat - 3.0) / (4.1 - 3.0));
                float batterylevel = 100 * (axp_->GetCoulombData() / this->battery_capacity_);
                ESP_LOGD(TAG, "Got Battery Level=%f (%f)", batterylevel, vbat);
                if (batterylevel > 100.)
                {
                    batterylevel = 100;
                }
                this->batterylevel_sensor_->publish_state(batterylevel);
            }

            auto input_status = axp_->GetInputPowerStatus();
            auto power_status = axp_->GetBatteryChargingStatus();
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
                     axp_->GetCoulombchargeData(),
                     axp_->GetCoulombdischargeData(),
                     axp_->GetCoulombData(),
                     axp_->GetBatChargeCurrent(),
                     axp_->GetBatCurrent());
        }

        void AXP192Component::loop()
        {
            auto input_status = axp_->GetInputPowerStatus();
            auto power_status = axp_->GetBatteryChargingStatus();
            auto irq_status = axp_->Read32bit(0x44);
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
            axp_->ScreenBreath(curr_brightness_ * 12);
        }
    } // namespace axp192
} // namespace esphome
