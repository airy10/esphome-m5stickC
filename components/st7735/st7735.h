#pragma once

#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/display/display_buffer.h"

// ST7735 specific commands used in init
#define ST7735_NOP			0x00
#define ST7735_SWRESET		0x01
#define ST7735_RDDID		0x04
#define ST7735_RDDST		0x09

#define ST7735_RDDPM		0x0A      // Read display power mode
#define ST7735_RDD_MADCTL	0x0B      // Read display MADCTL
#define ST7735_RDD_COLMOD	0x0C      // Read display pixel format
#define ST7735_RDDIM		0x0D      // Read display image mode
#define ST7735_RDDSM		0x0E      // Read display signal mode
#define ST7735_RDDSR		0x0F      // Read display self-diagnostic result (ST7735)

#define ST7735_SLPIN		0x10
#define ST7735_SLPOUT		0x11
#define ST7735_PTLON		0x12
#define ST7735_NORON		0x13

#define ST7735_INVOFF		0x20
#define ST7735_INVON		0x21
#define ST7735_GAMSET		0x26      // Gamma set
#define ST7735_DISPOFF		0x28
#define ST7735_DISPON		0x29
#define ST7735_CASET		0x2A
#define ST7735_RASET		0x2B
#define ST7735_RAMWR		0x2C
#define ST7735_RGBSET		0x2D      // Color setting for 4096, 64K and 262K colors
#define ST7735_RAMRD		0x2E

#define ST7735_PTLAR		0x30
#define ST7735_VSCRDEF		0x33      // Vertical scrolling definition (ST7735)
#define ST7735_TEOFF		0x34      // Tearing effect line off
#define ST7735_TEON			0x35      // Tearing effect line on
#define ST7735_MADCTL		0x36      // Memory data access control
#define ST7735_IDMOFF		0x38      // Idle mode off
#define ST7735_IDMON		0x39      // Idle mode on
#define ST7735_RAMWRC		0x3C      // Memory write continue (ST7735)
#define ST7735_RAMRDC		0x3E      // Memory read continue (ST7735)
#define ST7735_COLMOD		0x3A

#define ST7735_RAMCTRL		0xB0      // RAM control
#define ST7735_RGBCTRL		0xB1      // RGB control
#define ST7735_PORCTRL		0xB2      // Porch control
#define ST7735_FRCTRL1		0xB3      // Frame rate control
#define ST7735_PARCTRL		0xB5      // Partial mode control
#define ST7735_GCTRL		0xB7      // Gate control
#define ST7735_GTADJ		0xB8      // Gate on timing adjustment
#define ST7735_DGMEN		0xBA      // Digital gamma enable
#define ST7735_VCOMS		0xBB      // VCOMS setting
#define ST7735_LCMCTRL		0xC0      // LCM control
#define ST7735_IDSET		0xC1      // ID setting
#define ST7735_VDVVRHEN		0xC2      // VDV and VRH command enable
#define ST7735_VRHS			0xC3      // VRH set
#define ST7735_VDVSET		0xC4      // VDV setting
#define ST7735_VCMOFSET		0xC5      // VCOMS offset set
#define ST7735_FRCTR2		0xC6      // FR Control 2
#define ST7735_CABCCTRL		0xC7      // CABC control
#define ST7735_REGSEL1		0xC8      // Register value section 1
#define ST7735_REGSEL2		0xCA      // Register value section 2
#define ST7735_PWMFRSEL		0xCC      // PWM frequency selection
#define ST7735_PWCTRL1		0xD0      // Power control 1
#define ST7735_VAPVANEN		0xD2      // Enable VAP/VAN signal output
#define ST7735_CMD2EN		0xDF      // Command 2 enable
#define ST7735_PVGAMCTRL	0xE0      // Positive voltage gamma control
#define ST7735_NVGAMCTRL	0xE1      // Negative voltage gamma control
#define ST7735_DGMLUTR		0xE2      // Digital gamma look-up table for red
#define ST7735_DGMLUTB		0xE3      // Digital gamma look-up table for blue
#define ST7735_GATECTRL		0xE4      // Gate control
#define ST7735_SPI2EN		0xE7      // SPI2 enable
#define ST7735_PWCTRL2		0xE8      // Power control 2
#define ST7735_EQCTRL		0xE9      // Equalize time control
#define ST7735_PROMCTRL		0xEC      // Program control
#define ST7735_PROMEN		0xFA      // Program mode enable
#define ST7735_NVMSET		0xFC      // NVM setting
#define ST7735_PROMACT		0xFE      // Program action

namespace esphome {
namespace st7735 {

template <int N>
struct Color565 {
  operator Color () const
  {
    return Color((((N)&0xF800) >> 8), (((N)&0x07E0) >> 3), (((N)&0x001F) << 3));
  }
};

}  // namespace st7735
}  // namespace esphome

// Some ready-made 16-bit ('565') color settings:
#define ST77XX_BLACK       esphome::st7735::Color565<0x0000>{}    /*   0,   0,   0 */
#define ST77XX_WHITE       esphome::st7735::Color565<0xFFFF>{}    /* 255, 255, 255 */
#define ST77XX_RED         esphome::st7735::Color565<0xF800>{}    /* 255,   0,   0 */
#define ST77XX_GREEN       esphome::st7735::Color565<0x07E0>{}    /*   0, 255,   0 */
#define ST77XX_BLUE        esphome::st7735::Color565<0x001F>{}    /*   0,   0, 255 */
#define ST77XX_CYAN        esphome::st7735::Color565<0x07FF>{}    /*   0, 255, 255 */
#define ST77XX_MAGENTA     esphome::st7735::Color565<0xF81F>{}    /* 255,   0, 255 */
#define ST77XX_YELLOW      esphome::st7735::Color565<0xFFE0>{}    /* 255, 255,   0 */
#define ST77XX_ORANGE      esphome::st7735::Color565<0xFDA0>{}    /* 255, 180,   0 */

// Some ready-made 16-bit ('565') color settings:
#define ST7735_BLACK      ST77XX_BLACK
#define ST7735_WHITE      ST77XX_WHITE
#define ST7735_RED        ST77XX_RED
#define ST7735_GREEN      ST77XX_GREEN
#define ST7735_BLUE       ST77XX_BLUE
#define ST7735_CYAN       ST77XX_CYAN
#define ST7735_MAGENTA    ST77XX_MAGENTA
#define ST7735_YELLOW     ST77XX_YELLOW
#define ST7735_ORANGE     ST77XX_ORANGE

namespace esphome {
namespace st7735 {

class ST7735 : public PollingComponent, public display::DisplayBuffer, 
                 public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_HIGH, spi::CLOCK_PHASE_TRAILING,
                                         spi::DATA_RATE_8MHZ> {
 public:
  void set_dc_pin(GPIOPin *dc_pin) { this->dc_pin_ = dc_pin; }
  void set_reset_pin(GPIOPin *reset_pin) { this->reset_pin_ = reset_pin; }
  
  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;
  void loop() override;
  
  void write_display_data();
  
 protected:
  GPIOPin *dc_pin_;
  GPIOPin *reset_pin_{nullptr};
  
  void displayInit(const uint8_t *addr);
  void sendcommand(uint8_t cmd, const uint8_t* dataBytes, uint8_t numDataBytes);
  void senddata(const uint8_t* dataBytes, uint8_t numDataBytes);

  void init_reset_();
  void writecommand(uint8_t value);
  void writedata(uint8_t value);
  
  void spi_master_write_addr(uint16_t addr1, uint16_t addr2);
  void spi_master_write_color(uint16_t color, uint16_t size);

  void draw_absolute_pixel_internal(int x, int y, Color color) override;

  int get_height_internal() override;
  int get_width_internal() override;
  size_t get_buffer_length_();
};


}  // namespace st7735
}  // namespace esphome
