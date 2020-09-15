#include "st7735.h"
#include "esphome/core/log.h"
#include "esphome/components/display/display_buffer.h"

namespace esphome {
namespace st7735 {

#define ST_CMD_DELAY      0x80    // special signifier for command lists

#define ST77XX_NOP        0x00
#define ST77XX_SWRESET    0x01
#define ST77XX_RDDID      0x04
#define ST77XX_RDDST      0x09

#define ST77XX_SLPIN      0x10
#define ST77XX_SLPOUT     0x11
#define ST77XX_PTLON      0x12
#define ST77XX_NORON      0x13

#define ST77XX_INVOFF     0x20
#define ST77XX_INVON      0x21
#define ST77XX_DISPOFF    0x28
#define ST77XX_DISPON     0x29
#define ST77XX_CASET      0x2A
#define ST77XX_RASET      0x2B
#define ST77XX_RAMWR      0x2C
#define ST77XX_RAMRD      0x2E

#define ST77XX_PTLAR      0x30
#define ST77XX_TEOFF      0x34
#define ST77XX_TEON       0x35
#define ST77XX_MADCTL     0x36
#define ST77XX_COLMOD     0x3A

#define ST77XX_MADCTL_MY  0x80
#define ST77XX_MADCTL_MX  0x40
#define ST77XX_MADCTL_MV  0x20
#define ST77XX_MADCTL_ML  0x10
#define ST77XX_MADCTL_RGB 0x00

#define ST77XX_RDID1      0xDA
#define ST77XX_RDID2      0xDB
#define ST77XX_RDID3      0xDC
#define ST77XX_RDID4      0xDD


// some flags for initR() :(
#define INITR_GREENTAB    0x00
#define INITR_REDTAB      0x01
#define INITR_BLACKTAB    0x02
#define INITR_18GREENTAB  INITR_GREENTAB
#define INITR_18REDTAB    INITR_REDTAB
#define INITR_18BLACKTAB  INITR_BLACKTAB
#define INITR_144GREENTAB 0x01
#define INITR_MINI160x80  0x04
#define INITR_HALLOWING   0x05

// Some register settings
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH  0x04

#define ST7735_FRMCTR1    0xB1
#define ST7735_FRMCTR2    0xB2
#define ST7735_FRMCTR3    0xB3
#define ST7735_INVCTR     0xB4
#define ST7735_DISSET5    0xB6

#define ST7735_PWCTR1     0xC0
#define ST7735_PWCTR2     0xC1
#define ST7735_PWCTR3     0xC2
#define ST7735_PWCTR4     0xC3
#define ST7735_PWCTR5     0xC4
#define ST7735_VMCTR1     0xC5

#define ST7735_PWCTR6     0xFC

#define ST7735_GMCTRP1    0xE0
#define ST7735_GMCTRN1    0xE1

static const uint8_t PROGMEM
    Rcmd1[] = {                       // 7735R init, part 1 (red or green tab)
        15,                             // 15 commands in list:
        ST77XX_SWRESET,   ST_CMD_DELAY, //  1: Software reset, 0 args, w/delay
        150,                          //     150 ms delay
        ST77XX_SLPOUT,    ST_CMD_DELAY, //  2: Out of sleep mode, 0 args, w/delay
        255,                          //     500 ms delay
        ST7735_FRMCTR1, 3,              //  3: Framerate ctrl - normal mode, 3 arg:
        0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
        ST7735_FRMCTR2, 3,              //  4: Framerate ctrl - idle mode, 3 args:
        0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
        ST7735_FRMCTR3, 6,              //  5: Framerate - partial mode, 6 args:
        0x01, 0x2C, 0x2D,             //     Dot inversion mode
        0x01, 0x2C, 0x2D,             //     Line inversion mode
        ST7735_INVCTR,  1,              //  6: Display inversion ctrl, 1 arg:
        0x07,                         //     No inversion
        ST7735_PWCTR1,  3,              //  7: Power control, 3 args, no delay:
        0xA2,
        0x02,                         //     -4.6V
        0x84,                         //     AUTO mode
        ST7735_PWCTR2,  1,              //  8: Power control, 1 arg, no delay:
        0xC5,                         //     VGH25=2.4C VGSEL=-10 VGH=3 * AVDD
        ST7735_PWCTR3,  2,              //  9: Power control, 2 args, no delay:
        0x0A,                         //     Opamp current small
        0x00,                         //     Boost frequency
        ST7735_PWCTR4,  2,              // 10: Power control, 2 args, no delay:
        0x8A,                         //     BCLK/2,
        0x2A,                         //     opamp current small & medium low
        ST7735_PWCTR5,  2,              // 11: Power control, 2 args, no delay:
        0x8A, 0xEE,
        ST7735_VMCTR1,  1,              // 12: Power control, 1 arg, no delay:
        0x0E,
        ST77XX_INVOFF,  0,              // 13: Don't invert display, no args
        ST77XX_MADCTL,  1,              // 14: Mem access ctl (directions), 1 arg:
        0xC8,                         //     row/col addr, bottom-top refresh
        ST77XX_COLMOD,  1,              // 15: set color mode, 1 arg, no delay:
        0x05 },                       //     16-bit color

    Rcmd2green[] = {                  // 7735R init, part 2 (green tab only)
        2,                              //  2 commands in list:
        ST77XX_CASET,   4,              //  1: Column addr set, 4 args, no delay:
        0x00, 0x02,                   //     XSTART = 0
        0x00, 0x7F+0x02,              //     XEND = 127
        ST77XX_RASET,   4,              //  2: Row addr set, 4 args, no delay:
        0x00, 0x01,                   //     XSTART = 0
        0x00, 0x9F+0x01 },            //     XEND = 159
    Rcmd2green144[] = {               // 7735R init, part 2 (green 1.44 tab)
        2,                              //  2 commands in list:
        ST77XX_CASET,   4,              //  1: Column addr set, 4 args, no delay:
        0x00, 0x00,                   //     XSTART = 0
        0x00, 0x7F,                   //     XEND = 127
        ST77XX_RASET,   4,              //  2: Row addr set, 4 args, no delay:
        0x00, 0x00,                   //     XSTART = 0
        0x00, 0x7F },                 //     XEND = 127
    Rcmd3[] = {                       // 7735R init, part 3 (red or green tab)
        4,                              //  4 commands in list:
        ST7735_GMCTRP1, 16      ,       //  1: Gamma Adjustments (pos. polarity), 16 args + delay:
        0x02, 0x1c, 0x07, 0x12,       //     (Not entirely necessary, but provides
        0x37, 0x32, 0x29, 0x2d,       //      accurate colors)
        0x29, 0x25, 0x2B, 0x39,
        0x00, 0x01, 0x03, 0x10,
        ST7735_GMCTRN1, 16      ,       //  2: Gamma Adjustments (neg. polarity), 16 args + delay:
        0x03, 0x1d, 0x07, 0x06,       //     (Not entirely necessary, but provides
        0x2E, 0x2C, 0x29, 0x2D,       //      accurate colors)
        0x2E, 0x2E, 0x37, 0x3F,
        0x00, 0x00, 0x02, 0x10,
        ST77XX_NORON,     ST_CMD_DELAY, //  3: Normal display on, no args, w/delay
        10,                           //     10 ms delay
        ST77XX_DISPON,    ST_CMD_DELAY, //  4: Main screen turn on, no args w/delay
        100 };                        //     100 ms delay        

static const char *TAG = "st7735";

void ST7735::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SPI ST7735...");
  this->spi_setup();
  this->dc_pin_->setup();  // OUTPUT

  this->init_reset_();
  
  this->displayInit(Rcmd1);
  this->displayInit(Rcmd2green);
  this->displayInit(Rcmd3);
  this->writecommand(ST7735_INVON);

  delay(120);

  this->writecommand(ST7735_DISPON);    //Display on
  delay(120);
  
  this->init_internal_(this->get_buffer_length_());
  memset(this->buffer_, 0x00, this->get_buffer_length_());
}

void ST7735::write_display_data() {
  
  uint16_t offsetx = 26; 
  uint16_t offsety = 1; 

  uint16_t x1 = offsetx;
  uint16_t x2 = x1 + get_width_internal()-1;
  uint16_t y1 = offsety;
  uint16_t y2 = y1 + get_height_internal()-1;
	
	this->enable();

	// set column(x) address
	this->dc_pin_->digital_write(false);
	this->write_byte(ST77XX_CASET);
	this->dc_pin_->digital_write(true);
	this->spi_master_write_addr(x1, x2);
	
	// set Page(y) address
	this->dc_pin_->digital_write(false);
	this->write_byte(ST77XX_RASET);
	this->dc_pin_->digital_write(true);
	this->spi_master_write_addr(y1, y2);

	//  Memory Write
	this->dc_pin_->digital_write(false);
	this->write_byte(ST77XX_RAMWR);
	this->dc_pin_->digital_write(true);

	this->write_array(this->buffer_, this->get_buffer_length_());

	this->disable();
}

void ST7735::spi_master_write_addr(uint16_t addr1, uint16_t addr2)
{
	static uint8_t Byte[4];
	Byte[0] = (addr1 >> 8) & 0xFF;
	Byte[1] = addr1 & 0xFF;
	Byte[2] = (addr2 >> 8) & 0xFF;
	Byte[3] = addr2 & 0xFF;

	
	this->dc_pin_->digital_write(true);
	this->write_array(Byte, 4);
	
}

void ST7735::spi_master_write_color(uint16_t color, uint16_t size)
{
	static uint8_t Byte[1024];
	int index = 0;
	for(int i=0;i<size;i++) {
		Byte[index++] = (color >> 8) & 0xFF;
		Byte[index++] = color & 0xFF;
	}

	this->dc_pin_->digital_write(true);
	return write_array(Byte, size*2);

}


void ST7735::dump_config() {
  LOG_DISPLAY("", "SPI ST7735", this);
  LOG_PIN("  CS Pin: ", this->cs_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_UPDATE_INTERVAL(this);
}

float ST7735::get_setup_priority() const { 
	return setup_priority::PROCESSOR; 
}

void ST7735::update() {
  this->do_update_();
  this->write_display_data();
}

void ST7735::loop() {

}

int ST7735::get_width_internal() {
	return 80;
}

int ST7735::get_height_internal() {
	return 160;
}

size_t ST7735::get_buffer_length_() {
  return size_t(this->get_width_internal()) * size_t(this->get_height_internal()) * 2;
}


void HOT ST7735::draw_absolute_pixel_internal(int x, int y, Color color) {
	if (x >= this->get_width_internal() || x < 0 || y >= this->get_height_internal() || y < 0)
		return;

  auto color565 = color.to_rgb_565();
  uint16_t pos = (x + y * this->get_width_internal()) * 2;
  this->buffer_[pos++] = (color565 >> 8) & 0xff;
  this->buffer_[pos] = color565 & 0xff;
}

void ST7735::displayInit(const uint8_t *addr) {

  uint8_t  numCommands, cmd, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    cmd = pgm_read_byte(addr++);         // Read command
    numArgs  = pgm_read_byte(addr++);    // Number of args to follow
    ms       = numArgs & ST_CMD_DELAY;   // If hibit set, delay follows args
    numArgs &= ~ST_CMD_DELAY;            // Mask out delay bit
    this->sendcommand(cmd, addr, numArgs);
    addr += numArgs;

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }
}

void ST7735::init_reset_() {
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(true);
    delay(1);
    // Trigger Reset
    this->reset_pin_->digital_write(false);
    delay(10);
    // Wake up
    this->reset_pin_->digital_write(true);
  }
}

void ST7735::writecommand(uint8_t value) {
  this->enable();
  this->dc_pin_->digital_write(false);
  this->write_byte(value);
  this->dc_pin_->digital_write(true);
  this->disable();
}

void ST7735::writedata(uint8_t value) {
  this->dc_pin_->digital_write(true);
  this->enable();
  this->write_byte(value);
  this->disable();
}

void ST7735::sendcommand(uint8_t cmd, const uint8_t* dataBytes, uint8_t numDataBytes) { 
  this->writecommand(cmd);
  this->senddata(dataBytes,numDataBytes);
}

void ST7735::senddata(const uint8_t* dataBytes, uint8_t numDataBytes) {
  this->dc_pin_->digital_write(true); //pull DC high to indicate data
  this->cs_->digital_write(false);
  this->enable();
  for (uint8_t i=0; i<numDataBytes; i++) {
    this->transfer_byte(pgm_read_byte(dataBytes++));  //write byte - SPI library
  }
  this->cs_->digital_write(true);
  this->disable();

}

}  // namespace st7735
}  // namespace esphome
