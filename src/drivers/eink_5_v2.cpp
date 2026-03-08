/*
eink_5v2.cpp
Inkplate 5 V2 ESP-IDF

Ported from the Arduino Library Inkplate5V2.cpp:

David Zovko, Borna Biro, Denis Vajak, Zvonimir Haramustek @ Soldered
https://github.com/SolderedElectronics/Inkplate-Arduino-library

For support, please reach over forums: forum.e-radionica.com/en
For more info about the product, please check: www.inkplate.io

This code is released under the GNU Lesser General Public License v3.0:
https://www.gnu.org/licenses/lgpl-3.0.en.html
Please review the LICENSE file included with this example.
If you have any questions about licensing, please contact techsupport@e-radionica.com
Distributed as-is; no warranty is given.
*/

#if INKPLATE_5V2

#define __EINK5V2__ 1
#include "eink_5_v2.hpp"
#include "esp_log.h"

#include "esp.hpp"

#include <iostream>

// Waveform table for Inkplate 5 V2 (from Inkplate5V2.h).
// 9 phases per gray level (last element in each row is 0 / unused terminator).
const uint8_t EInk5V2::WAVEFORM_3BIT[8][9] = {
  {0, 0, 1, 1, 2, 1, 1, 1, 0}, {1, 1, 2, 2, 1, 2, 1, 1, 0},
  {0, 1, 2, 2, 1, 1, 2, 1, 0}, {0, 0, 1, 1, 1, 1, 1, 2, 0},
  {1, 2, 1, 2, 1, 1, 1, 2, 0}, {0, 1, 1, 1, 2, 0, 1, 2, 0},
  {1, 1, 1, 2, 2, 2, 1, 2, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0}};

const uint8_t EInk5V2::LUT2[16] = {
  0xAA, 0xA9, 0xA6, 0xA5, 0x9A, 0x99, 0x96, 0x95,
  0x6A, 0x69, 0x66, 0x65, 0x5A, 0x59, 0x56, 0x55 };

const uint8_t EInk5V2::LUTW[16] = {
  0xFF, 0xFE, 0xFB, 0xFA, 0xEF, 0xEE, 0xEB, 0xEA,
  0xBF, 0xBE, 0xBB, 0xBA, 0xAF, 0xAE, 0xAB, 0xAA };

const uint8_t EInk5V2::LUTB[16] = {
  0xFF, 0xFD, 0xF7, 0xF5, 0xDF, 0xDD, 0xD7, 0xD5,
  0x7F, 0x7D, 0x77, 0x75, 0x5F, 0x5D, 0x57, 0x55 };

bool
EInk5V2::setup()
{
  if (initialized) return true;

  ESP_LOGD(TAG, "Initializing...");

  wire.setup();

  if (!io_expander_int.setup()) {
    ESP_LOGE(TAG, "Initialization not completed (Main MCP Issue).");
    return false;
  }
  else {
    ESP_LOGD(TAG, "MCP initialized.");
  }

  io_expander_ext.setup();

  Wire::enter();

  io_expander_int.set_direction(VCOM,         IOExpander::PinMode::OUTPUT);
  io_expander_int.set_direction(PWRUP,        IOExpander::PinMode::OUTPUT);
  io_expander_int.set_direction(WAKEUP,       IOExpander::PinMode::OUTPUT);
  io_expander_int.set_direction(GPIO0_ENABLE, IOExpander::PinMode::OUTPUT);
  io_expander_int.digital_write(GPIO0_ENABLE, IOExpander::SignalLevel::HIGH);
 
  wakeup_set();

  wire_device = new WireDevice(PWRMGR_ADDRESS);
  if ((wire_device == nullptr) || !wire_device->is_initialized()) {
    ESP_LOGE(TAG, "Setup error: %s", wire_device == nullptr ? "NULL Device!" : "Not initialized!");
    return false;
  }

  uint8_t pgm[] = {
    0x09,       // cmd
    0b00011011, // Power up seq.
    0b00000000, // Power up delay (3mS per rail)
    0b00011011, // Power down seq.
    0b00000000  // Power down delay (6mS per rail)
  };

  ESP::delay(1);
  wire_device->write(pgm, sizeof(pgm));

  ESP::delay(1);

  wakeup_clear();


  // Set all pins of seconds I/O expander to outputs, low.
  // For some reason, it draw more current in deep sleep when pins are set as inputs...
  /************* TODO *************
    // Enable pull down resistors on SPI lines to reduce power consumption in deep sleep.
    pinMode(12, INPUT_PULLDOWN);
    pinMode(13, INPUT_PULLDOWN);
    pinMode(14, INPUT_PULLDOWN);
    pinMode(15, INPUT_PULLDOWN);  
  **************/
  //if (io_expander_ext.is_present()) {
  //  for (int i = 0; i < 15; i++) {
  //    io_expander_ext.set_direction((IOExpander::Pin) i, IOExpander::PinMode::OUTPUT);
  //    io_expander_ext.digital_write((IOExpander::Pin) i, IOExpander::SignalLevel::LOW);
  //  }
  //}

  // For same reason, unused pins of first I/O expander have to be also set as outputs, low.
  io_expander_int.set_direction(IOExpander::Pin::IOPIN_11, IOExpander::PinMode::OUTPUT);
  io_expander_int.set_direction(IOExpander::Pin::IOPIN_12, IOExpander::PinMode::OUTPUT);
  io_expander_int.set_direction(IOExpander::Pin::IOPIN_13, IOExpander::PinMode::OUTPUT);
  io_expander_int.set_direction(IOExpander::Pin::IOPIN_14, IOExpander::PinMode::OUTPUT);
  io_expander_int.set_direction(IOExpander::Pin::IOPIN_15, IOExpander::PinMode::OUTPUT);

  io_expander_int.digital_write(IOExpander::Pin::IOPIN_11, IOExpander::SignalLevel::LOW);
  io_expander_int.digital_write(IOExpander::Pin::IOPIN_12, IOExpander::SignalLevel::LOW);
  io_expander_int.digital_write(IOExpander::Pin::IOPIN_13, IOExpander::SignalLevel::LOW);
  io_expander_int.digital_write(IOExpander::Pin::IOPIN_14, IOExpander::SignalLevel::LOW);
  io_expander_int.digital_write(IOExpander::Pin::IOPIN_15, IOExpander::SignalLevel::LOW);

  // CONTROL PINS (no GPIO 0 on Inkplate 5 V2)
  gpio_set_direction(GPIO_NUM_2,  GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT);

  io_expander_int.set_direction(OE,   IOExpander::PinMode::OUTPUT);
  io_expander_int.set_direction(GMOD, IOExpander::PinMode::OUTPUT);
  io_expander_int.set_direction(SPV,  IOExpander::PinMode::OUTPUT);

  // Battery voltage switch MOSFET
  //io_expander_int.set_direction(IOExpander::Pin::IOPIN_9, IOExpander::PinMode::OUTPUT);

  if (i2s_comms.is_ready()) { // instanciated through the EInk constructor
    //i2s_comms.init(4);
    i2s_comms.init();
  }
  else {
    ESP_LOGE(TAG, "I2SComms is not ready!!!");
    return false;
  }

  d_memory_new = new_frame_buffer_1bit();
  p_buffer     = (uint8_t *) malloc(BITMAP_SIZE_1BIT * 2);

  // 9 waveform phases for Inkplate 5 V2 (vs 8 for Inkplate 6)
  GLUT  = (uint32_t *) malloc(256 * 9 * sizeof(uint32_t));
  GLUT2 = (uint32_t *) malloc(256 * 9 * sizeof(uint32_t));

  ESP_LOGD(TAG, "Memory allocation for bitmap buffers.");
  ESP_LOGD(TAG, "d_memory_new: %08x p_buffer: %08x.", (unsigned int)d_memory_new, (unsigned int)p_buffer);

  Wire::leave();

  if ((d_memory_new == nullptr) ||
      (p_buffer     == nullptr) ||
      (GLUT         == nullptr) ||
      (GLUT2        == nullptr)) {
    return false;
  }

  d_memory_new->clear();
  memset(p_buffer, 0, BITMAP_SIZE_1BIT * 2);

  // Build the grayscale LUTs — 9 phases for Inkplate 5 V2.
  for (int j = 0; j < 9; j++) {
    for (uint32_t i = 0; i < 256; i++) {
      GLUT [(j << 8) + i] =  (WAVEFORM_3BIT[i & 0x07][j] << 2) | (WAVEFORM_3BIT[(i >> 4) & 0x07][j]);
      GLUT2[(j << 8) + i] = ((WAVEFORM_3BIT[i & 0x07][j] << 2) | (WAVEFORM_3BIT[(i >> 4) & 0x07][j])) << 4;
    }
  }

  initialized = true;

  return true;
}

/**
 * @brief 1-bit (black & white) full display update.
 *
 * Inkplate 5 V2 specific differences from Inkplate 6:
 *   - Cleaning cycles use repeat count 11 (not 21/12)
 *   - 3 repetitions of black-pixel phase (not 4)
 *   - Row-flipped scan: each row is scanned forward in memory but pixels
 *     within each row are sent in reverse order.
 *   - No vscan_start() call at end of update.
 */
void
EInk5V2::update(FrameBuffer1Bit & frame_buffer)
{
  ESP_LOGD(TAG, "1bit Update...");

  Wire::enter();

  if (!turn_on()) {
    Wire::leave();
    return;
  }

  // Clear display — fewer cycles than Inkplate 6
  clean(PixelState::WHITE,     1);
  clean(PixelState::BLACK,    11);
  clean(PixelState::DISCHARGE, 1);
  clean(PixelState::WHITE,    11);
  clean(PixelState::DISCHARGE, 1);
  clean(PixelState::BLACK,    11);
  clean(PixelState::DISCHARGE, 1);
  clean(PixelState::WHITE,    11);

  uint8_t * data = frame_buffer.get_data();

  volatile uint8_t * line_buffer = i2s_comms.get_line_buffer();

  // Write only black pixels — 3 repetitions (Inkplate 6 uses 4).
  for (int k = 0; k < 3; k++) {

    uint8_t * ptr = data;

    vscan_start();

    for (int i = 0; i < HEIGHT; i++) {

      // Row-flipped scan: start at end of current row, walk backwards.
      uint8_t * row_ptr = (ptr + LINE_SIZE_1BIT) - 1;

      for (int n = 0; n < (WIDTH / 4); n += 4) {
        uint8_t dram1 = *row_ptr--;
        uint8_t dram2 = *row_ptr--;
        line_buffer[n    ] = LUTB[(dram2 >> 4) & 0x0F];
        line_buffer[n + 1] = LUTB[ dram2       & 0x0F];
        line_buffer[n + 2] = LUTB[(dram1 >> 4) & 0x0F];
        line_buffer[n + 3] = LUTB[ dram1       & 0x0F];
        ptr += 2;
      }

      // Advance base pointer to next row.
      //ptr += LINE_SIZE_1BIT;

      i2s_comms.send_data();
      vscan_end();
    }

    ESP::delay_microseconds(230);
  }

  // Write both black and white pixels — 1 repetition.
  for (int k = 0; k < 1; k++) {
    uint8_t * ptr = data;

    vscan_start();

    for (int i = 0; i < HEIGHT; i++) {

      uint8_t * row_ptr = (ptr + LINE_SIZE_1BIT) - 1;

      for (int n = 0; n < (WIDTH / 4); n += 4) {
        uint8_t dram1 = *row_ptr--;
        uint8_t dram2 = *row_ptr--;
        line_buffer[n    ] = LUT2[(dram2 >> 4) & 0x0F];
        line_buffer[n + 1] = LUT2[ dram2       & 0x0F];
        line_buffer[n + 2] = LUT2[(dram1 >> 4) & 0x0F];
        line_buffer[n + 3] = LUT2[ dram1       & 0x0F];
        ptr += 2;
      }
      //ptr += LINE_SIZE_1BIT;

      i2s_comms.send_data();
      vscan_end();
    }

    ESP::delay_microseconds(230);
  }

  // Discharge sequence.
  {
    vscan_start();

    for (int i = 0; i < HEIGHT; i++) {

      for (int n = 0; n < (WIDTH / 4); n += 4) {
        line_buffer[n    ] = 0;
        line_buffer[n + 1] = 0;
        line_buffer[n + 2] = 0;
        line_buffer[n + 3] = 0;
      }

      i2s_comms.send_data();
      vscan_end();
    }

    ESP::delay_microseconds(230);
  }

  // Note: Inkplate 5 V2 does NOT call vscan_start() here (unlike Inkplate 6).
  turn_off();

  Wire::leave();

  memcpy(d_memory_new->get_data(), frame_buffer.get_data(), BITMAP_SIZE_1BIT);
  allow_partial();
}

/**
 * @brief 3-bit (8-level grayscale) full display update.
 *
 * Inkplate 5 V2 specific differences from Inkplate 6:
 *   - 9 waveform phases (not 8)
 *   - Cleaning cycles use repeat count 11 (not 21/12)
 *   - Row-flipped scan direction
 *   - Ends with clean(SKIP, 1) but no vscan_start()
 */
void
EInk5V2::update(FrameBuffer3Bit & frame_buffer)
{
  ESP_LOGD(TAG, "3bit Update...");

  Wire::enter();
  if (!turn_on()) {
    Wire::leave();
    return;
  }

  // Clear display.
  clean(PixelState::WHITE,     1);
  clean(PixelState::BLACK,    11);
  clean(PixelState::DISCHARGE, 1);
  clean(PixelState::WHITE,    11);
  clean(PixelState::DISCHARGE, 1);
  clean(PixelState::BLACK,    11);
  clean(PixelState::DISCHARGE, 1);
  clean(PixelState::WHITE,    11);

  uint8_t * data = frame_buffer.get_data();

  volatile uint8_t * line_buffer = i2s_comms.get_line_buffer();

  // 9 waveform phases for Inkplate 5 V2 (Inkplate 6 uses 8).
  for (int k = 0, kk = 0; k < 9; k++, kk += 256) {

    uint8_t * dp = data;

    vscan_start();

    for (int i = 0; i < HEIGHT; i++) {

      // Row-flipped scan: walk backwards within each row.
      // Each row is LINE_SIZE_3BIT bytes. The 3-bit data is packed 2 pixels per byte.
      uint8_t * row_ptr = (dp + LINE_SIZE_3BIT) - 1;

      for (int j = 0; j < (WIDTH / 4); j += 4) {
        // Read pairs of bytes from the end of the row, going backwards.
        // row_ptr points at the odd byte (high pixel), row_ptr-1 at the even byte (low pixel).
        line_buffer[j + 2] = (GLUT2[kk + row_ptr[0]] | GLUT[kk + row_ptr[-1]]); row_ptr -= 2;
        line_buffer[j + 3] = (GLUT2[kk + row_ptr[0]] | GLUT[kk + row_ptr[-1]]); row_ptr -= 2;
        line_buffer[j    ] = (GLUT2[kk + row_ptr[0]] | GLUT[kk + row_ptr[-1]]); row_ptr -= 2;
        line_buffer[j + 1] = (GLUT2[kk + row_ptr[0]] | GLUT[kk + row_ptr[-1]]); row_ptr -= 2;
        dp += 8;
      }
      // Advance base pointer to next row.
      //dp += LINE_SIZE_3BIT;

      i2s_comms.send_data();

      vscan_end();
    }

    ESP::delay_microseconds(230);
  }

  // Discharge.
  clean(PixelState::SKIP, 1);

  // Note: Inkplate 5 V2 does NOT call vscan_start() here.
  turn_off();

  Wire::leave();
  block_partial();
}

/**
 * @brief Partial display update (1-bit mode only).
 *
 * Inkplate 5 V2 specific differences from Inkplate 6:
 *   - 4 repetitions (not 5)
 *   - uint32_t for position variable (buffer > 64K)
 *   - Row-flipped scan direction
 *   - No clean(SKIP, 1) + vscan_start() at end
 */
void
EInk5V2::partial_update(FrameBuffer1Bit & frame_buffer, bool force)
{
  if (!is_partial_allowed() && !force) {
    update(frame_buffer);
    return;
  }

  ESP_LOGD(TAG, "Partial update...");

  uint8_t * idata = frame_buffer.get_data();
  uint8_t * odata = d_memory_new->get_data();

  // Compute the difference buffer.
  // Walk backwards through the framebuffers (same as Arduino code).
  uint32_t n   = BITMAP_SIZE_1BIT * 2 - 1;
  uint32_t pos = BITMAP_SIZE_1BIT - 1;

  for (int i = 0; i < HEIGHT; i++) {
    for (int j = 0; j < LINE_SIZE_1BIT; j++) {
      uint8_t diffw =  odata[pos] & ~idata[pos];
      uint8_t diffb = ~odata[pos] &  idata[pos];
      pos--;
      p_buffer[n--] = LUTW[diffw >>   4] & (LUTB[diffb >>   4]);
      p_buffer[n--] = LUTW[diffw & 0x0F] & (LUTB[diffb & 0x0F]);
    }
  }

  Wire::enter();
  if (!turn_on()) {
    Wire::leave();
    return;
  }

  volatile uint8_t * line_buffer = i2s_comms.get_line_buffer();
  i2s_comms.init_lldesc();

  if (line_buffer == nullptr) return;

  // 4 repetitions for Inkplate 5 V2 (Inkplate 6 uses 5).
  for (int k = 0; k < 4; k++) {

    // Row-flipped scan through the p_buffer.
    // The p_buffer was filled in reverse order (backwards from end).
    // Each row has (WIDTH / 4) bytes in p_buffer.
    uint8_t * dp = p_buffer;

    vscan_start();

    for (int i = 0; i < HEIGHT; i++) {

      // Row-flipped: walk backwards within each row of the p_buffer.
      uint8_t * row_ptr = (dp + (WIDTH / 4)) - 1;

      for (int j = 0; j < (WIDTH / 4); j += 4) {
        line_buffer[j + 2] = *row_ptr--;
        line_buffer[j + 3] = *row_ptr--;
        line_buffer[j    ] = *row_ptr--;
        line_buffer[j + 1] = *row_ptr--;
      }

      dp += (WIDTH / 4);

      i2s_comms.send_data();

      vscan_end();
    }

    ESP::delay_microseconds(230);
  }

  clean(PixelState::DISCHARGE, 2);
  // Note: Inkplate 5 V2 does NOT call clean(SKIP) or vscan_start() here.

  turn_off();

  Wire::leave();
  memcpy(d_memory_new->get_data(), frame_buffer.get_data(), BITMAP_SIZE_1BIT);
}

/**
 * @brief Fill the entire screen with a single pixel state.
 *
 * Uses I2S DMA to send uniform line data for the entire display.
 */
void
EInk5V2::clean(PixelState pixel_state, uint8_t repeat_count)
{
  if (!turn_on()) return;

  volatile uint8_t * line_buffer = i2s_comms.get_line_buffer();
  for (int i = 0; i < (WIDTH / 4); i++) {
    line_buffer[i] = static_cast<uint8_t>(pixel_state);
  }

  i2s_comms.init_lldesc();

  for (int8_t k = 0; k < repeat_count; k++) {

    vscan_start();

    for (int i = 0; i < HEIGHT; i++) {
      i2s_comms.send_data();
      vscan_end();
    }

    ESP::delay_microseconds(230);
  }
}

#endif