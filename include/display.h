/**
 * @file display.h
 * @brief Display driver for Waveshare ESP32-S3-Touch-AMOLED-1.8
 *
 * Handles SH8601 AMOLED initialization via QSPI and LVGL integration.
 * Reset is controlled via TCA9554 I2C GPIO expander.
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <lvgl.h>

/**
 * @brief Initialize the display hardware and LVGL
 * @return true if successful, false otherwise
 */
bool display_init();

/**
 * @brief Set display brightness
 * @param brightness 0-255
 */
void display_set_brightness(uint8_t brightness);

/**
 * @brief Turn display on/off
 * @param on true to turn on, false to turn off
 */
void display_power(bool on);

/**
 * @brief Get the LVGL display driver
 * @return Pointer to the display driver
 */
lv_disp_t* display_get_driver();

#endif // DISPLAY_H
