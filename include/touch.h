/**
 * @file touch.h
 * @brief Touch driver for FT3168 capacitive touch controller
 *
 * The FT3168 is a FocalTech capacitive touch controller that communicates
 * over I2C. It shares the I2C bus with other peripherals on the board.
 */

#ifndef TOUCH_H
#define TOUCH_H

#include <Arduino.h>
#include <lvgl.h>

/**
 * @brief Initialize the touch controller
 * @return true if successful, false otherwise
 */
bool touch_init();

/**
 * @brief Read current touch state
 * @param x Pointer to store X coordinate (0-367)
 * @param y Pointer to store Y coordinate (0-447)
 * @param pressed Pointer to store pressed state
 * @return true if read successful, false otherwise
 */
bool touch_read(uint16_t *x, uint16_t *y, bool *pressed);

/**
 * @brief Get the LVGL input device driver
 * @return Pointer to the input device
 */
lv_indev_t* touch_get_driver();

#endif // TOUCH_H
