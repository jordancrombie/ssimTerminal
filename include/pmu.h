/**
 * @file pmu.h
 * @brief Power Management Unit driver for ssimTerminal
 *
 * The AXP2101 PMU controls power rails for the display, touch, and other peripherals.
 * Must be initialized before display_init() to ensure display has power.
 *
 * For boards without AXP2101 (like ESP32-S3-Touch-LCD-7), stub functions are provided.
 */

#ifndef PMU_H
#define PMU_H

#include <Arduino.h>
#include "pins_config.h"

/**
 * @brief Initialize the PMU
 *
 * Configures power rails for display and other peripherals.
 * Must be called before display_init().
 *
 * @return true if initialization successful (or no PMU present), false on error
 */
bool pmu_init();

/**
 * @brief Get battery percentage (0-100%)
 * @return Battery percentage or -1 if unavailable
 */
int pmu_get_battery_percent();

/**
 * @brief Check if USB power is connected
 * @return true if USB/VBUS is present
 */
bool pmu_is_usb_connected();

/**
 * @brief Check if battery is charging
 * @return true if currently charging
 */
bool pmu_is_charging();

/**
 * @brief Get battery voltage in mV
 * @return Battery voltage in millivolts
 */
uint16_t pmu_get_battery_voltage();

#endif // PMU_H
