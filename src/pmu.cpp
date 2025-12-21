/**
 * @file pmu.cpp
 * @brief PMU driver implementation
 *
 * For boards with AXP2101 (like ESP32-S3-Touch-AMOLED-1.8), this controls power rails.
 * For boards without I2C PMU (like ESP32-S3-Touch-LCD-7), stub functions are provided.
 */

#include "pmu.h"
#include "pins_config.h"
#include <Wire.h>

#if HAS_PMU_AXP2101

// =============================================================================
// AXP2101 PMU Implementation
// =============================================================================

#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>

static XPowersAXP2101 pmu;
static bool pmu_initialized = false;

bool pmu_init() {
    Serial.println("Initializing AXP2101 PMU...");

    // Initialize PMU with existing Wire instance
    if (!pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
        Serial.println("ERROR: AXP2101 PMU not found!");
        return false;
    }

    Serial.printf("PMU Chip ID: 0x%02X\n", pmu.getChipID());

    // Disable unused power channels to save power
    pmu.disableDC2();
    pmu.disableDC3();
    pmu.disableDC4();
    pmu.disableDC5();

    // DCDC1 - Main 3.3V rail (ESP32 core, typically always on)
    pmu.setDC1Voltage(3300);
    pmu.enableDC1();

    // ALDO1 - Often used for display power (3.3V)
    pmu.setALDO1Voltage(3300);
    pmu.enableALDO1();
    Serial.println("  ALDO1: 3.3V enabled (display power)");

    // ALDO2 - Touch/sensor power
    pmu.setALDO2Voltage(3300);
    pmu.enableALDO2();
    Serial.println("  ALDO2: 3.3V enabled (touch/sensor)");

    // ALDO3 - Backup/additional display power
    pmu.setALDO3Voltage(3300);
    pmu.enableALDO3();
    Serial.println("  ALDO3: 3.3V enabled");

    // ALDO4 - Audio codec or other peripherals
    pmu.setALDO4Voltage(3300);
    pmu.enableALDO4();
    Serial.println("  ALDO4: 3.3V enabled");

    // BLDO1 - Often display logic power
    pmu.setBLDO1Voltage(3300);
    pmu.enableBLDO1();
    Serial.println("  BLDO1: 3.3V enabled");

    // BLDO2 - Backup
    pmu.setBLDO2Voltage(3300);
    pmu.enableBLDO2();
    Serial.println("  BLDO2: 3.3V enabled");

    // Wait for power rails to stabilize
    delay(50);

    // Configure charging parameters
    pmu.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);
    pmu.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);
    pmu.enableCellbatteryCharge();

    // Enable ADC for battery monitoring
    pmu.enableBattDetection();
    pmu.enableVbusVoltageMeasure();
    pmu.enableBattVoltageMeasure();

    pmu_initialized = true;
    Serial.println("AXP2101 PMU initialized - all power rails enabled");

    // Print current power status
    Serial.printf("  Battery: %dmV, USB: %s, Charging: %s\n",
                  pmu.getBattVoltage(),
                  pmu.isVbusIn() ? "connected" : "disconnected",
                  pmu.isCharging() ? "yes" : "no");

    return true;
}

int pmu_get_battery_percent() {
    if (!pmu_initialized) return -1;
    return pmu.getBatteryPercent();
}

bool pmu_is_usb_connected() {
    if (!pmu_initialized) return false;
    return pmu.isVbusIn();
}

bool pmu_is_charging() {
    if (!pmu_initialized) return false;
    return pmu.isCharging();
}

uint16_t pmu_get_battery_voltage() {
    if (!pmu_initialized) return 0;
    return pmu.getBattVoltage();
}

#else

// =============================================================================
// Stub Implementation (for boards without I2C PMU)
// =============================================================================

bool pmu_init() {
    Serial.println("PMU: No AXP2101 on this board (power controlled by CS8501 or direct)");
    return true;  // Return success - no PMU is not an error
}

int pmu_get_battery_percent() {
    return -1;  // Not available
}

bool pmu_is_usb_connected() {
    return true;  // Assume USB connected (no way to detect)
}

bool pmu_is_charging() {
    return false;  // Unknown
}

uint16_t pmu_get_battery_voltage() {
    return 0;  // Not available
}

#endif // HAS_PMU_AXP2101
