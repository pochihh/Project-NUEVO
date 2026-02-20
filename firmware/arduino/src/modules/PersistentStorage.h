/**
 * @file PersistentStorage.h
 * @brief Non-volatile storage module using Arduino EEPROM
 *
 * The Arduino Mega 2560 has 4096 bytes (4 KB) of internal EEPROM.
 * EEPROM survives power-off and resets — data written once stays until
 * explicitly overwritten. The hardware supports approximately 100,000
 * write cycles per byte before wear-out, so avoid writing every loop().
 *
 * This module centralises all EEPROM access for the firmware. All other
 * modules that need persistent data should use this API instead of calling
 * EEPROM.put() / EEPROM.get() directly.
 *
 * EEPROM Layout (v1 — 36 bytes total, starting at address 0):
 * ─────────────────────────────────────────────────────────────
 *  Byte  0– 3  magic        uint32_t   0xDEAD2026 when layout is valid
 *  Byte  4     version      uint8_t    Layout version (currently 1)
 *  Byte  5– 7  reserved     uint8_t×3  Set to 0
 *  Byte  8–11  wheelDiamMm  float      Wheel diameter in mm (0 = not saved)
 *  Byte 12–15  wheelBaseMm  float      Wheel base in mm (0 = not saved)
 *  Byte 16     magCalValid  uint8_t    1 = valid mag calibration stored
 *  Byte 17–19  reserved     uint8_t×3  Set to 0
 *  Byte 20–23  magOffsetX   float      Mag hard-iron offset X (µT)
 *  Byte 24–27  magOffsetY   float      Mag hard-iron offset Y (µT)
 *  Byte 28–31  magOffsetZ   float      Mag hard-iron offset Z (µT)
 * ─────────────────────────────────────────────────────────────
 *
 * Usage:
 *   PersistentStorage::init();         // call once in setup()
 *
 *   // Write
 *   PersistentStorage::setWheelDiameter(65.0f);
 *
 *   // Read
 *   float d;
 *   if (PersistentStorage::getWheelDiameter(d)) { ... }
 *
 *   // Erase everything
 *   PersistentStorage::reset();
 */

#ifndef PERSISTENTSTORAGE_H
#define PERSISTENTSTORAGE_H

#include <Arduino.h>
#include <EEPROM.h>
#include <stdint.h>

// ============================================================================
// EEPROM LAYOUT CONSTANTS
// These are exposed so test sketches can read raw bytes for educational dumps.
// Production code should use the typed get/set methods below.
// ============================================================================

#define PS_MAGIC            0xDEAD2026UL
#define PS_VERSION          1
#define PS_BASE_ADDR        0

// Field offsets from PS_BASE_ADDR
#define PS_OFF_MAGIC        0   // uint32_t
#define PS_OFF_VERSION      4   // uint8_t
// 3 bytes reserved at 5–7
#define PS_OFF_WHEEL_DIAM   8   // float
#define PS_OFF_WHEEL_BASE   12  // float
#define PS_OFF_MAG_VALID    16  // uint8_t
// 3 bytes reserved at 17–19
#define PS_OFF_MAG_X        20  // float
#define PS_OFF_MAG_Y        24  // float
#define PS_OFF_MAG_Z        28  // float

#define PS_LAYOUT_SIZE      32  // total bytes used

// ============================================================================
// PERSISTENT STORAGE CLASS (Static)
// ============================================================================

class PersistentStorage {
public:
    /**
     * @brief Initialize and validate EEPROM storage
     *
     * Reads the magic number. If it matches, the stored layout is treated as
     * valid. If not (first boot or after reset()), the storage is initialised
     * with default values and the magic number is written.
     *
     * Always call this once in setup() before any get/set calls.
     */
    static void init();

    /**
     * @brief Erase all stored data and restore defaults
     *
     * Writes 0xFF over all managed bytes, then re-initialises with defaults.
     * Useful during development or to factory-reset a robot unit.
     */
    static void reset();

    /**
     * @brief Returns true if EEPROM contains a valid (non-corrupted) layout
     */
    static bool isValid() { return valid_; }

    /**
     * @brief Returns the stored layout version number
     */
    static uint8_t getVersion();

    // ========================================================================
    // WHEEL GEOMETRY
    // ========================================================================

    /**
     * @brief Load wheel diameter from EEPROM
     * @param mm Output — wheel diameter in millimetres
     * @return False if no value has been saved yet (mm unchanged)
     */
    static bool getWheelDiameter(float& mm);

    /**
     * @brief Persist wheel diameter to EEPROM
     * @param mm Wheel diameter in millimetres
     */
    static void setWheelDiameter(float mm);

    /**
     * @brief Load wheel base from EEPROM
     * @param mm Output — wheel base (centre-to-centre) in millimetres
     * @return False if no value has been saved yet (mm unchanged)
     */
    static bool getWheelBase(float& mm);

    /**
     * @brief Persist wheel base to EEPROM
     * @param mm Wheel base in millimetres
     */
    static void setWheelBase(float mm);

    // ========================================================================
    // MAGNETOMETER CALIBRATION
    // ========================================================================

    /**
     * @brief Load mag calibration offsets from EEPROM
     * @param ox, oy, oz Output — hard-iron offsets in µT
     * @return False if no calibration has been saved
     */
    static bool getMagCalibration(float& ox, float& oy, float& oz);

    /**
     * @brief Persist mag calibration offsets to EEPROM
     * @param ox, oy, oz Hard-iron offsets in µT
     */
    static void setMagCalibration(float ox, float oy, float oz);

    /**
     * @brief Clear saved mag calibration (marks it invalid without erasing bytes)
     */
    static void clearMagCalibration();

    /**
     * @brief Returns true if a valid mag calibration is stored
     */
    static bool hasMagCalibration();

private:
    static bool valid_;     // true if magic number matched on init()

    // Write the magic + version header (marks storage as valid)
    static void writeHeader();
};

#endif // PERSISTENTSTORAGE_H
