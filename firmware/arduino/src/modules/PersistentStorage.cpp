/**
 * @file PersistentStorage.cpp
 * @brief Implementation of EEPROM-backed persistent storage
 *
 * See PersistentStorage.h for the full EEPROM layout and usage notes.
 *
 * Key Arduino EEPROM facts for students:
 *  - EEPROM.put() only writes bytes that have changed (automatic wear-leveling)
 *  - EEPROM.get() reads without any wear effect — read as often as needed
 *  - Arduino Mega 2560: 4096 bytes available at addresses 0x000–0xFFF
 *  - ~100,000 write cycles per byte before wear-out
 *    → at 1 write/second that's ~27 hours; at 1 write/minute that's ~69 days
 *    → never write to EEPROM inside loop() without rate-limiting
 */

#include "PersistentStorage.h"

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

bool PersistentStorage::valid_ = false;

// ============================================================================
// INIT / RESET
// ============================================================================

void PersistentStorage::init() {
    uint32_t magic = 0;
    EEPROM.get(PS_BASE_ADDR + PS_OFF_MAGIC, magic);

    if (magic == PS_MAGIC) {
        valid_ = true;
        return;
    }

    // First boot (or after reset()): write header + zero all fields
    writeHeader();

    // Default wheel geometry: 0.0 = "not configured"
    float zero = 0.0f;
    EEPROM.put(PS_BASE_ADDR + PS_OFF_WHEEL_DIAM, zero);
    EEPROM.put(PS_BASE_ADDR + PS_OFF_WHEEL_BASE, zero);

    // No mag calibration by default
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_VALID, (uint8_t)0);
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_X, zero);
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_Y, zero);
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_Z, zero);

    valid_ = true;
}

void PersistentStorage::reset() {
    // Invalidate by writing 0xFF over magic; EEPROM.update only writes changed bytes
    uint32_t invalid = 0xFFFFFFFFUL;
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAGIC, invalid);

    // Zero all managed bytes
    for (int i = 1; i < PS_LAYOUT_SIZE; i++) {
        EEPROM.update(PS_BASE_ADDR + i, 0x00);
    }

    // Re-initialise with defaults
    valid_ = false;
    init();
}

// ============================================================================
// VERSION
// ============================================================================

uint8_t PersistentStorage::getVersion() {
    if (!valid_) return 0;
    uint8_t v = 0;
    EEPROM.get(PS_BASE_ADDR + PS_OFF_VERSION, v);
    return v;
}

// ============================================================================
// WHEEL GEOMETRY
// ============================================================================

bool PersistentStorage::getWheelDiameter(float& mm) {
    if (!valid_) return false;
    float v = 0.0f;
    EEPROM.get(PS_BASE_ADDR + PS_OFF_WHEEL_DIAM, v);
    if (v <= 0.0f) return false;  // 0 = not saved
    mm = v;
    return true;
}

void PersistentStorage::setWheelDiameter(float mm) {
    if (!valid_) return;
    EEPROM.put(PS_BASE_ADDR + PS_OFF_WHEEL_DIAM, mm);
}

bool PersistentStorage::getWheelBase(float& mm) {
    if (!valid_) return false;
    float v = 0.0f;
    EEPROM.get(PS_BASE_ADDR + PS_OFF_WHEEL_BASE, v);
    if (v <= 0.0f) return false;
    mm = v;
    return true;
}

void PersistentStorage::setWheelBase(float mm) {
    if (!valid_) return;
    EEPROM.put(PS_BASE_ADDR + PS_OFF_WHEEL_BASE, mm);
}

// ============================================================================
// MAGNETOMETER CALIBRATION
// ============================================================================

bool PersistentStorage::getMagCalibration(float& ox, float& oy, float& oz) {
    if (!valid_) return false;
    uint8_t valid = 0;
    EEPROM.get(PS_BASE_ADDR + PS_OFF_MAG_VALID, valid);
    if (valid != 1) return false;
    EEPROM.get(PS_BASE_ADDR + PS_OFF_MAG_X, ox);
    EEPROM.get(PS_BASE_ADDR + PS_OFF_MAG_Y, oy);
    EEPROM.get(PS_BASE_ADDR + PS_OFF_MAG_Z, oz);
    return true;
}

void PersistentStorage::setMagCalibration(float ox, float oy, float oz) {
    if (!valid_) return;
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_X, ox);
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_Y, oy);
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_Z, oz);
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_VALID, (uint8_t)1);  // mark valid last
}

void PersistentStorage::clearMagCalibration() {
    if (!valid_) return;
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_VALID, (uint8_t)0);
}

bool PersistentStorage::hasMagCalibration() {
    if (!valid_) return false;
    uint8_t v = 0;
    EEPROM.get(PS_BASE_ADDR + PS_OFF_MAG_VALID, v);
    return (v == 1);
}

// ============================================================================
// INTERNAL
// ============================================================================

void PersistentStorage::writeHeader() {
    uint32_t magic   = PS_MAGIC;
    uint8_t  version = PS_VERSION;
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAGIC,   magic);
    EEPROM.put(PS_BASE_ADDR + PS_OFF_VERSION, version);
}
