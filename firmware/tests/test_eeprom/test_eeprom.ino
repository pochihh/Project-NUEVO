/**
 * @file test_eeprom.ino
 * @brief EEPROM persistent storage test
 *
 * This test demonstrates and verifies the PersistentStorage module, which uses
 * the Arduino Mega's internal EEPROM to save data that survives power-off.
 *
 * ─── Background: What is EEPROM? ─────────────────────────────────────────────
 * EEPROM (Electrically Erasable Programmable Read-Only Memory) is a type of
 * non-volatile memory built into the Arduino Mega 2560:
 *   - Size:        4096 bytes (4 KB)
 *   - Survives:    power-off, reset, and firmware uploads
 *   - Erased by:   explicitly writing new values, or using reset()
 *   - Endurance:   ~100,000 write cycles per byte
 *                  (100,000 writes ÷ 1 write/minute ≈ 69 days of continuous writing)
 *   - Read speed:  fast, no wear effect — read freely
 *   - Write speed: slow (~3.3 ms/byte) — avoid writing every loop()
 *
 * Arduino's EEPROM library uses EEPROM.put() / EEPROM.get() which automatically
 * handle multi-byte types (float, uint32_t, structs). EEPROM.put() also performs
 * wear-leveling: it only physically writes bytes that have changed, which
 * dramatically extends EEPROM lifespan.
 *
 * ─── Test Procedure ───────────────────────────────────────────────────────────
 * Run 1 (first power-on):
 *   - Storage is uninitialised → test writes known values
 *   - Verifies immediate read-back matches what was written
 *   - Instructs you to power off the Arduino
 *
 * Run 2 (after power cycle):
 *   - Reads back the same values written in Run 1
 *   - Confirms data survived the power-off
 *   - This proves EEPROM is truly non-volatile
 *
 * Run 3+ (reset test):
 *   - Type 'r' to erase EEPROM and start fresh
 *   - Next run will be treated as Run 1 again
 *
 * ─── Hardware ─────────────────────────────────────────────────────────────────
 * No external hardware needed. Just the Arduino Mega 2560.
 * Open Serial Monitor at 115200 baud.
 *
 * ─── Commands ─────────────────────────────────────────────────────────────────
 *   d  - Dump raw EEPROM bytes for the managed region
 *   r  - Reset (erase) all stored data — next reboot is Run 1 again
 *   w  - Write test values again (useful for re-running without power cycle)
 *   h  - Show this help
 */

#include "src/modules/PersistentStorage.h"

// ============================================================================
// TEST VALUES — written on Run 1, verified on Run 2
// ============================================================================

static const float TEST_WHEEL_DIAM  = 65.0f;   // mm — typical robot wheel
static const float TEST_WHEEL_BASE  = 150.0f;  // mm — typical robot width
static const float TEST_MAG_OX      = -12.34f; // µT — example hard-iron offset
static const float TEST_MAG_OY      =  45.67f;
static const float TEST_MAG_OZ      =  -8.90f;

// Tolerance for float comparisons (EEPROM stores exact IEEE 754, so this should be 0)
static const float FLOAT_EPSILON = 0.001f;

// ============================================================================
// HELPERS
// ============================================================================

bool floatClose(float a, float b) {
    return abs(a - b) < FLOAT_EPSILON;
}

void printPass(const char* name) {
    Serial.print(F("  [PASS] "));
    Serial.println(name);
}

void printFail(const char* name, float expected, float got) {
    Serial.print(F("  [FAIL] "));
    Serial.print(name);
    Serial.print(F(": expected "));
    Serial.print(expected, 4);
    Serial.print(F(", got "));
    Serial.println(got, 4);
}

void printSeparator() {
    Serial.println(F("----------------------------------------"));
}

// ============================================================================
// WRITE TEST VALUES
// ============================================================================

void writeTestValues() {
    Serial.println(F("[Write] Writing test values to EEPROM..."));

    PersistentStorage::setWheelDiameter(TEST_WHEEL_DIAM);
    Serial.print(F("  wheelDiameter = "));
    Serial.print(TEST_WHEEL_DIAM, 2);
    Serial.println(F(" mm"));

    PersistentStorage::setWheelBase(TEST_WHEEL_BASE);
    Serial.print(F("  wheelBase     = "));
    Serial.print(TEST_WHEEL_BASE, 2);
    Serial.println(F(" mm"));

    PersistentStorage::setMagCalibration(TEST_MAG_OX, TEST_MAG_OY, TEST_MAG_OZ);
    Serial.print(F("  magOffset     = ("));
    Serial.print(TEST_MAG_OX, 2);
    Serial.print(F(", "));
    Serial.print(TEST_MAG_OY, 2);
    Serial.print(F(", "));
    Serial.print(TEST_MAG_OZ, 2);
    Serial.println(F(") µT"));

    Serial.println(F("[Write] Done."));
    Serial.println();
}

// ============================================================================
// VERIFY READ-BACK
// ============================================================================

bool verifyValues(const char* context) {
    Serial.print(F("[Verify] "));
    Serial.println(context);

    bool allPass = true;

    // --- Wheel diameter ---
    float diam = 0.0f;
    if (!PersistentStorage::getWheelDiameter(diam)) {
        Serial.println(F("  [FAIL] wheelDiameter: not found in storage"));
        allPass = false;
    } else if (!floatClose(diam, TEST_WHEEL_DIAM)) {
        printFail("wheelDiameter", TEST_WHEEL_DIAM, diam);
        allPass = false;
    } else {
        printPass("wheelDiameter");
    }

    // --- Wheel base ---
    float base = 0.0f;
    if (!PersistentStorage::getWheelBase(base)) {
        Serial.println(F("  [FAIL] wheelBase: not found in storage"));
        allPass = false;
    } else if (!floatClose(base, TEST_WHEEL_BASE)) {
        printFail("wheelBase", TEST_WHEEL_BASE, base);
        allPass = false;
    } else {
        printPass("wheelBase");
    }

    // --- Magnetometer calibration ---
    float ox = 0.0f, oy = 0.0f, oz = 0.0f;
    if (!PersistentStorage::getMagCalibration(ox, oy, oz)) {
        Serial.println(F("  [FAIL] magCalibration: not found in storage"));
        allPass = false;
    } else {
        if (!floatClose(ox, TEST_MAG_OX)) { printFail("magOffsetX", TEST_MAG_OX, ox); allPass = false; }
        else printPass("magOffsetX");
        if (!floatClose(oy, TEST_MAG_OY)) { printFail("magOffsetY", TEST_MAG_OY, oy); allPass = false; }
        else printPass("magOffsetY");
        if (!floatClose(oz, TEST_MAG_OZ)) { printFail("magOffsetZ", TEST_MAG_OZ, oz); allPass = false; }
        else printPass("magOffsetZ");
    }

    Serial.println();
    if (allPass) {
        Serial.println(F("  >>> All checks PASSED <<<"));
    } else {
        Serial.println(F("  >>> Some checks FAILED <<<"));
    }
    Serial.println();
    return allPass;
}

// ============================================================================
// RAW EEPROM DUMP
// ============================================================================

void dumpEeprom() {
    Serial.println();
    printSeparator();
    Serial.println(F("Raw EEPROM dump (managed region, 32 bytes):"));
    printSeparator();
    Serial.println(F("Addr  Hex   Dec  Meaning"));

    // Print byte-by-byte with field annotations
    struct { int addr; int len; const char* name; } fields[] = {
        { 0,  4, "magic (0xDEAD2026 = valid)" },
        { 4,  1, "version"                    },
        { 5,  3, "reserved"                   },
        { 8,  4, "wheelDiameterMm (float)"    },
        { 12, 4, "wheelBaseMm (float)"         },
        { 16, 1, "magCalValid (1=yes)"        },
        { 17, 3, "reserved"                   },
        { 20, 4, "magOffsetX (float)"         },
        { 24, 4, "magOffsetY (float)"         },
        { 28, 4, "magOffsetZ (float)"         },
    };

    const int numFields = sizeof(fields) / sizeof(fields[0]);

    for (int fi = 0; fi < numFields; fi++) {
        for (int b = 0; b < fields[fi].len; b++) {
            int addr = PS_BASE_ADDR + fields[fi].addr + b;
            uint8_t val = EEPROM.read(addr);

            // Address
            if (addr < 10) Serial.print(F(" "));
            Serial.print(addr);
            Serial.print(F(":   "));

            // Hex
            if (val < 0x10) Serial.print(F("0"));
            Serial.print(val, HEX);
            Serial.print(F("    "));

            // Decimal
            if (val < 100) Serial.print(F(" "));
            if (val < 10)  Serial.print(F(" "));
            Serial.print(val);
            Serial.print(F("  "));

            // Field name on first byte of field
            if (b == 0) Serial.println(fields[fi].name);
            else        Serial.println();
        }
    }

    // Also print the float values decoded
    Serial.println();
    Serial.println(F("Decoded floats:"));

    float v;
    EEPROM.get(PS_BASE_ADDR + 8,  v); Serial.print(F("  wheelDiam = ")); Serial.println(v, 3);
    EEPROM.get(PS_BASE_ADDR + 12, v); Serial.print(F("  wheelBase = ")); Serial.println(v, 3);
    EEPROM.get(PS_BASE_ADDR + 20, v); Serial.print(F("  magX      = ")); Serial.println(v, 4);
    EEPROM.get(PS_BASE_ADDR + 24, v); Serial.print(F("  magY      = ")); Serial.println(v, 4);
    EEPROM.get(PS_BASE_ADDR + 28, v); Serial.print(F("  magZ      = ")); Serial.println(v, 4);

    printSeparator();
    Serial.println();
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}

    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("  EEPROM Persistent Storage Test"));
    Serial.println(F("========================================"));
    Serial.println();
    Serial.println(F("Arduino Mega 2560 EEPROM facts:"));
    Serial.print(F("  - Total size:    "));
    Serial.print(EEPROM.length());
    Serial.println(F(" bytes (4 KB)"));
    Serial.println(F("  - Endurance:    ~100,000 writes/byte"));
    Serial.println(F("  - Survives:     power-off, reset, reflash"));
    Serial.println(F("  - Erased by:    writing new data or reset()"));
    Serial.println();

    // Initialise PersistentStorage — reads magic number, validates layout
    PersistentStorage::init();

    printSeparator();

    if (!PersistentStorage::isValid()) {
        // Should not happen after init(), but guard anyway
        Serial.println(F("ERROR: Storage failed to initialise."));
        while (true) {}
    }

    Serial.print(F("Storage layout version: "));
    Serial.println(PersistentStorage::getVersion());
    Serial.println();

    // ---- Determine run type ------------------------------------------------
    // We detect Run 1 vs Run 2+ by checking whether our test values exist.
    // If wheel diameter is not stored, this is the first run.
    float existingDiam = 0.0f;
    bool firstRun = !PersistentStorage::getWheelDiameter(existingDiam);

    if (firstRun) {
        // ── RUN 1 ────────────────────────────────────────────────────────────
        Serial.println(F("=== RUN 1: No previous data found ==="));
        Serial.println();
        Serial.println(F("This is the first run (or after a reset)."));
        Serial.println(F("Writing test values to EEPROM..."));
        Serial.println();

        writeTestValues();

        Serial.println(F("Verifying immediate read-back (sanity check):"));
        verifyValues("immediate read-back after write");

        printSeparator();
        Serial.println();
        Serial.println(F(">>> NOW: Power off the Arduino completely."));
        Serial.println(F("    Disconnect USB or turn off the power switch."));
        Serial.println(F("    Wait 3 seconds, then power back on."));
        Serial.println(F("    If the data survived, it proves EEPROM is non-volatile!"));
        Serial.println();

    } else {
        // ── RUN 2+ ───────────────────────────────────────────────────────────
        Serial.println(F("=== RUN 2+: Previous data found! ==="));
        Serial.println();
        Serial.println(F("Data survived the power cycle. Verifying values..."));
        Serial.println();

        bool pass = verifyValues("after power cycle");

        printSeparator();
        Serial.println();
        if (pass) {
            Serial.println(F("SUCCESS! All values match what was written before power-off."));
            Serial.println(F("EEPROM is confirmed non-volatile."));
        } else {
            Serial.println(F("FAILURE: Values do not match. EEPROM may be corrupted."));
        }
        Serial.println();
        Serial.println(F("Type 'r' to erase EEPROM and start over with Run 1."));
    }

    printSeparator();
    printHelp();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    if (!Serial.available()) return;

    char c = Serial.read();

    switch (c) {
        case 'd':
        case 'D':
            dumpEeprom();
            break;

        case 'r':
        case 'R':
            Serial.println();
            Serial.println(F("[Reset] Erasing all EEPROM storage..."));
            PersistentStorage::reset();
            Serial.println(F("[Reset] Done. Reboot the Arduino to see Run 1 again."));
            Serial.println(F("        (or type 'w' to write test values now)"));
            Serial.println();
            break;

        case 'w':
        case 'W':
            Serial.println();
            writeTestValues();
            verifyValues("immediate read-back");
            break;

        case 'h':
        case 'H':
        case '?':
            printHelp();
            break;

        case '\n':
        case '\r':
            break;

        default:
            Serial.print(F("Unknown command '"));
            Serial.print(c);
            Serial.println(F("'. Type 'h' for help."));
            break;
    }
}

// ============================================================================
// HELP
// ============================================================================

void printHelp() {
    Serial.println(F("Commands:"));
    Serial.println(F("  d  - Dump raw EEPROM bytes for the managed region"));
    Serial.println(F("  r  - Reset (erase) all stored data"));
    Serial.println(F("  w  - Write test values again"));
    Serial.println(F("  h  - Show this help"));
    Serial.println();
}
