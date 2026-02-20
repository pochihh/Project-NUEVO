/**
 * @file SensorManager.h
 * @brief Centralized sensor management with multi-rate updates
 *
 * SensorManager aggregates all sensor subsystems and exposes a unified interface
 * for the rest of the firmware. It is called from the scheduler at two rates:
 *
 *   update()          — called at SENSOR_UPDATE_FREQ_HZ (100 Hz): reads IMU,
 *                       runs Fusion AHRS, samples voltages at 10 Hz sub-rate
 *   updateRange()     — called at SENSOR_LIDAR_FREQ_HZ (50 Hz): reads lidar;
 *                       ultrasonic sensors polled internally at 15 Hz sub-rate
 *
 * Attached sensors are controlled by config.h compile-time flags:
 *   IMU_ENABLED, LIDAR_COUNT, ULTRASONIC_COUNT
 *
 * Magnetometer Calibration State Machine
 * ----------------------------------------
 * Calibration is only allowed in IDLE firmware state. The Bridge sends
 * SENSOR_MAG_CAL_CMD; the result is streamed back via SENSOR_MAG_CAL_STATUS.
 *
 *   IDLE → startMagCalibration() → SAMPLING (collecting min/max per axis)
 *        → saveMagCalibration()  → SAVED (offsets written to EEPROM, 9-DOF active)
 *        → cancelMagCalibration()→ IDLE (no change)
 *
 * Voltage Dividers (from PCB hardware design):
 *   VBAT  (A0): 50kΩ/10kΩ → 1:6 → 0–24 V maps to 0–4 V ADC
 *   V5    (A1): 10kΩ/10kΩ → 1:2 → 0–10 V maps to 0–5 V ADC
 *   VSERVO(A2): 20kΩ/10kΩ → 1:3 → 0–15 V maps to 0–5 V ADC
 */

#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

#include <Arduino.h>
#include <stdint.h>
#include "../config.h"
#include "../drivers/IMUDriver.h"
#include "../drivers/LidarDriver.h"
#include "../drivers/UltrasonicDriver.h"
#include "../lib/Fusion/FusionWrapper.h"
#include "PersistentStorage.h"

// Maximum number of range sensors of each type (must match config.h defines)
#define SENSOR_MAX_LIDARS       4
#define SENSOR_MAX_ULTRASONICS  4

// ============================================================================
// MAGNETOMETER CALIBRATION STATE
// ============================================================================

enum MagCalState : uint8_t {
    MAG_CAL_IDLE     = 0,   // No calibration in progress
    MAG_CAL_SAMPLING = 1,   // Collecting samples (spinning the robot)
    MAG_CAL_COMPLETE = 2,   // Sampling done, offsets computed, not yet saved
    MAG_CAL_SAVED    = 3,   // Offsets saved to EEPROM and active
    MAG_CAL_ERROR    = 4    // Error (e.g., insufficient samples)
};

struct MagCalData {
    MagCalState state;
    uint16_t    sampleCount;
    float       minX, maxX;    // µT
    float       minY, maxY;
    float       minZ, maxZ;
    float       offsetX, offsetY, offsetZ;  // (max+min)/2 per axis
    bool        savedToEeprom;
};

// ============================================================================
// SENSOR MANAGER (Static class)
// ============================================================================

class SensorManager {
public:
    /**
     * @brief Initialize all enabled sensors
     *
     * Initializes I2C, IMU, lidar(s), ultrasonic(s), and reads initial voltages.
     * Loads magnetometer calibration from EEPROM if available.
     * Call once in setup() before using any sensors.
     */
    static void init();

    /**
     * @brief Main sensor update — call at SENSOR_UPDATE_FREQ_HZ (100 Hz)
     *
     * - Reads IMU (if data ready) and runs Fusion AHRS
     * - Updates voltage readings at 10 Hz sub-rate
     * - Updates mag calibration sampling if active
     */
    static void update();

    /**
     * @brief Range sensor update — call at SENSOR_LIDAR_FREQ_HZ (50 Hz)
     *
     * - Reads all lidar sensors at 50 Hz
     * - Reads ultrasonic sensors at 15 Hz sub-rate (every ~3rd call)
     */
    static void updateRange();

    // ========================================================================
    // IMU / FUSION OUTPUT
    // ========================================================================

    /**
     * @brief Get the latest orientation quaternion from Fusion AHRS
     *
     * In 9-DOF mode (magnetometer calibrated): yaw is absolute (north-referenced).
     * In 6-DOF fallback: yaw drifts slowly with gyro integration.
     */
    static void getQuaternion(float& w, float& x, float& y, float& z);

    /**
     * @brief Get linear acceleration in the earth (global/NWU) frame
     *
     * Gravity removed. A stationary sensor returns approximately (0, 0, 0).
     * Units: g (1 g = 9.81 m/s²).
     */
    static void getEarthAcceleration(float& x, float& y, float& z);

    /**
     * @brief Check if the IMU is connected and producing data
     */
    static bool isIMUAvailable() { return imuInitialized_; }

    /**
     * @brief Check if the magnetometer calibration is active (9-DOF fusion)
     */
    static bool isMagCalibrated() { return magCal_.savedToEeprom || magCal_.state == MAG_CAL_COMPLETE; }

    /**
     * @brief Get latest raw IMU values (for TLV packing)
     */
    static int16_t getRawAccX();
    static int16_t getRawAccY();
    static int16_t getRawAccZ();
    static int16_t getRawGyrX();
    static int16_t getRawGyrY();
    static int16_t getRawGyrZ();
    static int16_t getRawMagX();
    static int16_t getRawMagY();
    static int16_t getRawMagZ();

    // ========================================================================
    // RANGE SENSORS
    // ========================================================================

    /**
     * @brief Get number of active lidar sensors
     */
    static uint8_t getLidarCount() { return lidarCount_; }

    /**
     * @brief Get number of active ultrasonic sensors
     */
    static uint8_t getUltrasonicCount() { return ultrasonicCount_; }

    /**
     * @brief Get latest lidar distance reading
     *
     * @param idx Sensor index (0-based, up to getLidarCount()-1)
     * @return Distance in mm, 0 = error / no reading yet
     */
    static uint16_t getLidarDistanceMm(uint8_t idx);

    /**
     * @brief Get latest ultrasonic distance reading
     *
     * @param idx Sensor index (0-based, up to getUltrasonicCount()-1)
     * @return Distance in mm, 0 = error / no reading yet
     */
    static uint16_t getUltrasonicDistanceMm(uint8_t idx);

    // ========================================================================
    // VOLTAGE MONITORING
    // ========================================================================

    static float getBatteryVoltage();
    static float get5VRailVoltage();
    static float getServoVoltage();
    static bool  isBatteryLow(float threshold = VBAT_LOW_THRESHOLD);

    // ========================================================================
    // MAGNETOMETER CALIBRATION API
    // ========================================================================

    /**
     * @brief Start magnetometer calibration sampling
     *
     * Resets min/max trackers and begins collecting samples.
     * Only accepted when firmware is in IDLE state (enforced by caller).
     */
    static void startMagCalibration();

    /**
     * @brief Stop calibration sampling without saving
     */
    static void cancelMagCalibration();

    /**
     * @brief Save current computed calibration offsets to EEPROM and activate
     *
     * Requires MAG_CAL_MIN_SAMPLES samples. Returns false if insufficient.
     *
     * @return True if offsets were saved successfully
     */
    static bool saveMagCalibration();

    /**
     * @brief Apply user-provided offsets directly and save to EEPROM
     *
     * Skips the sampling phase; offsets provided by the Bridge
     * (e.g., from a previous calibration run on the host side).
     */
    static void applyMagCalibration(float ox, float oy, float oz);

    /**
     * @brief Clear EEPROM calibration and revert to 6-DOF mode
     */
    static void clearMagCalibration();

    /**
     * @brief Read current calibration state (for SENSOR_MAG_CAL_STATUS TLV)
     */
    static const MagCalData& getMagCalData() { return magCal_; }

    // ========================================================================
    // STATISTICS
    // ========================================================================

    static uint32_t getUpdateCount()    { return updateCount_; }
    static uint32_t getLastUpdateTime() { return lastUpdateTime_; }

private:
    // ---- IMU + Fusion ----
    static IMUDriver    imu_;
    static FusionWrapper fusion_;
    static bool imuInitialized_;
    static uint32_t lastImuMicros_;     // micros() at last IMU update (for dt)

    // ---- Range sensors ----
    static LidarDriver      lidars_[SENSOR_MAX_LIDARS];
    static UltrasonicDriver ultrasonics_[SENSOR_MAX_ULTRASONICS];
    static uint8_t          lidarCount_;
    static uint8_t          ultrasonicCount_;
    static uint16_t         lidarDistMm_[SENSOR_MAX_LIDARS];
    static uint16_t         ultrasonicDistMm_[SENSOR_MAX_ULTRASONICS];

    // Sub-rate counters
    static uint8_t rangeCallCount_;     // increments each updateRange() call

    // ---- Voltages ----
    static float batteryVoltage_;
    static float rail5VVoltage_;
    static float servoVoltage_;
    static uint8_t voltageCallCount_;   // increments each update(); sample at 10 Hz

    // ---- Magnetometer calibration ----
    static MagCalData magCal_;

    // ---- Statistics ----
    static bool     initialized_;
    static uint32_t updateCount_;
    static uint32_t lastUpdateTime_;

    // ---- Internal helpers ----
    static void initMagCalFromStorage();
    static void updateVoltages();
    static void updateMagCalSampling();
    static uint16_t readADCAverage(uint8_t pin, uint8_t numSamples = 4);
    static float    adcToVoltage(uint16_t adcValue, float dividerRatio);
};

#endif // SENSORMANAGER_H
