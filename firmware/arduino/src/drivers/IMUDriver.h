/**
 * @file IMUDriver.h
 * @brief Wrapper for ICM-20948 9-DoF IMU (SparkFun library)
 *
 * Provides a simplified interface to the ICM-20948 combining:
 * - 3-axis accelerometer (output in mg)
 * - 3-axis gyroscope (output in DPS)
 * - 3-axis magnetometer / AK09916 (output in µT)
 * - Temperature sensor
 *
 * This driver does NOT use the DMP. Sensor fusion is handled externally
 * by FusionWrapper (Madgwick AHRS from the Fusion library).
 *
 * Magnetometer calibration offsets can be set via setMagOffset() and are
 * subtracted from each magnetometer reading before output.
 *
 * Usage:
 *   IMUDriver imu;
 *   imu.init(IMU_AD0_VAL);          // AD0_VAL: 0=0x68, 1=0x69
 *
 *   if (imu.dataReady()) {
 *       imu.update();               // reads getAGMT() from hardware
 *       float ax = imu.getAccX();   // mg
 *       float gx = imu.getGyrX();   // DPS
 *       float mx = imu.getMagX();   // µT (offset applied)
 *   }
 */

#ifndef IMUDRIVER_H
#define IMUDRIVER_H

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include "../config.h"

#if IMU_ENABLED
  #include "../lib/SparkFun_9DoF_IMU_Breakout_-_ICM_20948_-_Arduino_Library/src/ICM_20948.h"
#endif

// ============================================================================
// IMU DRIVER CLASS
// ============================================================================

class IMUDriver {
public:
    IMUDriver();

    /**
     * @brief Initialize IMU sensor over I2C
     *
     * @param ad0Val Address select: 0 = 0x68 (AD0 low), 1 = 0x69 (AD0 high)
     * @return True if initialization succeeded
     */
    bool init(uint8_t ad0Val = IMU_AD0_VAL);

    /**
     * @brief Check if IMU is connected and responding
     */
    bool isConnected() const { return initialized_; }

    /**
     * @brief Check if new data is ready to be read from the sensor
     */
    bool dataReady();

    /**
     * @brief Read all sensor data from ICM-20948 (calls getAGMT internally)
     *
     * Must call dataReady() first; call this when it returns true.
     *
     * @return True if read was successful
     */
    bool update();

    // ========================================================================
    // SCALED OUTPUTS (post-library conversion, float)
    // ========================================================================

    /** Accelerometer X in mg (milligravity). 1000 mg = 1 g = 9.81 m/s². */
    float getAccX() const { return accX_; }
    float getAccY() const { return accY_; }
    float getAccZ() const { return accZ_; }

    /** Gyroscope in degrees per second (DPS). */
    float getGyrX() const { return gyrX_; }
    float getGyrY() const { return gyrY_; }
    float getGyrZ() const { return gyrZ_; }

    /**
     * Magnetometer in µT with calibration offset subtracted.
     * Returns raw µT if no offset has been set (offsets default to 0.0).
     */
    float getMagX() const { return magX_; }
    float getMagY() const { return magY_; }
    float getMagZ() const { return magZ_; }

    /** Temperature from internal sensor in °C. */
    float getTemp() const { return temp_; }

    // ========================================================================
    // PACKED INT16 OUTPUTS (for TLV transmission)
    // ========================================================================

    /** Accel as int16 in mg: direct cast of getAccX() */
    int16_t getRawAccX() const { return (int16_t)accX_; }
    int16_t getRawAccY() const { return (int16_t)accY_; }
    int16_t getRawAccZ() const { return (int16_t)accZ_; }

    /** Gyro as int16 in 0.1 DPS units: (int16)(gyrX * 10) */
    int16_t getRawGyrX() const { return (int16_t)(gyrX_ * 10.0f); }
    int16_t getRawGyrY() const { return (int16_t)(gyrY_ * 10.0f); }
    int16_t getRawGyrZ() const { return (int16_t)(gyrZ_ * 10.0f); }

    /** Mag as int16 in µT (calibration offset already applied) */
    int16_t getRawMagX() const { return (int16_t)magX_; }
    int16_t getRawMagY() const { return (int16_t)magY_; }
    int16_t getRawMagZ() const { return (int16_t)magZ_; }

    // ========================================================================
    // MAGNETOMETER CALIBRATION
    // ========================================================================

    /**
     * @brief Set hard-iron calibration offsets for the magnetometer
     *
     * These offsets are subtracted from each magnetometer reading:
     *   corrected = raw - offset
     * Offset is typically (max + min) / 2 from a spinning calibration.
     *
     * @param ox, oy, oz Offsets in µT
     */
    void setMagOffset(float ox, float oy, float oz);

    /**
     * @brief Get current magnetometer calibration offsets
     */
    void getMagOffset(float& ox, float& oy, float& oz) const;

private:
#if IMU_ENABLED
    ICM_20948_I2C imu_;         // SparkFun ICM-20948 library instance
#endif

    bool initialized_;

    // Latest sensor readings (updated by update())
    float accX_, accY_, accZ_;  // mg
    float gyrX_, gyrY_, gyrZ_;  // DPS
    float magX_, magY_, magZ_;  // µT (offset applied)
    float temp_;                // °C

    // Hard-iron calibration offsets (default 0.0)
    float magOffX_, magOffY_, magOffZ_;
};

#endif // IMUDRIVER_H
