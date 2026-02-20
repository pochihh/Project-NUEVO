#pragma once

#include "Fusion.h"

/**
 * C++ wrapper for the Fusion AHRS library
 * Provides sensor fusion using accelerometer, gyroscope, and magnetometer data
 */
class FusionWrapper {
public:
    /**
     * Initialize the Fusion AHRS with default settings
     * @param sample_rate_hz Sampling rate in Hz
     * @param gyroscope_range_dps Gyroscope range in degrees per second
     */
    FusionWrapper(float sample_rate_hz = 100.0f, float gyroscope_range_dps = 2000.0f);

    /**
     * Reset the AHRS to initial state
     */
    void reset();

    /**
     * Update the AHRS with new sensor measurements
     * @param gyro_x, gyro_y, gyro_z Gyroscope data in degrees/second
     * @param accel_x, accel_y, accel_z Accelerometer data in g
     * @param mag_x, mag_y, mag_z Magnetometer data in µT
     * @param delta_time_s Time since last update in seconds
     */
    void update(float gyro_x, float gyro_y, float gyro_z,
                float accel_x, float accel_y, float accel_z,
                float mag_x, float mag_y, float mag_z,
                float delta_time_s);

    /**
     * Update without magnetometer (6-axis mode)
     * @param gyro_x, gyro_y, gyro_z Gyroscope data in degrees/second
     * @param accel_x, accel_y, accel_z Accelerometer data in g
     * @param delta_time_s Time since last update in seconds
     */
    void updateNoMag(float gyro_x, float gyro_y, float gyro_z,
                     float accel_x, float accel_y, float accel_z,
                     float delta_time_s);

    /**
     * Get the current orientation as Euler angles
     * @param roll Output roll angle in degrees
     * @param pitch Output pitch angle in degrees
     * @param yaw Output yaw angle in degrees
     */
    void getEulerAngles(float& roll, float& pitch, float& yaw) const;

    /**
     * Get the current orientation as a quaternion
     * @param w, x, y, z Output quaternion components
     */
    void getQuaternion(float& w, float& x, float& y, float& z) const;

    /**
     * Get gravity-removed acceleration in the earth (global) frame
     *
     * The earth frame uses the NWU (North-West-Up) convention. Gravity is removed,
     * so a stationary sensor returns approximately (0, 0, 0).
     *
     * @param x, y, z Output earth-frame linear acceleration in g
     */
    void getEarthAcceleration(float& x, float& y, float& z) const;

    /**
     * Get gravity-removed acceleration in the sensor (body) frame
     *
     * Same as earth acceleration but expressed in the sensor's own coordinate system.
     *
     * @param x, y, z Output sensor-frame linear acceleration in g
     */
    void getLinearAcceleration(float& x, float& y, float& z) const;

    /**
     * Configure AHRS settings
     * @param gain AHRS gain (default: 0.5, range: 0-1, higher = faster convergence)
     * @param accel_rejection Acceleration rejection threshold in g (default: 10.0)
     * @param mag_rejection Magnetic rejection threshold in µT (default: 10.0)
     */
    void setSettings(float gain = 0.5f,
                    float accel_rejection = 10.0f,
                    float mag_rejection = 10.0f);

private:
    FusionBias bias_;
    FusionAhrs ahrs_;
    float sample_rate_hz_;
    float gyroscope_range_dps_;
};
