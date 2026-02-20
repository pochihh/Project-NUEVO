/**
 * @file IMUDriver.cpp
 * @brief Implementation of ICM-20948 IMU driver (SparkFun library, no DMP)
 *
 * See IMUDriver.h for usage and design notes.
 */

#include "IMUDriver.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================

IMUDriver::IMUDriver()
    : initialized_(false)
    , accX_(0.0f), accY_(0.0f), accZ_(0.0f)
    , gyrX_(0.0f), gyrY_(0.0f), gyrZ_(0.0f)
    , magX_(0.0f), magY_(0.0f), magZ_(0.0f)
    , temp_(0.0f)
    , magOffX_(0.0f), magOffY_(0.0f), magOffZ_(0.0f)
{
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool IMUDriver::init(uint8_t ad0Val) {
#if IMU_ENABLED
    // Wire.begin() is called once globally in main setup; do not call here.
    // Set I2C clock to 400 kHz for fast sensor reads.
    Wire.setClock(400000);

    // Try to initialize the ICM-20948. The library will retry on failure.
    imu_.begin(Wire, ad0Val);

    if (imu_.status != ICM_20948_Stat_Ok) {
#ifdef DEBUG_SENSOR
        DEBUG_SERIAL.print(F("[IMU] Init failed: "));
        DEBUG_SERIAL.println(imu_.statusString());
#endif
        return false;
    }

    initialized_ = true;

#ifdef DEBUG_SENSOR
    DEBUG_SERIAL.print(F("[IMU] ICM-20948 initialized (AD0="));
    DEBUG_SERIAL.print(ad0Val);
    DEBUG_SERIAL.print(F(", addr=0x"));
    DEBUG_SERIAL.print(ad0Val ? 0x69 : 0x68, HEX);
    DEBUG_SERIAL.println(F(")"));
#endif

    return true;
#else
    (void)ad0Val;
    return false;
#endif
}

// ============================================================================
// DATA ACQUISITION
// ============================================================================

bool IMUDriver::dataReady() {
#if IMU_ENABLED
    if (!initialized_) return false;
    return imu_.dataReady();
#else
    return false;
#endif
}

bool IMUDriver::update() {
#if IMU_ENABLED
    if (!initialized_) return false;

    // Read all axes in a single I2C transaction (accel + gyro + mag + temp)
    imu_.getAGMT();

    // Accelerometer: accX() returns mg (milligravity)
    accX_ = imu_.accX();
    accY_ = imu_.accY();
    accZ_ = imu_.accZ();

    // Gyroscope: gyrX() returns DPS (degrees per second)
    gyrX_ = imu_.gyrX();
    gyrY_ = imu_.gyrY();
    gyrZ_ = imu_.gyrZ();

    // Magnetometer: magX() returns µT; subtract hard-iron calibration offsets
    magX_ = imu_.magX() - magOffX_;
    magY_ = imu_.magY() - magOffY_;
    magZ_ = imu_.magZ() - magOffZ_;

    // Temperature: temp() returns °C
    temp_ = imu_.temp();

    return true;
#else
    return false;
#endif
}

// ============================================================================
// MAGNETOMETER CALIBRATION
// ============================================================================

void IMUDriver::setMagOffset(float ox, float oy, float oz) {
    magOffX_ = ox;
    magOffY_ = oy;
    magOffZ_ = oz;
}

void IMUDriver::getMagOffset(float& ox, float& oy, float& oz) const {
    ox = magOffX_;
    oy = magOffY_;
    oz = magOffZ_;
}
