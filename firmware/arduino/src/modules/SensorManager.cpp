/**
 * @file SensorManager.cpp
 * @brief Implementation of centralized sensor management
 *
 * See SensorManager.h for design notes and usage.
 */

#include "SensorManager.h"
#include "PersistentStorage.h"
#include "../pins.h"

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

IMUDriver    SensorManager::imu_;
FusionWrapper SensorManager::fusion_(SENSOR_UPDATE_FREQ_HZ, 2000.0f);

bool     SensorManager::imuInitialized_ = false;
uint32_t SensorManager::lastImuMicros_  = 0;

LidarDriver      SensorManager::lidars_[SENSOR_MAX_LIDARS];
UltrasonicDriver SensorManager::ultrasonics_[SENSOR_MAX_ULTRASONICS];
uint8_t          SensorManager::lidarCount_        = 0;
uint8_t          SensorManager::ultrasonicCount_   = 0;
uint16_t         SensorManager::lidarDistMm_[SENSOR_MAX_LIDARS]           = {};
uint16_t         SensorManager::ultrasonicDistMm_[SENSOR_MAX_ULTRASONICS] = {};

float   SensorManager::batteryVoltage_   = 0.0f;
float   SensorManager::rail5VVoltage_    = 0.0f;
float   SensorManager::servoVoltage_     = 0.0f;

MagCalData SensorManager::magCal_ = {
    MAG_CAL_IDLE, 0,
    0.0f, 0.0f,  0.0f, 0.0f,  0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    false
};

bool     SensorManager::initialized_   = false;
uint32_t SensorManager::updateCount_   = 0;
uint32_t SensorManager::lastUpdateTime_ = 0;

// I2C addresses for configured sensors (from config.h)
static const uint8_t kLidarAddrs[4] = {
    LIDAR_0_I2C_ADDR, LIDAR_1_I2C_ADDR, LIDAR_2_I2C_ADDR, LIDAR_3_I2C_ADDR
};
static const uint8_t kUltrasonicAddrs[4] = {
    ULTRASONIC_0_I2C_ADDR, ULTRASONIC_1_I2C_ADDR,
    ULTRASONIC_2_I2C_ADDR, ULTRASONIC_3_I2C_ADDR
};

// ============================================================================
// INITIALIZATION
// ============================================================================

void SensorManager::init() {
    if (initialized_) return;

    // ADC reference: use default (AVCC = 5 V)
    analogReference(DEFAULT);

    // Initial voltage readings
    updateVoltages();

    // --- IMU ---
#if IMU_ENABLED
    imuInitialized_ = imu_.init(IMU_AD0_VAL);

    if (imuInitialized_) {
        // Apply fusion settings from config.h
        fusion_.setSettings(FUSION_GAIN, FUSION_ACCEL_REJECTION, FUSION_MAG_REJECTION);

        // Load magnetometer calibration from persistent storage if available
        initMagCalFromStorage();

#ifdef DEBUG_SENSOR
        DEBUG_SERIAL.println(F("[SensorManager] IMU OK"));
        if (magCal_.savedToEeprom) {
            DEBUG_SERIAL.println(F("[SensorManager] Mag cal loaded from EEPROM (9-DOF)"));
        } else {
            DEBUG_SERIAL.println(F("[SensorManager] No mag cal found; running 6-DOF"));
        }
#endif
    } else {
#ifdef DEBUG_SENSOR
        DEBUG_SERIAL.println(F("[SensorManager] WARNING: IMU not detected"));
#endif
    }
#endif

    // --- Lidar sensors ---
#if LIDAR_COUNT > 0
    lidarCount_ = 0;
    for (uint8_t i = 0; i < LIDAR_COUNT && i < SENSOR_MAX_LIDARS; i++) {
        if (lidars_[i].init(kLidarAddrs[i])) {
            lidarCount_++;
        }
    }
#ifdef DEBUG_SENSOR
    DEBUG_SERIAL.print(F("[SensorManager] Lidar sensors: "));
    DEBUG_SERIAL.println(lidarCount_);
#endif
#endif

    // --- Ultrasonic sensors ---
#if ULTRASONIC_COUNT > 0
    ultrasonicCount_ = 0;
    for (uint8_t i = 0; i < ULTRASONIC_COUNT && i < SENSOR_MAX_ULTRASONICS; i++) {
        if (ultrasonics_[i].init(kUltrasonicAddrs[i])) {
            ultrasonicCount_++;
        }
    }
#ifdef DEBUG_SENSOR
    DEBUG_SERIAL.print(F("[SensorManager] Ultrasonic sensors: "));
    DEBUG_SERIAL.println(ultrasonicCount_);
#endif
#endif

    lastImuMicros_ = micros();
    initialized_   = true;

#ifdef DEBUG_SENSOR
    DEBUG_SERIAL.print(F("[SensorManager] Battery: "));
    DEBUG_SERIAL.print(batteryVoltage_);
    DEBUG_SERIAL.println(F(" V"));
#endif
}

// ============================================================================
// TIMER4_OVF_vect ISR — 10 kHz carrier; /100 counter → 100 Hz dispatch
// ============================================================================

/**
 * @brief Timer4 Overflow ISR
 *
 * Fires at 10 kHz (same carrier as stepper Timer3).  An internal counter
 * divides this down to 100 Hz before invoking SensorManager::isrTick().
 *
 * sei() is called before isrTick() so the TWI (I2C) interrupt can fire
 * during the sensor reads — Wire.h requires the TWI_vect to be serviced.
 */
ISR(TIMER4_OVF_vect) {
    static uint8_t timerCounter = 0;
    if (++timerCounter < 100) return;   // 10 kHz / 100 = 100 Hz dispatch
    timerCounter = 0;
    sei();   // Re-enable interrupts: Wire/TWI needs its own ISR during I2C reads
    SensorManager::isrTick();
}

// ============================================================================
// isrTick — 100 Hz dispatcher (called from TIMER4_OVF_vect)
// ============================================================================

void SensorManager::isrTick() {
    if (!initialized_) return;

    static uint8_t counter = 0;

    lastUpdateTime_ = millis();
    updateCount_++;

    update100Hz();                        // every tick    (100 Hz)
    if ((counter & 1) == 0) update50Hz(); // even ticks   ( 50 Hz)
    if (counter == 0)        update10Hz(); // tick 0 only  ( 10 Hz)

    if (++counter >= 10) counter = 0;
}

// ============================================================================
// update100Hz — IMU + Fusion AHRS (100 Hz)
// ============================================================================

void SensorManager::update100Hz() {
#if IMU_ENABLED
    if (!imuInitialized_ || !imu_.dataReady()) return;

    imu_.update();

    uint32_t now   = micros();
    float    dtSec = (now - lastImuMicros_) * 1e-6f;
    lastImuMicros_ = now;

    // Clamp dt: ignore startup spike or any gap > 100 ms
    if (dtSec <= 0.0f || dtSec > 0.1f) dtSec = 1.0f / SENSOR_UPDATE_FREQ_HZ;

    // Convert accel from mg → g for Fusion library
    float ax = imu_.getAccX() * 0.001f;
    float ay = imu_.getAccY() * 0.001f;
    float az = imu_.getAccZ() * 0.001f;

    if (isMagCalibrated()) {
        // 9-DOF: fuse accelerometer, gyroscope, and calibrated magnetometer
        fusion_.update(
            imu_.getGyrX(), imu_.getGyrY(), imu_.getGyrZ(),
            ax, ay, az,
            imu_.getMagX(), imu_.getMagY(), imu_.getMagZ(),
            dtSec
        );
    } else {
        // 6-DOF fallback: no magnetometer (yaw drifts with gyro integration)
        fusion_.updateNoMag(
            imu_.getGyrX(), imu_.getGyrY(), imu_.getGyrZ(),
            ax, ay, az,
            dtSec
        );
    }

    if (magCal_.state == MAG_CAL_SAMPLING) {
        updateMagCalSampling();
    }
#endif
}

// ============================================================================
// update50Hz — Lidar reads (50 Hz)
// ============================================================================

void SensorManager::update50Hz() {
    for (uint8_t i = 0; i < lidarCount_ && i < SENSOR_MAX_LIDARS; i++) {
        lidarDistMm_[i] = lidars_[i].getDistanceMm();
    }
}

// ============================================================================
// update10Hz — Voltages + Ultrasonic (10 Hz)
// ============================================================================

void SensorManager::update10Hz() {
    updateVoltages();

    for (uint8_t i = 0; i < ultrasonicCount_ && i < SENSOR_MAX_ULTRASONICS; i++) {
        ultrasonicDistMm_[i] = ultrasonics_[i].getDistanceMm();
    }
}

// ============================================================================
// IMU / FUSION OUTPUT
// ============================================================================

void SensorManager::getQuaternion(float& w, float& x, float& y, float& z) {
    fusion_.getQuaternion(w, x, y, z);
}

void SensorManager::getEarthAcceleration(float& x, float& y, float& z) {
    fusion_.getEarthAcceleration(x, y, z);
}

int16_t SensorManager::getRawAccX() { return imu_.getRawAccX(); }
int16_t SensorManager::getRawAccY() { return imu_.getRawAccY(); }
int16_t SensorManager::getRawAccZ() { return imu_.getRawAccZ(); }
int16_t SensorManager::getRawGyrX() { return imu_.getRawGyrX(); }
int16_t SensorManager::getRawGyrY() { return imu_.getRawGyrY(); }
int16_t SensorManager::getRawGyrZ() { return imu_.getRawGyrZ(); }
int16_t SensorManager::getRawMagX() { return imu_.getRawMagX(); }
int16_t SensorManager::getRawMagY() { return imu_.getRawMagY(); }
int16_t SensorManager::getRawMagZ() { return imu_.getRawMagZ(); }

// ============================================================================
// RANGE SENSOR OUTPUT
// ============================================================================

uint16_t SensorManager::getLidarDistanceMm(uint8_t idx) {
    if (idx >= lidarCount_) return 0;
    return lidarDistMm_[idx];
}

uint16_t SensorManager::getUltrasonicDistanceMm(uint8_t idx) {
    if (idx >= ultrasonicCount_) return 0;
    return ultrasonicDistMm_[idx];
}

// ============================================================================
// VOLTAGE MONITORING
// ============================================================================

float SensorManager::getBatteryVoltage() { return batteryVoltage_; }
float SensorManager::get5VRailVoltage()  { return rail5VVoltage_;  }
float SensorManager::getServoVoltage()   { return servoVoltage_;   }

bool SensorManager::isBatteryLow(float threshold) {
    return (batteryVoltage_ > 0.0f && batteryVoltage_ < threshold);
}

bool SensorManager::isBatteryCritical() {
    return (batteryVoltage_ > 0.0f && batteryVoltage_ < VBAT_CUTOFF_V);
}

bool SensorManager::isBatteryOvervoltage() {
    return (batteryVoltage_ > VBAT_OVERVOLTAGE_V);
}

// ============================================================================
// MAGNETOMETER CALIBRATION
// ============================================================================

void SensorManager::startMagCalibration() {
    magCal_.state       = MAG_CAL_SAMPLING;
    magCal_.sampleCount = 0;
    magCal_.minX = magCal_.maxX = 0.0f;
    magCal_.minY = magCal_.maxY = 0.0f;
    magCal_.minZ = magCal_.maxZ = 0.0f;
    magCal_.offsetX = magCal_.offsetY = magCal_.offsetZ = 0.0f;

    // Zero IMU offsets during calibration so we collect raw magnetometer values.
    // The new offsets will be applied after the user saves the calibration.
    imu_.setMagOffset(0.0f, 0.0f, 0.0f);

    // Revert to 6-DOF during calibration (mag data is unreliable without offsets)
    fusion_.reset();

#ifdef DEBUG_SENSOR
    DEBUG_SERIAL.println(F("[MagCal] Calibration started — rotate robot through all orientations"));
#endif
}

void SensorManager::cancelMagCalibration() {
    magCal_.state = MAG_CAL_IDLE;
#ifdef DEBUG_SENSOR
    DEBUG_SERIAL.println(F("[MagCal] Calibration cancelled"));
#endif
}

bool SensorManager::saveMagCalibration() {
    if (magCal_.sampleCount < MAG_CAL_MIN_SAMPLES) {
#ifdef DEBUG_SENSOR
        DEBUG_SERIAL.print(F("[MagCal] Insufficient samples: "));
        DEBUG_SERIAL.print(magCal_.sampleCount);
        DEBUG_SERIAL.print(F(" / "));
        DEBUG_SERIAL.println(MAG_CAL_MIN_SAMPLES);
#endif
        magCal_.state = MAG_CAL_ERROR;
        return false;
    }

    applyMagCalibration(magCal_.offsetX, magCal_.offsetY, magCal_.offsetZ);
    return true;
}

void SensorManager::applyMagCalibration(float ox, float oy, float oz) {
    magCal_.offsetX = ox;
    magCal_.offsetY = oy;
    magCal_.offsetZ = oz;

    // Push offsets to IMU driver (subtracted from each mag reading)
    imu_.setMagOffset(ox, oy, oz);

    // Persist via PersistentStorage module
    PersistentStorage::setMagCalibration(ox, oy, oz);

    // Switch Fusion to 9-DOF mode (will use magnetometer on next update)
    magCal_.state         = MAG_CAL_SAVED;
    magCal_.savedToEeprom = true;

    // Reset the AHRS so it re-converges with the now-valid magnetometer
    fusion_.reset();

#ifdef DEBUG_SENSOR
    DEBUG_SERIAL.println(F("[MagCal] Calibration saved and 9-DOF mode activated"));
    DEBUG_SERIAL.print(F("  offsets (uT): "));
    DEBUG_SERIAL.print(ox); DEBUG_SERIAL.print(F(", "));
    DEBUG_SERIAL.print(oy); DEBUG_SERIAL.print(F(", "));
    DEBUG_SERIAL.println(oz);
#endif
}

void SensorManager::clearMagCalibration() {
    PersistentStorage::clearMagCalibration();

    imu_.setMagOffset(0.0f, 0.0f, 0.0f);
    magCal_.savedToEeprom = false;
    magCal_.state         = MAG_CAL_IDLE;

    // Revert to 6-DOF (no mag fusion)
    fusion_.reset();

#ifdef DEBUG_SENSOR
    DEBUG_SERIAL.println(F("[MagCal] Calibration cleared; reverted to 6-DOF"));
#endif
}

// ============================================================================
// INTERNAL: LOAD FROM PERSISTENT STORAGE
// ============================================================================

void SensorManager::initMagCalFromStorage() {
    float ox, oy, oz;
    if (!PersistentStorage::getMagCalibration(ox, oy, oz)) {
        magCal_.savedToEeprom = false;
        return;
    }

    magCal_.offsetX       = ox;
    magCal_.offsetY       = oy;
    magCal_.offsetZ       = oz;
    magCal_.savedToEeprom = true;
    magCal_.state         = MAG_CAL_SAVED;

    imu_.setMagOffset(ox, oy, oz);
}

// ============================================================================
// INTERNAL: MAG CAL SAMPLING (called every IMU update while SAMPLING)
// ============================================================================

void SensorManager::updateMagCalSampling() {
    // Offsets were zeroed by startMagCalibration(), so getMagX/Y/Z() returns raw µT.
    float mx = imu_.getMagX();
    float my = imu_.getMagY();
    float mz = imu_.getMagZ();

    if (magCal_.sampleCount == 0) {
        magCal_.minX = magCal_.maxX = mx;
        magCal_.minY = magCal_.maxY = my;
        magCal_.minZ = magCal_.maxZ = mz;
    } else {
        if (mx < magCal_.minX) magCal_.minX = mx;
        if (mx > magCal_.maxX) magCal_.maxX = mx;
        if (my < magCal_.minY) magCal_.minY = my;
        if (my > magCal_.maxY) magCal_.maxY = my;
        if (mz < magCal_.minZ) magCal_.minZ = mz;
        if (mz > magCal_.maxZ) magCal_.maxZ = mz;
    }

    magCal_.sampleCount++;

    // Compute running offsets (hard-iron = center of the ellipsoid)
    magCal_.offsetX = (magCal_.maxX + magCal_.minX) * 0.5f;
    magCal_.offsetY = (magCal_.maxY + magCal_.minY) * 0.5f;
    magCal_.offsetZ = (magCal_.maxZ + magCal_.minZ) * 0.5f;
}

// ============================================================================
// INTERNAL: VOLTAGE MONITORING
// ============================================================================

void SensorManager::updateVoltages() {
    // Battery (1:6 divider → multiply ADC voltage by 6)
    batteryVoltage_ = adcToVoltage(readADCAverage(PIN_VBAT_SENSE), 1.0f / 6.0f);

    // 5V rail (1:2 divider → multiply by 2)
    rail5VVoltage_  = adcToVoltage(readADCAverage(PIN_V5_SENSE), 1.0f / 2.0f);

    // Servo rail (1:3 divider → multiply by 3)
    servoVoltage_   = adcToVoltage(readADCAverage(PIN_VSERVO_SENSE), 1.0f / 3.0f);
}

uint16_t SensorManager::readADCAverage(uint8_t pin, uint8_t numSamples) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < numSamples; i++) {
        sum += analogRead(pin);
    }
    return (uint16_t)(sum / numSamples);
}

float SensorManager::adcToVoltage(uint16_t adcValue, float dividerRatio) {
    // ADC input voltage
    float vADC = (adcValue / 1023.0f) * ADC_VREF;
    // Actual voltage before the divider (dividerRatio = Vout / Vin, so Vin = Vout / ratio)
    return vADC / dividerRatio;
}
