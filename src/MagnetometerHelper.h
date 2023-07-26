#ifndef MAGNETOMETER_HELPER_H
#define MAGNETOMETER_HELPER_H

#include "HMC5883L.h"
#include "MPU6050_6Axis_MotionApps20.h"

class MagnetometerHelper
{
public:
    MagnetometerHelper();
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
    void initialize(MPU6050 mpu);
    void calibrate(boolean eeprom_write, byte xy_or_z);
    float getMagnetometerBearing();
    void getRollPitch(float *roll, float *pitch);
    void getRollPitchRawFloat(float *roll, float *pitch);
    float getMagnetometerTiltCompensatedBearing();
    void calc_offsets(void);
    byte magnetometerReady(void);
    void read_with_eeprom_min_max_for_xyz(void);
    void readXYZ();

    // Add other Magnetometer related methods here

private:
    HMC5883L compass;
    MPU6050 accelgyro;

    Quaternion q;        // [w, x, y, z]         quaternion container   // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity; // [x, y, z]            gravity vector
    float ypr[3];
    int minx, maxx, miny, maxy, minz, maxz;
    int offx, offy, offz;
    byte stat, ovfl, skipped;
};

#endif