#ifndef MAGNETOMETER_HELPER_H
#define MAGNETOMETER_HELPER_H

#include "HMC5883L.h"
#include "MPU6050_6Axis_MotionApps20.h"

class MagnetometerHelper
{
public:
    MagnetometerHelper();
    // int16_t ax, ay, az;
    // int16_t gx, gy, gz;
    // int16_t mx, my, mz;
    bool inversXY = true;
    void initialize(MPU6050 mpu);
    void calibrate(boolean eeprom_write, byte xy_or_z);
    float getMagnetometerBearing(int16_t x, int16_t y, int16_t z);
    void getRollPitch(float *roll, float *pitch);
    void getRollPitchRawFloat(float *roll, float *pitch);
    float getMagnetometerTiltCompensatedBearing(int16_t ix, int16_t iy, int16_t iz);
    void calc_offsets(void);
    byte magnetometerReady(void);
    void read_with_eeprom_min_max_for_xyz(void);
    void getMagnetometer(int16_t *x, int16_t *y, int16_t *z);
    byte getMagnetometerRaw(int16_t *x, int16_t *y, int16_t *z);
    void readMagnetometer(int16_t *x, int16_t *y, int16_t *z);
    // void readXYZ();

    // Add other Magnetometer related methods here

private:
    HMC5883L compass;
    MPU6050 accelgyro;
    int minx, maxx, miny, maxy, minz, maxz;
    int offx = 0;
    int offy = 0;
    int offz = 0;
};

#endif