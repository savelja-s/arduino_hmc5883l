#ifndef MPU_HELPER_H
#define MPU_HELPER_H

#include "MPU6050_6Axis_MotionApps20.h"

class MPUHelper
{
public:
    MPUHelper();
    void initialize();
    void calibrate();
    void readAccelerometer(int16_t &accX, int16_t &accY, int16_t &accZ);
    MPU6050 mpu;
    uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;
};

#endif