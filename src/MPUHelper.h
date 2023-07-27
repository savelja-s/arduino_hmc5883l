#ifndef MPU_HELPER_H
#define MPU_HELPER_H

#include "MPU6050_6Axis_MotionApps20.h"

class MPUHelper
{
public:
    MPUHelper();
    void initialize();
    MPU6050 mpu;
};

#endif