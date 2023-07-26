#include "MPUHelper.h"
#include "Constants.h"

MPUHelper::MPUHelper() {}


void MPUHelper::calibrate() {
    // MPU6050 calibration code
    // ...
}


void MPUHelper::initialize()
{
    // load and configure the DMP
    mpu.initialize();
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.print(F("DMP Initialization FIFOPacketSize"));
        Serial.print(packetSize);
        Serial.println("\t");
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    mpu.setI2CMasterModeEnabled(false);
    mpu.setI2CBypassEnabled(true);
    mpu.setSleepEnabled(false);

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void MPUHelper::readAccelerometer(int16_t &accX, int16_t &accY, int16_t &accZ)
{
    mpu.getAcceleration(&accX, &accY, &accZ);
}