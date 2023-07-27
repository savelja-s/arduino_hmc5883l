#include "MagnetometerHelper.h"
#include "HMC5883L.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "EEPROMHelper.h"
#include "Constants.h"

MagnetometerHelper::MagnetometerHelper() {}

void MagnetometerHelper::read_with_eeprom_min_max_for_xyz()
{
    EEPROMHelper::eeprom_read_min_max_for_xyz(minx, maxx, miny, maxy, minz, maxz);
}

void MagnetometerHelper::initialize(MPU6050 initMpu)
{
    accelgyro = initMpu;
    // initialize device
    Serial.println("Initializing I2C devices...");
    // Wire.beginTransmission(HMC5883L_ARRD);
    // Wire.write(0x02); // Адреса регістру конфігурації для режиму роботи
    // Wire.write(0x00); // Записуємо 0x00, щоб встановити режим роботи за замовчуванням (режим роботи 0)
    // Wire.endTransmission();
    compass.initialize();
    Serial.println(compass.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
}

void MagnetometerHelper::calc_offsets(void)
{
    offx = (maxx + minx) / 2; // maxx/2 - minx/2 + 2minx/2
    offy = (maxy + miny) / 2;
    offz = (maxz + minz) / 2;
}
void MagnetometerHelper::readMagnetometer(int16_t *x, int16_t *y, int16_t *z)
{
    while (!magnetometerReady())
        ;
    getMagnetometer(x, y, z); // Note: Addresses of pointers passed.
}
void MagnetometerHelper::getMagnetometer(int16_t *x, int16_t *y, int16_t *z)
{
    if (!getMagnetometerRaw(x, y, z))
        return;

    if (inversXY)
    {
        // modify for accel board orientation (board x = up, y to left).
        *y = -(*y);
    }
}
byte MagnetometerHelper::getMagnetometerRaw(int16_t *x, int16_t *y, int16_t *z)
{
    if (!magnetometerReady())
        return 0;
    *x = compass.getHeadingX();
    *y = compass.getHeadingY();
    *z = compass.getHeadingZ();
    // compass.getHeading(x, y, z);
    // Wire.beginTransmission(HMC5883L_ARRD);
    // Wire.write(HMC5883L_RA_DATAX_H); // read from address zero = x,y,z registers.
    // int err = Wire.endTransmission();

    // if (!err)
    // {
    //     Wire.requestFrom((byte)HMC5883L_ARRD, (byte)6); // Blocking?
    //     while (Wire.available() < 6)
    //         ; // Wait if above blocking then this not needed.
    //     *x = (int16_t)(Wire.read() | Wire.read() << 8);
    //     *y = (int16_t)(Wire.read() | Wire.read() << 8);
    //     *z = (int16_t)(Wire.read() | Wire.read() << 8);
    // }
    return 1;
}

byte MagnetometerHelper::magnetometerReady(void)
{
    byte stat, ovfl, skipped;
    // Data ready?
    Wire.beginTransmission(HMC5883L_ARRD); // HMC5883L I2C address
    Wire.write(HMC5883L_RA_STATUS);        // Read from status reg
    int num = Wire.endTransmission();
    Wire.requestFrom((byte)HMC5883L_ARRD, (byte)1);
    stat = Wire.read();    // RDY bit (bit 0)
    ovfl = stat & 0x02;    // Extract OVFL bit (bit 1)
    skipped = stat & 0x04; // Extract SKIPPED bit (bit 2)
    return (stat & 0x01);  // 0x01 is the RDY (Ready) flag
}
float MagnetometerHelper::getMagnetometerBearing(int16_t ix, int16_t iy, int16_t iz)
{
    int x = ix - offx;
    int y = iy - offy;
    int z = iz - offz;
    float atan2val = 180 / M_PI * atan2((float)y, (float)x); // NEW coords.
    int b = (int)(-atan2val + 360) % 360;
    return b;
}
float MagnetometerHelper::getMagnetometerTiltCompensatedBearing(int16_t ix, int16_t iy, int16_t iz)
{
    int x = ix - offx;
    int y = iy - offy;
    int z = iz - offz;
    float r, p, mx, my, mz;

    getRollPitch(&r, &p);

    // Convert back to radians
    int rdeg = r;
    int pdeg = p;
    r = (M_PI / 180) * r;
    p = (M_PI / 180) * p;

    mx = (float)x;
    my = (float)y;
    mz = (float)z;

    int cx = (int)(mx * cos(p) + my * sin(r) * sin(p) - mz * cos(r) * sin(p));
    int cy = (int)(my * cos(r) + mz * sin(r));
    // Also diffrerent here:
    // https://web.archive.org/web/20130624074336/http://www.loveelectronics.co.uk:80/Tutorials/13/tilt-compensated-compass-arduino-tutorial
    // Same as 1st!!!

    int tx = cx, ty = cy, tz = z; // Done offx,y above
    float atan2val = 180 / M_PI * atan2((float)(ty), (float)(tx));

    int tb = (int)(-atan2val + 360) % 360;

    return tb;
}
void MagnetometerHelper::getRollPitchRawFloat(float *roll, float *pitch)
{
    float r, x, y, z;
    int16_t ax, ay, az;
    accelgyro.getAcceleration(&ax, &ay, &az);
    x = (float)ax * ADXL345_LSBVAL_3V3 - 25E-3;
    y = (float)ay * ADXL345_LSBVAL_3V3 - 25E-3;
    z = (float)az * ADXL345_LSBVAL_2V5 + 20e-3;

    r = sqrt(x * x + y * y + z * z);

    if (inversXY)
    {
        // modify for accel board orientation (board x = up, y to left).
        y = -y;
    }
    *pitch = 180 / M_PI * asin(x / r);
    *roll = 180 / M_PI * -asin(y / r);
}
void MagnetometerHelper::getRollPitch(float *roll, float *pitch)
{
    static float avg_r[SMOOTH_ACCELL], avg_p[SMOOTH_ACCELL];
    static byte idx = 0;
    float r, p;
    getRollPitchRawFloat(roll, pitch);
    avg_r[idx] = *roll; // Smooth.
    avg_p[idx] = *pitch;
    idx++;
    if (idx >= SMOOTH_ACCELL)
        idx = 0;
    r = p = 0;
    for (int i = 0; i < SMOOTH_ACCELL; i++)
    {
        r += avg_r[i];
        p += avg_p[i];
    }
    r /= SMOOTH_ACCELL;
    p /= SMOOTH_ACCELL;
    *roll = r;
    *pitch = p;
}

void MagnetometerHelper::calibrate(boolean eeprom_write, byte xy_or_z)
{
    unsigned long calTimeWas = millis();
    int16_t x, y, z;
    readMagnetometer(&x, &y, &z);
    maxx = minx = x; // Set initial values to current magnetometer readings.
    maxy = miny = y;
    maxz = minz = z;
    delay(300); // Allow button release.
    while (1)
    { // Calibration loop.
        if (digitalRead(BUTTON_CAL) == 0 || digitalRead(BUTTON_TEST) == 0)
        {
            delay(300); // Allow button release.
            return;     // Abort
        }
        if (magnetometerReady())
            getMagnetometer(&x, &y, &z);
        if (x > maxx)
            maxx = x;
        if (x < minx)
            minx = x;
        if (y > maxy)
            maxy = y;
        if (y < miny)
            miny = y;
        if (z > maxz)
            maxz = z;
        if (z < minz)
            minz = z;
        if (eeprom_write)
            Serial.println("CALIB");
        else
            Serial.println("TEST");

        if (!xy_or_z)
            Serial.println("XY");
        else
            Serial.println("Z");
        int secmillis = millis() - calTimeWas;
        if (secmillis > CALTIME)
            break; // Exit after time up.
        Serial.println("--> ");
        Serial.println((int)((CALTIME - secmillis) / 1000));
        delay(10);
    } // while cal
    if (xy_or_z == 0)
    {
        offx = ((maxx - minx) / 2) + minx;
        offy = ((maxy - miny) / 2) + miny;
        offz = ((maxz - minz) / 2) + minz; // TODO
    }
    else
        // offz = ((maxz - minz) / 2) + minz;
        if (eeprom_write)
        {
            EEPROMHelper::eeprom_write_min_max_for_xyz(xy_or_z, minx, maxx, miny, maxy, minz, maxz);
        }

    unsigned long dispExitTimeWas = millis();
    while (1)
    {
        // Make sure this does not repeat endlessly!
        if (eeprom_write)
            Serial.println("EEPROM Written");
        else
            Serial.println("TEST DMY Write");
        if (millis() - dispExitTimeWas > 2000)
            break;
        delay(10);
    }
    calc_offsets();
}
// Add other Magnetometer related methods implementation here