#include "MagnetometerHelper.h"
#include "HMC5883L.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "EEPROMHelper.h"
#include "Constants.h"

MagnetometerHelper::MagnetometerHelper()
{
    int offx = 0, offy = 0, offz = 0;
}
void MagnetometerHelper::read_with_eeprom_min_max_for_xyz()
{
    EEPROMHelper::eeprom_read_min_max_for_xyz(minx, maxx, miny, maxy, minz, maxz);
}

void MagnetometerHelper::initialize(MPU6050 initMpu)
{
    accelgyro = initMpu;
    // initialize device
    Serial.println("Initializing I2C devices...");
    compass.initialize();
    Serial.println(compass.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
}

void MagnetometerHelper::calc_offsets(void)
{
    offx = (maxx + minx) / 2; // maxx/2 - minx/2 + 2minx/2
    offy = (maxy + miny) / 2;
    offz = (maxz + minz) / 2;
}
void MagnetometerHelper::readXYZ()
{
    compass.getHeading(&mx, &my, &mz);
}

byte MagnetometerHelper::magnetometerReady(void)
{
    // Data ready?
    Wire.beginTransmission(0x1E); // HMC5883L I2C address
    Wire.write(0x09);             // Read from status reg
    int num = Wire.endTransmission();
    Wire.requestFrom((byte)0x1E, (byte)1);
    stat = Wire.read();    // RDY bit (bit 0)
    ovfl = stat & 0x02;    // Extract OVFL bit (bit 1)
    skipped = stat & 0x04; // Extract SKIPPED bit (bit 2)
    return (stat & 0x01);  // 0x01 is the RDY (Ready) flag
}
float MagnetometerHelper::getMagnetometerBearing()
{
    int16_t x = mx - offx;
    int16_t y = my - offy;
    int16_t z = mz - offz;
    float atan2val = 180 / M_PI * atan2((float)y, (float)x); // NEW coords.
    int b = (int)(-atan2val + 360) % 360;
    return b;
}
float MagnetometerHelper::getMagnetometerTiltCompensatedBearing()
{
    int x = mx - offx;
    int y = my - offy;
    int z = mz - offz;
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

    // modify for accel board orientation (board x = up, y to left).
    y = -y;
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

    //    int x,y,z;
    //    float deg=0,deg2=0;

    //    readMagnetometer(&x, &y, &z);

    maxx = minx = mx; // Set initial values to current magnetometer readings.
    maxy = miny = my;
    maxz = minz = mz;

    delay(300); // Allow button release.

    while (1)
    { // Calibration loop.

        if (digitalRead(BUTTON_CAL) == 0 || digitalRead(BUTTON_TEST) == 0)
        {
            delay(300); // Allow button release.
            return;     // Abort
        }

        if (magnetometerReady())
            readXYZ();
        if (mx > maxx)
            maxx = mx;
        if (mx < minx)
            minx = mx;
        if (my > maxy)
            maxy = my;
        if (my < miny)
            miny = my;
        if (mz > maxz)
            maxz = mz;
        if (mz < minz)
            minz = mz;

        //   display.clearDisplay();

        //   display.setTextSize(2);
        //   display.setCursor(0,0);

        if (eeprom_write)
            Serial.println("CALIB");
        else
            Serial.println("TEST");

        //   display.setCursor(0,16);
        if (!xy_or_z)
            Serial.println("XY");
        else
            Serial.println("Z");

        int secmillis = millis() - calTimeWas;
        if (secmillis > CALTIME)
            break; // Exit after time up.

        int secs = (int)((CALTIME - secmillis + 1000) / 1000);
        //   display.setCursor(0,32);
        Serial.println("--> ");
        Serial.println((int)((CALTIME - secmillis) / 1000));

        //   drawBearing((int)deg2, midx, midy , radius);
        //   drawBearing((int)deg, midx, midy , radius);

        //   deg = (360.0/CALTIME)*secmillis; // Rotate a line for countdown duration.

        //   deg2 += deg; // Rotate a line for countdown duration. Fun.
        //   deg = fmod(deg,360);

        //   for(int i=0;i<360;i+=45)   // 45 Degree spokes (rotating)
        // drawBearing(i + (45/secs)*10, midx, midy, radius-7);

        //   display.display();  // Update display.
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

        //   display.clearDisplay();

        //   display.setTextSize(2);
        //   display.setCursor(0,0);

        // Make sure this does not repeat endlessly!
        if (eeprom_write)
            Serial.println("EEPROM Written");
        else
            Serial.println("TEST DMY Write");
        if (millis() - dispExitTimeWas > 2000)
            break;

        //   display.display();  // Update display.
        delay(10);
    }
    calc_offsets();
}
// Add other Magnetometer related methods implementation here