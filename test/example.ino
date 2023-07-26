#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "HMC5883L.h"
#include <EEPROM.h>

#define BUTTON_CAL 15
#define BUTTON_TEST 14
#define EEADDR 66     // Start location to write EEPROM data.
#define CALTIME 20000 // In ms.
#define SMOOTH_ACCELL 20

MPU6050 accelgyro;
HMC5883L mag;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

Quaternion q;        // [w, x, y, z]         quaternion container   // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;
uint8_t fifoBuffer[64];
#define LED_PIN 17
bool blinkState = false;
static byte stat, ovfl, skipped;
static int minx, maxx, miny, maxy, minz, maxz;
static int offx = 0, offy = 0, offz = 0;

void calc_offsets(void)
{
    offx = (maxx + minx) / 2; // maxx/2 - minx/2 + 2minx/2
    offy = (maxy + miny) / 2;
    offz = (maxz + minz) / 2;
}

byte magnetometerReady(void)
{
    // Data ready?
    Wire.beginTransmission(0x1E); // HMC5883L I2C address
    Wire.write(0x09);             // Read from status reg
    int num = Wire.endTransmission();
    Wire.requestFrom((byte)0x1E, (byte)1);
    stat = Wire.read(); // RDY bit (bit 0)

    ovfl = stat & 0x02;    // Extract OVFL bit (bit 1)
    skipped = stat & 0x04; // Extract SKIPPED bit (bit 2)
    return (stat & 0x01);  // 0x01 is the RDY (Ready) flag
}

void calibrate(boolean eeprom_write, byte xy_or_z)
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
            mag.getHeading(&mx, &my, &mz);
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
            int EEAddr = EEADDR;

            if (xy_or_z == 0)
            {
                int EEAddr = EEADDR;
                EEPROM.put(EEAddr, minx);
                EEAddr += sizeof(minx);
                EEPROM.put(EEAddr, maxx);
                EEAddr += sizeof(maxx);
                EEPROM.put(EEAddr, miny);
                EEAddr += sizeof(miny);
                EEPROM.put(EEAddr, maxy);
                EEAddr += sizeof(maxy);
                EEAddr = EEADDR + 4 * sizeof(minx);
                EEPROM.put(EEAddr, minz);
                EEAddr += sizeof(minz);
                EEPROM.put(EEAddr, maxz);
                EEAddr += sizeof(maxz);
            }
            else
            {
                int EEAddr = EEADDR + 4 * sizeof(minx);
                EEPROM.put(EEAddr, minz);
                EEAddr += sizeof(minz);
                EEPROM.put(EEAddr, maxz);
                EEAddr += sizeof(maxz);
            }
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

void calibrate2()
{
    unsigned long calTimeWas = millis();
    byte cal = 1, startCal = 1;
    //   int16_t x, y, z;
    float deg = 0, deg2 = 0;

    //   readMagnetometer(&x, &y, &z);

    maxx = minx = mx; // Set initial values to current magnetometer readings.
    maxy = miny = my;

    while (cal)
    {
        if (magnetometerReady())
            mag.getHeading(&mx, &my, &mz);
        if (mx > maxx)
            maxx = mx;
        if (mx < minx)
            minx = mx;
        if (my > maxy)
            maxy = my;
        if (my < miny)
            miny = my;

        Serial.println("CALIBRATE ");

        int secmillis = millis() - calTimeWas;
        int secs = (int)((CALTIME - secmillis + 1000) / 1000);
        Serial.println("--> ");
        Serial.println((CALTIME - secmillis) / 1000);

        if (secs == 0)
        { // Cal has ended
            calc_offsets();
            cal = 0;

            int EEAddr = EEADDR;
            EEPROM.put(EEAddr, minx);
            EEAddr += sizeof(minx);
            EEPROM.put(EEAddr, maxx);
            EEAddr += sizeof(maxx);
            EEPROM.put(EEAddr, miny);
            EEAddr += sizeof(miny);
            EEPROM.put(EEAddr, maxy);
            EEAddr += sizeof(maxy);
            Serial.println("EEPROM Written"); // Make sure this does not repeat on the terminal!
        }

        delay(10);
    } // while cal
}

float getMagnetometerTiltCompensatedBearing(int x, int y, int z)
{
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

void getRollPitch(float *roll, float *pitch)
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

void getRollPitchRawFloat(float *roll, float *pitch)
{
    float r, x, y, z;
    int16_t ax, ay, az;

// Datasheet: OPERATION AT VOLTAGES OTHER THAN 2.5 V
// 3v3 X,Y 25mg too high, z 20mg too low
// 3V3 lsb value 265/g c  (g/265)=0.03698
// 2V5 lsb value 256/g   (g/256)=0.03828 z axis unaffected by voltage supply.
#define ADXL345_LSBVAL_3V3 3.698E-3
#define ADXL345_LSBVAL_2V5 3.828E-3

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

float getMagnetometerBearing(int16_t x, int16_t y, int16_t z)
{

    float atan2val = 180 / M_PI * atan2((float)y, (float)x); // NEW coords.
    int b = (int)(-atan2val + 360) % 360;
    return b;
}
void setup()
{
    Wire.begin();
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    mag.initialize();
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = accelgyro.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    accelgyro.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        accelgyro.CalibrateAccel(6);
        accelgyro.CalibrateGyro(6);
        accelgyro.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        accelgyro.setDMPEnabled(true);

        // get expected DMP packet size for later comparison
        packetSize = accelgyro.dmpGetFIFOPacketSize();
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
    accelgyro.setI2CMasterModeEnabled(false);
    accelgyro.setI2CBypassEnabled(true);
    accelgyro.setSleepEnabled(false);

    Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_CAL, INPUT_PULLUP);
    pinMode(BUTTON_TEST, INPUT_PULLUP);

    // Read EEPROM offset data
    int EEAddr = EEADDR;
    EEPROM.get(EEAddr, minx);
    EEAddr += sizeof(minx);
    EEPROM.get(EEAddr, maxx);
    EEAddr += sizeof(maxx);
    EEPROM.get(EEAddr, miny);
    EEAddr += sizeof(miny);
    EEPROM.get(EEAddr, maxy);
    EEAddr += sizeof(maxy);
    EEPROM.get(EEAddr, minz);
    EEAddr += sizeof(minz);
    EEPROM.get(EEAddr, maxz);
    EEAddr += sizeof(maxz);
    calc_offsets();
}

void loop()
{
    static unsigned long BLTimeWas = millis();
    int bearing, i;
    if (digitalRead(BUTTON_CAL) == 0)
    {
        calibrate(1, 0);
        // calibrate(1, 1);
    }
    if (digitalRead(BUTTON_TEST) == 0)
    {
        calibrate(0, 0);
        // calibrate(0, 1);
    }
    // accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mag.getHeading(&mx, &my, &mz);

    // display tab-separated accel/gyro x/y/z values
    // Serial.print("a/g:\t");
    // Serial.print(ax); Serial.print("\t");
    // Serial.print(ay); Serial.print("\t");
    // Serial.print(az); Serial.print("\t");
    // Serial.print(gx); Serial.print("\t");
    // Serial.print(gy); Serial.print("\t");
    // Serial.print(gz);Serial.print("\t");

    Serial.print("mag:\t");
    Serial.print(mx);
    Serial.print("\t");
    Serial.print(my);
    Serial.print("\t");
    Serial.print(mz);
    Serial.print("\t");

    // To calculate heading in degrees. 0 degree indicates North
    // float heading = atan2((float)(my - offy), (float)(mx - offx));
    // if(heading < 0)
    //   heading += 2 * M_PI;
    // Serial.print("heading:\t");
    // Serial.print(heading * 180/M_PI);
    // static int bearing;
    // int atan2val = 180 / M_PI * atan2((float)(mx - offx), (float)(my - offy));
    // bearing = (-atan2val + 360) % 360;

    bearing = (int)getMagnetometerBearing(mx - offx, my - offy, mz - offz);

    int tbearing = getMagnetometerTiltCompensatedBearing(mx - offx, my - offy, mz - offz);

    Serial.print("\tbearing:\t");
    Serial.print(bearing);
    Serial.print("\tt_bearing:\t");
    Serial.println(tbearing);
    // blink LED to indicate activity
    if (millis() - BLTimeWas > 400)
    { // LED toggle
        BLTimeWas = millis();
        static byte togLED = 0;
        togLED = !togLED;
        if (togLED)
            digitalWrite(LED_PIN, HIGH);
        else
            digitalWrite(LED_PIN, LOW);
    }
}