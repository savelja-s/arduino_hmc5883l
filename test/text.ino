// QST5883L compass and SSD1306 OLED display
// 3 Axis Magnetometer Tilt compensation using ADXl345
//
// Coords x ahead, y right - follows aircraft convention.
//
// Copyright John Main - Free for non commercial use.
//
#include "Wire.h"; #For 5883
#include <SPI.h>  ;# For SSD1306 board.
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "I2Cdev.h"
#include "math.h"
#include "ADXL345.h"

#define OLED_MOSI 11
#define OLED_CLK 13 // LED_BUILTIN
#define OLED_DC 9
#define OLED_CS 8
#define OLED_RESET 10

#define BUTTON_CAL 2
#define BUTTON_TEST 5
#define LED 4 // LED_BUILTIN pin 13 is also SPICLK SCK

#define EEADDR 98     // 66 // Start location to write EEPROM data.
#define CALTIME 10000 // In ms.
#define SMOOTH_ACCELL 20

#define SCREEN_HEIGHT 128 // The screen is rotated in sw.
#define SCREEN_WIDTH 64   // This is default height.

ADXL345 accel;

static byte stat, ovfl, skipped;
static int minx, maxx, miny, maxy, minz, maxz;
static int offx = 0, offy = 0, offz = 0;

static int degTest = 0, updateSpeed = 1000;
static byte showMT = 0;

const int radius = SCREEN_WIDTH / 2;
const int midx = radius; // Middle of compass rose.
const int midy = radius; // Middle of compass rose.

Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS); // hw spi

/////////////////////////////////////////////////////////////
// Generic I2C write to I2C device with address, i2cAddr
// Write to address regaddr within device
// Write the byte d
void I2C_write_AddrDev_AddrReg_Byte(byte i2cAddr, byte regaddr, byte d)
{
   Wire.beginTransmission(i2cAddr);
   Wire.write(regaddr);
   Wire.write(d);
   Wire.endTransmission();
}

/////////////////////////////////////////////////////////////
void calc_offsets(void)
{
   offx = (maxx + minx) / 2; // maxx/2 - minx/2 + 2minx/2
   offy = (maxy + miny) / 2;
   offz = (maxz + minz) / 2;
}

/////////////////////////////////////////////////////////////
byte magnetometerReady(void)
{
   // Data ready?
   Wire.beginTransmission(0x0d); // Read from status reg
   Wire.write(0x06);
   int num = Wire.requestFrom((byte)0x0d, (byte)1);
   stat = Wire.read(); // DOR Data out Ready (SKIPPED).
   Wire.endTransmission();

   ovfl = stat & 0x02;
   skipped = stat & 0x04;

   return (stat && 0x01); // 0x01 is the DRDY Data Ready flag
}

/////////////////////////////////////////////////////////////
// If data is not ready x,y,z are not changed.
// raw data is aligned to HMC5883 coords
byte getMagnetometerRaw(int16_t *x, int16_t *y, int16_t *z)
{

   if (!magnetometerReady())
      return 0;

   Wire.beginTransmission(0x0d);
   Wire.write(0x00); // read from address zero = x,y,z registers.
   int err = Wire.endTransmission();

   if (!err)
   {
      Wire.requestFrom((byte)0x0d, (byte)6); // Blocking?
      while (Wire.available() < 6)
         ; // Wait if above blocking then this not needed.
      *x = (int16_t)(Wire.read() | Wire.read() << 8);
      *y = (int16_t)(Wire.read() | Wire.read() << 8);
      *z = (int16_t)(Wire.read() | Wire.read() << 8);
   }
   return 1;
}

/////////////////////////////////////////////////////////////
// Orient to board coordinates
void getMagnetometer(int16_t *x, int16_t *y, int16_t *z)
{

   if (!getMagnetometerRaw(x, y, z))
      return;

   // modify for accel board orientation (board x = up, y to left).
   *y = -(*y);
}

/////////////////////////////////////////////////////////////
float getMagnetometerBearing(int16_t x, int16_t y, int16_t z)
{

   float atan2val = 180 / M_PI * atan2((float)y, (float)x); // NEW coords.
   int b = (int)(-atan2val + 360) % 360;
   return b;
}

/////////////////////////////////////////////////////////////
// Blocking: Waits in this function for reading to be ready.
void readMagnetometer(int16_t *x, int16_t *y, int16_t *z)
{
   while (!magnetometerReady())
      ;
   getMagnetometer(x, y, z); // Note: Addresses of pointers passed.
}

///////////////////////////////////////////////////////////////
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

///////////////////////////////////////////////////////////////
// Returns angle calculated from z, y, z accelerometers.
//
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

   accel.getAcceleration(&ax, &ay, &az);
   x = (float)ax * ADXL345_LSBVAL_3V3 - 25E-3;
   y = (float)ay * ADXL345_LSBVAL_3V3 - 25E-3;
   z = (float)az * ADXL345_LSBVAL_2V5 + 20e-3;

   r = sqrt(x * x + y * y + z * z);

   // modify for accel board orientation (board x = up, y to left).
   y = -y;
   *pitch = 180 / M_PI * asin(x / r);
   *roll = 180 / M_PI * -asin(y / r);
}

///////////////////////////////////////////////////////////////
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

/////////////////////////////////////////////////////////////
void setup()
{

   pinMode(LED, OUTPUT);

   pinMode(BUTTON_CAL, INPUT_PULLUP);
   pinMode(BUTTON_TEST, INPUT_PULLUP);

   Wire.begin();          // Start I2C
   Wire.setClock(100000); // Test at high speed

   display.begin(SSD1306_SWITCHCAPVCC);
   display.clearDisplay();
   display.setTextSize(1);
   //   display.setTextColor(WHITE);       // Does NOT erase background.
   display.setTextColor(WHITE, BLACK); // Does erase background.

   // Datasheet suggests this for chip startup HMC5883L.
   // Set reset register.Datasheet suggests this as 0x01.
   I2C_write_AddrDev_AddrReg_Byte(0x0d, 0x0b, 1);
   // Control reg : Mode:continuous, ODR:10Hz, RNG:2G, OSR:512 (over sample)
   //   I2C_write_AddrDev_AddrReg_Byte(0x0d,0x09,B00000001);
   // In lab gauss > 2G so need higher range.
   I2C_write_AddrDev_AddrReg_Byte(0x0d, 0x09, B00010001);

   // Read EEPROM
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

   display.setRotation(3);

   Serial.begin(115200);
   Serial.println("Compass hmc5883L");

   accel.initialize();
   Serial.println(F("Testing device connections..."));
   Serial.print(F("ADXL345 connection "));
   Serial.println(accel.testConnection() ? F("successful") : F("failed"));

   accel.setFullResolution(ADXL345_RANGE_2G);
   accel.setRate(ADXL345_RATE_100); // This is default but shows the value.

   accel.setFullResolution(1); // 0 => 10 bit mode.
   accel.setLowPowerEnabled(0);
   accel.setRange(0); // 0 => 2g, 3 => 16g
}

/////////////////////////////////////////////////////////////
// Normalized line drawing (0,0) = bot left
void drawLine(int x1, int y1, int x2, int y2)
{
   display.drawLine(x1, SCREEN_HEIGHT - y1, x2, SCREEN_HEIGHT - y2, 1);
}

/////////////////////////////////////////////////////////////
// Normalized line drawing (0,0) = bot left
void drawCircle(int x, int y, int radius)
{
   display.drawCircle(x, SCREEN_HEIGHT - y, radius - 1, 1);
}

/////////////////////////////////////////////////////////////
void drawBearing(float bearing, int midx, int midy, int radius)
{
   bearing += 90; // Rotate arctan2 to vertical
   int opp = sin(bearing * M_PI / 180) * radius;
   int adj = cos(bearing * M_PI / 180) * radius;
   // Screen +y is down but drawLine adjusts it up.
   drawLine(midx, midy, midx + adj, midy + opp);
}

/////////////////////////////////////////////////////////////
// Can choose whether to write to EEPROM for testing.
void calibrate(boolean eeprom_write, byte xy_or_z)
{
   unsigned long calTimeWas = millis();

   int x, y, z;
   float deg = 0, deg2 = 0;

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

      display.clearDisplay();

      display.setTextSize(2);
      display.setCursor(0, 0);

      if (eeprom_write)
         display.print("CALIB");
      else
         display.print("TEST");

      display.setCursor(0, 16);
      if (!xy_or_z)
         display.print("XY");
      else
         display.print("Z");

      int secmillis = millis() - calTimeWas;
      if (secmillis > CALTIME)
         break; // Exit after time up.

      int secs = (int)((CALTIME - secmillis + 1000) / 1000);
      display.setCursor(0, 32);
      display.print("--> ");
      display.print((int)((CALTIME - secmillis) / 1000));

      drawBearing((int)deg2, midx, midy, radius);
      drawBearing((int)deg, midx, midy, radius);

      deg = (360.0 / CALTIME) * secmillis; // Rotate a line for countdown duration.

      deg2 += deg; // Rotate a line for countdown duration. Fun.
      deg = fmod(deg, 360);

      for (int i = 0; i < 360; i += 45) // 45 Degree spokes (rotating)
         drawBearing(i + (45 / secs) * 10, midx, midy, radius - 7);

      display.display(); // Update display.
      delay(10);
   } // while cal

   if (xy_or_z == 0)
   {
      offx = ((maxx - minx) / 2) + minx;
      offy = ((maxy - miny) / 2) + miny;
   }
   else
      offz = ((maxz - minz) / 2) + minz;

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

      display.clearDisplay();

      display.setTextSize(2);
      display.setCursor(0, 0);

      // Make sure this does not repeat endlessly!
      if (eeprom_write)
         display.print("EEPROM Written");
      else
         display.print("TEST DMY Write");
      if (millis() - dispExitTimeWas > 2000)
         break;

      display.display(); // Update display.
      delay(10);
   }
   calc_offsets();
}

/////////////////////////////////////////////////////////////
void draw(int Mx, int My, int bearing, int tbearing)
{
   int lineH, lineNum;

   // SSD1306 init.
   display.clearDisplay();

   // display fancy stuff
   drawCircle(midx, midy, radius - 1); // -1 as circle is 1 bigger
   drawCircle(midx, midy, (radius - 1) / 2);

   for (int i = 0; i < 360; i += 45) // 45 Degree spokes
      drawBearing(i, midx, midy, radius - 7);

   drawBearing(bearing, midx, midy, radius);
   drawBearing(0, midx, midy, radius); // North

   display.setTextSize(2);
   lineH = 16;

   lineNum = 0;
   // Bearing
   display.setCursor(15, lineH * lineNum);
   display.print(bearing);
   display.print((char)247);
   lineNum++; // Next line

   // Tilt Bearing
   display.setCursor(15, lineH * lineNum);
   display.print(tbearing);
   display.print((char)247);
   lineNum++; // Next line

   drawBearing(tbearing, midx, midy, radius - 10);

   if (Mx > maxy || Mx < minx || My > maxy || My < miny)
   {
      display.setCursor(0, lineH * lineNum++);
      display.print("*CAL*");
   }

   display.setTextSize(1);
   lineH = 8;
   lineNum = 0;

   // Roll Pitch
   float r, p;
   getRollPitch(&r, &p);

   display.setCursor(0, 120);
   display.print(r, 1);
   display.print((char)247);

   display.setCursor(40, 120);
   display.print(p, 1);
   display.print((char)247);

   display.display(); // Update display.
}

/////////////////////////////////////////////////////////////
void loop(void)
{
   static unsigned long BLTimeWas = millis();
   int x, y, z; // Raw compass output values.
   int bearing, i;

   if (digitalRead(BUTTON_CAL) == 0)
   {
      calibrate(1, 0);
      calibrate(1, 1);
   }
   if (digitalRead(BUTTON_TEST) == 0)
   {
      calibrate(0, 0);
      calibrate(0, 1);
   }

   getMagnetometer(&x, &y, &z);

   bearing = (int)getMagnetometerBearing(x - offx, y - offy, z - offz);

   int tbearing = getMagnetometerTiltCompensatedBearing(x - offx, y - offy, z - offz);

   draw(x, y, bearing, tbearing);

   if (millis() - BLTimeWas > 400)
   { // LED toggle
      BLTimeWas = millis();
      static byte togLED = 0;
      togLED = !togLED;
      if (togLED)
         digitalWrite(LED, HIGH);
      else
         digitalWrite(LED, LOW);
   }
}

// End of QST5883L OLED compass -
// 3 axis magnetometer tilt compensation example.
