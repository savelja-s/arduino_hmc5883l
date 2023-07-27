#include <Arduino.h>
#include "Wire.h"
#include "MPUHelper.h"
#include "Constants.h"
#include "MagnetometerHelper.h"
#include "ButtonHelper.h"

MPUHelper mpuHelper;
MagnetometerHelper magHelper;
ButtonHelper buttonHelper;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  mpuHelper.initialize();
  magHelper.initialize(mpuHelper.mpu);
  magHelper.read_with_eeprom_min_max_for_xyz();
  magHelper.calc_offsets();
  buttonHelper.initialize();
}

void loop()
{
  int16_t ax, ay, az;
  int16_t mx, my, mz;
  int bearing;
  static unsigned long BLTimeWas = millis();
  if (digitalRead(BUTTON_CAL) == 0)
  {
    magHelper.calibrate(1, 0);
    // calibrate(1, 1);
  }
  if (digitalRead(BUTTON_TEST) == 0)
  {
    magHelper.calibrate(0, 0);
    // calibrate(0, 1);
  }
  mpuHelper.mpu.getAcceleration(&ax, &ay, &az);
  magHelper.getMagnetometer(&mx, &my, &mz);

  Serial.print("Accelerometer: ");
  Serial.print("X = ");
  Serial.print(ax);
  Serial.print(", Y = ");
  Serial.print(ay);
  Serial.print(", Z = ");
  Serial.print(az);
  Serial.print("\t mag:x=");
  Serial.print(ax);
  Serial.print("\ty=");
  Serial.print(ay);
  Serial.print("\tz=");
  Serial.print(az);
  Serial.print("\t");
  bearing = (int)magHelper.getMagnetometerBearing(ax, ay, az);

  int tbearing = magHelper.getMagnetometerTiltCompensatedBearing(ax, ay, az);

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