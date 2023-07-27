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
  static unsigned long BLTimeWas = millis();
  int16_t x, y, z; // Raw compass output values.
  int bearing;
  if (digitalRead(BUTTON_CAL) == 0)
  {
    magHelper.calibrate(1, 0);
  }
  if (digitalRead(BUTTON_TEST) == 0)
  {
    magHelper.calibrate(0, 0);
  }
  magHelper.getMagnetometer(&x, &y, &z);

  // Serial.print("Accel: ");
  // Serial.print("X = ");
  // Serial.print(accX);
  // Serial.print(", Y = ");
  // Serial.print(accY);
  // Serial.print(", Z = ");
  // Serial.print(accZ);
  Serial.print("mag: x=");
  Serial.print(x);
  Serial.print("\t y=");
  Serial.print(y);
  Serial.print("\t z=");
  Serial.println(z);
  // bearing = (int)magHelper.getMagnetometerBearing(x, y, z);

  // int tbearing = magHelper.getMagnetometerTiltCompensatedBearing(x, y, z);

  // Serial.print("\tbearing:\t");
  // Serial.print(bearing);
  // Serial.print("\tt_bearing:\t");
  // Serial.println(tbearing);
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