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
  int16_t accX, accY, accZ;
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
  Serial.println("------------------REEEEAAADDD MPU--------------2222");
  mpuHelper.readAccelerometer(accX, accY, accZ);
  Serial.println("------------------REEEEAAADDD COMPASS--------------");
  magHelper.readXYZ();

  Serial.print("Accelerometer: ");
  Serial.print("X = ");
  Serial.print(accX);
  Serial.print(", Y = ");
  Serial.print(accY);
  Serial.print(", Z = ");
  Serial.println(accZ);
  Serial.print("mag:\t");
  Serial.print(magHelper.mx);
  Serial.print("\t");
  Serial.print(magHelper.my);
  Serial.print("\t");
  Serial.print(magHelper.mz);
  Serial.print("\t");
  bearing = (int)magHelper.getMagnetometerBearing();

  int tbearing = magHelper.getMagnetometerTiltCompensatedBearing();

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

  delay(1000); // Затримка для читабельного виводу
}