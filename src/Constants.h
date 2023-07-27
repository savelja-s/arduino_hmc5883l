#ifndef CONSTANTS_H
#define CONSTANTS_H

#define HMC5883L_ARRD 0x1E
#define HMC5883L_RA_STATUS 0x09
#define HMC5883L_RA_DATAX_H 0x03

#define BUTTON_CAL 10
#define BUTTON_TEST 11
// #define BUTTON_CAL 15
// #define BUTTON_TEST 14

#define LED_PIN 17

#define EEADDR 66     // Start location to write EEPROM data.
#define CALTIME 20000 // In ms.
#define SMOOTH_ACCELL 20

// Datasheet: OPERATION AT VOLTAGES OTHER THAN 2.5 V
// 3v3 X,Y 25mg too high, z 20mg too low
// 3V3 lsb value 265/g c  (g/265)=0.03698
// 2V5 lsb value 256/g   (g/256)=0.03828 z axis unaffected by voltage supply.
#define ADXL345_LSBVAL_3V3 3.698E-3
#define ADXL345_LSBVAL_2V5 3.828E-3

#endif