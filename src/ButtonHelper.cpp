#include <Arduino.h>
#include "ButtonHelper.h"
#include "Constants.h"

ButtonHelper::ButtonHelper() {}
void ButtonHelper::initialize()
{
    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_CAL, INPUT_PULLUP);
    pinMode(BUTTON_TEST, INPUT_PULLUP);
}