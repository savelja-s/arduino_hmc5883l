#ifndef EEPROM_HELPER_H
#define EEPROM_HELPER_H
#include <Arduino.h>

class EEPROMHelper
{
public:
    static void eeprom_read_min_max_for_xyz(int &minx, int &maxx, int &miny, int &maxy, int &minz, int &maxz);
    static void eeprom_write_min_max_for_xyz(byte xy_or_z, int minx, int maxx, int miny, int maxy, int minz, int maxz);
};

#endif