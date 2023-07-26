#include "EEPROMHelper.h"
#include <EEPROM.h>
#include "Constants.h"

void EEPROMHelper::eeprom_read_min_max_for_xyz(int &minx, int &maxx, int &miny, int &maxy, int &minz, int &maxz)
{
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
}

void EEPROMHelper::eeprom_write_min_max_for_xyz(byte xy_or_z, int minx, int maxx, int miny, int maxy, int minz, int maxz)
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