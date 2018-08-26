#ifndef EEMEM_H
#define EEMEM_H

#include <Arduino.h>

struct eeSet // EEPROM backed data
{
  uint16_t size;          // if size changes, use defauls
  uint16_t sum;           // if sum is diiferent from memory struct, write
  char     szSSID[32];
  char     szSSIDPassword[64];
  int8_t   tz;            // Timezone offset
  uint8_t  dst;
  bool     bEnableOLED;
  uint16_t rate;
};

extern eeSet ee;

class eeMem
{
public:
  eeMem();
  void update(void);
private:
  uint16_t Fletcher16( uint8_t* data, int count);
};

extern eeMem eemem;

#endif // EEMEM_H
