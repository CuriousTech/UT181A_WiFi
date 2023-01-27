#ifndef EEMEM_H
#define EEMEM_H

#include <Arduino.h>

#define EESIZE (offsetof(eeMem, end) - offsetof(eeMem, size) )

class eeMem
{
public:
  eeMem();
  void update(void);
private:
  uint16_t Fletcher16( uint8_t* data, int count);
public:
  uint16_t size;          // if size changes, use defauls
  uint16_t sum;           // if sum is diiferent from memory struct, write
  char     szSSID[32] = "";
  char     szSSIDPassword[64] = "";
  int8_t   tz = -5;            // Timezone offset
  uint8_t  dst = 0;
  bool     bEnableOLED = false;
  uint8_t  exportFormat = 0;
  uint16_t rate = 0;
  uint32_t updateTime = 0;
  uint8_t  end;
};

extern eeMem ee;

#endif // EEMEM_H
