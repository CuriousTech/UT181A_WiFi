/**************************************************************
 * WiFiManager is a library for the ESP8266/Arduino platform
 * (https://github.com/esp8266/Arduino) to enable easy
 * configuration and reconfiguration of WiFi credentials and
 * store them in EEPROM.
 * inspired by http://www.esp8266.com/viewtopic.php?f=29&t=2520
 * https://github.com/chriscook8/esp-arduino-apboot
 * Built by AlexT https://github.com/tzapu
 * Licensed under MIT license
 **************************************************************/

#include "WiFiManager.h"
#include "eeMem.h"

#define OLED_ENABLE

#ifdef OLED_ENABLE
#include "ssd1306_i2c.h"
#include "icons.h"
extern SSD1306 display;
#endif

WiFiManager::WiFiManager()
{
}

// Start WiFi connection or AP if no SSID
void WiFiManager::autoConnect(char const *apName, const char *pPass)
{
  _apName = apName;
  _pPass = pPass;

//  DEBUG_PRINT("");
//    DEBUG_PRINT("AutoConnect");
    
  if ( ee.szSSID[0] ) {
    DEBUG_PRINT("Waiting for Wifi to connect");

    WiFi.mode(WIFI_STA);
    WiFi.begin(ee.szSSID, ee.szSSIDPassword);
    WiFi.setHostname(apName);
    _state = ws_connecting;
    _timer = 50;
  }
  else
  {
    startAP();
  }
}

// Start AP mode
void WiFiManager::startAP()
{
  //setup AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(_apName);
  DEBUG_PRINT("Started Soft Access Point");
#ifdef OLED_ENABLE
  display.print("AP started:");
  IPAddress apIp = WiFi.softAPIP();
  display.print(apIp.toString());
#endif

  DEBUG_PRINT(WiFi.softAPIP());

  if (!MDNS.begin(_apName))
    DEBUG_PRINT("Error setting up MDNS responder!");
  WiFi.scanNetworks();

  _state = ws_config;
}

// return current connection sate
int WiFiManager::state()
{
  return _state;
}

// returns true once after a connection is made (for time)
bool WiFiManager::connectNew()
{
  if(_state == ws_connectSuccess)
  {
    _state = ws_connected;
    return true;
  }
  return false; 
}

// returns true if in config/AP mode
bool WiFiManager::isCfg(void)
{
  return (_state == ws_config);
}

void WiFiManager::setPass(const char *p)
{
  strncpy(ee.szSSIDPassword, p, sizeof(ee.szSSIDPassword) );
  eemem.update();
  DEBUG_PRINT("Updated EEPROM.  Restaring.");
  autoConnect(_apName, _pPass);
}

// Called at any frequency
void WiFiManager::service()
{
  static int s = 1; // do first list soon
  static uint32_t m;
  static uint16_t ticks;

  if((millis() - m) > 200)
  {
    m = millis();
    ticks++;
    if(_state == ws_connecting)
    {
#ifdef DEBUG
      Serial.print(".");
#endif
#ifdef OLED_ENABLE
      display.clear();
      display.drawXbm(34,10, 60, 36, WiFi_Logo_bits);
      display.setColor(INVERSE);
      display.fillRect(10, 10, 108, 44);
      display.setColor(WHITE);
      drawSpinner(4, (_timer / 10) % 4);
      display.display();
#endif
      if(_timer)
      {
        if (WiFi.status() == WL_CONNECTED)
        {
          DEBUG_PRINT("Connected");
          _state = ws_connectSuccess;
        }
        else if(--_timer == 0)
        {
          DEBUG_PRINT("");
          DEBUG_PRINT("Could not connect to WiFi");
          startAP();
        }
      }
      return;
    }
  }

  if(ticks < 5)
    return;
  ticks = 0;

  if(_state != ws_config)
    return;
  if(--s)
    return;
  s = 60;
  int n = WiFi.scanNetworks(); // scan for stored SSID each minute
  if(n == 0 )
    return;

  for (int i = 0; i < n; i++)
  {
#ifdef OLED_ENABLE
    display.print(WiFi.SSID(i));
#endif
    if(WiFi.SSID(i) == ee.szSSID) // found cfg SSID
    {
      DEBUG_PRINT("SSID found.  Restarting.");
      autoConnect(_apName, _pPass);
      s = 5; // set to 5 seconds in case it fails again
    }
  }
}

String WiFiManager::page()
{
  String s = HTTP_HEAD;
  s += HTTP_SCRIPT;
  s += HTTP_STYLE;
  s += HTTP_HEAD_END;

  for (int i = 0;  WiFi.SSID(i).length(); ++i)
  {
    DEBUG_PRINT(WiFi.SSID(i));
    DEBUG_PRINT(WiFi.RSSI(i));
    String item = HTTP_ITEM;
    item.replace("{v}", WiFi.SSID(i) );
    s += item;
  }
  WiFi.scanDelete();
  String form = HTTP_FORM;
  form.replace("$key", _pPass );
  s += form;
  s += HTTP_END;
  return s;
}

void WiFiManager::drawSpinner(int count, int active) {
#ifdef OLED_ENABLE
  for (int i = 0; i < count; i++) {
    const char *xbm;
    if (active == i) {
       xbm = active_bits;
    } else {
       xbm = inactive_bits;  
    }
    display.drawXbm(64 - (12 * count / 2) + 12 * i,56, 8, 8, xbm);
  }
#endif
}
