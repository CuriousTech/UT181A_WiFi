/**The MIT License (MIT)

Copyright (c) 2018 by Greg Cunningham, CuriousTech

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// Build with Arduino IDE 1.8.5, esp8266 SDK 2.4.2

//uncomment to enable Arduino IDE Over The Air update code
#define OTA_ENABLE

//#define USE_SPIFFS

#include <ssd1306_i2c.h> // https://github.com/CuriousTech/WiFi_Doorbell/tree/master/Libraries/ssd1306_i2c

#include <EEPROM.h>
#include <ESP8266mDNS.h>
#include <ESP8266NetBIOS.h>
#include "WiFiManager.h"
#include <ESPAsyncWebServer.h> // https://github.com/me-no-dev/ESPAsyncWebServer
#include <TimeLib.h> // http://www.pjrc.com/teensy/td_libs_Time.html
#include <UdpTime.h>
#include "eeMem.h"
#include <JsonParse.h> // https://github.com/CuriousTech/ESP8266-HVAC/tree/master/Libraries/JsonParse
#ifdef OTA_ENABLE
#include <FS.h>
#include <ArduinoOTA.h>
#include "ut181if.h"
#endif
#ifdef USE_SPIFFS
#include <FS.h>
#include <SPIFFSEditor.h>
#else
#include "pages.h"
#endif

int serverPort = 80;

#define BUTTON     0 // the prog button
#define ESP_LED    2 // low turns on ESP blue LED
#define SCL        4
#define SDA        5
#define SER2_RX   13  //
#define SER2_TX   15

SSD1306 display(0x3c, 5, 4); // Initialize the oled display for address 0x3c, sda=5, sdc=4

WiFiManager wifi;  // AP page:  192.168.4.1
AsyncWebServer server( serverPort );
AsyncWebSocket ws("/ws"); // access at ws://[esp ip]/ws
AsyncWebSocket wsb("/bin"); // access at ws://[esp ip]/bin

uint32_t binClientID; // connected binary client

void jsonCallback(int16_t iEvent, uint16_t iName, int iValue, char *psValue);
JsonParse jsonParse(jsonCallback);
UdpTime utime;
eeMem eemem;

UT181Interface ut;

uint16_t displayTimer;
uint8_t oldSW, oldSel;

String dataJson()
{
  int nMin, nMax, nMod;
  ut.RangeMod(nMin, nMax, nMod);

  String s = "state;{";
  s += "\"t\":";    s += now() - ( (ee.tz + utime.getDST() ) * 3600);
  s += ",\"v\":\""; s += ut.ValueText(0);
  s += "\",\"u\":\""; s += ut.UnitText();
  s += "\",\"v1\":\""; s += ut.ValueText(1);
  s += "\",\"v2\":\""; s += ut.ValueText(2);
  s += "\",\"v3\":\""; s += ut.ValueText(3);
  s += "\",\"cnt\":"; s += ut.DisplayCnt();
  s += ",\"mn\":"; s += nMin;
  s += ",\"mx\":"; s += nMax;
  s += ",\"md\":"; s += nMod;
  s += ",\"st\":"; s += ut.StatusBits();
  s += "}\n";
  return s;
}

String rangesJson()
{
  static char *rangeList[10][3][10] = {
    {{"Auto", "6", "60", "600", "1000", NULL}, {"Auto", NULL}, {"Auto", NULL}},
    {{"Auto", "600", "60", NULL}, {"Auto", NULL}, {"Auto",NULL}},
    {{"Auto", "6", "60", "600", "1000", NULL}, {"Auto",NULL}, {"Auto",NULL}},
    {{"Auto", "60", "600", NULL}, {"Auto",NULL}, {"Auto",NULL}},
    {{"Auto", "600", "6k", "60k", "600k", "6M", "60M", NULL}, {"Auto",NULL}, {"Auto",NULL} },
    {{"Auto",NULL}, {"Auto", "6n", "60n", "600n", "6µ", "60µ", "600µ", "6m", "60m", NULL},{"Auto",NULL}},
    {{"Auto", "60", "600", "6k", "60k", "600k", "6M", "60M", NULL},
      {"Auto", "60", "600", "6k", "60k", NULL}, {"Auto",NULL} },
    {{"Auto", "600", "6000", NULL}, {"Auto", "600", "6000", NULL}, {"Auto",NULL} },
    {{"Auto", "60", "600", NULL}, {"Auto", "60", "600", NULL}, {"Auto",NULL} },
    {{"Auto",NULL}, {"Auto",NULL}, {"Auto",NULL} },
  };

  static char *optList[10][3][10] = {
    {{"VAC", "VAC,Hz", "Peak", "LowPass", "dBV", "dBM", NULL}, {NULL}, {NULL}},
    {{"mVAC", "mVAC,Hz", "Peak", "AC+DC", NULL}, {NULL}, {NULL}},
    {{"VDC", "AC+DC", "Peak", NULL}, {NULL}, {NULL} },
    {{"mVDC", "Peak", NULL}, {"T1,T2", "T2,T1", "T1-T2", "T2-T1", NULL}, {"T1,T2", "T2,T1", "T1-T2", "T2-T1", NULL}},
    {{"Ohms",NULL}, {"Short", "Open", NULL}, {"nS", NULL}},
    {{"Normal", "Alarm", NULL}, {"Cap", NULL}, {NULL}},
    {{"Hz", NULL}, {"%", NULL}, {"Pulse", NULL}},
    {{"µADC", "AC+DC", "Peak", NULL}, {"µAAC", "µAAC,Hz", "Peak", NULL}, {NULL}},
    {{"mADC", "AC+DC", "Peak", NULL}, {"mAAC", "mAAC,Hz", "Peak", NULL}, {NULL}},
    {{"ADC", "AC+DC", "Peak", NULL}, {"AAC", "AAC,Hz", "Peak", NULL}, {NULL}},
  };

  uint8_t sw = ut.m_MData.Switch - 1;
  if(sw >= 10) sw = 0;
  uint8_t sel = ut.m_MData.Select - 1;
  if(sel >= 3) sel = 0;

  int i;
  String s = "range;{\"rs\":";
  s += ut.m_MData.Range;
  s += ",\"os\":";
  s += ut.m_MData.Select;

  s += ",\"r\":[";
  for(i = 0; rangeList[sw][sel][i]; i++)
  {
    if(i) s += ",";
    s += "[\"";
    s += rangeList[sw][sel][i];
    s += "\"]";
  }
  s += "],\"o\":[";

  for(i = 0; optList[sw][sel][i]; i++)
  {
    if(i) s += ",";
    s += "[\"";
    s += optList[sw][sel][i];
    s += "\"]";
  }

  s += "]}";
  return s;
}

String settingsJson()
{
  String s = "settings;{";
  s += "\"tz\":";    s += ee.tz;
  s += ",\"o\":";  s += ee.bEnableOLED;
  s += "}\n";
  return s;
}

void parseParams(AsyncWebServerRequest *request)
{
  char sztemp[100];
  char password[64];
 
  if(request->params() == 0)
    return;

//  Serial.println("parseParams");

  for ( uint8_t i = 0; i < request->params(); i++ ) {
    AsyncWebParameter* p = request->getParam(i);
    p->value().toCharArray(sztemp, 100);
    String s = wifi.urldecode(sztemp);
    bool which = (tolower(p->name().charAt(1) ) == 'd') ? 1:0;
    int val = s.toInt();
 
    switch( p->name().charAt(0)  )
    {
      case 'O': // OLED
          ee.bEnableOLED = (s == "true") ? true:false;
          display.clear();
          display.display();
          break;
      case 'r': // WS update rate
          ee.rate = val;
          break;
      case 's': // ssid
          s.toCharArray(ee.szSSID, sizeof(ee.szSSID));
          break;
      case 'p': // pass
          wifi.setPass(s.c_str());
          break;
    }
  }
}

const char *jsonList1[] = { "cmd",
  "oled",
  "TZ",
  "hold",
  "mm", // minmax
  "range",
  "sel",
  "rel",
  NULL
};

void jsonCallback(int16_t iEvent, uint16_t iName, int iValue, char *psValue)
{
  uint8_t sel;

  switch(iEvent)
  {
    case 0: // cmd
      switch(iName)
      {
        case 0: // OLED
          ee.bEnableOLED = iValue ? true:false;
          break;
        case 1: // TZ
          ee.tz = iValue;
          break;
        case 2: // hold
          ut.Hold();
          break;
        case 3: // mm
          ut.MinMax();
          break;
        case 4: // range
          ut.SetRange(iValue);
          break;
        case 5: // select
          ut.SetSelect(iValue, false, 0);
          break;
        case 6: // rel
          sel = ut.m_MData.Select;
          if(ut.RelState())
            ut.SetSelect(sel, false, 0);
          else
            ut.SetSelect(sel, true, ut.GetfValue());
          break;
      }
      break;
  }
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len)
{  //Handle WebSocket event
  static bool bRestarted = true;
  String s;

  switch(type)
  {
    case WS_EVT_CONNECT:      //client connected
      if(bRestarted)
      {
        bRestarted = false;
        client->text("alert;Restarted");
      }
      client->keepAlivePeriod(50);
      client->text(dataJson());
      client->text(settingsJson());
      client->ping();
      oldSW = 20;
      break;
    case WS_EVT_DISCONNECT:    //client disconnected
      break;
    case WS_EVT_ERROR:    //error was received from the other end
      break;
    case WS_EVT_PONG:    //pong message was received (in response to a ping request maybe)
      break;
    case WS_EVT_DATA:  //data packet
      AwsFrameInfo * info = (AwsFrameInfo*)arg;
      if(info->final && info->index == 0 && info->len == len){
        //the whole message is in a single frame and we got all of it's data
        if(info->opcode == WS_TEXT){
          data[len] = 0;

          char *pCmd = strtok((char *)data, ";"); // assume format is "name;{json:x}"
          char *pData = strtok(NULL, "");

          if(pCmd == NULL || pData == NULL) break;

          jsonParse.process(pCmd, pData);
        }
      }
      break;
  }
}

void onBinEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len)
{  //Handle WebSocket event
  String s;
  uint8_t buf[] = {1,8,1,0xA};

  switch(type)
  {
    case WS_EVT_CONNECT:      //client connected
      client->keepAlivePeriod(50);
      binClientID = client->id();
      client->binary(buf, sizeof(buf));
      client->ping();
      break;
    case WS_EVT_DISCONNECT:    //client disconnected
      binClientID = 0;
      break;
    case WS_EVT_ERROR:    //error was received from the other end
      break;
    case WS_EVT_PONG:    //pong message was received (in response to a ping request maybe)
      break;
    case WS_EVT_DATA:  //data packet
      AwsFrameInfo * info = (AwsFrameInfo*)arg;
      if(info->final && info->index == 0 && info->len == len){
        //the whole message is in a single frame and we got all of it's data
          ut.WriteData(data, len); // Raw command. No ABCD, length, or checksum
      }
      break;
  }
}

volatile bool bButtonPressed;

void btnISR() // Prog button on board
{
  bButtonPressed = true;
}

void setup()
{
  const char hostName[] ="UT181A";

  pinMode(ESP_LED, OUTPUT);
  digitalWrite(ESP_LED, LOW);

  // initialize dispaly
  display.init();
//  display.flipScreenVertically();
  display.clear();
  display.display();

  Serial.begin(9600);

  WiFi.hostname(hostName);
  wifi.autoConnect(hostName, "password");

  MDNS.begin ( hostName, WiFi.localIP() );

#ifdef USE_SPIFFS
  SPIFFS.begin();
  server.addHandler(new SPIFFSEditor("admin", controlPassword));
#endif

  // attach AsyncWebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  wsb.onEvent(onBinEvent);
  server.addHandler(&wsb);

  server.on( "/", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request){
      parseParams(request);
#ifdef USE_SPIFFS
      request->send(SPIFFS, "/index.html");
#else
      request->send_P(200, "text/html", page1);
#endif
  });
  server.on( "/s", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request){
    parseParams(request);
    request->send( 200, "text/html", wifi.page() );
  });
  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncWebServerResponse *response = request->beginResponse_P(200, "image/x-icon", favicon, sizeof(favicon));
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404);
  });

  server.onFileUpload([](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
  });
  server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
  });

  server.begin();

  MDNS.addService("http", "tcp", serverPort);
  NBNS.begin(hostName);

#ifdef OTA_ENABLE
  ArduinoOTA.begin();
#endif

  jsonParse.addList(jsonList1);

  utime.start();
  if(ee.rate == 0)
    ee.rate = 60;

  attachInterrupt(BUTTON, btnISR, FALLING);
  digitalWrite(ESP_LED, HIGH); // blue LED off
}

void sendBinData(uint8_t *p, int len)
{
  wsb.binary(binClientID, p, len);
}

void loop()
{
  static uint8_t hour_save, sec_save;
  static uint8_t cnt = 0;

  MDNS.update();
#ifdef OTA_ENABLE
  ArduinoOTA.handle();
#endif
  utime.check(ee.tz);

  ut.service(); // read serial data

  if(ut.Updated())
    ws.textAll(dataJson());

  if(oldSW != ut.m_MData.Switch || oldSel != ut.m_MData.Select)
  {
    oldSW = ut.m_MData.Switch;
    oldSel = ut.m_MData.Select;
    ws.textAll(rangesJson());
  }

  if(sec_save != second()) // only do stuff once per second (loop is maybe 20-30 Hz)
  {
    sec_save = second();
    if (hour_save != hour())
    {
      hour_save = hour();
      if(hour_save == 2)
      {
        utime.start(); // update time daily at DST change
      }
      eemem.update(); // update EEPROM if needed while we're at it (give user time to make many adjustments)
    }

    static uint8_t connTime = 1;
    if(--connTime == 0)
    {
      connTime = 5;
      if(ut.m_bConnected == false)
        ut.Connect(true);
    }

    if(displayTimer) // temp display on thing
      displayTimer--;

    static uint8_t pulseTime = 1;
    if(--pulseTime == 0)
    {
      pulseTime = 5;
//      digitalWrite(ESP_LED, LOW);
  //    delay(10);
    //  digitalWrite(ESP_LED, HIGH);
    }
  }

  if(wifi.isCfg()) // WiFi cfg will draw it
    return;

  // draw the screen here
  if(ee.bEnableOLED)
  {
    display.clear();
    display.drawPropString( 2, 23, ut.ValueText(0) );
    display.drawPropString(80, 47, ut.UnitText() );
    display.display();
  }
}
