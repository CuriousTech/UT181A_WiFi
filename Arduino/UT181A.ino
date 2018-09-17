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

String timeFmt(uint32_t val)
{
  int s = val % 60;
  int m = val / 60 % 60;
  static String st;

  st = String(val/3600);
  st += ":";
  if(m < 10) st += "0";
  st += m;
  st += ":";
  if(s < 10) st += "0";
  st += s;
  return st;
}

String dataJson()
{
  int nMin, nMax, nMod;
  ut.RangeMod(nMin, nMax, nMod);

  String s = "state;{";
  s += "\"t\":";    s += now() - ( (ee.tz + utime.getDST() ) * 3600);
  s += ",\"v\":\""; s += ut.ValueText(0);
  s += "\",\"u\":\""; s += ut.UnitText();
  s += "\",\"v1\":\"";
  if(ut.m_MData.Recording)
  {
    s += timeFmt(ut.m_MData.u.RecTimer.dwElapsed);
    s += "\",\"v2\":\""; s += timeFmt(ut.m_MData.u.RecTimer.dwRemain);
    s += "\",\"v3\":\""; s += ut.m_MData.u.RecTimer.dwSamples;
  }
  else
  {
    if(ut.m_MData.Comp)
      s += ut.m_MData.u.Comp.Fail ? "FAIL" : "PASS";
    else
      s += ut.ValueText(1);
    s += "\",\"v2\":\""; s += ut.ValueText(2);
    s += "\",\"v3\":\""; s += ut.ValueText(3);
  }
  s += "\",\"mn\":"; s += nMin;
  s += ",\"mx\":"; s += nMax;
  s += ",\"md\":"; s += nMod;
  s += ",\"st\":"; s += ut.StatusBits();

  if(ut.m_MData.MinMax)
  {
    s += ",\"t1\":"; s += ut.m_MData.u.MM.dwTime1;
    s += ",\"t2\":"; s += ut.m_MData.u.MM.dwTime2;
    s += ",\"t3\":"; s += ut.m_MData.u.MM.dwTime3;
  }
  else if(ut.m_MData.Recording)
  {
    s += ",\"t1\":"; s += ut.m_MData.u.RecTimer.dwElapsed;
    s += ",\"t2\":"; s += ut.m_MData.u.RecTimer.dwRemain;
    s += ",\"t3\":"; s += ut.m_MData.u.RecTimer.dwSamples;
  }
  s += "}\n";
  return s;
}

void fixDeg(char *p)
{
  if(*p == 0xB0) *p = '@'; // convert degree to ampersand
  if(p[1] == 0xB0) p[1] = '@';
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
  s += ']';

  if(ut.m_MData.MinMax)
  {
    fixDeg(ut.m_MData.u.MM.szUnit);    
    s += ",\"u1\":\""; s += ut.m_MData.u.MM.szUnit;
    s += "\",\"u2\":\""; s += ut.m_MData.u.MM.szUnit;
    s += "\",\"u3\":\""; s += ut.m_MData.u.MM.szUnit; s += "\"";
  }
  else if(ut.m_MData.Peak)
  {
    fixDeg(ut.m_MData.u.Ext.szUnit1);
    s += ",\"u1\":\""; s += ut.m_MData.u.Ext.szUnit1;
    s += "\",\"u2\":\"";
    s += "\",\"u3\":\"\"";
  }
  else if(ut.m_MData.Rel)
  {
    fixDeg(ut.m_MData.u.Ext.szUnit1);
    fixDeg(ut.m_MData.u.Ext.szUnit2);
    s += ",\"u1\":\""; s += ut.m_MData.u.Ext.szUnit1;
    s += "\",\"u2\":\""; s += ut.m_MData.u.Ext.szUnit2;
    s += "\",\"u3\":\"\"";
  }
  else if(ut.m_MData.Switch==4&&ut.m_MData.Select==2)
  {
    fixDeg(ut.m_MData.u.Std.szUnit);
    s += ",\"u1\":\""; s += ut.m_MData.u.Std.szUnit;
    s += "\",\"u2\":\"";
    if(ut.m_MData.tempReverse || ut.m_MData.tempSubReverse)
      s += ut.m_MData.u.Std.szUnit;
    s += "\",\"u3\":\"\"";
  }
  else if(ut.m_MData.dataType == 1)
  {
    fixDeg(ut.m_MData.u.Ext.szUnit1);
    fixDeg(ut.m_MData.u.Ext.szUnit2);
    fixDeg(ut.m_MData.u.Ext.szUnit3);
    s += ",\"u1\":\""; s += ut.m_MData.u.Ext.szUnit1;
    s += "\",\"u2\":\""; s += ut.m_MData.u.Ext.szUnit2;
    s += "\",\"u3\":\""; s += ut.m_MData.u.Ext.szUnit3; s += "\"";
  }
  else
  {
    s += ",\"u1\":\"\""; 
    s += ",\"u2\":\"\"";
    s += ",\"u3\":\"\"";
  }

  if(ut.m_MData.MinMax)
  {
      s += ",\"l0\":\"MAX MIN\"";
      s += ",\"l1\":\"Maximum\"";
      s += ",\"l2\":\"Average\"";
      s += ",\"l3\":\"Minimum\"";
  }
  else if(ut.m_MData.Recording)
  {
      s += ",\"l0\":\"REC\""; 
      s += ",\"l1\":\"Elapsed Time:\"";
      s += ",\"l2\":\"Remaining Time:\"";
      s += ",\"l3\":\"Samples:\"";
  }
  else if(ut.m_MData.Rel)
  {
      s += ",\"l0\":\"REL\""; 
      s += ",\"l1\":\"Reference:\"";
      s += ",\"l2\":\"Measurement:\"";
      s += ",\"l3\":\"\"";
  }
  else if(ut.m_MData.Switch==4 && ut.m_MData.Select==2) // Temp
  {
      s += ",\"l0\":\"\"";
      if(ut.m_MData.tempSubReverse)
      {
        s += ",\"l1\":\"T2\"";
        s += ",\"l2\":\"T1\"";
      }
      else if(ut.m_MData.tempReverse)
      {
        s += ",\"l1\":\"T1\"";
        s += ",\"l2\":\"\"";
      }
      else
      {
        s += ",\"l1\":\"T2\"";
        s += ",\"l2\":\"\"";
      }
      s += ",\"l3\":\"\"";
  }
  else if(ut.m_MData.Comp)
  {
      static char *compM[] = {"INNER", "OUTER", "< VALUE", "> VALUE"};
      s += ",\"l0\":\"\"";
      s += ",\"l1\":\"MODE: "; s += compM[ut.m_MData.u.Comp.CompMode]; s+= "\"";
      s += ",\"l2\":\"LOW:  "; s += ut.m_MData.u.Comp.fLow; s += "\"";
      s += ",\"l3\":\"HIGH: "; s += ut.m_MData.u.Comp.fHigh; s += "\"";
  }
  else // normal
  {
      s += ",\"l0\":\"\"";
      s += ",\"l1\":\"\"";
      s += ",\"l2\":\"\"";
      s += ",\"l3\":\"\"";
  }
  s += "}";
  return s;
}

String settingsJson()
{
  String s = "settings;{";
  s += "\"tz\":"; s += ee.tz;
  s += ",\"o\":"; s += ee.bEnableOLED;
  s += ",\"f\":"; s += ee.exportFormat;
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
  "oled", // 0
  "TZ",
  "hold",
  "mm", // minmax
  "range",
  "sel",
  "rel",
  "rec",
  "svs",
  "name", // 9
  "int",
  "sav",
  "stop",
  "file",
  "snap", //14
  "fmt",
  NULL
};

void jsonCallback(int16_t iEvent, uint16_t iName, int iValue, char *psValue)
{
  uint8_t sel;
  static char szName[16];
  static uint32_t wInterval;

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
          if(ut.m_MData.Rel)
            ut.SetSelect(sel, false, 0);
          else
            ut.SetSelect(sel, true, ut.GetfValue());
          break;
        case 7: // records
          ut.getRecordCount();
          break;
        case 8: // saves
          ut.getSaveCount();
          break;
        case 9: // name
          strncpy(szName, psValue, sizeof(szName)-1);
          break;
        case 10: // interval
          wInterval = iValue;
          break;
        case 11: //save (duration)
          if(iValue == 0)
            ut.Save();
          else
          {
            if(strlen(szName) == 0)
              strcpy(szName, "Record_01");
            if(wInterval == 0)
              break;
            ut.StartRecord(szName, wInterval, iValue);
          }
          break;
        case 12: // stop
          if(ut.m_MData.Recording)
            ut.StopRecord();
          break;
        case 13: // file
          ut.startRecordRetreval(iValue, szName, wInterval);
          break;
        case 14: // snap
//        ut.getSave(iValue, true);
          break;
        case 15: // fmt
          ee.exportFormat = iValue;
          break;
      }
      break;
  }
}

void sendSaveEntry(SaveRec *pRec)
{
  String s = "save;{\"a\":[[\"";

  s += ut.convertDate(pRec->Date);
  s += "\"],[\"";
  s += ut.ValueText(pRec->Value0);
  s += "\"],[\"";

  switch(pRec->switchSel)
  {
    case 0x08:
      fixDeg(pRec->u.a.szLabel0);
      s += pRec->u.a.szLabel0;
      s += "\"]]";
      break;
    case 0x02:
    case 0x0A:
    case 0x42:
      fixDeg(pRec->u.a.szLabel1);
      fixDeg(pRec->u.a.szLabel2);
      s += pRec->u.a.szLabel1;
      s += "\"],[\""; s += ut.ValueText(pRec->u.a.Value11);
      s += "\"],[\""; s += pRec->u.a.szLabel2;
      s += "\"]]";
      break;
    case 0x06:
    case 0x0E:
    case 0x1E:
      fixDeg(pRec->u.a.szLabel0);
      fixDeg(pRec->u.a.szLabel1);
      fixDeg(pRec->u.a.szLabel2);
      s += pRec->u.a.szLabel0;
      s += "\"],[\""; s += ut.ValueText(pRec->u.a.Value11);
      s += "\"],[\""; s += pRec->u.a.szLabel1;
      s += "\"],[\""; s += ut.ValueText(pRec->u.a.Value12);
      s += "\"],[\""; s += pRec->u.a.szLabel2;
      s += "\"]]";
      break;
    case 0x27:
    case 0x2F:
      fixDeg(pRec->u.b.szLabel);
      s += pRec->u.b.szLabel;
      s += "\"],[\""; s += ut.ValueText(pRec->u.b.Value21);
      s += "\"],[\""; s += ut.ValueText(pRec->u.b.Value22);
      s += "\"],[\""; s += ut.ValueText(pRec->u.b.Value23);
      s += "\"]]";
      break;
  }

  s += "}";
  ws.textAll(s);
}

void sendRecordEntry(Record *pRecord)
{
  String s = "record;{\"a\":[[\"";
  s += pRecord->szName;
  s += "\"],[\""; s += ut.convertDate(pRecord->Date);
  fixDeg(pRecord->szUnit);
  s += "\"],[\""; s += pRecord->szUnit;
  s += "\"],[\""; s += pRecord->dwSamples;
  s += "\"],[\""; s += pRecord->wInterval;
  s += "\"],[\""; s += pRecord->dwDuration;
  s += "\"],[\""; s += ut.ValueText(pRecord->mMin);
  s += "\"],[\""; s += ut.ValueText(pRecord->mAvg);
  s += "\"],[\""; s += ut.ValueText(pRecord->mMax);
  s += "\"]]}";
  ws.textAll(s);
}

void WsSend(String s)
{
  ws.textAll(s);  
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len)
{  //Handle WebSocket event
  static bool bRestarted = true;

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

void handleFileList(AsyncWebServerRequest *request)
{
  String s = "";
  request->send( 200, "text/html", s );
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
  server.addHandler(new SPIFFSEditor("admin", "admin"));
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
  server.on( "/files.html", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request){
#ifdef USE_SPIFFS
    request->send( SPIFFS, "/files.html" );
#else
    request->send_P( 200, "text/html", files_html );
#endif
  });
  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });

#ifdef USE_SPIFFS
  server.serveStatic("/beep.png", SPIFFS, "/beep.png");
  server.serveStatic("/shock.png", SPIFFS, "/shock.png");
  server.serveStatic("/diode.png", SPIFFS, "/diode.png");
  server.serveStatic("/favicon.ico", SPIFFS, "/favicon.ico");
#else
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncWebServerResponse *response = request->beginResponse_P(200, "image/x-icon", favicon, sizeof(favicon));
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });
  server.on("/beep.png", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", beep_png, sizeof(beep_png));
    request->send(response);
  });
  server.on("/shock.png", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", shock_png, sizeof(shock_png));
    request->send(response);
  });
  server.on("/diode.png", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", diode_png, sizeof(diode_png));
    request->send(response);
  });
#endif

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
  static uint8_t cnt;
  static uint32_t oldOpt;
  time_t nw;

  MDNS.update();
#ifdef OTA_ENABLE
  ArduinoOTA.handle();
#endif
  utime.check(ee.tz);

  nw = now();
  ut.service(nw); // read serial data

  if(ut.Updated())
    ws.textAll(dataJson());

  if(oldSW != ut.m_MData.Switch || oldSel != ut.m_MData.Select || oldOpt != *(uint32_t*)(&ut.m_MData))
  {
    oldOpt = *(uint32_t*)(&ut.m_MData);
    oldSW = ut.m_MData.Switch;
    oldSel = ut.m_MData.Select;
    ws.textAll(rangesJson());
  }

  int s = second(nw);
  if(sec_save != s) // only do stuff once per second (loop is maybe 20-30 Hz)
  {
    sec_save = s;
    if(s == 0 && hour_save != hour(nw))
    {
      hour_save = hour(nw);
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
  }

  // draw the screen here
  if(ee.bEnableOLED && wifi.isCfg() == false)
  {
    display.clear();
    display.drawPropString( 2, 23, ut.ValueText(0) );
    display.drawPropString(80, 47, ut.UnitText() );
    display.display();
  }
}
