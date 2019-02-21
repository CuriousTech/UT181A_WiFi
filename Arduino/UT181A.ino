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
#include <DNSServer.h>
#include <TimeLib.h> // http://www.pjrc.com/teensy/td_libs_Time.html
#include <UdpTime.h> // https://github.com/CuriousTech/
#include "eeMem.h"
#include <JsonParse.h> // https://github.com/CuriousTech/ESP8266-HVAC/tree/master/Libraries/JsonParse
#include "ut181if.h"
#include "jsonstring.h"
#ifdef OTA_ENABLE
#include <FS.h>
#include <ArduinoOTA.h>
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
#define V_PULSE   14
#define SER2_TX   15

SSD1306 display(0x3c, 5, 4); // Initialize the oled display for address 0x3c, sda=5, sdc=4

WiFiManager wifi;  // AP page:  192.168.4.1
DNSServer dnsServer;
AsyncWebServer server( serverPort );
AsyncWebSocket ws("/ws"); // access at ws://[esp ip]/ws
AsyncWebSocket wsb("/bin"); // access at ws://[esp ip]/bin

uint32_t binClientID; // connected binary client
void jsonCallback(int16_t iEvent, uint16_t iName, int iValue, char *psValue);
JsonParse jsonParse(jsonCallback);
UdpTime utime;
eeMem eemem;

UT181Interface ut;
uint32_t oldOpt;
uint16_t skipCnt;
uint8_t tick;
uint16_t volts;
uint32_t voltTm;

String timeFmt(uint32_t val) // convert seconds to hh:mm:ss
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

String dataJson() // main meter data 10Hz
{
  uint32_t nw = now();
  static uint16_t lastV;

  jsonString js("state");
  js.Var("t", nw - ( (ee.tz + utime.getDST() ) * 3600));
  js.Var("tk", tick);
  js.Var("v", ut.ValueText(0) ); // primary value
  if(ut.m_MData.ShowBar == 0 || ut.m_MData.MinMax) // invalid bar graph (use main value)
  {
    js.Var("bv", ut.ValueText(0));
  }
  else  // 10Hz bar value
  {
    js.Var("bv", (ut.m_MData.type == 6) ? ut.m_MData.u.Ext.Value3.fValue : ut.m_MData.u.Std.fBarValue);
  }

  js.Var("u", ut.UnitText() ); // unit of measurement
  if(ut.m_MData.Recording) // recording timer
  {
    js.Var("v1", timeFmt(ut.m_MData.u.RecTimer.dwElapsed) );
    js.Var("v2", timeFmt(ut.m_MData.u.RecTimer.dwRemain) );
    js.Var("v3", ut.m_MData.u.RecTimer.dwSamples );
  }
  else // normal and others
  {
    if(ut.m_MData.Comp) // comp mode
    {
      char *p = "PASS";
      if(ut.m_MData.Switch == 4 && ut.m_MData.Select >1) // temp
      {
        if(ut.m_MData.u.CompTemp.Fail) p = "FAIL";
      }
      else if (ut.m_MData.u.Std.Fail) p = "FAIL";
      js.Var("v1", p);
    }
    else
      if(ut.DisplayCnt()>1) js.Var("v1", ut.ValueText(1) );
    if(ut.DisplayCnt() > 2) js.Var("v2", ut.ValueText(2) );
    if(ut.DisplayCnt() > 3) js.Var("v3", ut.ValueText(3) );
  }

  int nMin, nMax, nMod;
  ut.RangeMod(nMin, nMax, nMod);  // min/max/modulo for bar display
  String s = String(nMin) + ",";
  s += nMax; s += ","; s += nMod; 
  js.Var("mm", s);

  bool bAdd = (ut.m_MData.Value.L || ut.m_MData.Value.Blank || ut.m_MData.LeadErr || ut.m_MData.Hold) ? false:true; // skip blanks
  if(ut.Connected() == false) bAdd = false;
  static uint16_t skipList[] = {0,1,4,9,19,49,99,299,599}; // chart logging frequency
  if(ee.rate)
  {
    if(skipCnt++ >= skipList[ee.rate])
      skipCnt = 0;
    else bAdd = false;
  }
  uint16_t st = ut.StatusBits();
  if(bAdd) st |= 1; // add to chart
  js.Var("st", String(st, HEX) ); // bit flags in HEX

  if(ut.m_MData.MinMax) // min max trigger times
  {
    js.Var("t1", ut.m_MData.u.MM.dwTime1);
    js.Var("t2", ut.m_MData.u.MM.dwTime2);
    js.Var("t3", ut.m_MData.u.MM.dwTime3);
  }
  else if(ut.m_MData.Recording)
  {
    js.Var("t1", ut.m_MData.u.RecTimer.dwElapsed);
    js.Var("t2", ut.m_MData.u.RecTimer.dwRemain);
    js.Var("t3", ut.m_MData.u.RecTimer.dwSamples);
  }

  if(nw - voltTm >= 10 || volts != lastV) // every 10 seconds
  {
    voltTm = nw;
    lastV = volts;
    js.Var("vlt", (float)volts / 1000);
  }

  return js.Close();
}

void fixDeg(char *p) // deg kills websocket (try \xB0)
{
  if(*p == 0xB0) *p = '@'; // convert degree to ampersand
  if(p[1] == 0xB0) p[1] = '@';
}

String rangesJson() // get range and select options for dropdowns
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

  static char *selList[10][4] = {
    {"VAC", NULL},
    {"mVAC", NULL},
    {"VDC", NULL },
    {"mVDC", "C", "F",NULL},
    {"Ohms", "Cont", "nS", NULL},
    {"Diode", "Cap", NULL},
    {"Hz", "%", "Pulse",NULL},
    {"µADC", "µAAC", NULL},
    {"mADC", "mAAC", NULL},
    {"ADC", "AAC", NULL},
  };

  uint8_t sw = ut.m_MData.Switch - 1;
  if(sw >= 10) sw = 0;
  uint8_t sel = ut.m_MData.Select - 1;
  if(sel >= 3) sel = 0;

  int i;

  jsonString js("range");

  js.Var("rs", ut.m_MData.Range);
  js.Var("os", ut.m_MData.Select);
  js.Array("s", selList[sw] );
  js.Array("r", rangeList[sw][sel] );
  js.Array("o", optList[sw][sel] );

  char *szUnits[4];
  szUnits[0] = NULL;
  szUnits[1] = NULL;
  szUnits[2] = NULL;
  szUnits[3] = NULL;
  if(ut.m_MData.MinMax) // extended units of measurement
  {
    fixDeg(ut.m_MData.u.MM.szUnit);
    szUnits[0] = ut.m_MData.u.MM.szUnit;
    szUnits[1] = ut.m_MData.u.MM.szUnit;
    szUnits[2] = ut.m_MData.u.MM.szUnit;
  }
  else if(ut.m_MData.Peak)
  {
    fixDeg(ut.m_MData.u.Ext.szUnit1);
    szUnits[0] = ut.m_MData.u.Ext.szUnit1;
  }
  else if(ut.m_MData.Switch==4 && ut.m_MData.Select > 1) // temp C or F
  {
    fixDeg(ut.m_MData.u.Std.szUnit);
    szUnits[0] = ut.m_MData.u.Std.szUnit;
    if(ut.m_MData.Mode >= 3)
     szUnits[1] = ut.m_MData.u.Std.szUnit;
  }
  else if(ut.m_MData.type == 3 || ut.m_MData.type == 6)
  {
    fixDeg(ut.m_MData.u.Ext.szUnit1);
    fixDeg(ut.m_MData.u.Ext.szUnit2);
    szUnits[0] = ut.m_MData.u.Ext.szUnit1;
    szUnits[1] = ut.m_MData.u.Ext.szUnit2;
  }
  else if(ut.m_MData.type == 7)
  {
    fixDeg(ut.m_MData.u.Ext.szUnit1);
    fixDeg(ut.m_MData.u.Ext.szUnit2);
    fixDeg(ut.m_MData.u.Ext.szUnit3);
    szUnits[0] = ut.m_MData.u.Ext.szUnit1;
    szUnits[1] = ut.m_MData.u.Ext.szUnit2;
    szUnits[2] = ut.m_MData.u.Ext.szUnit3;
  }

  js.Array("u", szUnits );

  String sLabels[5];

  if(ut.m_MData.MinMax) // labels
  {
      sLabels[0] = "MAX MIN";
      sLabels[1] = "Maximum";
      sLabels[2] = "Average";
      sLabels[3] = "Minimum";
  }
  else if(ut.m_MData.Recording)
  {
      sLabels[0] = "REC";
      sLabels[1] = "Elapsed Time:";
      sLabels[2] = "Remain Time:";
      sLabels[3] = "Samples:";
  }
  else if(ut.m_MData.Rel)
  {
      sLabels[0] = "REL"; 
      sLabels[1] = "Reference:";
      sLabels[2] = "Measurement:";
  }
  else if(ut.m_MData.Switch==4 && ut.m_MData.Select > 1) // Temp C or F
  {
      sLabels[0] = "";
      if(ut.m_MData.Mode >= 3) // subtractive
      {
        sLabels[1] = "T1";
        sLabels[2] = "T2";
      }
      else if(ut.m_MData.Mode == 2) // reverse
      {
        sLabels[1] = "T1";
      }
      else // normal
      {
        sLabels[1] = "T2";
      }
  }
  else if(ut.m_MData.Comp)
  {
      static char *compM[] = {"INNER", "OUTER", "< VALUE", "> VALUE"};
      sLabels[0] = "";

      if(ut.m_MData.Switch == 4 && ut.m_MData.Select >1) // temp
      {
        sLabels[1] = "MODE: "; sLabels[1] += compM[ut.m_MData.u.CompTemp.CompMode];
        sLabels[2] = "LOW:  "; sLabels[2] += ut.m_MData.u.CompTemp.fLow;
        sLabels[3] = "HIGH: "; sLabels[3] += ut.m_MData.u.CompTemp.fHigh;
      }
      else
      {
        sLabels[1] = "MODE: "; sLabels[1 ]+= compM[ut.m_MData.u.Std.CompMode];
        switch(ut.m_MData.u.Std.CompMode)
        {
          case 0:
          case 1:
            sLabels[2] = "LOW:  "; sLabels[2] += ut.m_MData.u.Std.fLow;
            sLabels[3] = "HIGH: "; sLabels[3] += ut.m_MData.u.Std.fHigh;
            break;
          case 2:
          case 3:
            sLabels[2] = "VALUE:  "; sLabels[2] += ut.m_MData.u.Std.fHigh;
            break;
        }
      }
  }

  js.Array("l", sLabels );

  return js.Close();
}

String settingsJson() // EEPROM settings
{
  jsonString js("settings");
  js.Var("tz", ee.tz);
  js.Var("o", ee.bEnableOLED);
  js.Var("f", ee.exportFormat);
  js.Var("r", ee.rate);
  return js.Close();
}

void parseParams(AsyncWebServerRequest *request) // parse URL params
{
  char sztemp[100];
  char password[64];
 
  if(request->params() == 0)
    return;

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
      case 's': // ssid
          s.toCharArray(ee.szSSID, sizeof(ee.szSSID));
          break;
      case 'p': // pass
          wifi.setPass(s.c_str());
          break;
    }
  }
}

const char *jsonList1[] = { "cmd", // WebSocket commands
  "oled", // 0
  "TZ",
  "hold",
  "mm", // minmax
  "range",
  "sel",
  "opt",
  "rel",
  "rec",
  "svs",
  "name", // 10
  "int",
  "sav",
  "stop",
  "file",
  "snap", //15
  "fmt",
  "delf",
  "dels",
  "rate",
  "rclk", // 20
  NULL
};

void jsonCallback(int16_t iEvent, uint16_t iName, int iValue, char *psValue) // handle WebSocket commands
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
          ut.SetSelect(iValue, 0, false, 0);
          break;
        case 6: // opt
          ut.SetSelect(ut.m_MData.Select, iValue, false, 0);
          break;
        case 7: // rel
          sel = ut.m_MData.Select;
          if(ut.m_MData.Rel)
            ut.SetSelect(sel, 0, false, 0);
          else
            ut.SetSelect(sel, 0, true, ut.GetfValue());
          break;
        case 8: // records
          ut.getRecordCount();
          break;
        case 9: // saves
          ut.getSaveCount();
          break;
        case 10: // name
          strncpy(szName, psValue, sizeof(szName)-1);
          break;
        case 11: // interval
          if(strlen(psValue)==0)
            iValue = 1;
          wInterval = iValue;
          break;
        case 12: //save (duration)
          if(iValue == 0)
            ut.Save();
          else
          {
            if(strlen(szName) == 0) strcpy(szName, "Record_01");
            if(wInterval == 0) wInterval = 1;
            ut.StartRecord(szName, wInterval, iValue);
            String s = "print;Start record ";
            s += szName;
            s += " ";
            s += wInterval;
            s += " ";
            s += iValue;
            WsSend(s);
          }
          break;
        case 13: // stop
          if(ut.m_MData.Recording)
            ut.StopRecord();
          break;
        case 14: // file
          ut.startRecordRetreval(iValue, szName, wInterval);
          break;
        case 15: // snap
//        ut.getSave(iValue, true);
          break;
        case 16: // fmt
          ee.exportFormat = iValue;
          break;
        case 17: // delf
          ut.deleteRecord(iValue + 1);
          break;
        case 18: //dels
          ut.deleteSave(iValue + 1);
          break;
        case 19: // rate
          ee.rate = iValue;
          skipCnt = 0;
          break;
        case 20: // sclk
          ut.setClock();
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

  if(pRec->type == 3) // MaxMin
  {
      fixDeg(pRec->u.b.szLabel);
      s += pRec->u.b.szLabel;
      s += "\"],[\""; s += ut.ValueText(pRec->u.b.Value21);
      s += "\"],[\""; s += ut.ValueText(pRec->u.b.Value22);
      s += "\"],[\""; s += ut.ValueText(pRec->u.b.Value23);
      s += "\"]]";
  }
  else if( pRec->type == 0) // normal
  {
      fixDeg(pRec->u.a.szLabel0);
      s += pRec->u.a.szLabel0;
      s += "\"]]";    
  }
  else if(pRec->type == 6) // 3 values
  {
      fixDeg(pRec->u.a.szLabel0);
      fixDeg(pRec->u.a.szLabel1);
      fixDeg(pRec->u.a.szLabel2);
      s += pRec->u.a.szLabel0;
      s += "\"],[\""; s += ut.ValueText(pRec->u.a.Value11);
      s += "\"],[\""; s += pRec->u.a.szLabel1;
      s += "\"],[\""; s += ut.ValueText(pRec->u.a.Value12);
      s += "\"],[\""; s += pRec->u.a.szLabel2;
      s += "\"]]";    
  }
  else if(pRec->type == 7) // 4 values
  {
      fixDeg(pRec->u.b.szLabel);
      s += pRec->u.b.szLabel;
      s += "\"],[\""; s += ut.ValueText(pRec->u.b.Value21);
      s += "\"],[\""; s += pRec->u.b.szLabel;
      s += "\"],[\""; s += ut.ValueText(pRec->u.b.Value22);
      s += "\"],[\""; s += pRec->u.b.szLabel;
      s += "\"],[\""; s += ut.ValueText(pRec->u.b.Value23);
      s += "\"],[\""; s += pRec->u.b.szLabel;
      s += "\"]]";    
  }
  else // 2 values
  {
      fixDeg(pRec->u.a.szLabel1);
      fixDeg(pRec->u.a.szLabel2);
      s += pRec->u.a.szLabel1;
      s += "\"],[\""; s += ut.ValueText(pRec->u.a.Value11);
      s += "\"],[\""; s += pRec->u.a.szLabel2;
      s += "\"]]";
  }

  s += ",\"b\":"; s+=pRec->type; s+="}";
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

void WsSend(String s) // send packat directly (preformatted)
{
  ws.textAll(s);  
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len)
{  //Handle WebSocket event

  switch(type)
  {
    case WS_EVT_CONNECT:      //client connected
      client->keepAlivePeriod(50);
      client->text(dataJson());
      client->text(settingsJson());
      client->ping();
      oldOpt = 0;
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
{  //Handle binary WebSocket event (/bin)
  String s;
  uint8_t buf[] = {1,8,1,0xA}; // identifier response

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

IPAddress apIP(192, 168, 4, 1);

void setup()
{
  const char hostName[] ="UT181A";

  pinMode(ESP_LED, OUTPUT);
  pinMode(V_PULSE, OUTPUT);
  digitalWrite(ESP_LED, LOW);

  // initialize dispaly
  display.init();
//  display.flipScreenVertically();
  display.clear();
  display.display();

  Serial.begin(9600);

  WiFi.hostname(hostName);
  wifi.autoConnect(hostName, "password");

#ifdef USE_SPIFFS
  SPIFFS.begin();
  server.addHandler(new SPIFFSEditor("admin", "admin"));
#endif

  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", apIP);
  
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
  server.serveStatic("/del-btn.png", SPIFFS, "/del-btn.png");
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
  server.on("/del-btn.png", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", delbtn_png, sizeof(delbtn_png));
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

//  MDNS.addService("http", "tcp", serverPort);
//  NBNS.begin(hostName);
  MDNS.begin ( hostName );
  MDNS.addService("http", "tcp", 80);

#ifdef OTA_ENABLE
  ArduinoOTA.begin();
#endif

  jsonParse.addList(jsonList1);

  utime.start();

  attachInterrupt(BUTTON, btnISR, FALLING);
  digitalWrite(ESP_LED, HIGH); // blue LED off

  display.clear();
  display.display();
}

void sendBinData(uint8_t *p, int len)
{
  wsb.binary(binClientID, p, len);
}

void loop()
{
  static uint8_t hour_save, sec_save, lastS;
  static uint8_t cnt;
  time_t nw;

//  MDNS.update();
  dnsServer.processNextRequest();
#ifdef OTA_ENABLE
  ArduinoOTA.handle();
#endif
  utime.check(ee.tz);

  nw = now();
  int s = second(nw);
  if(s !=lastS)
  {
    lastS = s;
    tick = 0;
  }
  ut.service(nw); // read serial data

  if(ut.Updated() || (tick==0 && !ut.Connected()) ) // new packet ready, or not receiving
  {
    ws.textAll(dataJson());
    tick++;
  }
  // only update the extras when settings change
  if(oldOpt != *(uint32_t*)(&ut.m_MData))
  {
    oldOpt = *(uint32_t*)(&ut.m_MData); // switch/sel is here as well
    ws.textAll(rangesJson());
  }

  if(sec_save != s) // only do stuff once per second
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
        ut.start(true);
    }
    digitalWrite(V_PULSE, LOW);
    uint16_t v = analogRead(0);
    volts = (v * 5.25);
    digitalWrite(V_PULSE, HIGH);
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
