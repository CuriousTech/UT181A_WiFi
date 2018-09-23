#include "ut181if.h"
#include "eeMem.h"
#include <TimeLib.h>

extern void sendBinData(uint8_t *p, int len);
extern void sendSaveEntry(SaveRec *pRec);
extern void sendRecordEntry(Record *pRecord);
extern void WsSend(String s);

void UT181Interface::service(time_t nw)
{
  while(Serial.available())
  {
    uint8_t c = Serial.read();
    m_keepAlive = nw;
    switch(m_state)
    {
      case 0:     // data packet: AB CD len len ............. chk chk
        if(c == 0xAB)
          m_state = 1;
        break;
      case 1:
        if(c == 0xCD)
          m_state = 2;
        break;
      case 2:
        m_len = (uint16_t)c;
        m_state = 3;
        break;
      case 3:
        m_len |= (uint16_t)(c << 8);
        m_state = 4;
        m_idx = 0;
        break;
      case 4:
        m_buffer[m_idx++] = c;
        if(m_idx == m_len || m_idx >= sizeof(m_buffer) )
        {
          if( sum(m_buffer, m_len-2) == (m_buffer[m_len-2] | (m_buffer[m_len-1]<<8)) )
            process_sentence(m_len-2);
          else
          {
            String s;

            s += "print;chk error ";
            s += String(sum(m_buffer, m_len-2), HEX);
            s += " ";
            s += String(m_buffer[m_len-2], HEX);
            s += String(m_buffer[m_len-1], HEX);

            WsSend(s);
            digitalWrite(2, LOW); // strobe LED
            delay(5);
            digitalWrite(2, HIGH);
          }
          m_state = 0;
          m_idx = 0;
          m_len = 0;
        }
        break;
    }
  }
  if((nw - m_keepAlive) > 3) // timeout should cause a restart if stopped
  {
    m_keepAlive = nw;
    m_bConnected = false;
    String s = "print; timeout ";
    s += m_idx;
    s += " ";
    s += m_len;
    WsSend(s);
    m_state = 0;
    m_idx = 0;
    m_len = 0;
    m_nRecReq = 0;
    m_nRecordItem = 0;
  }
}

bool UT181Interface::Updated() // check for new primary data packet update
{
  bool u = false;
  static int n;

  if(n != m_Updated) u = true;
  n = m_Updated;
  return u;
}

void UT181Interface::process_sentence(uint16_t len)
{
  sendBinData(m_buffer, len);
  switch(m_buffer[0])
  {
    case 0x01:  // 1 = command respsonses
          // 01 4F 4B A0 = 'OK '
          // 01 45 52 9D = 'ER'
          // 01 49 4E 56 = 'INV'
      if(m_nRecReq) // serialized start of file list retrieval
      {
        GetRecordSeq();
      }
      if(m_bGetRecStart)  // serialized start of large data retrieval
      {
        m_bGetRecStart = false;
        getRecordData();
      }
      break;
    case 0x02: // main meter data
      m_bConnected = true;
      memcpy(&m_MData, m_buffer + 1, sizeof(MData));
      m_Updated++;
      break;
    case 0x03: // save entry
      if(m_nRecIdx >= m_nSaves)
      {
        m_nRecReq = 0; // too many
        break;
      }
      sendSaveEntry( (SaveRec*)(m_buffer + 1));

      if(++m_nRecIdx < m_nSaves)
        GetRecordSeq(); // get more
      else // finished
      {
        m_nRecReq = 0;
        Connect(true);
      }
      break;

    case 0x04: // record entry
      if(m_nRecIdx >= m_nRecords)
      {
        m_nRecReq = 0;
        break;
      }

      sendRecordEntry( (Record*)(m_buffer + 1));

      if(++m_nRecIdx < m_nRecords)
      {
        GetRecordSeq();
      }
      else // end, reconnect
      {
        m_nRecReq = 0;
        Connect(true);
      }
      break;
    case 0x05: // record data
      decodeSamples(m_buffer + 2, m_buffer[1]); // first byte is count
      m_nRecDataIndex += m_buffer[1];
      if(m_nRecDataIndex < m_nRecDataIndexEnd)
      {
        getRecordData();
      }
      else // end of record, restart
      {
        m_nRecordItem = 0;
        Connect(true);
        WsSend("finish;0");
      }
      break;
    case 0x72:  // number of saves/records
      switch(m_buffer[1])
      {
        case 0x08: // saves
          m_nSaves = getWord(m_buffer + 2); // 72 08 03 00 83
          Connect(false); // disable to read
          m_nRecReq = 1; // setup to start on next ack
          m_nRecIdx = 0;
          break;
        case 0x0E: // records
          m_nRecords = getWord(m_buffer + 2); // 72 0E 02 00 88
          Connect(false); // disable main data to read
          m_nRecReq = 2;
          m_nRecIdx = 0;
          break;
      }
      break;
  }
}

uint16_t UT181Interface::getWord(uint8_t *p)
{
  uint16_t *wP = (uint16_t*)p;
  return *wP;
}

void UT181Interface::startRecordRetreval(int nItem, char *pszUnit, uint32_t dwSamples)
{
  if(nItem >= m_nRecords)
    return;

  strcpy(m_szRecUnit, pszUnit);
  m_nRecDataIndex = 0;
  m_nRecDataIndexEnd = dwSamples;
  m_bGetRecStart = true;
  Connect(false);
  m_nRecordItem = nItem + 1;
}

void UT181Interface::Connect(bool bCon) // start/stop meter data transmit
{
  static uint8_t cmd[]= {CMD_CONNECT,0x00};

  if( m_nRecReq || m_nRecordItem) // in download mode
    return;
  if(!bCon)  m_bConnected = false;

  cmd[1] = bCon ? 1 : 0;
  Write(cmd, sizeof(cmd) );
}

uint16_t UT181Interface::sum(uint8_t *p, uint16_t len)
{
  uint16_t ch = 2;

  for(uint16_t i = 0; i < len; i++)
    ch += p[i];
  ch += (len >> 8);
  return ch + (len & 0xFF);
}

 // Write data from binary socket
 bool UT181Interface::WriteData(uint8_t *pData, int len)
{
  Write(pData, len);
}

// Write data to serial IR LED
bool UT181Interface::Write(uint8_t *p, uint8_t len)
{
  uint8_t buf[64] = {0};

  buf[0] = 0xAB;
  buf[1] = 0xCD;
  buf[2] = len + 2;
  buf[3] = 0;

  uint16_t ck = buf[2];
  int i;
  for(i = 0; i < len; i++)
  {
    buf[i+4] = p[i];
    ck += p[i];
  }
  buf[i+4] = ck & 0xFF;
  buf[i+5] = ck >> 8;

  return Serial.write(buf, len+6);
}

// Select and REL + rel value
void UT181Interface::SetSelect(uint8_t nSel, uint8_t nOpt, bool bRel, float fValue)
{
  uint8_t cmd[]= {CMD_SET_OPTION,0, (m_MData.Switch << 4) | nSel};

  cmd[1] = ((nOpt + 1) << 4) | (bRel ? 2:1);

  if(cmd[2] == 0x52 || cmd[2] == 0x61) // sub mode
  {
    cmd[1] = 0x10 | (nOpt + 1);
  }

  Write(cmd, sizeof(cmd) );
  if(bRel)
  {
      #pragma pack(push, 1)
      struct relCmd
      {
        uint8_t Cmd;
        float fVal;
      };
      #pragma pack(pop)

      relCmd rc = {CMD_SET_REL, fValue};

      Write((uint8_t*)&rc, sizeof(rc) );
  }
}

float UT181Interface::GetfValue()
{
  return m_MData.Value.fValue;
}

void UT181Interface::SetRange(uint8_t n)  // 0 = auto, 1 = range 1...8
{
  static uint8_t cmd[]= {CMD_SET_RANGE,0x00};

  cmd[1] = n;
  Write(cmd, sizeof(cmd) );
}

void UT181Interface::Hold()
{
  static uint8_t cmd[]= {CMD_HOLD,0x5A}; // Z
  Write(cmd, sizeof(cmd) );
}

void UT181Interface::MinMax() // toggle
{
  static uint8_t cmd[]= {CMD_SET_MINMAX,0x00};
  cmd[1] = m_MData.MinMax ? 0:1;
  Write(cmd, sizeof(cmd) );
}

void UT181Interface::GetRecordSeq() // get next record or save (redundant?)
{
  switch(m_nRecReq)
  {
    case 1:
      if(m_nRecIdx > m_nSaves)
      {
        m_nRecReq = 0;
        Connect(true);
      }
      else
        getSave(m_nRecIdx+1);
      break;
    case 2:
      if(m_nRecIdx > m_nRecords)
      {
        m_nRecReq = 0;
        Connect(true);
      }
      else
        getRecord(m_nRecIdx+1);
      break;
  }
}

void UT181Interface::getRecord(uint16_t nItem) // get a record entry
{
  uint8_t cmd[]= {CMD_GET_RECORD, 0, 0};

  cmd[1] = nItem & 0xFF;
  cmd[2] = nItem >> 8;
  Write(cmd, sizeof(cmd) );
}

void UT181Interface::getRecordCount()
{
  static uint8_t cmd[]= {CMD_GET_RECORD_COUNT};

  Write(cmd, sizeof(cmd) );
}

void UT181Interface::decodeSamples(uint8_t *p, uint8_t count) // decode record file samples
{
  String s;
  s.reserve(4800);

  s = "chunk;";
  for(int i = 0; i < count; i++)
  {
    RecItem item;
    memcpy(&item, p, sizeof(RecItem));
    s += ValueText(item.Value);
    s += ",";
    s += uniDateToGTC(item.t);
    s += "\r\n";
    p += sizeof(RecItem);
  }
  WsSend(s);
}

void UT181Interface::getRecordData()
{
  recReq rr;

  rr.Cmd = CMD_GET_RECORD_DATA;
  rr.wItem = m_nRecordItem;
  rr.dwOffset = m_nRecDataIndex + 1;
  Write((uint8_t*)&rr, sizeof(rr) );
}

void UT181Interface::Save()
{
  static uint8_t cmd[]= {CMD_SAVE};

  Write(cmd, sizeof(cmd) );
}

void UT181Interface::StartRecord(char *pName, uint16_t wInterval, uint32_t dwDuration)
{
  recCmd rc;

  rc.Cmd = CMD_RECORD_START;
  strcpy(rc.szName, pName);
  rc.wInterval = wInterval;
  rc.dwDuration = dwDuration;
  Write((uint8_t*)&rc, sizeof(rc) );
}

void UT181Interface::StopRecord()
{
  uint8_t cmd[]= {CMD_STOP_RECORD};

  Write(cmd, sizeof(cmd) );
}

void UT181Interface::getSave(uint16_t nItem)
{
  uint8_t cmd[]= {CMD_GET_SAVE, 0, 0};

  cmd[1] = nItem & 0xFF;
  cmd[2] = nItem >> 8;
  Write(cmd, sizeof(cmd) );
}

void UT181Interface::getSaveCount()
{
  static uint8_t cmd[]= {CMD_GET_SAVE_COUNT};

  Write(cmd, sizeof(cmd) );
}

void UT181Interface::DeleteAllSave() // Saves
{
  static uint8_t cmd[]= {CMD_DELETE_SAVE_ITEM, 0xFF, 0xFF};

  Write(cmd, sizeof(cmd) );
}

void UT181Interface::deleteSave(int nItem)
{
  static uint8_t cmd[]= {CMD_DELETE_SAVE_ITEM, 0, 0};

  cmd[1] = nItem & 0xFF;
  cmd[2] = nItem >> 8;

  Write(cmd, sizeof(cmd) );
}

void UT181Interface::deleteRecord(int nItem)
{
  static uint8_t cmd[]= {CMD_DELETE_RECORD_ITEM, 0, 0};

  cmd[1] = nItem & 0xFF;
  cmd[2] = nItem >> 8;

  Write(cmd, sizeof(cmd) );
}

String UT181Interface::convertDate(uniDate &dt)
{
  static String s;
  s = String(dt.year + 2000) + "/";
  if(dt.month<10) s += "0";
  s += dt.month;
  s += "/";
  if(dt.day<10) s += "0";
  s += dt.day;
  s += " ";
  if(dt.hours<10) s += "  ";
  s += dt.hours;
  s += ":";
  if(dt.minutes<10) s += "0";
  s += dt.minutes;
  s += ":";
  if(dt.seconds<10) s += "0";
  s += dt.seconds;
  return s;
}

time_t UT181Interface::uniDateToGTC(uniDate &dt)
{
  tmElements_t t;

  t.Year = dt.year;
  t.Month = dt.month;
  t.Day = dt.day;
  t.Hour = dt.hours;
  t.Minute = dt.minutes;
  t.Second = dt.seconds;

  return makeTime(t);
}

void UT181Interface::setClock()
{
  #pragma pack(push, 1)
  struct setCmd
  {
    uint8_t Cmd;
    uddw ud;
  };
  #pragma pack(pop)

  setCmd cmd;
  cmd.Cmd = CMD_SET_CLOCK;

  cmd.ud.ud.year = year() - 2000;
  cmd.ud.ud.month = month();
  cmd.ud.ud.day = day();
  cmd.ud.ud.hours = hour();
  cmd.ud.ud.minutes = minute();
  cmd.ud.ud.seconds = second();

  uddw cv;
  cv.dw = ee.updateTime;
  if(cv.ud.day != day()){ // set time daily
    ee.updateTime = cmd.ud.dw;
    Write((uint8_t*)&cmd, sizeof(cmd) );
  }
}

int UT181Interface::readPercent()
{
  return  m_nRecDataIndex * 100 / m_nRecDataIndexEnd;
}

int UT181Interface::DisplayCnt()
{
  if(m_MData.MinMax)  return 4;
  if(m_MData.Rel)     return 3;
  if(m_MData.Peak)    return 2;

  switch(m_MData.dataType)
  {
      case 0x5:// dbV
      case 0x1: // T1
        return 2;
      case 0x3: // T1-T2 // Todo: fix data for mVac+dc
      case 0x7: // Hz Ms
        return 3;
      default: // 0x4
        return 1;
    }
}

char *UT181Interface::UnitText()
{
  char *p = (m_MData.MinMax) ? m_MData.u.MM.szUnit : m_MData.u.Std.szUnit;
  if(*p == 0xB0) *p = '@'; // convert degree to ampersand
  if(p[1] == 0xB0) p[1] = '@'; // convert degree to ampersand
}

char *UT181Interface::ValueText(int which)
{
  MValue Value;

  if(DisplayCnt()-1 < which)
    return "";

  if(m_MData.MinMax)
    switch(which)
    {
      default:  Value = m_MData.Value; break;
      case 1:   Value = m_MData.u.MM.Value1; break;
      case 2:   Value = m_MData.u.MM.Value2; break;
      case 3:   Value = m_MData.u.MM.Value3;
        break;
    }
  else
    switch(which)
    {
      default: Value = m_MData.Value; break;
      case 1:  Value = m_MData.u.Ext.Value1; break;
      case 2:  Value = m_MData.u.Ext.Value2; break;
      case 3:  Value = m_MData.u.Ext.Value3; break;
    }

  return ValueText(Value);
}

char *UT181Interface::ValueText(MValue &Value)
{
  static char szVal[16];

  if(Value.Blank)
  {
    switch(Value.Precision)
    {
      case 0: return "-----"; break;
      case 1: return "----.-"; break;
      case 2: return "---.--"; break;
      case 3: return "--.---"; break;
      case 4: return "-.----"; break;
      default: return ".-----"; break;
    }
  }
  else if(Value.L) // bit 0 invalid
  {
    return "0L";
  }

  char szFmt[8] = "%.4f";

  szFmt[2] = Value.Precision + '0';
  sprintf(szVal, szFmt, Value.fValue);
  return szVal;
}

void UT181Interface::RangeMod(int &nMin, int &nMax, int &nMod)
{
  getSign();

  nMin = 0;
  nMax = 60;
  nMod = 5;

  static short rangeListMax[10][3][8] =
  { // 1, 2, 3...
    {{6,60,600,1000,60},{60},{60}},               // VAC
    {{60,600,60,60,60},{60},{60}},                // mVAC
    {{6,60,600,1000,60},{60},{60}},               // VDC
    {{60,600,60,60,60},{60,600,60,60,60},{60,600,60,60,60}},  // mVDC, C, F
    {{600,6,60,600,6,60},{600,600,600,600,600},{60,60,60,60,60}}, // Ohms,cont,nS
    {{30,30,30,30,30},{6,60,600,6,60,600,6,60},{60}}, // Diode, cap
    {{60,600,6,60,600,6,60},{60,600,6,60,600,6,60},{60,600,6,60,600,6,60}}, //Hz, %, pulse
    {{600,6000,60,60,60},{600,6000,60,60,60},{60}},       //uADC, uAAC
    {{60,600,60,60,60},{60,600,60,60,60},{60}},         //mADC, mAAC
    {{20},{20},{60}},                     //ADC, AAC
  };

  static bool rangeListMod[10][3][8] =
  { // 1, 2, 3...
    {{true ,false,true,true},{false},{false}},  // VAC
    {{false,true,false,false},{false},{false}}, // mVAC
    {{false,false,true,true},{false},{false}},  // VDC
    {{false,true,false},{false,false,false},{false,false,false}}, // mVDC, C, F
    {{true,true,false,true,true},{true,true,true},{false}}, // Ohms,cont,nS
    {{true,false,false},{false,false,true,true,false,true,true},{false}}, // Diode, cap
    {{false,true,true,false,true,true},{false,true,true,false},{false,true,true,false}},  //Hz, %, pulse
    {{true,true},{true,true},{false}},  //uADC, uAAC
    {{false},{false},{false}},  //mADC, mAAC
    {{false},{false},{false}},  //ADC, AAC
  };

  uint8_t sw = m_MData.Switch - 1;
  uint8_t sel = m_MData.Select - 1;
  uint8_t r = m_MData.Range - 1;

  if(sw >= 10) sw = 0;
  if(sel >= 3) sel = 0;
  if(r >= 8) r = 0;

  nMax = rangeListMax[sw][sel][r];

  if( m_bSign)
    nMin = -nMax;

  nMod = rangeListMod[sw][sel][r] ? 10:5;
  if(nMax == 1000)
    nMod = 12;

  if(m_MData.Peak || m_MData.MinMax) // no bar
    nMod = 0;

  switch(m_MData.dataType) // no bar modes
  {
    case 0x1: // T1
      nMod = 0;
    case 0x3: // T1-T2
      if(m_MData.Switch == eSwitch_mVDC)
        nMod = 0;
      break;
  }
}

void UT181Interface::getSign()
{
  if(m_MData.Value.fValue < 0)
  {
    m_bSign = true;
    return;
  }
  static bool rangeListSigned[10][3] =
  { // 1, 2, 3...
    {false,false,false},  // VAC
    {false,false,false},  // mVAC
    {true ,true ,true },  // VDC
    {true ,true ,true },  // mVDC, C, F
    {false,false,false},  // Ohms,cont,nS
    {false,false,false},  // Diode, cap
    {false,false,false},  //Hz, %, pulse
    {true ,false,false},  //uADC, uAAC
    {true ,false,false},  //mADC, mAAC
    {true ,false,false},  //ADC, AAC
  };

  uint8_t sw = m_MData.Switch - 1;
  uint8_t sel = m_MData.Select - 1;

  if(sw > 10) sw = 0;
  if(sel > 10) sel = 0;

  m_bSign = rangeListSigned[sw][sel];
}

uint16_t UT181Interface::StatusBits()
{
  uint16_t bits = 0;

    // Switch modes
  switch(m_MData.Switch)
  {
    case eSwitch_Ohms: // Ohms
      switch(m_MData.Select)
      {
        case 2: // cont check
          bits |= (1<<1);
          break;
      }
      break;
    case eSwitch_Diode:
      switch(m_MData.Select)
      {
        case 1: // Diode
          bits |= (1<<2);
          break;
      }
      break;
  }

  if(m_bSign)         bits |= 1;
  if(m_MData.Over)    bits |= (1<<3);
  if(m_MData.Auto)    bits |= (1<<4);
  if(m_MData.Hold)    bits |= (1<<5);
  if(m_MData.Comp)    bits |= (1<<6);
  if(m_MData.Peak)    bits |= (1<<7);
  if(m_MData.Recording) bits |= (1<<8);
  if(m_MData.LeadErr) bits |= (1<<9);
  if(m_MData.Rel)     bits |= (1<<10);
  if(m_MData.MinMax)  bits |= (1<<11);
  if(m_MData._T1)     bits |= (1<<12);

  return bits;
}
