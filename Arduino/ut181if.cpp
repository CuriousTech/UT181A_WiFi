#include "ut181if.h"
#include <TimeLib.h>

extern void sendBinData(uint8_t *p, int len);
extern void sendSaveEntry(SaveRec *pRec);
extern void sendRecordEntry(Record *pRecord);
extern void sendDbg(String s);

void UT181Interface::service()
{
  while(Serial.available())
  {
    uint8_t c = Serial.read();
    m_keepAlive = now();

    switch(m_state)
    {
      case 0:
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
            digitalWrite(2, LOW);
            delay(5);
            digitalWrite(2, HIGH);
          }
          m_state = 0;
        }
        break;
    }
  }
  if( (now() - m_keepAlive) > 4)
  {
    m_keepAlive = now();
    m_bConnected = false;
  }
}

bool UT181Interface::Updated()
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
      m_bConnected = true;
      if(m_nRecReq)
      {
        GetRecordSeq();
      }
      if(m_bGetRecStart)
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
        m_nRecReq = 0;
        break;
      }
      sendSaveEntry( (SaveRec*)(m_buffer + 1));

      if(++m_nRecIdx < m_nSaves)
        GetRecordSeq(); // get more
      else
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
        GetRecordSeq();
      else // end, reconnect
      {
        m_nRecReq = 0;
        Connect(true);
      }
      break;
    case 0x05: // record data
      decodeSamples(m_buffer + 2, m_buffer[1]);
      m_nRecDataIndex += m_buffer[1];
      if(m_nRecDataIndex < m_nRecDataIndexEnd)
        getRecordData();
      else // end of record, restart
        Connect(true);
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

void UT181Interface::startRecordRetreval(int nItem)
{
  if(nItem >= m_nRecords)
    return;

//  AllocChart(m_pRecords[nItem].dwSamples);
  m_nRecDataIndex = 0;
//  m_nRecDataIndexEnd = m_pRecords[nItem].dwSamples;
  m_nRecordItem = nItem + 1;
  m_bGetRecStart = true;
  m_nRecordSampleItem = nItem;
  Connect(false);
}

void UT181Interface::Connect(bool bCon)
{
  static uint8_t cmd[]= {0x05,0x00};

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
void UT181Interface::SetSelect(uint8_t nSel, bool bRel, float fValue)
{
  uint8_t cmd[]= {0x01,0, (m_MData.Switch << 4) | m_MData.Select};

  cmd[1] = ((nSel + 1) << 4) | (bRel ? 2:1);

  if(cmd[2] == 0x52 || cmd[2] == 0x61) // sub mode
  {
    cmd[1] = 0x10 | (nSel + 1);
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

      relCmd rc = {3, fValue};

      Write((uint8_t*)&rc, sizeof(rc) );
  }
}

float UT181Interface::GetfValue()
{
  return m_MData.Value.fValue;
}

void UT181Interface::SetRange(uint8_t n)  // 0 = auto, 1 = range 1...8
{
  static uint8_t cmd[]= {0x02,0x00};

  cmd[1] = n;
  Write(cmd, sizeof(cmd) );
}

void UT181Interface::Hold()
{
  static uint8_t cmd[]= {0x12,0x5A}; // Z
  Write(cmd, sizeof(cmd) );
}

void UT181Interface::MinMax() // toggle
{
  static uint8_t cmd[]= {0x04,0x00};
  bool bOn = false;
  switch(m_MData.dataType)
  {
    case 0x2F: // Max min
    case 0x27: // max min temp
      bOn = true;
      break;
  }
  cmd[1] = bOn ? 0:1;
  Write(cmd, sizeof(cmd) );
}

void UT181Interface::GetRecordSeq()
{
  switch(m_nRecReq)
  {
    case 1:
      if(m_nRecIdx > m_nSaves)
        Connect(true);
      else
        getSave(m_nRecIdx+1);
      break;
    case 2:
      if(m_nRecIdx > m_nRecords)
        Connect(true);
      else
        getRecord(m_nRecIdx+1);
      break;
  }
}

void UT181Interface::getRecord(uint16_t nItem)
{
  uint8_t cmd[]= {0x0C, 0, 0};

  cmd[1] = nItem & 0xFF;
  cmd[2] = nItem >> 8;
  Write(cmd, sizeof(cmd) );
}

void UT181Interface::getRecordCount()
{
  static uint8_t cmd[]= {0x0E};

  Write(cmd, sizeof(cmd) );
}

void UT181Interface::decodeSamples(uint8_t *p, uint8_t count)
{
/*
  float fScale = GetValueScale(m_pRecords[m_nRecordSampleItem].szUnit);
  int n = m_nRecDataIndex;
  int tick = 0;
  uint8_t lastSec = 0;

  for(int i = 0; i < count; i++)
  {
    MValue *pM = (MValue*)p;
    memcpy(&m_cData.pData[m_cData.dwIndex].Value[0], pM, sizeof(MValue) );
    m_cData.pData[m_cData.dwIndex].fScale = fScale;
    p += 5;

    m_cData.pData[m_cData.dwIndex].t.dw = *(uint32_t*)p;
    if(m_cData.pData[m_cData.dwIndex].t.seconds != lastSec || tick > 9)
    {
      lastSec = m_cData.pData[m_cData.dwIndex].t.seconds;
      tick = 0;
    }
    m_cData.pData[m_cData.dwIndex].tick = tick++;
    p += 4;
    if(m_cData.dwCount < m_pRecords[m_nRecordSampleItem].dwSamples)
    {
      m_cData.dwIndex++;
      m_cData.dwCount++;
    }
  }
  */
}

float UT181Interface::GetValueScale(char *pszUnit)
{
  switch(pszUnit[0])
  {
    case 'k': return 1000;
    case 'M': return 1000000;
    case 'm': return 0.001f;
    case 'u': return 0.000001f;
    case 'n': return 0.000000001f;
    default: return 1;
  }
}

void UT181Interface::getRecordData()
{
  #pragma pack(push, 1)
  struct recReq
  {
    uint8_t Cmd;
    uint16_t wItem;
    uint32_t dwOffset;
  };
  #pragma pack(pop)

  recReq rr;

  rr.Cmd = 0x0D;
  rr.wItem = m_nRecordItem;
  rr.dwOffset = m_nRecDataIndex + 1;
  Write((uint8_t*)&rr, sizeof(rr) );
}

void UT181Interface::Save()
{
  static uint8_t cmd[]= {0x06};

  Write(cmd, sizeof(cmd) );
}

void UT181Interface::getSave(uint16_t nItem)
{
  uint8_t cmd[]= {0x07, 0, 0};

  cmd[1] = nItem & 0xFF;
  cmd[2] = nItem >> 8;
  Write(cmd, sizeof(cmd) );
}

void UT181Interface::getSaveCount()
{
  static uint8_t cmd[]= {0x08};

  Write(cmd, sizeof(cmd) );
}

void UT181Interface::DeleteAllSave() // Saves
{
  static uint8_t cmd[]= {0x09, 0xFF, 0xFF};

  Write(cmd, sizeof(cmd) );
}

const char *UT181Interface::convertDate(uniDate &dt)
{
  static String s;
  s = String(dt.year + 2000) + "/";
  if(dt.month<10) s += "0";
  s += dt.month;
  s += "/";
  if(dt.day<10) s += "0";
  s += dt.day;
  s += " ";
  if(dt.hours<10) s += " ";
  s += dt.hours;
  s += ":";
  if(dt.minutes<10) s += "0";
  s += dt.minutes;
  s += ":";
  if(dt.seconds<10) s += "0";
  s += dt.seconds;
  return s.c_str();
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

  if(m_MData.Peak)
  {
    nMin = nMax = nMod = 0;
    return;
  }

  switch(m_MData.dataType) // no bar modes
  {
    case 0x01: // T1
      nMin = nMax = nMod = 0;
      return;
    case 0x3: // T1-T2 // Todo: fix data for mVac+dc
      if(m_MData.Switch == eSwitch_mVDC)
      {
        nMin = nMax = nMod = 0;
        return;
      }
      break;
  }

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
