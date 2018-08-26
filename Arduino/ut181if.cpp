#include "ut181if.h"

extern void sendBinData(uint8_t *p, int len);

UT181Interface::UT181Interface()
{
}

void UT181Interface::service()
{
  while(Serial.available())
  {
    uint8_t c = Serial.read();

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
//            sendBinData("ERR", 3);
          }
          m_state = 0;
        }
        break;
    }
  }
}

bool UT181Interface::Connected()
{
  return m_bConnected;
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
  m_Updated++;
  switch(m_buffer[0])
  {
    case 0x01:  // 1 = command respsonses
          // 01 4F 4B A0 = 'OK '
          // 01 45 52 9D = 'ER'
          // 01 49 4E 56 = 'INV'
      m_bConnected = true;
      break;
    case 0x72:  // number of saves/records
      switch(m_buffer[1])
      {
        case 0x08: // saves
          m_nSaves = getWord(m_buffer + 2); // 72 08 03 00 83
          Connect(false); // disable to read
          if(m_pSave)
            delete m_pSave;
          m_pSave = new SaveRec[m_nSaves];
          m_nRecReq = 1;
          m_nRecIdx = 0;
          break;
        case 0x0E: // records
          m_nRecords = getWord(m_buffer + 2); // 72 0E 02 00 88
          Connect(false); // disable main data to read
          if(m_pRecords)
            delete m_pRecords;
          m_pRecords = new Record[m_nRecords];
          m_nRecReq = 2;
          m_nRecIdx = 0;
          break;
      }
      break;
    case 0x03: // save data
      if(m_pSave == NULL || m_nRecIdx >= m_nSaves)
      {
        m_nRecReq = 0;
        break;
      }

      memcpy(&m_pSave[m_nRecIdx], m_buffer + 1, sizeof(SaveRec));

      if(++m_nRecIdx < m_nSaves)
        GetRecordSeq(); // get more
      else
      {
        m_nRecReq = 0;
        Connect(true);
      }
      break;

    case 0x04: // record data
      if(m_pRecords == NULL || m_nRecIdx >= m_nRecords)
      {
        m_nRecReq = 0;
        break;
      }

      memcpy(&m_pRecords[m_nRecIdx], m_buffer + 1, sizeof(Record));

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
    case 0x02: // main meter data
      m_bConnected = true;
      memset(&m_exValues, 0, sizeof(m_exValues)); // just easy
      memset(&m_MM, 0, sizeof(m_MM));
      memcpy(&m_MData, m_buffer + 1, sizeof(MData));

      if(m_MData.Recording)
      {
        memcpy(&m_recTime, m_buffer + 19, sizeof(m_recTime));
      }
      if(m_MData.Comp)
      {
        memcpy(&m_Comp, m_buffer + 19, sizeof(Comp));
      }
      else switch(m_MData.dataType)
      {
        case 0x08: // normal
          break;
        case 0x0A: // peak / dbV
        case 0x42:
        case 0x02:
        case 0x06:
        case 0x16: // AC+DC, temps
        case 0x0E: // relative
        case 0x1E:
          memcpy(&m_exValues, m_buffer + 19, sizeof(m_exValues));
          break;
        case 0x2F:
        case 0x27: // min/max
          memcpy(&m_MM, m_buffer + 11, sizeof(m_MM));
          strcpy(m_MData.szUnit, (char *) m_buffer+38);
          break;
      }
      break;
  }
}

float UT181Interface::bin2float(uint8_t *val)
{
  uint32_t *dwP = (uint32_t*)val;
  uint32_t dwValue = *dwP;
  float *pf = (float *)&dwValue;
  return *pf;
}

void UT181Interface::getMValue(MValue *mv, uint8_t *p)
{
  memcpy((uint8_t*)mv, p, sizeof(MValue));
}

void UT181Interface::float2bin(float fValue, uint8_t *p)
{
  uint32_t *pd = (uint32_t*)&fValue;
  uint32_t *dwP = (uint32_t*)p;
  *dwP = *pd;
}

uint32_t UT181Interface::getLong(uint8_t *p)
{
  uint32_t *dwP = (uint32_t*)p;
  return *dwP;
}

uint16_t UT181Interface::getWord(uint8_t *p)
{
  uint16_t *wP = (uint16_t*)p;
  return *wP;
}

void UT181Interface::startRecordRetreval(int nItem)
{
  if(m_pRecords == NULL || nItem >= m_nRecords)
    return;

  AllocChart(m_pRecords[nItem].dwSamples);
  m_nRecDataIndex = 0;
  m_nRecDataIndexEnd = m_pRecords[nItem].dwSamples;
  m_nRecordItem = nItem + 1;
  m_bGetRecStart = true;
  m_nRecordSampleItem = nItem;
  Connect(false);
}

void UT181Interface::AllocChart(uint32_t dwSize)
{
  if(m_cData.pData)
    delete m_cData.pData;
  memset(&m_cData, 0, sizeof(m_cData));
  m_cData.pData = new ItemData[dwSize + 254];
  memset(m_cData.pData, 0, sizeof(ItemData) * (dwSize + 254) );
  m_cData.dwAvail = dwSize;
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
    static uint8_t val[]= {3, 0,0,0,0};
    float2bin(fValue, val + 1 );
    Write(val, sizeof(val) );
  }
}

float UT181Interface::GetfValue()
{
  return m_MData.Value.fValue;
}

uint8_t UT181Interface::GetSelect(void)
{
  return m_MData.Select;
}

bool UT181Interface::RelState()
{
   switch(m_MData.dataType)
   {
      case 0x10: // rel
      case 0x16: // rel temp
      case 0x1E: // dbV
        return true;
   }
   return false;
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
  uint8_t cmd[]= {0x0E};

  Write(cmd, sizeof(cmd) );
}

void UT181Interface::decodeSamples(uint8_t *p, uint8_t count)
{
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
  uint8_t cmd[]= {0x06};

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
  uint8_t cmd[]= {0x08};

  Write(cmd, sizeof(cmd) );
}

void UT181Interface::DeleteAllSave() // Saves
{
  uint8_t cmd[]= {0x09, 0xFF, 0xFF};

  Write(cmd, sizeof(cmd) );
}

char *UT181Interface::TimeText(uniDate dt)
{
  static char szDate[24];
  sprintf(szDate, "%d/%02d/%02d %2d:%02d:%02d",
    dt.year + 2000, dt.month, dt.day, dt.hours, dt.minutes, dt.seconds);
  return szDate;
}

int UT181Interface::dataType()
{
  return m_MData.dataType;
}

char *UT181Interface::UnitText()
{
  return m_MData.szUnit;
}

int UT181Interface::DisplayCnt()
{
  if(m_MData.dataType == 0x27 || m_MData.dataType == 0x2F ) // min/max
  {
    return 4;
  }
  switch(m_MData.dataType)
  {
      case 0x0A:// dbV
      case 0x42: // Peak
      case 0x02: // T1
        return 2;
      case 0x10: // rel
      case 0x16: // rel temp
      case 0x1E: // dbV
      case 0x06: // T1-T2 // Todo: fix data for mVac+dc
      case 0x0E: // Hz Ms
        return 3;
      case 0x2F: // Max min
      case 0x27: // max min temp
        return 4;
      default:
        return 1;
    }
}

char *UT181Interface::ValueText(int which)
{
  MValue Value;

  if(DisplayCnt() < which)
    return "";

  switch(which)
  {
    default:  Value = m_MData.Value; break;
    case 1:
      if(m_MData.dataType == 0x27 || m_MData.dataType == 0x2F) // MM
        Value = m_MM.Value1;
      else
        Value = m_exValues.Value1;
      break;
    case 2:
      if(m_MData.dataType == 0x27 || m_MData.dataType == 0x2F) // MM
        Value = m_MM.Value2;
      else
        Value = m_exValues.Value2;
      break;
    case 3:
      if(m_MData.dataType == 0x27 || m_MData.dataType == 0x2F) // MM
        Value = m_MM.Value3;
      else
        Value = m_exValues.Value3;
      break;
  }
  
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
