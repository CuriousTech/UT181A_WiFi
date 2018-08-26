
#ifndef UT181IF_H
#define UT181IF_H

#include <Arduino.h>

enum UT_CMD{
  CMD_SET_OPTION = 1,
  CMD_SET_RANGE,
  CMD_03,
  CMD_SET_MINMAX,
  CMD_CONNECT,
  CMD_SAVE,
  CMD_GET_SAVE,
  CMD_GET_SAVE_COUNT,
  CMD_DELETE_ALL_SAVE,
  CMD_DELETE_SAVE_ITEM,
  CMD_RECORD_START,
  CMD_STOP_RECORD,
  CMD_GET_RECORD,
  CMD_GET_RECORD_DATA,
  CMD_GET_RECORD_COUNT,
  CMD_DELETE_ALL_RECORDS,
  CMD_DELETE_RECORD_ITEM,
  CMD_10,
  CMD_11,
  CMD_HOLD,
};

enum eSwitch{
  eSwitch_None,
  eSwitch_VAC,
  eSwitch_mVAC,
  eSwitch_VDC,
  eSwitch_mVDC,
  eSwitch_Ohms,
  eSwitch_Diode,
  eSwitch_Hz,
  eSwitch_uA,
  eSwitch_mA,
  eSwitch_A,
};

#pragma pack(push, 1)

union uniDate{
  uint32_t dw;
  uint32_t year:6;
  uint32_t month:4;
  uint32_t day:5;
  uint32_t hours:5;
  uint32_t minutes:6;
  uint32_t seconds:6;
};

struct MValue
{
  float fValue;
  uint8_t L:1;
  uint8_t Blank:1;
  uint8_t Unk1:1;
  uint8_t Unk2:1;
  uint8_t Precision:4;
};

struct ItemData{
  MValue  Value[4];
  float   fScale;
  uint8_t tick;
  uniDate t;
};

struct Record
{
  char  szName[11];
  char  szUnit[8];
  uint16_t  wInterval;
  uint32_t dwDuration;
  uint32_t dwSamples;
  MValue mMax;
  MValue mAvg;
  MValue mMin;
  uniDate Date;
};

struct SaveRec
{
  uniDate Date;
  uint8_t dataType;
  uint8_t Unk[2];
  uint8_t Switch:4;
  uint8_t Select:4;
  uint8_t Range;
  MValue Value0;

  union tag_u
  {
    uint8_t bytes[60];
    struct a{
      char szLabel0[8];
      MValue Value11;
      char szLabel1[8];
      MValue Value12;
      char szLabel2[8];
      MValue Value13;
      char szLabel3[8];
      MValue Value14;
      char szLabel4[8];
    };
    struct b{
      MValue Value21;
      uint32_t dwTime1;
      MValue Value22;
      uint32_t dwTime2;
      MValue Value23;
      uint32_t dwTime3;
      char szLabel[8];
    };
  }u;
};

struct MData
{
  uint8_t  dataType:7;
  uint8_t  Hold:1;   // byte 0

  uint8_t  Auto:1;   // byte 1
  uint8_t  Over:1;   // byte 1
  uint8_t  Unk1:1;
  uint8_t  LeadError:1;
  uint8_t  Comp:1;
  uint8_t  Recording:1;
  uint8_t  Unk2:2;   // byte 1

  uint8_t  option1:4;  // byte 2
  uint8_t  option2:4;
  uint8_t  Select:4; // byte 3
  uint8_t  Switch:4; // byte 3
  uint8_t  Range:4;  // byte 4
  uint8_t  nUnk:4;   // byte 4
  MValue  Value;
  char  szUnit[8];
  uint8_t  Unk[4];
  char  szUnit2[8];
};

struct Comp  // offset 19
{
  uint8_t Unk[4];
  char szUnit[8];
  uint8_t CompMode:4;
  uint8_t BeepMode:4;
  uint8_t Fail;
  uint8_t Precision;
  float fHigh;
  float fLow;
};

struct MinMaxx
{
  MValue Value1;
  uint32_t dwTime1;
  MValue Value2;
  uint32_t dwTime2;
  MValue Value3;
  uint32_t dwTime3;
  char szUnit[8];
};

struct exData // peak mode, dbV, relative, temp rel
{
  MValue Value1;
  char szUnit1[8];
  MValue Value2;
  char szUnit2[8];
  MValue Value3;
  char szUnit3[8];
};

struct RecTimer
{
  uint32_t dwElapsed;
  uint32_t dwRemain;
  uint32_t dwSamples;
};

#pragma pack(pop)

struct ChartData
{
  uint32_t dwAvail;
  uint32_t dwIndex;
  uint32_t dwCount;
  ItemData *pData;
};

class UT181Interface
{
public:
  UT181Interface();
  void service(void);
  void Connect(bool bCon);
  bool Connected(void);
  bool Updated(void);
  bool WriteData(uint8_t *pData, int len);
  char *TimeText(uniDate dt);
  char *ValueText(int which);
  char *UnitText(void);
  int dataType(void);
  int DisplayCnt(void);
  void Hold(void);
  void MinMax(void);
  void SetRange(uint8_t n);
  void SetSelect(uint8_t nSel, bool bRel, float fValue);
  bool RelState(void);
  float GetfValue(void);
  uint8_t GetSelect(void);
private:
  float bin2float(uint8_t *val);
  void getMValue(MValue *mv, uint8_t *p);
  void float2bin(float fValue, uint8_t *p);
  uint32_t getLong(uint8_t *p);
  uint16_t getWord(uint8_t *p);
  float GetValueScale(char *pszUnit);
  uint16_t sum(uint8_t *p, uint16_t len);
  bool Write(uint8_t *p, uint8_t len);
  void process_sentence(uint16_t len);
  void GetRecordSeq(void);
  void getRecord(uint16_t nItem);
  void getRecordCount(void);
  void decodeSamples(uint8_t *p, uint8_t count);
  void getRecordData(void);
  void startRecordRetreval(int nItem);
  void AllocChart(uint32_t dwSize);
  void Save(void);
  void getSave(uint16_t nItem);
  void getSaveCount(void);
  void DeleteAllSave(void);

  uint8_t m_buffer[64]; // 3200 required for records
  uint8_t m_idx;
  uint8_t m_state;
  uint16_t m_len;
  bool    m_bConnected;

  uint16_t  m_nSaves;
  uint16_t  m_nRecords;
  MData     m_MData;
  Comp      m_Comp;
  RecTimer  m_recTime;
  MinMaxx   m_MM;
  exData    m_exValues;
  SaveRec  *m_pSave;
  Record   *m_pRecords;
  ChartData m_cData;
  uint8_t   m_nRecReq;
  int   m_nRecIdx;
  bool  m_bGetRecStart;
  int   m_nRecordItem;
  int   m_nRecDataIndexEnd;
  int   m_nRecDataIndex;
  int   m_nRecordSampleItem;
  int   m_Updated;
};

#endif
