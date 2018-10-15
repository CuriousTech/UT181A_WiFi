
#ifndef UT181IF_H
#define UT181IF_H

#include <Arduino.h>

enum UT_CMD{
  CMD_SET_OPTION = 1,
  CMD_SET_RANGE,
  CMD_SET_REL,
  CMD_SET_MINMAX,
  CMD_CONT_DATA,
  CMD_SAVE,
  CMD_GET_SAVE,
  CMD_GET_SAVE_COUNT,
  CMD_DELETE_SAVE_ITEM,
  CMD_RECORD_START,
  CMD_STOP_RECORD,
  CMD_GET_RECORD,
  CMD_GET_RECORD_DATA,
  CMD_GET_RECORD_COUNT,
  CMD_DELETE_RECORD_ITEM,
  CMD_SET_CLOCK,
  CMD_QUERY_MODEL,
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

enum RXID{
  RX_ACK = 1,
  RX_MDATA,
  RX_SAVE_ENT,
  RX_REC_ENT,
  RX_REC_DATA,
  RX_REC_CNT = 0x72
};

#pragma pack(push, 1)

struct uniDate{
  uint32_t year:6;
  uint32_t month:4;
  uint32_t day:5;
  uint32_t hours:5;
  uint32_t minutes:6;
  uint32_t seconds:6;
};

union uddw{
    uint32_t dw;
    uniDate ud;
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

struct RecItem{
  MValue  Value;
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
  uniDate Date; // 4 bytes
  uint8_t type:3;
  uint8_t ShowBar:1;
  uint8_t Rel:1;
  uint8_t MinMax:1;
  uint8_t Peak:1;
  uint8_t Hold:1;
  uint8_t Auto:1;
  uint8_t Over:1;
  uint8_t Unk1:1;
  uint8_t LeadError:1;
  uint8_t Comp:1;
  uint8_t Recording:1;
  uint8_t Unk2:2;
  uint8_t Beeper:2;
  uint8_t Unk3:2;
  uint8_t Mode:4;
  uint8_t Select:4;
  uint8_t Switch:4;
  uint8_t Range:4;
  uint8_t pad:4;
  MValue Value0; // 5 bytes

  union tag_u
  {
    uint8_t bytes[60];
    struct{
      char szLabel0[8];
      MValue Value11;
      char szLabel1[8];
      MValue Value12;
      char szLabel2[8];
      MValue Value13;
      char szLabel3[8];
      MValue Value14;
      char szLabel4[8];
    } a;
    struct{
      MValue Value21;
      uint32_t dwTime1;
      MValue Value22;
      uint32_t dwTime2;
      MValue Value23;
      uint32_t dwTime3;
      char szLabel[8];
    } b;
  } u;
};

union UData
{
  struct // normal and Comp mode
  {
    char    szUnit[8];
    float   fBarValue;
    char    szUnit2[8];
    uint8_t CompMode:2;
    uint8_t Unk1:1;
    uint8_t BeepMode:3;
    uint8_t Unk2:2;
    uint8_t Fail:1;
    uint8_t Unk3:7;
    uint8_t Precision;
    float   fHigh;
    float   fLow;
    char pad2[15];
  } Std; // 31
  struct // temp mode comp
  {
    char    szUnit[8];
    uint8_t CompMode:2;
    uint8_t Unk1:1;
    uint8_t BeepMode:3;
    uint8_t Unk2:2;
    uint8_t Fail:1;
    uint8_t Unk3:7;
    uint8_t Precision;
    float   fHigh;
    float   fLow;
    char    pad3[26];
  } CompTemp; //20
  struct // peak mode, dbV, relative, temp rel
  {
    char   szUnit[8];
    MValue Value1;
    char   szUnit1[8];
    MValue Value2;
    char   szUnit2[8];
    MValue Value3;
    char   szUnit3[8];
  } Ext; //46
  struct
  {
    char     szUnit[8];
    uint32_t dwElapsed;
    uint32_t dwRemain;
    uint32_t dwSamples;
    char     pad3[26];
  } RecTimer; // 20
  struct
  {
    MValue   Value1;
    uint32_t dwTime1;
    MValue   Value2;
    uint32_t dwTime2;
    MValue   Value3;
    uint32_t dwTime3;
    char     szUnit[8];
    char     pad4[11];
  } MM; // 35
};

struct MData
{
  uint8_t type:3;
//  uint8_t  MinMax2:1;
//  uint8_t  Ext:1; // 4 value
//  uint8_t  Triple:1; // 3 value
  uint8_t  ShowBar:1;
  uint8_t  Rel:1;
  uint8_t  MinMax:1;
  uint8_t  Peak:1;
  uint8_t  Hold:1;   // byte 0

  uint8_t  Auto:1;   // byte 1
  uint8_t  Over:1;
  uint8_t  Unk1:1;
  uint8_t  LeadErr:1;
  uint8_t  Comp:1;
  uint8_t  Recording:1;
  uint8_t  Unk2:2;   // byte 1
  uint8_t  Beeper:2; // 1=high, 2=low
  uint8_t  Unk3:2;   // byte 2
  uint8_t  Mode:4;
  uint8_t  Select:4; // byte 3
  uint8_t  Switch:4; // byte 3
  uint8_t  Range:4;  // byte 4
  uint8_t  nUnk4:4;  // byte 4
  MValue   Value;    // 5 bytes
  UData    u;        // union based on modes
};

struct recReq
{
  uint8_t Cmd;
  uint16_t wItem;
  uint32_t dwOffset;
};

struct recCmd
{
  uint8_t Cmd;
  char szName[11];
  uint16_t wInterval;
  uint32_t dwDuration;
};

#pragma pack(pop)

class UT181Interface
{
public:
  UT181Interface(){};
  void service(time_t nw);
  void start(bool bCont);
  bool Updated(void);
  bool WriteData(uint8_t *pData, int len);
  String convertDate(uniDate &dt);
  char *ValueText(int which);
  char *ValueText(MValue &mv);
  char *UnitText(void);
  int DisplayCnt(void);
  void SetRange(uint8_t n);
  void SetSelect(uint8_t nSel, uint8_t nOpt, bool bRel, float fValue);
  float GetfValue(void);
  void Hold(void);
  void MinMax(void);
  void RangeMod(int &nMin, int &nMax, int &nMod);
  void getSign(void);
  uint16_t StatusBits(void);
  void getRecordCount(void);
  void getSaveCount(void);
  void DeleteAllSave(void);
  void Save(void);
  void StartRecord(char *pName, uint16_t wInterval, uint32_t dwDuration);
  void StopRecord(void);
  void startRecordRetreval(int nItem, char *pszUnit, uint32_t dwSamples);
  int readPercent(void);
  void deleteSave(int nItem);
  void deleteRecord(int nItem);
  void setClock(void);
  void QueryModel(void);
private:
  uint16_t getWord(uint8_t *p);
  uint16_t sum(uint8_t *p, uint16_t len);
  bool Write(uint8_t *p, uint8_t len);
  void process_sentence(uint16_t len);
  void GetRecordSeq(void);
  void getRecord(uint16_t nItem);
  void decodeSamples(uint8_t *p, uint8_t count);
  void getRecordData(void);
  void getSave(uint16_t nItem);
  time_t uniDateToGTC(uniDate &dt);

  uint8_t  m_buffer[3200]; // 2254 required for records
  uint16_t m_idx;
  uint8_t  m_state;
  uint16_t m_len;
  int      m_Updated;

  uint16_t m_nSaves;
  uint16_t m_nRecords;
  uint8_t  m_nRecReq;
  bool     m_bGetRecStart;
  bool     m_bSign;
  int      m_nRecIdx;
  int      m_nRecordItem;
  int      m_nRecDataIndexEnd;
  int      m_nRecDataIndex;
  uint32_t m_keepAlive;
  char     m_szRecUnit[12];
  char     m_szModel[12];
  char     m_szSerial[32];
public:
  MData    m_MData;
  bool     m_bConnected;
};

#endif
