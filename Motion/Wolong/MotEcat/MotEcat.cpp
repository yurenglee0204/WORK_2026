
/***************************************
 FName : MotEcat.cpp
 Date  : 115.01.23
 Writer: Chen, Shih-tung
 Desc. : . 07.01.16, 新建立, MOT interface for ECAT;
             .  不直接使用 [EcMaster.lib];
         . 07.02.23 .. 07.01.24, 新增 _DUMP_INFO_AX_...;
         . 07.03.19 .. 07.03.13, 新增 SDP-E (Shihlin Electric)的支援;
         . 07.08.30 .. 07.06.01, 調整 _MAX_YAS_AXIS_ == 6 -> 18;
             .  新增 dwMotEcatStatus;
             .  修正 _NOT_READY_SON_ 的處理;
         . 07.09.26 .. 07.09.18, 修正 hrmb_phase_process(); 
         . 08.01.07, 08.01.02, 07.12.27 .. 07.10.24, 新建立, for CE60_EV355;
             .  HalRmb 對應的是 CNC-IO, Slave-type == 3;
             .  Panasonic drives, Slave-type == 4;
         . 08.02.27, 修正 opreation_loop(), 補充 wTouchFunc[] 的來源; 
         . 08.04.16, 08.03.21, 新增 Maxsine drives, Slave-type == 5; 
         . 08.05.06, 修正為不自動進入 _OPERATION_;
         . 08.05.24, 支援 New CNC_IO Pulse mode; 
         . 08.06.24, 08.06.20, 修正 New CNC_IO Ax == 1 的支援; 
         . 09.03.11, 09.02.26, 08.12.11, 修正 CNC_IO 的命令處理流程, 區隔 CNC_RMB/CNC_IO:
         . 09.04.14, 補充 [Panasonic] 對 sTorqVal[] 的支援;
         . 09.05.07, 啟用 wUsrTorqMax[] 設定介面;
         . 10.03.16, 10.03.08, 新增對 [高創]的支援;
         . 10.03.25, 修正脫離 _OPERATION_ 的命令維護;  
         . 10.07.15, 新增對 [固高]的支援, wSlaveType[] == 9;
         . 10.07.29, 10.03.08, 修正對 [高創]的支援, 調整 PDO 內容;
         . 10.08.16, 修正 _sdo_read/_sdo_write;
         . 10.10.08, 10.10.07, 新增選項: 直接呼叫 mot_r0_process();
         . 10.10.13, 新增 log_ecat_comm_info_ex();
         . 10.10.20, 修正對 [固高]的支援, 調整 PDO 內容;
         . 10.11.08, _NO_ECAT_R0_THREAD_ == 1 -> 0;
         . 11.03.01, 11.02.25, 11.01.28, 11.01.10, 新增對 [菲仕]的支援, wSlaveType[] == 10, 11;
         . 11.06.21, 新增 Nidec 的支援, == 12;
         . 11.07.11, 修正 Warning 不視為 _FAULT_;
         . 11.12.09, 11.11.22, 新增對 [Schneider]的支援 == 13, 14;
         . 12.07.21, 12.07.20, 補充修正 == 14 的遺漏;
             .  新增 CNC-IO, iDebugInf[];
         . 12.11.28, 新增 iCmdPosRet[];
         . 13.03.14, 13.03.07, 新增對 [SA3_EC301]的支援 == 16;
         . 13.04.02, 補充 log info.;
         . 13.07.09, 13.05.27, 新增 dwDOutVelRd[];
         . 13.10.18, 13.09.03, 13.08.16, 新增 Beijing CTB 的支援, == 17;
             .  新增 Lenze == 15;
         . 13.12.24, 修正  == 17 的支援;
         . 14.08.12, .. 14.07.22, 新增 pISR_ecat_r0_func 介面;
         . 15.01.09, 14.12.23, 新增 HCFA == 18; (禾川)
         . 15.01.15, 新增 ACS880 == 19; (弘遠)
         . 15.01.23, 修正 _SW_ON_DISABLE_ 流程, 處理 FAULT 後的影響;
***************************************/

#include "stdafx.h"

#include <AtEthercat.h>

#include <windows.h>

#include "MotEcat_def.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include <memory.h>
#include "WolongPdo.h"

  //  14.07.22, 10.10.07;
//  #define _NO_ECAT_R0_THREAD_         0
#define _NO_ECAT_R0_THREAD_         1

#if  ( _MOT_CE600_ == 1 )
  #include "HalEcatLib.h"
  
  #define  RtCreateEvent                CreateEvent
  #define  RtOpenEvent                  OpenEvent
  #define  RtCloseHandle                CloseHandle
  #define  RtWaitForSingleObject        WaitForSingleObject
  #define  RtPulseEvent                 PulseEvent
  #define  RtGetClockTime( a, b)        QueryPerformanceCounter( b )
  #define  RtPulseEvent                 PulseEvent
  #define  RtSetEvent                   SetEvent
  #define  RTFCNDCL         
  #define  RtPrintf                     printf
#endif

#define _ECAT_MAX_REG_          20000
#define _ECAT_MAX_OP_           2000
#define _MAX_YAS_AXIS_          18

typedef void (*FUNC_PTR)(void);

extern "C"
{
  __declspec (dllexport) void MotAppInit( );
  __declspec (dllexport) void MotAppClose( );
  __declspec (dllexport) void MotAppPrepare( void **pSlaveList, DWORD SlaveNum );
  __declspec (dllexport) void MotAppWorkpd( BYTE *pbyPDIn, BYTE *pbyPDOut, DWORD dwState );
  __declspec (dllexport) void oprtss_ptime_start( int iTm );
  __declspec (dllexport) void oprtss_ptime_end( int iTm );  
  //  __declspec (dllexport) int  MECAT_set_r0_func( FUNC_PTR );
  //  __declspec (dllexport) int  iLastProcTm[10];
}

extern "C"
{
  void ISR_ecat_r0_func(void);
  void MECAT_initial(void);
  void MECAT_close(void);
  
  int   iLastProcTm[];
  DWORD dwMEcatR0Cnt;
}

extern "C" DWORD dwMotApiLoadFlag;

//char szDateMsg[] = "115.06.23.101.Sw";				//"115.01.23 102.Sw";
char szDateMsg[] = "115.09.22.101.Sw";					//"115.01.23 102.Sw";
unsigned long dwVersionMark = 0x0DB260731;				// 0x0DB260123;

DWORD WINAPI oprtss_thread(LPVOID pp);
DWORD WINAPI mot_r0_thread(LPVOID pp);
DWORD WINAPI mot_r3_thread(LPVOID pp);

void SaveEcatOpData( );     //  07.01.31;   
void SaveEcatCommData( );   //  07.01.30;

int  switchon_loop(int ax);     //  13.10.18;

int  mot_r0_process();
void mot_r3_initial( );
int  mot_r3_get_user_info( );
void block_info_input();
void block_info_output();
void log_ecat_comm_info( int  i );
void log_ecat_comm_info_ex( int  i, int type );     //  10.10.13;

int Type20ParamService(int ax);										// 2026/07/29, Ulysses
int Type20UserVFFProcess(int ax, int nPosCmd, int nVFFGain);		// 2026/07/29, Ulysses
void DebugPrintTypeVFF(FILE *fout);									// 2026/07/31, Ulysses
int Type20VCmdFilterProcess(int ax, int nVCmd);						// 2026/08/16, Ulysses

#define TYPE_20_VCMD_FILTER											// 2026/08/16, Ulysses

void  *my_aSlaveList[MAX_NUM_SLAVES];
//  WORD  *pwHalRmbTxBase, *pwHalRmbRxBase;
BYTE  *pbyHalRmbTxBase, *pbyHalRmbRxBase;
BYTE  *pbyEcatPDIn;
BYTE  *pbyEcatPDOut;
DWORD dwYasFoundSlaves, dwHalRmbBase, dwHalRmbStartF, 
      dwHalRmbTxNum, dwHalRmbRxNum, dwHRmbNum;
DWORD dwEcatMasterState;
//  BYTE  HRmbInBuff[5][22], HRmbOutBuff[5][8];     for Hal-Rmb module;
BYTE  HRmbInBuff[3][28], HRmbOutBuff[3][18];     //  for CNC-IO module;
DWORD dwHalRmbSAddr;

SYSTEMTIME  sysHalCurrTime;        //  07.12.27;

// for periodic timer code
LARGE_INTEGER  liPeriod;

HANDLE   hOpShMem, hOpRtssEvent, hOpRtssThd;
HANDLE   hMotR0Thd, hMotR3Thd, 
         hEcatR0Event, hEcatR3fEvent, hEcat4msEvent;

unsigned char *pOpShMem;

DWORD   *p_McmdComd,   *p_McmdParm1,   *p_McmdParm2,  
        *p_McmdParm3,  *p_McmdParm4,   *p_McmdSysInfo,
        *p_McmdRetSts, *p_McmdRetVal1, 
        *p_McmdRetVal2,*p_McmdRetVal3,
        *p_McmdBlkIn,  *p_McmdBlkOut,  *p_McmdInfoOut;
DWORD   *p_McmdHRmbIn, *p_McmdHRmbOut;  
DWORD   dwOpRtssFlag=0, dwOpRtssCount=0, dwOpRtssExecFlag=0;
DWORD   dwOpRtssTid,  dwOpRtssPara,  dwOpRtssActiveF; 
DWORD   dwMotR0Count=0, dwMotR3Count=0;
DWORD   dwMotR0Tid,  dwMotR0Para,  dwMotR0ActiveF; 
DWORD   dwMotR3Tid,  dwMotR3Para,  dwMotR3ActiveF; 
DWORD   g_dwParaSize,  g_dwParaVal; 
//  10.08.16, 10.08.13;
BYTE    strSdoWrBuff[30], strSdoRdBuff[30];
//  10.10.08;
int     iCurrThdPri, iPreThdPri, iLstThdPri;
//  13.07.09; 
DWORD   dwCncIoIsrCnt, dwCncIoSync0Cnt, dwCncIoDiffCnt; 
DWORD   dwCncIoDOut_0, dwCncIoDOut_1, dwCncIoDOut_2; 

//  Planning data; 
USHORT  wOldCtrlCmd[MAX_NUM_SLAVES], wCurrPhase[MAX_NUM_SLAVES];
int     iCurrGlbPos[MAX_NUM_SLAVES];
int     oprtss_cmd_read_idx[MAX_NUM_SLAVES], oprtss_cmd_write_idx[MAX_NUM_SLAVES];
int     oprtss_cmd_buff[MAX_NUM_SLAVES][_MAX_CMD_NUM_];

//  Command request from client(User); 
int     iUsrPosCmd[MAX_NUM_SLAVES], iUsrVelCmd[MAX_NUM_SLAVES];
short   shUsrTorqCmd[MAX_NUM_SLAVES];  
USHORT  wUsrTorqMax[MAX_NUM_SLAVES], wUsrCtrlCmd[MAX_NUM_SLAVES],
        wUsrTouchFunc[MAX_NUM_SLAVES], wOldUsrCtrlCmd[MAX_NUM_SLAVES];  
BYTE    bUsrModeWrite[MAX_NUM_SLAVES];
BYTE    boldUsrModeWr[MAX_NUM_SLAVES];  //  13.10.18;
DWORD   dwUsrReqFlag=0, dwUsrReqFunFlag[MAX_NUM_SLAVES];  //  08.04.16;
int     iUsrVffCmd[MAX_NUM_SLAVES];     //  11.12.09;

LARGE_INTEGER   liCurrProcTm[10], liPrevProcTm[10], liDiffProcTm[10];
int     iMaxProcTm[10];
//  int     iLastProcTm[10];

USHORT  wTouchSts[MAX_NUM_SLAVES], wSlaveType[MAX_NUM_SLAVES];  
USHORT  wPhaseCnt[MAX_NUM_SLAVES];      //  11.12.09;  
int     iTouchPos[MAX_NUM_SLAVES];
int     iDebugInf[MAX_NUM_SLAVES];      //  12.07.20;
int     iCmdPosRet[MAX_NUM_SLAVES];     //  12.11.28;
USHORT  wTouchStsOld[MAX_NUM_SLAVES];   //  10.03.16;  
USHORT  wTouchFuncOld[MAX_NUM_SLAVES];  //   
DWORD   dwDInputOld[MAX_NUM_SLAVES];    //   
DWORD   dwDOutVelRd[MAX_NUM_SLAVES];    //  13.05.27; 

DWORD   dwAuToCtrlCmd[MAX_NUM_SLAVES];  //  07.02.02;  
DWORD   dwDInput[MAX_NUM_SLAVES];       //  07.02.12;  
//  07.01.30;
DWORD   dwEcatCmIdx=0, dwEcatCmCnt=0;
int     iEcatCommBuff[_ECAT_MAX_REG_][15];
DWORD   dwEcatOpIdx=0, dwEcatOpCnt=0;
int     iEcatOpBuff[_ECAT_MAX_OP_][10];
DWORD   dwUbitVal[32], dwAxUsrReqFlag=0;
//  07.02.12, 07.02.08;
WORD    wHRmbStatus[5], wHRmbErrcode[5], wHRmbLatchSts[5],
        wHRmbCtrl[5], wHRmbLatFunc[5], wHRmbPhase[5];
int     iHRmbEncPos[5], iHRmbLatchP2[5], 
        iHRmbLatchP1[5], iHRmbVelCmd[5];
DWORD   dwHRmbDInp[5], dwHRmbAuToCtrl[5]; 
WORD    wUsrHRCtrl[5], wUsrHRLatFunc[5],
        wOldUsrHRCtrl[5], wOldHRmbCtrl[5], wOldUsrHRLatFunc[5];
DWORD   dwUsrHRDOutp[5], dwOldUsrHRDOutp[5], dwHRmbDOutp[5];
int     iUsrHRVelCmd[5], iOldUsrHRVelCmd[5];
WORD    wHRmbActiveNum, wHRmbInitFlag=0;
DWORD   dwHrmbInitCnt=0;
DWORD   dwMotEcatStatus=0, dwMotEcatRdyFlag=0;
int     iLastCmdPos[MAX_NUM_SLAVES], iNewPosCmd[MAX_NUM_SLAVES];
USHORT  wOldSlavSts[MAX_NUM_SLAVES];
WORD    wOldHRmbSts[MAX_NUM_SLAVES];
WORD    wFollowErrCnt[MAX_NUM_SLAVES];  //  08.05.06;
//  08.05.24;
int     iHRmbPulCmd[5], iHRmbHwCmdPos[5],
        iUsrHRPulCmd[5], iOldUsrHRPulCmd[5]; 
int     iHwCmdPos[MAX_NUM_SLAVES]; 
//  09.02.26;
DWORD   dwWarnCount[MAX_NUM_SLAVES];  
//  09.04.14;
WORD    wOldErrCode[MAX_NUM_SLAVES];
//  13.03.06;
DWORD   dwParam0[MAX_NUM_SLAVES], dwParam1[MAX_NUM_SLAVES],  
        dwParam2[MAX_NUM_SLAVES], dwParam3[MAX_NUM_SLAVES],  
        dwParam4[MAX_NUM_SLAVES], dwParam5[MAX_NUM_SLAVES];  

//  09.02.26;
//  extern BYTE     YasInBuff[][40], YasOutBuff[][40];
extern BYTE     YasInBuff[][64], YasOutBuff[][40];
extern int      iEncPos[], iFolwErrVal[], iCmdPos[], 
                iVelCmd[], iVelRdVal[];     //  13.09.03;
extern short    shTorqVal[], shTorqCmd[], shVffCmd[];  //  11.12.09;
extern USHORT   wSlavSts[], wCtrlCmd[], wTorqMax[];  
extern USHORT   wTouchFunc[], wErrCode[];  
extern BYTE     bModeWrite[], bModeRead[];
extern DWORD    dwYasFoundSlaves; 
extern int      iMaxProcTime;
extern DWORD    dwMotAppCount;
//  10.10.20;
extern DWORD    dwMotAppBusyCount;

///////////////////////////////////////////////////////////////////////////				// 2026/07/31, Ulysses
double g_dType20VFFExpTau[_MAX_YAS_AXIS_] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
double g_dTypae20VFF[_MAX_YAS_AXIS_] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int g_nType20PosCmd[_MAX_YAS_AXIS_] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int g_nType20UserVFFEnable[_MAX_YAS_AXIS_] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int g_nType20UserVFFGain[_MAX_YAS_AXIS_] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};		// %
int g_nType20UserVFFFilter[_MAX_YAS_AXIS_] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};		// 0.01ms
int g_nType20VUnit[_MAX_YAS_AXIS_] = {3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3};				// 0: PPS, 3: RPM
bool g_bVFFStartUp = true;

double g_dType20VCmdFExpTau[_MAX_YAS_AXIS_] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int g_nType20VCmd[_MAX_YAS_AXIS_] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
double g_dTypae20VCmdF[_MAX_YAS_AXIS_] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

///////////////////////////////////////////////////////////////////////////
// 2026/07/06, New Log directory
BOOL g_bNewLogDirectort = FALSE;
BOOL IsDirectoryExist(const wchar_t* path)
{
    DWORD dwAttrib = GetFileAttributes(path);
    return (dwAttrib != INVALID_FILE_ATTRIBUTES &&
            (dwAttrib & FILE_ATTRIBUTE_DIRECTORY));

}

///////////////////////////////////////////////////////////////////////////

/*:: ec301_set_init ------------------*/
int  ec301_set_init( int iNo )
{
  if  ( iNo < 0 || iNo >= MAX_NUM_SLAVES )  return( -1 );
  
  /***  13.07.08;
  p_McmdBlkIn[380+iNo] = dwParam0[iNo] = 5000;     //  Ramp function time(U32), ms; 
  p_McmdBlkIn[390+iNo] = dwParam1[iNo] = 5000;     //  Ramp function time(U32) 
  p_McmdBlkIn[400+iNo] = dwParam2[iNo] = 61340;    //  Profile velocity(U32), pulse/sec; 
  p_McmdBlkIn[410+iNo] = dwParam3[iNo] = 5100;     //  Profile acceleration(U32), pulse/sec2; 
  p_McmdBlkIn[420+iNo] = dwParam4[iNo] = 5100;     //  Profile deceleration(U32), pulse/sec2; 
  p_McmdBlkIn[430+iNo] = dwParam5[iNo] = 25500;    //  Quick stop deceleration(U32), pulse/sec2; 
  ***/
  bModeWrite[iNo] = 0x02;   //  2: Velocitymode;
  
  return( 0 );
}

/*:: oprtss_init ---------------------*/
int  oprtss_init()
{
  int   i;
     
  // RtPrintf("VVVVV: CstOpRtss_init().. [%s] \n", szDateMsg);
  if  ( dwOpRtssFlag == 1 )  return( 1 );

  #if  ( _MOT_CE600_ == 1 )
    hOpRtssEvent  = RtCreateEvent(NULL, FALSE, FALSE, _T("ECAT_OP_EVENT") );
    hEcatR0Event  = RtCreateEvent(NULL, FALSE, FALSE, _T("ECAT_R0_EVENT") );
    hEcatR3fEvent = RtCreateEvent(NULL, FALSE, FALSE, _T("ECAT_R3F_EVENT") );
    hEcat4msEvent  = RtCreateEvent(NULL, FALSE, FALSE, _T("ECAT_4ms_EVENT") );
  #else
  hOpRtssEvent  = RtCreateEvent(NULL, FALSE, FALSE, "ECAT_OP_EVENT" );
  hEcatR0Event  = RtCreateEvent(NULL, FALSE, FALSE, "ECAT_R0_EVENT" );
  hEcatR3fEvent = RtCreateEvent(NULL, FALSE, FALSE, "ECAT_R3F_EVENT" );
  hEcat4msEvent  = RtCreateEvent(NULL, FALSE, FALSE, "ECAT_4ms_EVENT" );
  #endif

  #if  ( _MOT_CE600_ == 1 )
  hOpShMem = CreateFileMapping(
                           INVALID_HANDLE_VALUE, 
                           NULL,                //  lpFileMappingAttributes, 
                           PAGE_READWRITE,      //  flProtect, 
                           0,                   //  dwMaximumSizeHigh, 
                           0x4000,              //  dwMaximumSizeLow, 16KB; 
                           _T("ECAT_OP_SMEM")   //  lpName 
                           );
  if  ( hOpShMem == NULL )
      {
      printf("CreateFileMapping() failed... [ECAT_OP_SMEM] \n");
      return( -2 );
      }
  pOpShMem = (unsigned char *)MapViewOfFile( 
                           hOpShMem,        //  hFileMappingObject, 
                           FILE_MAP_WRITE,  //  dwDesiredAccess, 
                           0,               //  dwFileOffsetHigh, 
                           0,               //  dwFileOffsetLow, 
                           0                //  dwNumberOfBytesToMap, 0 for 全部;
                           );
  #else
  if  (! (hOpShMem = RtCreateSharedMemory(
                                       PAGE_READWRITE,      //  flProtect
                                       0,                   // MaximumSizeHigh
                                       0x4000,              // MaximumSizeLow, 16KB;
                                       "ECAT_OP_SMEM" ,     // lpName
                                       (void **)&pOpShMem   // location
                                       )  ))
      {
      RtPrintf("OpRtss: RtCreateSharedMemory error = %d\n",GetLastError() );
      return( -1 );
      }
  #endif

  p_McmdComd    = (DWORD *)(pOpShMem + _MCMD_COMD_);
  p_McmdParm1   = (DWORD *)(pOpShMem + _MCMD_PARM1_);
  p_McmdParm2   = (DWORD *)(pOpShMem + _MCMD_PARM2_);
  p_McmdParm3   = (DWORD *)(pOpShMem + _MCMD_PARM3_);
  p_McmdParm4   = (DWORD *)(pOpShMem + _MCMD_PARM4_);
  
  p_McmdRetSts  = (DWORD *)(pOpShMem + _MCMD_RET_STS_);
  p_McmdRetVal1 = (DWORD *)(pOpShMem + _MCMD_RET_VAL1_);
  p_McmdRetVal2 = (DWORD *)(pOpShMem + _MCMD_RET_VAL2_);
  p_McmdRetVal3 = (DWORD *)(pOpShMem + _MCMD_RET_VAL3_);    //  10.08.16;
  
  p_McmdSysInfo = (DWORD *)(pOpShMem + _MCMD_SYS_INFO_);
  p_McmdBlkIn   = (DWORD *)(pOpShMem + _MCMD_BLK_IN_);
  p_McmdBlkOut  = (DWORD *)(pOpShMem + _MCMD_BLK_OUT_);
  p_McmdInfoOut = (DWORD *)(pOpShMem + _MCMD_INFO_OUT_);

  p_McmdHRmbIn  = (DWORD *)(pOpShMem + _MCMD_HRMB_IN_);
  p_McmdHRmbOut = (DWORD *)(pOpShMem + _MCMD_HRMB_OUT_);

  //  mot_r3_initial( );
  //  10.10.08;
  iCurrThdPri = CeGetThreadPriority( GetCurrentThread() );
  iPreThdPri = iLstThdPri = iCurrThdPri;
  
  dwOpRtssActiveF = 1;
  hOpRtssThd = CreateThread(
                          NULL,	            // pointer to thread security attributes  
                          0,	            // initial thread stack size, in bytes 
                          oprtss_thread,	// pointer to thread function 
                          &dwOpRtssPara,    // argument for new thread 
                          0,	            // creation flags, runs immediately 
                          &dwOpRtssTid 	    // pointer to returned thread identifier 
                          );	
  #if  ( _MOT_CE600_ == 1 )
    CeSetThreadPriority( hOpRtssThd, 110 );    //  0..255, 0最高;
  #else
  RtSetThreadPriority( hOpRtssThd, 100);
  #endif

  #if  (_NO_ECAT_R0_THREAD_ == 1 )
  #else  
  /***/
  dwMotR0ActiveF = 1;
  hMotR0Thd = CreateThread( NULL, 0, mot_r0_thread,	 
                            &dwMotR0Para, 0, &dwMotR0Tid );	
  #if  ( _MOT_CE600_ == 1 )
    CeSetThreadPriority( hMotR0Thd, 3 );    //  0..255, 0最高;
  #else
  RtSetThreadPriority( hMotR0Thd, 125);
  #endif
  /***/
  #endif
  
  dwMotR3ActiveF = 1;
  hMotR3Thd = CreateThread( NULL, 0, mot_r3_thread,	 
                            &dwMotR3Para, 0, &dwMotR3Tid );	
  #if  ( _MOT_CE600_ == 1 )
    CeSetThreadPriority( hMotR3Thd, 100 );    //  0..255, 0最高;
  #else
  RtSetThreadPriority( hMotR3Thd, 110);
  #endif

  for (i = 0; i < MAX_NUM_SLAVES; i++)
      {
      wCurrPhase[i] = _NOT_READY_SON_;
      wOldCtrlCmd[i] = wCtrlCmd[i] = 0x06;
      bModeWrite[i] = 0x08;
      wTorqMax[i] = 7000;
      iVelCmd[i] = 0;
      iCmdPos[i] = 0;
      shTorqCmd[i] = 2000;
      wOldUsrCtrlCmd[i] = wUsrCtrlCmd[i] = 0x0;
      wTouchFunc[i] = 0;
      dwAuToCtrlCmd[i] = 1;     //  07.02.02;
      wOldSlavSts[i] = 0x70;    //  07.08.29;
      shVffCmd[i] = 0;          //  11.12.09;
      iUsrVffCmd[i] = 0;        //  
      boldUsrModeWr[i] = bUsrModeWrite[i] = 0x00;   //  13.10.18;
      
      iCurrGlbPos[i] = 0;
      //  清除 Cmd-buffer 資料;
      //  memset( &oprtss_cmd_buff[i][0], 0, _MAX_CMD_NUM_*sizeof(int) );
      ZeroMemory( &oprtss_cmd_buff[i][0], _MAX_CMD_NUM_*sizeof(int) );
      oprtss_cmd_read_idx[i] = 0;
      oprtss_cmd_write_idx[i] = 0;
      ZeroMemory( &YasOutBuff[i][0], 40 );
      //  08.05.06;
      wFollowErrCnt[i] = 0;
      dwWarnCount[i] = 0;   //  09.02.26;
      wOldErrCode[i] = 0;   //  09.04.14;
      wTouchStsOld[i] = 0;  //  10.03.16;
      wTouchFuncOld[i] = 0; //  
      dwDInputOld[i] = 0;   //  
      }

  for (i = 0; i < 10; i++)
      iMaxProcTm[i] = 0;
  
  for ( i = 0; i < 32; i++)
      dwUbitVal[i] = 0x00000001 << i;
  //  07.02.08;
  //  pwHalRmbTxBase = pwHalRmbRxBase = NULL;
  pbyHalRmbTxBase = pbyHalRmbRxBase = NULL;
  dwHalRmbTxNum = dwHalRmbRxNum = dwHalRmbBase = dwHRmbNum = 0;
  dwHalRmbStartF = 0;
  ZeroMemory( &HRmbInBuff[0][0], 84 );      //  84 = 3 * 28, 13.07.08;
  ZeroMemory( &HRmbOutBuff[0][0], 54 );     //  54 = 3 * 18;
  wHRmbActiveNum = 0x03;
  for (i = 0; i < 5; i++)
      {
      wHRmbPhase[i] = _NOT_READY_SON_;
      //  wHRmbCtrl[i] = wOldHRmbCtrl[i] = 0x06;
      wHRmbCtrl[i] = 0x80;          //  08.06.24;
      wOldHRmbCtrl[i] = 0x00;       //
      iHRmbVelCmd[i] = 0;
      wHRmbLatFunc[i] = 0;
      wUsrHRCtrl[i] = wOldUsrHRCtrl[i] = 0; 
      wUsrHRLatFunc[i] = wOldUsrHRLatFunc[i] = 0; 
      iUsrHRVelCmd[i] = iOldUsrHRVelCmd[i] = 0;
      dwHRmbAuToCtrl[i] = 1;    //  07.09.21;
      wOldHRmbSts[i] = 0x70;    // 
      dwUsrHRDOutp[i] = dwOldUsrHRDOutp[i] = 0; 
      }

  p_McmdSysInfo[9] = dwVersionMark;  
  //  pISR_ecat_r0_func = default_null_func;

  MECAT_initial();  //  14.08.06;
  
  dwOpRtssFlag = 1;
    
  return( 0 );
}

/*:: oprtss_close --------------------*/
int  oprtss_close()
{
  if  ( dwOpRtssFlag == 0 )  return( 1 );
  dwOpRtssFlag = 0;
  
  dwMotR3ActiveF = 0;
  dwMotR0ActiveF = 0;

  MECAT_close( );  //  14.08.11;

  dwOpRtssActiveF = 0;
  *p_McmdComd = _OPRT_CLOSE_ ;
  RtSetEvent( hOpRtssEvent );
  Sleep( 200 );
  SaveEcatOpData();
  SaveEcatCommData();
  
  RtCloseHandle( hEcatR0Event );
  RtCloseHandle( hOpRtssThd );
  RtCloseHandle( hOpRtssEvent );
  RtCloseHandle( hEcatR3fEvent );
  RtCloseHandle( hEcat4msEvent );
  RtCloseHandle( hOpShMem );
  
  return( 0 );
}

/*:: oprtss_ptime_start --------------*/
void oprtss_ptime_start( int iTm )
{
  if  ( iTm < 0 || iTm >= 10 )  return;
  
  RtGetClockTime( CLOCK_2, &liPrevProcTm[iTm] );  
}

/*:: oprtss_ptime_end ----------------*/
void oprtss_ptime_end( int iTm )
{
  if  ( iTm < 0 || iTm >= 10 )  return;

  RtGetClockTime( CLOCK_2, &liCurrProcTm[iTm] );  

  liDiffProcTm[iTm].QuadPart = liCurrProcTm[iTm].QuadPart - liPrevProcTm[iTm].QuadPart;
  iLastProcTm[iTm] = (int)liDiffProcTm[iTm].LowPart;
  if  ( iLastProcTm[iTm] > iMaxProcTm[iTm] )
      iMaxProcTm[iTm] = iLastProcTm[iTm];
}

/*:: block_info_input ----------------*/
void block_info_input()
{
  int       i;
  BYTE      byTmp;
  USHORT    usTmp;

  //  07.06.01, _MAX_YAS_AXIS_ <= 20;  
  //  Total: 256 Dword, 0 .. 255;  
  if  ( dwYasFoundSlaves > _MAX_YAS_AXIS_ )   dwYasFoundSlaves = _MAX_YAS_AXIS_; 
  for (i = 0; i < (int)dwYasFoundSlaves; i++)
      {
      iDebugInf[i]   = 0;
      dwDOutVelRd[i] = 0;   //  13.05.27;
      /***/
      if (wSlaveType[i] == TYPE_WOLONG)
          {
          WOLONG_TX_DATA tx;
          WolongDecodeTx(&YasInBuff[i][0], &tx);
          wErrCode[i] = tx.errorCode;
          wSlavSts[i] = tx.statusword;
          bModeRead[i] = tx.mode;
          iEncPos[i] = tx.position;
          wTouchSts[i] = tx.probeStatus;
          iTouchPos[i] = tx.probePosition;
          iFolwErrVal[i] = tx.followingError;
          dwDInput[i] = tx.digitalInputs;
          shTorqVal[i] = tx.torque;
          iHwCmdPos[i] = iLastCmdPos[i]; /* software value, not PDO feedback */
          iCmdPosRet[i] = 0; /* not present in this Tx mapping */
          }
      else if  ( wSlaveType[i] == 2 )
          {  // == SDP-E, 士林電機; 
          iEncPos[i]     = *(int *)&YasInBuff[i][2];
          shTorqVal[i]   = *(short *)&YasInBuff[i][6];
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][8];
          bModeRead[i]   = YasInBuff[i][10];            
          dwDInput[i]    = *(DWORD *)&YasInBuff[i][11];     //
          wTouchSts[i]   = *(USHORT *)&YasInBuff[i][15];            
          iTouchPos[i]   = *(int *)&YasInBuff[i][17];
          iCmdPosRet[i]  = *(int *)&YasInBuff[i][21];       //  12.11.28;
          iFolwErrVal[i] = 0;
          iHwCmdPos[i]   = iLastCmdPos[i];      //  08.05.24;            
          iDebugInf[i]   = 0;
          }
      else if  ( wSlaveType[i] == 3 )
          {  // == CNC-IO, 08.05.24, 22 -> 27; 
          iEncPos[i]     = *(int *)&YasInBuff[i][3];
          iHwCmdPos[i]   = *(int *)&YasInBuff[i][7];    //  Position demand;
          wErrCode[i]    = *(short *)&YasInBuff[i][11];
          dwDInput[i]    = *(DWORD *)&YasInBuff[i][13];
          wTouchSts[i]   = *(USHORT *)&YasInBuff[i][17];            
          iTouchPos[i]   = *(int *)&YasInBuff[i][19];
          iDebugInf[i]   = *(int *)&YasInBuff[i][23];   //  12.07.20;
          bModeRead[i]   = YasInBuff[i][2];           
          iFolwErrVal[i] = 0;
          shTorqVal[i]   = 0;            
          iCmdPosRet[i]  = 0;
          dwDOutVelRd[i] = *(DWORD *)&YasOutBuff[i][13];    //  13.05.27;
          }
      else if  ( wSlaveType[i] == 4 )
          {  // == Panasonic; 
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][0];
          wSlavSts[i]    = *(USHORT *)&YasInBuff[i][2];            
          bModeRead[i]   = YasInBuff[i][4];            
          iEncPos[i]     = *(int *)&YasInBuff[i][5];
          wTouchSts[i]   = *(USHORT *)&YasInBuff[i][9];            
          iTouchPos[i]   = *(int *)&YasInBuff[i][11];
          iFolwErrVal[i] = *(int *)&YasInBuff[i][15];
          dwDInput[i]    = *(DWORD *)&YasInBuff[i][19];
          shTorqVal[i]   = *(short *)&YasInBuff[i][23];  //  09.04.14;            
          iHwCmdPos[i]   = iLastCmdPos[i];      //  08.05.24;            
          iDebugInf[i]   = 0;
          iCmdPosRet[i]  = 0;
          }
      else if  ( wSlaveType[i] == 5 )
          {  // == Maxsine, 邁信; 
          bModeRead[i]   = YasInBuff[i][2];            
          iEncPos[i]     = *(int *)&YasInBuff[i][3];
          //  iVelValue[i]   = *(int *)&YasInBuff[i][7];            
          shTorqVal[i]   = *(short *)&YasInBuff[i][11];            
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][13];
          dwDInput[i]    = *(DWORD *)&YasInBuff[i][15];
          wTouchSts[i]   = *(USHORT *)&YasInBuff[i][19];            
          iTouchPos[i]   = *(int *)&YasInBuff[i][21];
          iFolwErrVal[i] = 0;
          iHwCmdPos[i]   = iLastCmdPos[i];      //  08.05.24;            
          iDebugInf[i]   = 0;
          iCmdPosRet[i]  = 0;
          }
      else if  ( wSlaveType[i] == 7 )
          {  // == [匯川], IS620N, 11.12.09; 
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][0];
          //  bModeRead[i]   = YasInBuff[i][2];            
          iEncPos[i]     = *(int *)&YasInBuff[i][4];
          shTorqVal[i]   = *(short *)&YasInBuff[i][8];            
          iFolwErrVal[i] = *(int *)&YasInBuff[i][10];
          wTouchSts[i]   = *(USHORT *)&YasInBuff[i][14];            
          iTouchPos[i]   = *(int *)&YasInBuff[i][16];
          //  iTouchPos2[i]  = *(int *)&YasInBuff[i][20];            
          dwDInput[i]    = *(DWORD *)&YasInBuff[i][24];
          iHwCmdPos[i]   = iLastCmdPos[i];      //  08.05.24;            
          iDebugInf[i]   = 0;
          iCmdPosRet[i]  = 0;
          }
      else if  ( wSlaveType[i] == 8 )
          {  // == CDHD, 高創, 10.03.08, 10.07.29, 23 bytes; 
          bModeRead[i]   = YasInBuff[i][2];            
          shTorqVal[i]   = *(short *)&YasInBuff[i][3];            
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][5];
          wTouchSts[i]   = *(USHORT *)&YasInBuff[i][7];            
          iTouchPos[i]   = *(int *)&YasInBuff[i][9];
          iEncPos[i]     = *(int *)&YasInBuff[i][13];
          //  shAInput[i]    = *(short *)&YasInBuff[i][17];
          dwDInput[i]    = *(DWORD *)&YasInBuff[i][19];
          iFolwErrVal[i] = 0;
          iHwCmdPos[i]   = iLastCmdPos[i];      //  08.05.24;            
          iDebugInf[i]   = 0;
          iCmdPosRet[i]  = 0;
          /***
          // == CDHD, 高創, 10.03.08, 33 bytes; 
          bModeRead[i]   = YasInBuff[i][2];            
          shTorqVal[i]   = *(short *)&YasInBuff[i][3];            
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][5];
          wTouchSts[i]   = *(USHORT *)&YasInBuff[i][7];            
          iTouchPos[i]   = *(int *)&YasInBuff[i][9];
          iEncPos[i]     = *(int *)&YasInBuff[i][13];
          //  shTorqCmdVal[i]   = *(short *)&YasInBuff[i][17];            
          //  shAInput[i]    = *(short *)&YasInBuff[i][19];
          dwDInput[i]    = *(DWORD *)&YasInBuff[i][21];
          //  iManHWExtCmd[i] = *(int *)&YasInBuff[i][25];
          iFolwErrVal[i] = *(int *)&YasInBuff[i][29];
          iHwCmdPos[i]   = iLastCmdPos[i];      //  08.05.24;            
          ***/
          }
      else if  ( wSlaveType[i] == 9 )
          {  // == GSHD_EC, 固高, 10.10.20, 33 Bytes -> 29 Bytes; 
          bModeRead[i]   = YasInBuff[i][2];            
          shTorqVal[i]   = *(short *)&YasInBuff[i][3];            
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][5];
          wTouchSts[i]   = *(USHORT *)&YasInBuff[i][7];            
          iTouchPos[i]   = *(int *)&YasInBuff[i][9];
          iEncPos[i]     = *(int *)&YasInBuff[i][13];
          dwDInput[i]    = *(DWORD *)&YasInBuff[i][17];
          //  iTouchPosN[i] = *(int *)&YasInBuff[i][21];
          iFolwErrVal[i] = *(int *)&YasInBuff[i][25];
          //  shTorqCmdVal[i]   = *(short *)&YasInBuff[i][17];            
          //  shCurrent[i]    = *(short *)&YasInBuff[i][19];
          //  dwDInput[i]    = *(DWORD *)&YasInBuff[i][21];
          //  iTouchPosN[i] = *(int *)&YasInBuff[i][25];
          //  iFolwErrVal[i] = *(int *)&YasInBuff[i][29];
          iHwCmdPos[i]   = iLastCmdPos[i];             
          iDebugInf[i]   = 0;
          iCmdPosRet[i]  = 0;
          }
      else if  ( wSlaveType[i] == 10 || wSlaveType[i] == 11 )
          {  // == Physis, 菲仕, 11.01.10; 
          wSlavSts[i]    = *(USHORT *)&YasInBuff[i][0];
          bModeRead[i]   = YasInBuff[i][2];            
          shTorqVal[i]   = *(short *)&YasInBuff[i][3];
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][5];
          wTouchSts[i]   = *(USHORT *)&YasInBuff[i][7];            
          iTouchPos[i]   = *(int *)&YasInBuff[i][9];
          iEncPos[i]     = *(int *)&YasInBuff[i][13];
          iFolwErrVal[i] = 0;
          //  byTmp          = YasInBuff[i][17];            //  11.02.25;
          usTmp          = *(USHORT *)&YasInBuff[i][17];    //  11.03.01;
          dwDInput[i]    = (DWORD)usTmp;                    //  
          iCmdPosRet[i]  = *(int *)&YasInBuff[i][19];       //  12.11.28;
          iHwCmdPos[i]   = iLastCmdPos[i];            
          iDebugInf[i]   = 0;
          }
      else if  ( wSlaveType[i] == 12 )
          {  // == Nidec, 11.06.21; 
          wSlavSts[i]    = *(USHORT *)&YasInBuff[i][0];
          bModeRead[i]   = YasInBuff[i][2];            
          shTorqVal[i]   = *(short *)&YasInBuff[i][3];
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][5];
          wTouchSts[i]   = *(USHORT *)&YasInBuff[i][7];            
          iTouchPos[i]   = *(int *)&YasInBuff[i][9];
          iEncPos[i]     = *(int *)&YasInBuff[i][13];
          iFolwErrVal[i] = 0;
          //  usTmp          = *(USHORT *)&YasInBuff[i][17]; 
          //  dwDInput[i]    = (DWORD)usTmp;               
          iHwCmdPos[i]   = iLastCmdPos[i];            
          iDebugInf[i]   = 0;
          iCmdPosRet[i]  = 0;
          }
      else if  ( wSlaveType[i] == 13 )
          {  // == Schneider, 11.11.22; 
          wSlavSts[i]    = *(USHORT *)&YasInBuff[i][0];
          bModeRead[i]   = YasInBuff[i][2];            
          iEncPos[i]     = *(int *)&YasInBuff[i][3];
          iFolwErrVal[i] = *(int *)&YasInBuff[i][7];
          shTorqVal[i]   = *(short *)&YasInBuff[i][11];
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][13];
          usTmp          = *(USHORT *)&YasInBuff[i][15]; 
          dwDInput[i]    = (DWORD)usTmp;               
          wTouchSts[i]   = 0;            
          iTouchPos[i]   = 0;
          //  usTmp          = *(USHORT *)&YasInBuff[i][17]; 
          //  dwDInput[i]    = (DWORD)usTmp;               
          iHwCmdPos[i]   = iLastCmdPos[i];            
          iDebugInf[i]   = 0;
          iCmdPosRet[i]  = 0;
          }
      else if  ( wSlaveType[i] == 14 )
          {  // == Schneider, 11.12.09; 
          wSlavSts[i]    = *(USHORT *)&YasInBuff[i][0];
          shTorqVal[i]   = *(short *)&YasInBuff[i][2];  //  Control Effort;
          iEncPos[i]     = 0;                      
          wErrCode[i]    = 0;                      
          iFolwErrVal[i] = 0;                      
          dwDInput[i]    = 0;                      
          wTouchSts[i]   = 0;            
          iTouchPos[i]   = 0;
          iHwCmdPos[i]   = iLastCmdPos[i];            
          iDebugInf[i]   = 0;
          iCmdPosRet[i]  = 0;
          }
      else if  ( wSlaveType[i] == 15 )
          {  // == Lenze, 12.12.14, 13.10.18; 
          wSlavSts[i]    = *(USHORT *)&YasInBuff[i][0];
          shTorqVal[i]   = *(short *)&YasInBuff[i][2];  //  Actual speed, 借用;
          iEncPos[i]     = 0;                      
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][4];                      
          iFolwErrVal[i] = 0;                      
          dwDInput[i]    = 0;                      
          wTouchSts[i]   = 0;            
          iTouchPos[i]   = 0;
          iHwCmdPos[i]   = iLastCmdPos[i];            
          iDebugInf[i]   = 0;
          iCmdPosRet[i]  = 0;
          }
      else if  ( wSlaveType[i] == 16 )
          {  // == SA3_EC301, 13.03.14; 
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][0];                      
          wSlavSts[i]    = *(USHORT *)&YasInBuff[i][2];
          iCmdPosRet[i]  = (int)(*(short *)&YasInBuff[i][4]);   //  Velocity demand(S16);
          //  shTorqVal[i]   = *(short *)&YasInBuff[i][6];          //  Control Effort(S16);
          //  bModeRead[i]   = YasInBuff[i][8];                     //  ModeRead(S8);           
          iEncPos[i]     = *(int *)&YasInBuff[i][6];            //  Position actual(S32);                      
          bModeRead[i]   = YasInBuff[i][10];                    //  ModeRead(S8);           
          //  iEncPos[i]     = 0;                            
          iFolwErrVal[i] = 0;                      
          dwDInput[i]    = 0;                      
          wTouchSts[i]   = 0;            
          iTouchPos[i]   = 0;
          iDebugInf[i]   = 0; 
          iHwCmdPos[i]   = iLastCmdPos[i];            
          }
      else if  ( wSlaveType[i] == 17 )
          {  //  13.09.03;  
          iEncPos[i]     = *(int *)&YasInBuff[i][2];
          shTorqVal[i]   = *(short *)&YasInBuff[i][6];
          iVelRdVal[i]   = *(int *)&YasInBuff[i][8];
          bModeRead[i]   = YasInBuff[i][12];            
          //  wTouchSts[i]   = *(USHORT *)&YasInBuff[i][14];            
          //  iTouchPos[i]   = *(int *)&YasInBuff[i][16];
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][13];        //  07.02.12;
          //  dwDInput[i]    = *(DWORD *)&YasInBuff[i][22];     //
          iHwCmdPos[i]   = iLastCmdPos[i];      //  08.05.24;            
          iDebugInf[i]   = 0;
          iCmdPosRet[i]  = 0;
          iFolwErrVal[i] = 0;   //  13.09.03;
          wTouchSts[i]   = 0;   //            
          iTouchPos[i]   = 0;
          }
      else if  ( wSlaveType[i] == 18 )
          {  //  14.12.23;  
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][0];
          wSlavSts[i]    = *(USHORT *)&YasInBuff[i][2];     //  !!!!;
          bModeRead[i]   = YasInBuff[i][4];            
          iEncPos[i]     = *(int *)&YasInBuff[i][5];
          iVelRdVal[i]   = *(int *)&YasInBuff[i][9];
          shTorqVal[i]   = *(short *)&YasInBuff[i][13];
          wTouchSts[i]   = *(USHORT *)&YasInBuff[i][15];            
          iTouchPos[i]   = *(int *)&YasInBuff[i][17];
          dwDInput[i]    = *(DWORD *)&YasInBuff[i][21];     //
          iHwCmdPos[i]   = iLastCmdPos[i];                 
          iDebugInf[i]   = 0;
          iCmdPosRet[i]  = 0;
          iFolwErrVal[i] = 0; 
          }
      else if  ( wSlaveType[i] == 19 )
          {  //  15.01.15;  
          iEncPos[i]     = *(int *)&YasInBuff[i][0];
          iVelRdVal[i]   = *(int *)&YasInBuff[i][4];
          iTouchPos[i]   = *(int *)&YasInBuff[i][8];
          dwDInput[i]    = *(DWORD *)&YasInBuff[i][12];     //
          shTorqVal[i]   = *(short *)&YasInBuff[i][16];
          wTouchSts[i]   = *(USHORT *)&YasInBuff[i][18];            
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][20];
          wSlavSts[i]    = *(USHORT *)&YasInBuff[i][22];     //  !!!!;
          bModeRead[i]   = YasInBuff[i][24];            
          iHwCmdPos[i]   = iLastCmdPos[i];                 
          iDebugInf[i]   = 0;
          iCmdPosRet[i]  = 0;
          iFolwErrVal[i] = 0; 
          }
      else if  ( wSlaveType[i] == 6 )
          {  // == CNC-RMB, 09.02.26; 
          iEncPos[i]     = *(int *)&YasInBuff[i][2];
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][6];
          dwDInput[i]    = *(DWORD *)&YasInBuff[i][8];
          wTouchSts[i]   = *(USHORT *)&YasInBuff[i][12];            
          //  iTouchPos2[i]  = *(int *)&YasInBuff[i][14];            
          iTouchPos[i]   = *(int *)&YasInBuff[i][18];
          iDebugInf[i]   = 0;
          iCmdPosRet[i]  = 0;
          }
	  else if  ( wSlaveType[i] == 20 )
          {  // == 眾辰 US800-C06, 15.06.04; 
		  wSlavSts[i]    = *(USHORT *)&YasInBuff[i][0];
          iEncPos[i]     = *(int *)&YasInBuff[i][2];
          shTorqVal[i]   = *(short *)&YasInBuff[i][6];
          iFolwErrVal[i] = *(int *)&YasInBuff[i][8];
          bModeRead[i]   = YasInBuff[i][12];            
          wTouchSts[i]   = *(USHORT *)&YasInBuff[i][14];            
          iTouchPos[i]   = *(int *)&YasInBuff[i][16];
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][20];    //  07.02.12;
          dwDInput[i]    = *(DWORD *)&YasInBuff[i][22];     //
          iHwCmdPos[i]   = iLastCmdPos[i];      //  08.05.24;            
          iDebugInf[i]   = 0;
          iCmdPosRet[i]  = 0;
          }
      else
          {
          iEncPos[i]     = *(int *)&YasInBuff[i][2];
          shTorqVal[i]   = *(short *)&YasInBuff[i][6];
          iFolwErrVal[i] = *(int *)&YasInBuff[i][8];
          bModeRead[i]   = YasInBuff[i][12];            
          wTouchSts[i]   = *(USHORT *)&YasInBuff[i][14];            
          iTouchPos[i]   = *(int *)&YasInBuff[i][16];
          wErrCode[i]    = *(USHORT *)&YasInBuff[i][20];    //  07.02.12;
          dwDInput[i]    = *(DWORD *)&YasInBuff[i][22];     //
          iHwCmdPos[i]   = iLastCmdPos[i];      //  08.05.24;            
          iDebugInf[i]   = 0;
          iCmdPosRet[i]  = 0;
          }
      /***/
      
      //  From driver, MAX axis == 20 的設計;
      p_McmdBlkIn[i]    = (DWORD)iEncPos[i]; 
      p_McmdBlkIn[20+i]  = (DWORD)shTorqVal[i]; 
      p_McmdBlkIn[40+i] = (DWORD)iFolwErrVal[i]; 
      p_McmdBlkIn[60+i] = (DWORD)wSlavSts[i]; 
      p_McmdBlkIn[80+i] = (DWORD)bModeRead[i];
      p_McmdBlkIn[100+i] = (DWORD)wTouchSts[i];
      p_McmdBlkIn[120+i] = (DWORD)iTouchPos[i];
      p_McmdBlkIn[140+i] = (DWORD)wErrCode[i];   //  07.02.12; 
      p_McmdBlkIn[160+i] = dwDInput[i];          //   
      p_McmdBlkIn[180+i] = (DWORD)iHwCmdPos[i];  //  08.05.24; 
      p_McmdBlkIn[400+i] = (DWORD)iDebugInf[i];  //  12.07.20; 
      p_McmdBlkIn[420+i] = (DWORD)iCmdPosRet[i]; //  12.11.28; 
      p_McmdBlkIn[440+i] = dwDOutVelRd[i];       //  13.05.27; 
      //  10.10.13, 10.03.16;
      if  ( wSlaveType[i] == 8 || wSlaveType[i] == 9 )
          {
          if  ( wTouchStsOld[i] != wTouchSts[i] )
              log_ecat_comm_info_ex( i, 99 );
          //  if  ( dwDInputOld[i] != dwDInput[i] )
          //      log_ecat_comm_info_ex( i, 88 );
          }
      wTouchStsOld[i] = wTouchSts[i];
      dwDInputOld[i] = dwDInput[i];
      }  //  END_of [for (i = 0; i < (int)dwYasFoundSlaves; i++)];
  //  08.05.24, 07.12.26, CNC-IO;
  if  ( dwHRmbNum > 0 && pbyHalRmbTxBase != NULL )
      {
      //  memcpy( &HRmbInBuff[0][0], (BYTE *)pwHalRmbTxBase, 110 );     //  110 = 5 * 22;
      //  memcpy( &HRmbInBuff[0][0], (BYTE *)pwHalRmbTxBase, 81 );     //  81 = 3 * 27;
      for (i = 0; i < 3; i++)
          {
          memcpy( &HRmbInBuff[i][0], &pbyHalRmbTxBase[i*27], 27 );
          wHRmbStatus[i]   = *(WORD *)&HRmbInBuff[i][0];
          iHRmbEncPos[i]   = *(int *)&HRmbInBuff[i][3];
          iHRmbHwCmdPos[i] = *(int *)&HRmbInBuff[i][7];
          wHRmbErrcode[i]  = *(WORD *)&HRmbInBuff[i][11];
          dwHRmbDInp[i]    = *(DWORD *)&HRmbInBuff[i][13];            
          wHRmbLatchSts[i] = *(WORD *)&HRmbInBuff[i][17];            
          iHRmbLatchP1[i]  = *(int *)&HRmbInBuff[i][19];
          iHRmbLatchP2[i]  = *(int *)&HRmbInBuff[i][23];
          p_McmdHRmbIn[i]    = (DWORD)wHRmbStatus[i]; 
          p_McmdHRmbIn[5+i]  = (DWORD)iHRmbEncPos[i]; 
          p_McmdHRmbIn[10+i] = (DWORD)wHRmbErrcode[i]; 
          p_McmdHRmbIn[15+i] = dwHRmbDInp[i]; 
          p_McmdHRmbIn[20+i] = (DWORD)wHRmbLatchSts[i];
          p_McmdHRmbIn[25+i] = (DWORD)iHRmbLatchP1[i];
          p_McmdHRmbIn[30+i] = (DWORD)iHRmbLatchP2[i];
          p_McmdHRmbIn[35+i] = (DWORD)wHRmbPhase[i];
          p_McmdHRmbIn[40+i] = (DWORD)iHRmbHwCmdPos[i];     //  08.05.24;
          }
      //  13.07.09;
      dwCncIoIsrCnt   = (DWORD)iHRmbLatchP1[2];
      dwCncIoSync0Cnt = (DWORD)iHRmbLatchP2[2];
      dwCncIoDiffCnt  = (DWORD)iHRmbHwCmdPos[2];
      }
}

/*:: block_info_output ---------------*/
void block_info_output()
{
  int       i, *piVal;
  short     *pshVal, shTmp;
  WORD      *pwVal;
  DWORD     *pdwVal, dwTmp;

  for (i = 0; i < (int)dwYasFoundSlaves; i++)
      {
      /***/
      //  07.12.26;
      //  if  ( dwOpRtssCount > 10000 )
      if (wSlaveType[i] == TYPE_WOLONG)
          {
          WolongEncodeRx(&YasOutBuff[i][0], wCtrlCmd[i], bModeWrite[i],
                         wTorqMax[i], iCmdPos[i], wTouchFunc[i], iVelCmd[i]);
          /* No target torque, VFF or digital output entry in the 15-byte Rx. */
          }
      else if  ( wSlaveType[i] == 3 )
          {  //  CNC-IO;
          piVal = (int *)&YasOutBuff[i][3];
          *piVal = iVelCmd[i];
          piVal = (int *)&YasOutBuff[i][7];
          *piVal = iCmdPos[i];
          pwVal = (WORD *)&YasOutBuff[i][11];
          *pwVal = wTouchFunc[i]; 
          //  pdwVal = (DWORD *)&YasOutBuff[i][13];
          //  *pdwVal = dwDOutVel[i];
          iHRmbVelCmd[0] = iVelCmd[i];
          iHRmbPulCmd[0] = iCmdPos[i];
          wHRmbLatFunc[0] = wTouchFunc[i]; 
          }
      else if  ( wSlaveType[i] == 4 )
          {  //  Panasonic;
          //  09.04.14;
          pwVal = (WORD *)&YasOutBuff[i][3];
          *pwVal = wTorqMax[i]; 
          piVal = (int *)&YasOutBuff[i][5];
          *piVal = iCmdPos[i];
          pwVal = (WORD *)&YasOutBuff[i][9];
          *pwVal = wTouchFunc[i]; 
          piVal = (int *)&YasOutBuff[i][11];
          *piVal = iVelCmd[i];
          YasOutBuff[i][2] = bModeWrite[i];
          /***
          piVal = (int *)&YasOutBuff[i][3];
          *piVal = iCmdPos[i];
          pwVal = (WORD *)&YasOutBuff[i][7];
          *pwVal = wTouchFunc[i]; 
          YasOutBuff[i][2] = bModeWrite[i];
          ***/
          }
      else if  ( wSlaveType[i] == 5 )
          {  //  == Maxsine, 邁信;
          piVal = (int *)&YasOutBuff[i][3];
          *piVal = iCmdPos[i];
          piVal = (int *)&YasOutBuff[i][7];
          *piVal = iVelCmd[i];
          pshVal = (short *)&YasOutBuff[i][11];
          *pshVal = shTorqCmd[i]; 
          pwVal = (WORD *)&YasOutBuff[i][13];
          *pwVal = wTouchFunc[i]; 
          pwVal = (WORD *)&YasOutBuff[i][15];
          *pwVal = wTorqMax[i]; 
          YasOutBuff[i][2] = bModeWrite[i];
          }
      else if  ( wSlaveType[i] == 7 )
          {  //  == IS620N, [匯川], 11.12.09;
          piVal = (int *)&YasOutBuff[i][2];
          *piVal = iCmdPos[i];
          pwVal = (WORD *)&YasOutBuff[i][6];
          *pwVal = wTouchFunc[i]; 
          //  pwVal = (DWORD *)&YasOutBuff[i][10];
          //  *pwVal = dwDOutput[i]; 
          //  YasOutBuff[i][2] = bModeWrite[i];
          }
      else if  ( wSlaveType[i] == 8 )
          {  //  == CDHD, 高創, 10.03.17, 10.07.29, 17 bytes;
          piVal = (int *)&YasOutBuff[i][3];
          *piVal = iCmdPos[i];
          piVal = (int *)&YasOutBuff[i][7];
          *piVal = iVelCmd[i];
          //  pdwVal = (DWORD *)&YasOutBuff[i][11];
          //  *pdwVal = dwDOutVel[i];
          pwVal = (WORD *)&YasOutBuff[i][15];
          *pwVal = wTouchFunc[i]; 
          YasOutBuff[i][2] = bModeWrite[i];
          /***  10.07.29;
          piVal = (int *)&YasOutBuff[i][3];
          *piVal = iCmdPos[i];
          //  pdwVal = (DWORD *)&YasOutBuff[i][7];
          //  *pdwVal = dwProfVel[i];
          piVal = (int *)&YasOutBuff[i][11];
          *piVal = iVelCmd[i];
          pshVal = (short *)&YasOutBuff[i][15];
          *pshVal = shTorqCmd[i]; 
          //  pdwVal = (DWORD *)&YasOutBuff[i][17];
          //  *pdwVal = dwDOutVel[i];
          pwVal = (WORD *)&YasOutBuff[i][21];
          *pwVal = wTouchFunc[i]; 
          YasOutBuff[i][2] = bModeWrite[i];
          ***/
          /***  10.03.08;
          pwVal = (WORD *)&YasOutBuff[i][3];
          *pwVal = wTouchFunc[i]; 
          piVal = (int *)&YasOutBuff[i][5];
          *piVal = iCmdPos[i];
          //  pdwVal = (DWORD *)&YasOutBuff[i][9];
          //  *pdwVal = dwProfVel[i];
          piVal = (int *)&YasOutBuff[i][13];
          *piVal = iVelCmd[i];
          pshVal = (short *)&YasOutBuff[i][17];
          *pshVal = shTorqCmd[i]; 
          //  pdwVal = (DWORD *)&YasOutBuff[i][19];
          //  *pdwVal = dwDOutVel[i];
          //  pshVal = (short *)&YasOutBuff[i][23];
          //  *pshVal = shTorqOff[i]; 
          ***/
          }
      else if  ( wSlaveType[i] == 9 )
          {  //  == GSHD_EC, 固高, 10.10.20, 25 -> 21 Bytes;
          piVal = (int *)&YasOutBuff[i][3];
          *piVal = iCmdPos[i];
          piVal = (int *)&YasOutBuff[i][7];
          *piVal = iVelCmd[i];
          pwVal = (WORD *)&YasOutBuff[i][11];
          *pwVal = wTouchFunc[i]; 
          pshVal = (short *)&YasOutBuff[i][13];
          *pshVal = shTorqCmd[i]; 
          //  pdwVal = (DWORD *)&YasOutBuff[i][15];
          //  *pdwVal = dwDOutVel[i];
          //  11.12.09;
          //  pshVal = (short *)&YasOutBuff[i][19];
          //  *pshVal = shTorqOffset[i];           
          pshVal = (short *)&YasOutBuff[i][19];
          
		  // 2026/09/22, Ulysses Debug
//		  if (shVffCmd[i] != *pshVal)
//			  printf(">>>>> Ax: %d, VFF [%d] --> [%d]\n", i, *pshVal, shVffCmd[i]);
		  // 2026/09/22, Ulysses Debug

		  *pshVal = shVffCmd[i];

          YasOutBuff[i][2] = bModeWrite[i];
          }
      else if  ( wSlaveType[i] == 10 || wSlaveType[i] == 11 )
          {  //  Physis, 菲仕, 11.01.10;
          piVal = (int *)&YasOutBuff[i][3];
          *piVal = iCmdPos[i];
          piVal = (int *)&YasOutBuff[i][7];
          //  *piVal = iVelCmd[i];
          *piVal = 0;       //  11.03.01;          
          pwVal = (WORD *)&YasOutBuff[i][11];
          *pwVal = wTouchFunc[i]; 
          YasOutBuff[i][2] = bModeWrite[i];
          }
      else if  ( wSlaveType[i] == 12 )
          {  //  Nidec, 11.06.21;
          piVal = (int *)&YasOutBuff[i][3];
          *piVal = iCmdPos[i];
          piVal = (int *)&YasOutBuff[i][7];
          //  *piVal = iVelCmd[i];
          *piVal = 0;                 
          pwVal = (WORD *)&YasOutBuff[i][11];
          *pwVal = wTouchFunc[i]; 
          YasOutBuff[i][2] = bModeWrite[i];
          }
      else if  ( wSlaveType[i] == 13 )
          {  //  Schneider, 11.11.22;
          piVal = (int *)&YasOutBuff[i][3];
          *piVal = iCmdPos[i];
          piVal = (int *)&YasOutBuff[i][7];
          *piVal = 0;   //  Target velocity;                 
          pwVal = (WORD *)&YasOutBuff[i][11];
          *pwVal = 0;   //  Target torque; 
          pwVal = (WORD *)&YasOutBuff[i][13];
          *pwVal = 0;   //  IO_DQ_set; 
          YasOutBuff[i][2] = bModeWrite[i];
          }
      else if  ( wSlaveType[i] == 14 )
          {  //  Schneider, 11.12.09;
          pshVal = (short *)&YasOutBuff[i][2];
          shTmp = (short)iVelCmd[i];
          *pshVal = shTmp;
          bModeWrite[i] = 0x09;     //  13.01.05;
          }
      else if  ( wSlaveType[i] == 15 )
          {  //  Lenze, 12.12.14, 13.10.18;
          pshVal = (short *)&YasOutBuff[i][2];
          shTmp = (short)iVelCmd[i];
          *pshVal = shTmp;
          bModeWrite[i] = 0x09;     //  13.01.05;
          }
      else if  ( wSlaveType[i] == 16 )
          {  //  == SA3_EC301, 13.03.14;
          pshVal = (short *)&YasOutBuff[i][2];  //  Target velocity(S16);
          *pshVal = (short)iVelCmd[i];
          piVal = (int *)&YasOutBuff[i][4];     //  Target position(S32);
          *piVal = iCmdPos[i];
          //  pdwVal = (DWORD *)&YasOutBuff[i][4];  //  Ramp function time(U32);
          //  *pdwVal = dwParam0[i];
          //  pdwVal = (DWORD *)&YasOutBuff[i][8];  //  Slow down time(U32);
          //  *pdwVal = dwParam1[i];
          YasOutBuff[i][8] = bModeWrite[i];
          /***  13.03.07;
          piVal = (int *)&YasOutBuff[i][13];    //  Target position(S32);
          *piVal = iCmdPos[i];
          pdwVal = (DWORD *)&YasOutBuff[i][17]; //  Profile velocity(U32);
          *pdwVal = dwParam2[i];
          pdwVal = (DWORD *)&YasOutBuff[i][21]; //  Profile acceleration(U32);
          *pdwVal = dwParam3[i];
          pdwVal = (DWORD *)&YasOutBuff[i][25]; //  Profile deceleration(U32);
          *pdwVal = dwParam4[i];
          pdwVal = (DWORD *)&YasOutBuff[i][29]; //  Quick stop deceleration(U32);
          *pdwVal = dwParam5[i];
          ***/
          }
      else if  ( wSlaveType[i] == 17 )
          {  //  13.09.03, 13.08.15, Beijing CTB;
          piVal = (int *)&YasOutBuff[i][2];
          *piVal = iCmdPos[i];
          piVal = (int *)&YasOutBuff[i][6];
          *piVal = iVelCmd[i];
          //  pshVal = (short *)&YasOutBuff[i][10];
          //  *pshVal = shTorqCmd[i]; 
          YasOutBuff[i][10] = bModeWrite[i];
          }
      else if  ( wSlaveType[i] == 18 )
          {  //  14.12.23, HCFA;
          YasOutBuff[i][2] = bModeWrite[i];
          pshVal = (short *)&YasOutBuff[i][3];
          *pshVal = shTorqCmd[i]; 
          pwVal = (WORD *)&YasOutBuff[i][5];
          *pwVal = wTorqMax[i]; 
          piVal = (int *)&YasOutBuff[i][7];
          *piVal = iCmdPos[i];
          piVal = (int *)&YasOutBuff[i][11];
          //  *piVal = iVelCmd[i];      //  Max speed;
          *piVal = 10000;      //  Max speed, 不該更動;
          pwVal = (WORD *)&YasOutBuff[i][15];
          *pwVal = wTouchFunc[i];
          piVal = (int *)&YasOutBuff[i][17];
          *piVal = iVelCmd[i];
          }
      else if  ( wSlaveType[i] == 19 )
          {  //  15.01.15;
          piVal = (int *)&YasOutBuff[i][0];
          *piVal = iCmdPos[i];
          piVal = (int *)&YasOutBuff[i][4];
          *piVal = iVelCmd[i];
          piVal = (int *)&YasOutBuff[i][8];
          *piVal = iVelCmd[i];          //  Profile velocity;
          pshVal = (short *)&YasOutBuff[i][12];
          *pshVal = shTorqCmd[i]; 
          pwVal = (WORD *)&YasOutBuff[i][14];   //  !!!;
          *pwVal = wCtrlCmd[i];                 //  !!!;
          YasOutBuff[i][16] = bModeWrite[i];
          YasOutBuff[i][17] = 0;
          pwVal = (WORD *)&YasOutBuff[i][18];
          *pwVal = wTouchFunc[i];
          }
      else if  ( wSlaveType[i] == 6 )
          {  //  CNC-RMB, 09.02.26;
          piVal = (int *)&YasOutBuff[i][2];
          *piVal = iVelCmd[i];
          pwVal = (WORD *)&YasOutBuff[i][6];
          *pwVal = wTouchFunc[i]; 
          }
	  else if  ( wSlaveType[i] == 20 )
          {  //  眾辰, US800-C06, 15.06.04;
          piVal = (int *)&YasOutBuff[i][2];
          *piVal = iCmdPos[i];
          piVal = (int *)&YasOutBuff[i][6];
#ifdef TYPE_20_VCMD_FILTER
		   *piVal = Type20VCmdFilterProcess(i, iVelCmd[i]);					// 2026/08/16, Ulysses, VCmd Filter: 0.01
#else
		   *piVal = iVelCmd[i];
#endif
          pshVal = (short *)&YasOutBuff[i][10];
          *pshVal = shTorqCmd[i]; 
          pwVal = (WORD *)&YasOutBuff[i][12];
          *pwVal = wTorqMax[i]; 
          pwVal = (WORD *)&YasOutBuff[i][16];
          *pwVal = wTouchFunc[i];
          YasOutBuff[i][14] = bModeWrite[i];
		  piVal = (int *)&YasOutBuff[i][18];								// 2026/07/29, Ulysses
		  if (g_bVFFStartUp == false)
			*piVal = Type20UserVFFProcess(i, iCmdPos[i], shVffCmd[i]);		// 2026/07/29, Ulysses
		  else
		    *piVal = 0;
		  g_nType20PosCmd[i] = iCmdPos[i];									// 2026/07/31, Ulysses
          }
      else
          {
          piVal = (int *)&YasOutBuff[i][2];
          *piVal = iCmdPos[i];
          piVal = (int *)&YasOutBuff[i][6];
          *piVal = iVelCmd[i];
          pshVal = (short *)&YasOutBuff[i][10];
          *pshVal = shTorqCmd[i]; 
          pwVal = (WORD *)&YasOutBuff[i][12];
          *pwVal = wTorqMax[i]; 
          pwVal = (WORD *)&YasOutBuff[i][16];
          *pwVal = wTouchFunc[i];
          YasOutBuff[i][14] = bModeWrite[i];
          }
      /***/
      p_McmdBlkOut[i]    = (DWORD)iCmdPos[i]; 
      p_McmdBlkOut[20+i]  = (DWORD)iVelCmd[i]; 
      p_McmdBlkOut[40+i] = (DWORD)shTorqCmd[i]; 
      p_McmdBlkOut[60+i] = (DWORD)wTorqMax[i]; 
      p_McmdBlkOut[80+i] = (DWORD)wCtrlCmd[i]; 
      p_McmdBlkOut[100+i] = (DWORD)bModeWrite[i]; 
      p_McmdBlkOut[120+i] = (DWORD)wTouchFunc[i]; 
      p_McmdBlkOut[140+i] = (DWORD)wCurrPhase[i]; 
      //  13.04.02, 10.03.16;
      if  ( wSlaveType[i] == 8 && wTouchFuncOld[i] != wTouchFunc[i] )
          log_ecat_comm_info_ex( i, 18 );
      //  13.04.02, 10.07.15;
      if  ( wSlaveType[i] == 9 && wTouchFuncOld[i] != wTouchFunc[i] )
          log_ecat_comm_info_ex( i, 19 );
      wTouchFuncOld[i] = wTouchFunc[i];
      }  //  END_of [for (i = 0; i < (int)dwYasFoundSlaves; i++)];

	  g_bVFFStartUp = false;								// 2026/07/31, Ulysses

  //  07.12.26, 07.02.09;
  //  if  ( dwHRmbNum > 0 && pwHalRmbRxBase != NULL && dwOpRtssCount > 10000 )
  if  ( dwHRmbNum > 0 && pbyHalRmbRxBase != NULL )
      {
      for (i = 1; i < 2; i++)
          {
          memcpy( &HRmbOutBuff[i][0], &pbyHalRmbRxBase[i*17], 17);
          wUsrHRCtrl[i]    = (WORD)p_McmdHRmbOut[i]; 
          iUsrHRVelCmd[i]  = (int)p_McmdHRmbOut[5+i]; 
          wUsrHRLatFunc[i] = (WORD)p_McmdHRmbOut[10+i];           
          dwUsrHRDOutp[i]  = (DWORD)p_McmdHRmbOut[15+i];           
          iUsrHRPulCmd[i]  = (int)p_McmdHRmbOut[20+i];  //  08.06.25;           
          if  ( wUsrHRCtrl[i] != wOldUsrHRCtrl[i] && 
                wUsrHRCtrl[i] != 0 && 
                dwHRmbAuToCtrl[i] != 1 )
              {
              wHRmbCtrl[i] = wUsrHRCtrl[i];
              wOldUsrHRCtrl[i] = wUsrHRCtrl[i];
              pwVal = (WORD *)&HRmbOutBuff[i][0];
              *pwVal = wHRmbCtrl[i]; 
              }
          //  08.06.24;
          if  ( dwHRmbAuToCtrl[i] == 1 )
              {
              pwVal = (WORD *)&HRmbOutBuff[i][0];
              *pwVal = wHRmbCtrl[i]; 
              }
          if  ( iUsrHRVelCmd[i] != iOldUsrHRVelCmd[i] )
              {
              iHRmbVelCmd[i] = iUsrHRVelCmd[i];
              iOldUsrHRVelCmd[i] = iUsrHRVelCmd[i];
              piVal = (int *)&HRmbOutBuff[i][3];
              *piVal = iHRmbVelCmd[i];
              }
          //  08.05.24;
          if  ( iUsrHRPulCmd[i] != iOldUsrHRPulCmd[i] )
              {
              iHRmbPulCmd[i] = iUsrHRPulCmd[i];
              iOldUsrHRPulCmd[i] = iUsrHRPulCmd[i];
              piVal = (int *)&HRmbOutBuff[i][7];
              *piVal = iHRmbPulCmd[i];
              }
          if  ( wUsrHRLatFunc[i] != wOldUsrHRLatFunc[i] )
              {
              wHRmbLatFunc[i] = wUsrHRLatFunc[i];
              wOldUsrHRLatFunc[i] = wUsrHRLatFunc[i];
              pwVal = (WORD *)&HRmbOutBuff[i][11];
              *pwVal = wHRmbLatFunc[i]; 
              }
          /***   13.07.09;
          if  ( (i == 2) && (dwUsrHRDOutp[i] != dwOldUsrHRDOutp[i]) )
              {
              dwHRmbDOutp[i] = dwUsrHRDOutp[i];
              dwOldUsrHRDOutp[i] = dwUsrHRDOutp[i];
              pdwVal = (DWORD *)&HRmbOutBuff[i][13];
              dwTmp = *pdwVal;
              *pdwVal = dwTmp | (dwHRmbDOutp[2] & 0x030f); 
              }
          ***/
          // ---
          //  p_McmdHRmbIn[40+i] = (DWORD)wHRmbCtrl[i];  有重覆!!
          p_McmdHRmbIn[45+i] = (DWORD)iHRmbVelCmd[i];
          p_McmdHRmbIn[50+i] = (DWORD)wHRmbLatFunc[i];
          p_McmdHRmbIn[55+i] = dwHRmbDOutp[i];
          p_McmdHRmbIn[60+i] = (DWORD)iHRmbPulCmd[i];   //  08.06.21;
          p_McmdHRmbIn[65+i] = (DWORD)wHRmbCtrl[i];     //  
          }      
      //  13.07.09, 09.02.26, 取消;
      /*** 不處理 DOutp[]!!;
      dwHRmbDOutp[2] = 0x03;    //  for 0x70FE object;          
      memcpy( &HRmbOutBuff[2][0], &pbyHalRmbRxBase[34], 17);
      pdwVal = (DWORD *)&HRmbOutBuff[2][13];
      dwTmp = *pdwVal;
      *pdwVal = dwTmp | (dwHRmbDOutp[2] & 0x030f); 
      memcpy( &pbyHalRmbRxBase[34], &HRmbOutBuff[2][0], 17); 
      ***/
      //  13.07.09, 讀取 dwCncIoDOut;
      memcpy( &HRmbOutBuff[0][0], &pbyHalRmbRxBase[0], 17);
      memcpy( &HRmbOutBuff[2][0], &pbyHalRmbRxBase[34], 17);
      pdwVal = (DWORD *)&HRmbOutBuff[2][13];
      dwCncIoDOut_2 = *pdwVal;
      pdwVal = (DWORD *)&HRmbOutBuff[1][13];
      dwCncIoDOut_1 = *pdwVal;
      pdwVal = (DWORD *)&HRmbOutBuff[0][13];
      dwCncIoDOut_0 = *pdwVal;
      //  if  ( wHRmbInitFlag == 1 )
      //      {
      //  08.06.20, 避免衝突; 
      //  memcpy( &pbyHalRmbRxBase[0], &HRmbOutBuff[0][0], 17);  //  51 = 3 * 17;
      memcpy( &pbyHalRmbRxBase[17], &HRmbOutBuff[1][0], 13);    //  17 -> 13, 不處理 DOutp;  
      //      }
      }
}

/*:: oprtss_write_dda_cmd ------------*/
int  oprtss_write_dda_cmd( int ax, int iCmd)
{
  int  tidx;
  
  if  ( dwOpRtssFlag != 1 )  return( -1 );
  if  ( ax >= MAX_NUM_SLAVES || ax < 0 )  return( -2 );

  tidx = oprtss_cmd_write_idx[ax];

  if  ( (tidx+1) == oprtss_cmd_read_idx[ax] ||
        (tidx+1) == (oprtss_cmd_read_idx[ax]+_MAX_CMD_NUM_) )
      { //  Full!! 
      return( -2 );
      } 

  oprtss_cmd_buff[ax][tidx] = iCmd;

  tidx++;
  if  ( tidx >= _MAX_CMD_NUM_ )  tidx = 0;
  oprtss_cmd_write_idx[ax] = tidx;
         
  return( 0 );
}

/*:: safe_reset_ax -------------------*/
void safe_reset_ax( int ax )
{
  //  10.08.16, 10.08.13;
  if  ( ax >= MAX_NUM_SLAVES || ax < 0 )  return;

  iCmdPos[ax] = iEncPos[ax];
  iVelCmd[ax] = 0;
  shTorqCmd[ax] = 2000;
  wTorqMax[ax]  = 7000;  //  x 0.1, Limit: 8000;
}

/*:: switchon_loop -------------------*/
//  13.10.18, 處理 mode 切換; (Standby 狀態)
int  switchon_loop( int ax )
{
  DWORD dwTmp;
  
  if  ( ax >= MAX_NUM_SLAVES || ax < 0 )  return( -2 );
  
  //  15.01.08;
  //  if  ( wSlaveType[ax] == 17 )
  if  ( wSlaveType[ax] == 17 || wSlaveType[ax] == 18 ||
        wSlaveType[ax] == 19 ||
		wSlaveType[ax] == 20)	// 眾辰, US800-C06, 15.06.04;
      {
      dwTmp = p_McmdBlkIn[360+ax];
      if  ( (dwTmp & dwUbitVal[2]) != 0x0 )
          {
          wUsrCtrlCmd[ax] = (USHORT)p_McmdBlkIn[280+ax];
          if  ( wUsrCtrlCmd[ax] != wOldUsrCtrlCmd[ax] )
              wCtrlCmd[ax] = wUsrCtrlCmd[ax];
          wOldUsrCtrlCmd[ax] = wUsrCtrlCmd[ax];
          }
      if  ( (dwTmp & dwUbitVal[3]) != 0x0 )
          {
          bUsrModeWrite[ax] = (BYTE)p_McmdBlkIn[300+ax];
          if  ( bUsrModeWrite[ax] != boldUsrModeWr[ax] )
              bModeWrite[ax] = bUsrModeWrite[ax];
          boldUsrModeWr[ax] = bUsrModeWrite[ax];
          }
      //  13.12.24;
      if  ( (dwTmp & dwUbitVal[5]) != 0x0 )
          {
          iUsrVffCmd[ax] = (int)p_McmdBlkIn[340+ax];
          }
      p_McmdBlkIn[360+ax] = 0x0;
      }

  return( 0 );
}

/*:: opreation_loop ------------------*/
int  opreation_loop( int ax )
{
  int  tidx, iTmpPul, iDiff;
  
  //  if  ( p_McmdSysInfo[10] == 907 )
  //      {  //  [MotionAPI] actived!!;
  //  mot_r3_get_user_info();
  //      }
  //  10.08.16, 10.08.13;  
  if  ( ax >= MAX_NUM_SLAVES || ax < 0 )  return( -2 );
  
  //  if  ( dwUsrReqFlag == 1 )
  if  ( (dwAxUsrReqFlag & dwUbitVal[ax]) != 0 )
      {
      iVelCmd[ax]    = iUsrVelCmd[ax];
      shVffCmd[ax]   = (short)iUsrVffCmd[ax];       //  11.12.09;
      //  08.04.16;
      if  ( wSlaveType[ax] == 4 && 
            (dwUsrReqFunFlag[ax] & dwUbitVal[0]) != 0 )
          {  // == Panasonic; 
          iNewPosCmd[ax] = iUsrPosCmd[ax];
          }
      else  if  ( wSlaveType[ax] == 3 )
          {  //  08.05.24;
          iDiff = iUsrPosCmd[ax];
          if  ( iDiff < 1000000 && iDiff > -1000000 )
              iNewPosCmd[ax] = iUsrPosCmd[ax];
          else
              iNewPosCmd[ax] = 0;
          }
      else  if  ( wSlaveType[ax] == 17 && wCtrlCmd[ax] == 0x180f )
          {  //  13.12.24;
          iNewPosCmd[ax] = iUsrVffCmd[ax];
          }
      else  if  ( wSlaveType[ax] == 18 && wCtrlCmd[ax] == 0x80f )
          {  //  15.01.08;
          iNewPosCmd[ax] = iUsrVffCmd[ax];
          }
      else  if  ( wSlaveType[ax] == 19 && wCtrlCmd[ax] == 0x40f )
          {  //  15.01.15;
          iNewPosCmd[ax] = iUsrVffCmd[ax];
          }
      else
          {
          iDiff = iUsrPosCmd[ax] - iLastCmdPos[ax];
          if  ( iDiff < 1000000 && iDiff > -1000000 )
              iNewPosCmd[ax] = iUsrPosCmd[ax];
          else
              iNewPosCmd[ax] = iLastCmdPos[ax];
          }
      //  shTorqCmd[ax] = shUsrTorqCmd[ax];
      wTorqMax[ax]   = wUsrTorqMax[ax];     //  09.05.07;
      //  13.03.07;
      if  ( wSlaveType[ax] == 16 )
          {
          if  ( bUsrModeWrite[ax] == 1 || bUsrModeWrite[ax] == 2 ||
                bUsrModeWrite[ax] == 4 )
              bModeWrite[ax] = bUsrModeWrite[ax];
          if  ( bModeWrite[ax] == 2 )
              {
              if  ( iVelCmd[ax] >= 32768 || iVelCmd[ax] < -32768 )
                  iVelCmd[ax] = 0;
              if  ( iVelCmd[ax] != 0 )
                  wCtrlCmd[ax] = 0x7f;
              else
                  wCtrlCmd[ax] = 0x0f;
              }
          }
      //  13.10.18;
      //  else
      //      bModeWrite[ax] = bUsrModeWrite[ax];
      if  ( wUsrCtrlCmd[ax] != wOldUsrCtrlCmd[ax] )
          {
          //  13.03.06, 11.12.09;
          if  ( wSlaveType[ax] != 14 && wSlaveType[ax] != 16 && 
                wSlaveType[ax] != 15 )
              wCtrlCmd[ax] = wUsrCtrlCmd[ax];
          wOldUsrCtrlCmd[ax] = wUsrCtrlCmd[ax];
          }          
      wTouchFunc[ax] = wUsrTouchFunc[ax];   //  08.02.27;
      dwAxUsrReqFlag &= ~dwUbitVal[ax];
      }
  else
      {
      iNewPosCmd[ax] = iLastCmdPos[ax];
      }
  
  //  07.07.31;
  if  ( p_McmdSysInfo[10] == 900 )
      {
      //  輸出補間命令, Read command queue;
      iTmpPul = 0;
      if  ( oprtss_cmd_read_idx[ax] != oprtss_cmd_write_idx[ax] )
          {
          tidx = oprtss_cmd_read_idx[ax];
      
          iTmpPul = oprtss_cmd_buff[ax][tidx];
          
          tidx++;
          if  ( tidx >= _MAX_CMD_NUM_ ) tidx = 0;              
          oprtss_cmd_read_idx[ax] = tidx;      
          }
      iCurrGlbPos[ax] += iTmpPul;
      }
  else if  ( p_McmdSysInfo[10] == 907 )
      {
      //  iCurrGlbPos[ax] = iUsrPosCmd[ax];
      iCurrGlbPos[ax] = iNewPosCmd[ax];
      }
  else
      iCurrGlbPos[ax] = iLastCmdPos[ax];
        
  if  ( wSlaveType[ax] == 3 )
      {
      iCmdPos[ax]   = iCurrGlbPos[ax];
      }
  else if  ( wSlaveType[ax] == 17 && wCtrlCmd[ax] == 0x180f )
      {  //  13.12.24;
      iCmdPos[ax]   = iUsrVffCmd[ax];
      }
  else if  ( wSlaveType[ax] == 18 && wCtrlCmd[ax] == 0x80f )
      {  //  15.01.08;
      //  iCmdPos[ax]   = iUsrVffCmd[ax];
      }
  else if  ( wSlaveType[ax] == 19 && wCtrlCmd[ax] == 0x40f )
      {  //  15.01.15;
      //  iCmdPos[ax]   = iUsrVffCmd[ax];
      }
  else if  ( bModeRead[ax] == 9 || bModeRead[ax] == 10 )
      { // Vel-mode/Torque-mode;
      iCmdPos[ax]   = iEncPos[ax];
      //  iCurrGlbPos[ax] = iEncPos[ax];
      }
  else
      {
      iCmdPos[ax]   = iCurrGlbPos[ax];
      }
  iLastCmdPos[ax] = iCmdPos[ax];    //  07.07.31;
  
  return( 0 );
}

/*:: log_ecat_opcmd_info -------------*/
void log_ecat_opcmd_info( )
{
  DWORD     dwIdx;

  dwIdx = dwEcatOpIdx; 
  iEcatOpBuff[dwIdx][0] = (int)*p_McmdComd;
  iEcatOpBuff[dwIdx][1] = (int)*p_McmdParm1;
  iEcatOpBuff[dwIdx][2] = (int)*p_McmdParm2;
  iEcatOpBuff[dwIdx][3] = (int)*p_McmdParm3;
  iEcatOpBuff[dwIdx][4] = (int)*p_McmdParm4;
  iEcatOpBuff[dwIdx][5] = (int)dwOpRtssCount;

  dwEcatOpIdx++;
  if  ( dwEcatOpIdx >= _ECAT_MAX_OP_ )
      dwEcatOpIdx = 0;
  if  ( dwEcatOpCnt < _ECAT_MAX_OP_ )
      dwEcatOpCnt++;
}

/*:: log_ecat_comm_info_ex -----------*/
void log_ecat_comm_info_ex( int  i, int type )
{    //  10.10.13;
  DWORD dwTmp, dwIdx;

  //  08.06.24, MAX_NUM_SLAVES -> _MAX_YAS_AXIS_;
  if  ( dwEcatCmCnt >= _ECAT_MAX_REG_ )  return;
  if  ( i < 0 || i >= (_MAX_YAS_AXIS_+5) )  return;

  dwIdx = dwEcatCmIdx; 
  iEcatCommBuff[dwIdx][0] = i;
  if  ( i >= _MAX_YAS_AXIS_ )
      {
      i -= _MAX_YAS_AXIS_;
      iEcatCommBuff[dwIdx][1] = (int)wHRmbCtrl[i];
      iEcatCommBuff[dwIdx][2] = (int)wHRmbStatus[i];
      iEcatCommBuff[dwIdx][3] = iHRmbVelCmd[i];
      iEcatCommBuff[dwIdx][4] = iHRmbEncPos[i];
      iEcatCommBuff[dwIdx][5] = 0;
      iEcatCommBuff[dwIdx][6] = (int)wHRmbLatchSts[i];
      iEcatCommBuff[dwIdx][7] = (int)wHRmbPhase[i];
      }
  else
      {
      iEcatCommBuff[dwIdx][1] = (int)wCtrlCmd[i];
      iEcatCommBuff[dwIdx][2] = (int)wSlavSts[i];
      iEcatCommBuff[dwIdx][3] = iCmdPos[i];
      iEcatCommBuff[dwIdx][4] = iEncPos[i];
      if  ( wSlaveType[i] == 17 && wCtrlCmd[i] == 0x180f )
          iEcatCommBuff[dwIdx][5] = iUsrVffCmd[i];
      else if  ( wSlaveType[i] == 18 && wCtrlCmd[i] == 0x80f )
          iEcatCommBuff[dwIdx][5] = iUsrVffCmd[i];
      else if  ( wSlaveType[i] == 19 && wCtrlCmd[i] == 0x40f )
          iEcatCommBuff[dwIdx][5] = iUsrVffCmd[i];  //  15.01.15;
      else
          iEcatCommBuff[dwIdx][5] = (int)shTorqVal[i];
      //  10.10.13, 10.03.16, 09.04.14, 09.03.17;
      //  if  ( wSlaveType[i] == 8 || wSlaveType[i] == 9 )
      if  ( type > 10 )
          {
          dwTmp = ((DWORD)wTouchFunc[i]) << 16;
          dwTmp |= (DWORD)wTouchSts[i];
          iEcatCommBuff[dwIdx][7] = type;
          }
      else
          {
          //  dwTmp = ((DWORD)bModeRead[i]) << 16;
          dwTmp = ((DWORD)wSlaveType[i]) << 24;
          dwTmp |= (((DWORD)bModeRead[i]) << 16);
          dwTmp |= (DWORD)wErrCode[i];
          iEcatCommBuff[dwIdx][7] = (int)wCurrPhase[i];
          }
      iEcatCommBuff[dwIdx][6] = (int)dwTmp;
      //  iEcatCommBuff[dwIdx][7] = (int)wCurrPhase[i];
      }
  iEcatCommBuff[dwIdx][8] = (int)dwMEcatR0Cnt;
  iEcatCommBuff[dwIdx][9] = (int)dwOpRtssCount;
  iEcatCommBuff[dwIdx][10] = (int)dwMotAppCount;
  iEcatCommBuff[dwIdx][11] = iLastProcTm[1];

  dwEcatCmIdx++;
  dwEcatCmCnt++;
}

/*:: log_ecat_comm_info --------------*/
void log_ecat_comm_info( int  i )
{
	log_ecat_comm_info_ex( i, 0 );
}

/*:: hrmb_phase_process --------------*/
int  hrmb_phase_process()
{    //  07.02.12;
  int   i, hax;

  if  ( dwHRmbNum <= 0 || pbyHalRmbRxBase == NULL )
      return( 0 );

  //  for ( i = 0; i < 5; i++ )
  for ( i = 1; i < _MAX_RMB_AXIS_; i++ )
      {
      //  08.06.24, MAX_NUM_SLAVES -> _MAX_YAS_AXIS_;
      hax = i + _MAX_YAS_AXIS_;
      //  07.09.18, Check Mode Switch, Fault/Following-error/Warning;
      //  if  ( (wHRmbStatus[i] & 0x0f) == 0x08 )
      if  ( (wHRmbStatus[i] & 0x08) != 0 ||
            //  (wHRmbStatus[i] & 0x2000) != 0 || 
            (wHRmbStatus[i] & 0x80) != 0 )
          {
          if  ( wHRmbPhase[i] != _FAULT_ )
              {
              log_ecat_comm_info( hax );
              wHRmbPhase[i] = _FAULT_;
              }
          }
      //  else if  ( (wHRmbStatus[i] & 0x4f) == 0x40 )
      else if  ( (wHRmbStatus[i] & 0x40) != 0 )
          {
          if  ( wHRmbPhase[i] != _SW_ON_DISABLE_ )
              {
              log_ecat_comm_info( hax );
              wHRmbPhase[i] = _SW_ON_DISABLE_;
              }
          }
      else if  ( (wHRmbStatus[i] & 0x4f) == 0x00 )
          {  //  07.09.18;
          if  ( wHRmbPhase[i] != _NOT_READY_SON_ )
              {
              log_ecat_comm_info( hax );
              wHRmbPhase[i] = _NOT_READY_SON_;
              }
          }
      else
          {
          switch( wHRmbPhase[i] )
              {
              case _FAULT_:  //  08.12.11, 07.09.18, 脫離;
                  log_ecat_comm_info( hax );
                  wHRmbPhase[i] = _SWITCH_ON_;
                  break;
              case _NOT_READY_SON_:  //  07.09.18;
                  /***  08.12.11;
                  if  ( (wHRmbStatus[i] & 0x40) != 0 || (wHRmbStatus[i] & 0x0f) != 0 )
                      {
                      if  ( dwOpRtssCount > 20 )
                          {
                          log_ecat_comm_info( hax );
                          wHRmbPhase[i] = _SW_ON_DISABLE_;
                          }
                      }
                  ***/
                  break;
              case _SW_ON_DISABLE_:
                  if  ( (wHRmbStatus[i] & 0x6f) == 0x21 )
                      {
                      log_ecat_comm_info( hax );
                      wHRmbPhase[i] = _READY_SW_ON_;
                      }
                  break;
              case _READY_SW_ON_:
                  //  if  ( (wHRmbStatus[i] & 0x27f) == 0x233 )
                  if  ( (wHRmbStatus[i] & 0x7f) == 0x33 ||
                        (wHRmbStatus[i] & 0x7f) == 0x37 )
                      {
                      log_ecat_comm_info( hax );
                      wHRmbPhase[i] = _SWITCH_ON_;
                      //  dwHRmbAuToCtrl[i] = 0;    //  08.06.25;
                      }
                  else if  ( (wHRmbStatus[i] & 0x0f) == 0x00 ) 
                      {
                      log_ecat_comm_info( hax );
                      wHRmbPhase[i] = _SW_ON_DISABLE_;                  
                      }
                  break;
              case _SWITCH_ON_:
                  //  if  ( (wHRmbStatus[i] & 0x27f) == 0x237 )
                  if  ( (wHRmbStatus[i] & 0x7f) == 0x37 )
                      {
                      log_ecat_comm_info( hax );
                      safe_reset_ax( hax );
                      dwHRmbAuToCtrl[i] = 0;  
                      wHRmbPhase[i] = _OPERATION_;
                      }
                  else if  ( (wHRmbStatus[i] & 0x0f) == 0x01 ) 
                      {
                      //  08.06.21, 排除;
                      //  log_ecat_comm_info( hax );
                      //  wHRmbPhase[i] = _READY_SW_ON_;                  
                      }
                  break;
              case _OPERATION_:
                  /***  08.06.21;
                  if  ( (wHRmbStatus[i] & 0x10) == 0x00 )
                      {  //  07.09.18;
                      log_ecat_comm_info( hax );
                      wHRmbPhase[i] = _SW_ON_DISABLE_;
                      }
                  else if  ( (wHRmbStatus[i] & 0x7) == 0x03 )
                      {
                      log_ecat_comm_info( hax );
                      wHRmbPhase[i] = _SWITCH_ON_;
                      }
                  else if  ( (wHRmbStatus[i] & 0x0f) == 0x01 ) 
                      {
                      log_ecat_comm_info( hax );
                      wHRmbPhase[i] = _READY_SW_ON_;                  
                      }
                  else if  ( (wHRmbStatus[i] & 0x0f) == 0x00 ) 
                      {
                      log_ecat_comm_info( hax );
                      wHRmbPhase[i] = _SW_ON_DISABLE_;                  
                      }
                 ***/
                 break;
              }
          }          
      //  Mode Action;
      switch( wHRmbPhase[i] )
          {
          case _FAULT_:
              //  08.12.11;
              wUsrHRCtrl[i] = (WORD)p_McmdHRmbOut[i]; 
              if  ( wOldHRmbCtrl[i] != wUsrHRCtrl[i] &&
                    wUsrHRCtrl[i] != 0x00 )
                  wHRmbCtrl[i] = wUsrHRCtrl[i];
              if  ( dwHRmbAuToCtrl[i] == 1 )
                  wHRmbCtrl[i] = 0x80;  //  Fault Reset;
              break;
          case _NOT_READY_SON_:
              //  08.12.11, 07.09.18;
              if  ( wOldHRmbCtrl[i] != 0x80 && wHRmbCtrl[i] != 0x07 )
                  wHRmbCtrl[i] = 0x80;
              else if  ( wOldHRmbCtrl[i] == 0x80 )
                  wHRmbCtrl[i] = 0x07;
              if  ( wOldHRmbCtrl[i] == 0x07 &&
                   (wHRmbStatus[i] & 0x0f) != 0x00 )
                  {  //  至少經過一個週期;
                  log_ecat_comm_info( hax );
                  wHRmbCtrl[i] = 0x06;
                  wHRmbPhase[i] = _SWITCH_ON_;
                  }
              break;
          case _SW_ON_DISABLE_:  //  08.12.11, 07.02.02;
              wHRmbPhase[i] = _SWITCH_ON_;
              /***
              if  ( dwHRmbAuToCtrl[i] == 1 )
                  {
                  wHRmbCtrl[i] = 0x0f;
                  }
              ***/
              break;
          case _READY_SW_ON_:  //  08.12.11, 08.06.25, 07.09.18, 07.02.02;
              wHRmbPhase[i] = _SWITCH_ON_;
              /***
              if  ( dwHRmbAuToCtrl[i] == 1 )
                  {
                  wHRmbCtrl[i] = 0x0f;
                  }
              ***/
              break;
          case _SWITCH_ON_:  //  08.12.11;
              if  ( dwHRmbAuToCtrl[i] == 1 )
                  {
                  if  ( wOldHRmbCtrl[i] != 0x07 )
                      wHRmbCtrl[i] = 0x07;
                  dwHRmbAuToCtrl[i] = 0;
                  }
              else
                  {
                  wUsrHRCtrl[i] = (WORD)p_McmdHRmbOut[i]; 
                  if  ( wOldHRmbCtrl[i] != wUsrHRCtrl[i] &&
                        wUsrHRCtrl[i] != 0x00 )
                      wHRmbCtrl[i] = wUsrHRCtrl[i];
                  }
              break;
          case _OPERATION_:
              dwMotEcatRdyFlag |= dwUbitVal[hax];   //  07.09.18;
              dwHRmbAuToCtrl[i] = 0;
              //  opreation_loop( i );
              break;
          }
      wOldHRmbCtrl[i] = wHRmbCtrl[i];
      /***  08.06.21;
      if  ( (wOldHRmbSts[i] & 0x10) == 0 && (wHRmbStatus[i] & 0x10) != 0 )
          {
          dwHRmbAuToCtrl[i] = 1;
          wHRmbCtrl[i] = 0x06;
          }
      ***/
      wOldHRmbSts[i] = wHRmbStatus[i];
      }
      
  return( 1 );
}

USHORT wPrevPhase[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};						// 2026/06/06
int g_nLogCnt = 0;
DWORD g_dwAxUsrReqFlagPrev = 0;																				// 2026/06/08
BYTE g_bModeWritePrev[MAX_NUM_SLAVES] =  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};		// 2026/06/08	
int g_iVelCmdPrev[MAX_NUM_SLAVES] =  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};			// 2026/06/08
void record_slave_sts()																						// 2026/06/06
{
	int   i;
  
	if  ( dwOpRtssFlag != 1 )  return;
	if  ( dwOpRtssExecFlag != 0 )  return;
	bool bAllInOperation = true;
	bool bPhaseChange = false, bCtrlCmdChange = false, bUsrReqFlagChange = false, bModeWriteChange = false, bVelCmdChange = false;
	for ( i = 0; i < (int)dwYasFoundSlaves; i++ )
		if (wCurrPhase[i] != _OPERATION_ && bAllInOperation == true)
			bAllInOperation = false;

	for ( i = 0; i < (int)dwYasFoundSlaves; i++ )
		if (wPrevPhase[i] != wCurrPhase[i] && bPhaseChange == false)
			bPhaseChange = true;

	for ( i = 0; i < (int)dwYasFoundSlaves; i++ )
		if (wOldCtrlCmd[i] != wCtrlCmd[i] && bCtrlCmdChange == false)
			bCtrlCmdChange = true;
	for ( i = 0; i < (int)dwYasFoundSlaves; i++ )
		if (g_bModeWritePrev[i] != bModeWrite[i] && bModeWriteChange == false)
			bModeWriteChange = true;
	for ( i = 0; i < (int)dwYasFoundSlaves; i++ )
		if (g_iVelCmdPrev[i] != iVelCmd[i] && bVelCmdChange == false)
			bVelCmdChange = true;

	if (g_dwAxUsrReqFlagPrev != dwAxUsrReqFlag)
		bUsrReqFlagChange = true;

	if ((bCtrlCmdChange || bPhaseChange || bModeWriteChange || bVelCmdChange) && g_nLogCnt++ < 5000)
	{
		RtPrintf(">>> ");
		for ( i = 0; i < (int)dwYasFoundSlaves; i++ )
			RtPrintf("%d. %x, %x, %x, %x, %d; ", i, wCurrPhase[i], wCtrlCmd[i], wSlavSts[i], bModeWrite[i], iVelCmd[i]);
		RtPrintf(" %d\n", dwOpRtssCount);
	}

	if (bUsrReqFlagChange)
		RtPrintf(" %x, %x\n", dwAxUsrReqFlag, dwUbitVal);

	for ( i = 0; i < (int)dwYasFoundSlaves; i++ )
	{
		wPrevPhase[i] = wCurrPhase[i];
		g_bModeWritePrev[i] = bModeWrite[i];
		g_iVelCmdPrev[i] = iVelCmd[i];
	}
	g_dwAxUsrReqFlagPrev = dwAxUsrReqFlag;
}

/*:: oprtss_process ------------------*/
int  oprtss_process()
{
  int   i;
  
  if  ( dwOpRtssFlag != 1 )  return( _NOT_INITIATED_ );
  if  ( dwOpRtssExecFlag != 0 )  return( 1 );
  
//  record_slave_sts();																	// 2026/06/06

  dwOpRtssCount++;
  dwOpRtssExecFlag = 1;  //  保護重複呼叫;
    
  block_info_input();
  
  //  14.08.12, 架構保留;
  //  ISR_ecat_r0_func( );
  //  14.08.04;
  mot_r3_get_user_info( );
  //
  for ( i = 0; i < (int)dwYasFoundSlaves; i++ )
      {
      //  08.05.06, Following-error, 獨立偵測;
      //  連續發生 20個週期以上, 只記錄, 不報警;
      if  ( (wSlavSts[i] & 0x2000) != 0 )
          {
          if  ( wFollowErrCnt[i] < 20 )
              {
              wFollowErrCnt[i]++;
              if  ( wFollowErrCnt[i] >= 20 )
                  log_ecat_comm_info( i );
              }
          } 
      else
          {
          wFollowErrCnt[i] = 0;
          }
      //  07.07.27, Check Mode Switch, Fault/Following-error/Warning;
      if  ( (wSlavSts[i] & 0x08) != 0 ||
            //  (wSlavSts[i] & 0x2000) != 0 || 
            (wSlavSts[i] & 0x80) != 0 )
          {
          if  ( wCurrPhase[i] != _FAULT_ )
              {
              log_ecat_comm_info( i );
              //  11.07.11, Warning 不切換;
              if  ( (wSlavSts[i] & 0x08) != 0 )
                  wCurrPhase[i] = _FAULT_;
              }
          }
      else if  ( (wSlavSts[i] & 0x40) != 0 )
          {  
          if  ( wCurrPhase[i] != _SW_ON_DISABLE_ )
              {
              if  ( wSlaveType[i] == 3 || wSlaveType[i] == 6 ||
                    wSlaveType[i] == 14 || wSlaveType[i] == 15 )
                  {  //  Do nothing;
                  }
              else
                  {
                  log_ecat_comm_info( i );
                  wCurrPhase[i] = _SW_ON_DISABLE_;
                  }
              }
          }
      //  else if  ( (wSlavSts[i] & 0x4f) == 0x00 )
      else if  ( ((wSlavSts[i] & 0x47) == 0x00) && 
                 (dwOpRtssCount > 20) )
          {  //  09.02.26, 09.01.08;
          if  ( wCurrPhase[i] != _NOT_READY_SON_ )
              {
              if  ( dwWarnCount[i] == 0 )
                  {
                  log_ecat_comm_info( i );
                  dwWarnCount[i]++;
                  if  ( wSlaveType[i] == 3 || wSlaveType[i] == 6 )
                      wCurrPhase[i] = _SWITCH_ON_;
                  else
                      wCurrPhase[i] = _NOT_READY_SON_;
                  }
              }
          }
      else
          {
          switch( wCurrPhase[i] )
              {
              case _FAULT_:  //  07.08.01, 脫離;
                  //  13.03.06, 11.12.09, 08.12.11;
                  if  ( wSlaveType[i] == 3 || wSlaveType[i] == 6 ||
                        wSlaveType[i] == 15 ||
                        wSlaveType[i] == 14 || wSlaveType[i] == 16 )
                      {
                      log_ecat_comm_info( i );
                      wCurrPhase[i] = _SWITCH_ON_;
                      } 
                  else if  ( (wSlavSts[i] & 0x6f) == 0x21 )
                      {
                      log_ecat_comm_info( i );
                      wCurrPhase[i] = _SW_ON_DISABLE_;  //  07.08.17;
                      //  wCurrPhase[i] = _READY_SW_ON_;
                      }
                  else if  ( (wSlavSts[i] & 0x7f) == 0x33 )
                      {
                      log_ecat_comm_info( i );
                      wCurrPhase[i] = _SW_ON_DISABLE_;
                      //  wCurrPhase[i] = _SWITCH_ON_;
                      }
                  else if  ( (wSlavSts[i] & 0x0f) == 0x00 ) 
                      {
                      log_ecat_comm_info( i );
                      wCurrPhase[i] = _SW_ON_DISABLE_;                  
                      }
                  else
                      {
                      log_ecat_comm_info( i );
                      wCurrPhase[i] = _SW_ON_DISABLE_;                  
                      }
                  //  已經檢查排除;
                  //  if  ( (wSlavSts[i] & 0x40) != 0 )
                  //    wCurrPhase[i] = _SW_ON_DISABLE_;
                  break;
              case _NOT_READY_SON_:
                  //  13.03.06, 11.12.09, 08.12.11, 排除 CNC_IO;
                  if  ( wSlaveType[i] == 3 || wSlaveType[i] == 6 ||
                        wSlaveType[i] == 15 ||
                        wSlaveType[i] == 14 || wSlaveType[i] == 16 )
                      break;
                  if  ( (wSlavSts[i] & 0x40) != 0 || (wSlavSts[i] & 0x0f) != 0 )
                      {
                      log_ecat_comm_info( i );
                      wCurrPhase[i] = _SW_ON_DISABLE_;
                      }
                  break;
              case _SW_ON_DISABLE_:
                  if  ( (wSlavSts[i] & 0x6f) == 0x21 )
                      {
                      log_ecat_comm_info( i );
                      wCurrPhase[i] = _READY_SW_ON_;
                      }
                  break;
              case _READY_SW_ON_:
                  //  if  ( (wSlavSts[i] & 0x27f) == 0x233 )
                  if  ( (wSlavSts[i] & 0x7f) == 0x33 )
                      {
                      log_ecat_comm_info( i );
                      wCurrPhase[i] = _SWITCH_ON_;
                      dwAuToCtrlCmd[i] = 0;     //  08.05.06;
                      }
                  else if  ( (wSlavSts[i] & 0x0f) == 0x00 ) 
                      {
                      log_ecat_comm_info( i );
                      wCurrPhase[i] = _SW_ON_DISABLE_;                  
                      }
                  else if  ( wPhaseCnt[i] > 90 ) 
                      {  //  11.12.09;
                      log_ecat_comm_info( i );
                      wCurrPhase[i] = _SWITCH_ON_;                  
                      }
                  break;
              case _SWITCH_ON_:
                  //  if  ( (wSlavSts[i] & 0x27f) == 0x237 )
                  if  ( (wSlavSts[i] & 0x7f) == 0x37 )
                      {
                      log_ecat_comm_info( i );
                      safe_reset_ax( i );
                      dwAuToCtrlCmd[i] = 0;     //  07.02.02;
                      wCurrPhase[i] = _OPERATION_;
                      }
                  else if  ( (wSlavSts[i] & 0x0f) == 0x01 ) 
                      {  //  13.03.06, 11.12.09, 07.12.27, 排除 CNC-IO;
                      if  ( wSlaveType[i] != 3 && wSlaveType[i] != 6 &&
                            wSlaveType[i] != 15 && 
                            wSlaveType[i] != 14 && wSlaveType[i] != 16 )
                          {
                          log_ecat_comm_info( i );
                          wCurrPhase[i] = _READY_SW_ON_;                  
                          }
                      }
                  break;
              case _OPERATION_:
                  if  ( wSlaveType[i] == 3 || wSlaveType[i] == 6 )
                      break;  //  07.12.27, 排除 CNC-IO;  
                  if  ( wSlaveType[i] == 14 || wSlaveType[i] == 16 ||
                        wSlaveType[i] == 15 )
                      break;  //  11.12.09;  
                  if  ( (wSlavSts[i] & 0x10) == 0x00 )
                      {  //  07.08.30, voltage disabled;
                      log_ecat_comm_info( i );
                      wCurrPhase[i] = _SW_ON_DISABLE_;
                      wCtrlCmd[i] = 0x06;  //  07.09.26;
                      dwMotEcatRdyFlag &= ~dwUbitVal[i];    //  10.03.25;
                      }
                  else if  ( (wSlavSts[i] & 0x7) == 0x03 )
                      {
                      log_ecat_comm_info( i );
                      wCurrPhase[i] = _SWITCH_ON_;
                      wCtrlCmd[i] = 0x07;  //  07.09.26;
                      dwMotEcatRdyFlag &= ~dwUbitVal[i];    //  10.03.25;
                      }
                  else if  ( (wSlavSts[i] & 0x0f) == 0x01 ) 
                      {
                      log_ecat_comm_info( i );
                      wCurrPhase[i] = _READY_SW_ON_;                  
                      wCtrlCmd[i] = 0x06;  //  07.09.26;
                      dwMotEcatRdyFlag &= ~dwUbitVal[i];    //  10.03.25;
                      }
                  else if  ( (wSlavSts[i] & 0x0f) == 0x00 ) 
                      {
                      log_ecat_comm_info( i );
                      wCurrPhase[i] = _SW_ON_DISABLE_;                  
                      wCtrlCmd[i] = 0x06;  //  07.09.26;
                      dwMotEcatRdyFlag &= ~dwUbitVal[i];    //  10.03.25;
                      }
                 break;
              }
          }          
      //  Mode Action;
      //  wCtrlCmd[i] &= 0x0f;
      switch( wCurrPhase[i] )
          {
          case _FAULT_:
              //  wCtrlCmd[i] |= 0x80;  //  Fault Reset;
              wUsrCtrlCmd[i] = (USHORT)p_McmdBlkIn[280+i];
              if  ( wOldCtrlCmd[i] != wUsrCtrlCmd[i] && 
                    wUsrCtrlCmd[i] != 0x00 )
                  wCtrlCmd[i] = wUsrCtrlCmd[i];
              //  07.02.21;
              if  ( dwAuToCtrlCmd[i] == 1 )
                  wCtrlCmd[i] = 0x80;
              break;
          case _NOT_READY_SON_:
              //  13.03.06, 11.12.09, ... 07.08.29, for CNC_IO;
              if  ( wSlaveType[i] == 3 || wSlaveType[i] == 6 ||
                    wSlaveType[i] == 15 || 
                    wSlaveType[i] == 14 || wSlaveType[i] == 16 )
                  {
                  wPhaseCnt[i] = 0;     //  11.12.09;
                  if  ( wOldCtrlCmd[i] != 0x80 && wCtrlCmd[i] != 0x07 )
                      wCtrlCmd[i] = 0x80;
                  else if  ( wOldCtrlCmd[i] == 0x80 )
                      wCtrlCmd[i] = 0x07;                  
                  if  ( wOldCtrlCmd[i] == 0x07 &&
                       (wSlavSts[i] & 0x0f) != 0x00 )
                      {
                      log_ecat_comm_info( i );
                      wCurrPhase[i] = _SWITCH_ON_;
                      }
                  break;
                  }
              if  ( wOldCtrlCmd[i] != 0x06 )
                  wCtrlCmd[i] = 0x06;
              dwWarnCount[i] = 0;
              break;
          case _SW_ON_DISABLE_:  //  07.02.02;
              //  09.02.26, 08.12.11;
              if  ( wSlaveType[i] == 3 || wSlaveType[i] == 6 )
                  {
                  wCurrPhase[i] = _SWITCH_ON_;
                  break;
                  }
              //  15.01.23;
              //  if  ( dwAuToCtrlCmd[i] == 1 )
              //      {
              //  07.08.29, 07.02.21;
              //  if  ( wOldCtrlCmd[i] == 0x80 )
              if  ( (wOldCtrlCmd[i] & 0x80) != 0x0 )
                  wCtrlCmd[i] = 0x06;
              if  ( (wOldCtrlCmd[i] & 0x06) != 0x02 &&
                    (wOldCtrlCmd[i] & 0x02) != 0x0 )
                  wCtrlCmd[i] = 0x06;
              //      }
              //  else
              //      {
              //      wUsrCtrlCmd[i] = (USHORT)p_McmdBlkIn[280+i];
              //      if  ( wOldCtrlCmd[i] != wUsrCtrlCmd[i] && 
              //            wUsrCtrlCmd[i] != 0x00 )
              //          wCtrlCmd[i] = wUsrCtrlCmd[i];
              //      }
              break;
          case _READY_SW_ON_:  //  07.08.01, 07.02.02;
              //  09.02.26, 08.12.11;
              if  ( wSlaveType[i] == 3 || wSlaveType[i] == 6 )
                  {
                  wCurrPhase[i] = _SWITCH_ON_;
                  break;
                  }
              //  15.01.23;
              //  if  ( dwAuToCtrlCmd[i] == 1 )
              //      {
              if  ( wOldCtrlCmd[i] == 0x06 || wOldCtrlCmd[i] == 0x80 )
                  {
                  wPhaseCnt[i] = 0;  //  11.12.09;
                  wCtrlCmd[i] = 0x07;
                  }
              else
                  wPhaseCnt[i]++;                      
              //  15.01.08, 13.10.18;
              if  ( wSlaveType[i] == 17 || wSlaveType[i] == 18 || wSlaveType[i] == 19 || 
//				    wSlaveType[i] == 20)	// 眾辰, US800-C06, 15.06.04;)
				    (wSlaveType[i] == 20 && i >= (int)dwYasFoundSlaves-2))	// 眾辰, US800-C06, 15.06.04;)
                  {  //  初始化;
                  bModeWrite[i] = 0x09;     //  預設為: CSV;
                  p_McmdBlkIn[300+i] = 0x09;
                  boldUsrModeWr[i] = bUsrModeWrite[i] = 0x09;
                  }              
              //      }
              //  else
              //      {
              //      wUsrCtrlCmd[i] = (USHORT)p_McmdBlkIn[280+i];
              //      if  ( wOldCtrlCmd[i] != wUsrCtrlCmd[i] && 
              //            wUsrCtrlCmd[i] != 0x00 )
              //          wCtrlCmd[i] = wUsrCtrlCmd[i];
              //      }
              break;
          case _SWITCH_ON_:
              if  ( dwAuToCtrlCmd[i] == 1 )
                  {
                  //  08.05.06;
                  //  if  ( wOldCtrlCmd[i] == 0x07 || wOldCtrlCmd[i] == 0x06 )
                  //      wCtrlCmd[i] = 0x0f;
                  //  08.12.11;
                  if  ( (wSlaveType[i] == 3) && wOldCtrlCmd[i] != 0x07 )
                      wCtrlCmd[i] = 0x07;
                  //  13.03.06, 11.12.09;
                  if  ( wSlaveType[i] == 14 || wSlaveType[i] == 15 || 
                        wSlaveType[i] == 16 )
                      {
                      wCtrlCmd[i] = 0x0f;
                      log_ecat_comm_info( i );
                      wCurrPhase[i] = _OPERATION_;
                      wPhaseCnt[i] = 0;
                      }
                  dwAuToCtrlCmd[i] = 0;
                  }
              else
                  {
                  wUsrCtrlCmd[i] = (USHORT)p_McmdBlkIn[280+i];
                  if  ( wOldCtrlCmd[i] != wUsrCtrlCmd[i] && 
                        wUsrCtrlCmd[i] != 0x00 )
                      wCtrlCmd[i] = wUsrCtrlCmd[i];
                  }
              switchon_loop( i );     //  13.10.18;
              break;
          case _OPERATION_:
              dwMotEcatRdyFlag |= dwUbitVal[i];     //  07.07.31;
              opreation_loop( i );
              break;
          }
      //  09.03.11;
      if  ( wOldCtrlCmd[i] != wCtrlCmd[i] )
          log_ecat_comm_info( i );
      wOldCtrlCmd[i] = wCtrlCmd[i];
      //  09.04.14, 09.03.17;
      if  ( wOldErrCode[i] != wErrCode[i] )
          {
          log_ecat_comm_info( i );
          wOldErrCode[i] = wErrCode[i];
          }
      //  13.12.24, 10.03.25, 07.07.31;
      if  ( (dwMotEcatRdyFlag & dwUbitVal[i]) == 0 )
          {
          if  ( wSlaveType[i] == 3 )
              iCmdPos[i] = 0;
          else if  ( wSlaveType[i] == 17 && wCtrlCmd[i] == 0x180f )
              iCmdPos[i] = iUsrVffCmd[i];
          else if  ( wSlaveType[i] == 18 && wCtrlCmd[i] == 0x80f )
              iCmdPos[i] = iUsrVffCmd[i];   //  15.01.08;
          else if  ( wSlaveType[i] == 19 && wCtrlCmd[i] == 0x40f )
              iCmdPos[i] = iUsrVffCmd[i];   //  15.01.15;
          else
              iCmdPos[i] = iEncPos[i];
          iLastCmdPos[i] = iCmdPos[i];
          }
      //  11.12.09, 09.02.26, 07.12.27, 07.08.29, 排除 CNC-IO;
      if  ( (wOldSlavSts[i] & 0x10) == 0 && 
            (wSlavSts[i] & 0x10) != 0 &&
            (wSlaveType[i] != 16) &&
            (wSlaveType[i] != 15) &&
            (wSlaveType[i] != 14) &&
            (wSlaveType[i] != 6) &&
            (wSlaveType[i] != 3) )
          {
          dwAuToCtrlCmd[i] = 1;
          wCtrlCmd[i] = 0x06;
          }
      wOldSlavSts[i] = wSlavSts[i];
      }

  if  ( dwUsrReqFlag == 1 )
      dwUsrReqFlag = 0;

  hrmb_phase_process();  //  08.06.21;
  //
  block_info_output();
  
  dwOpRtssExecFlag = 0;  //  保護重複呼叫;    
  return( 0 );
}

//======================================================================

/*:: mot_get_HalRmb_offset -----------*/
int  mot_get_HalRmb_offset()
{    //  07.12.27, 07.02.07;
  int   iNum, i;
  WORD  wByOffset;

  EC_T_CFG_SLAVE_INFO* pMySlave = EC_NULL;

  //  HAL_ECAT_LoadSlaveFile(); //解析ENI轉換後的檔案資料
  //  pwHalRmbTxBase = pwHalRmbRxBase = NULL;
  pbyHalRmbTxBase = pbyHalRmbRxBase = NULL;
  dwHalRmbTxNum = dwHalRmbRxNum = 0;
  //  07.12.27;
  iNum = 0;
  dwHalRmbSAddr = 1004;
  for ( i = 0; i < (int)dwYasFoundSlaves; i++ )
      {
      if  ( wSlaveType[i] == 3 )
          {
          pMySlave = (EC_T_CFG_SLAVE_INFO *)my_aSlaveList[i];    
          wByOffset = pMySlave->dwPdOffsIn/8;                
          //  pwHalRmbTxBase = (WORD *)&pbyEcatPDIn[wByOffset];
          pbyHalRmbTxBase = &pbyEcatPDIn[wByOffset];
          wByOffset = pMySlave->dwPdOffsOut/8;                
          pbyHalRmbRxBase = &pbyEcatPDOut[wByOffset];
          dwHalRmbSAddr = (DWORD)pMySlave->wStationAddress;
          dwHalRmbBase = 0x1210;
          iNum = 1;
          break;
          }
      }
  dwHRmbNum = (DWORD)iNum;
  /***  07.12.27, 保留;
  iNum = HAL_ECAT_GetServoNumber( HAL_RMB );
  //  for ( i = 0; i < n; i++)
  //      {
  // 取得第n個Servo Device的Tx PDO Offset與其數量
  if  ( iNum > 0 )
      {
      HAL_ECAT_GetServoTxOffset( HAL_RMB, 0, &dwHalRmbBase, &dwHalRmbTxNum);
      pwHalRmbTxBase = (WORD *)&pbyEcatPDIn[dwHalRmbBase/8];
      // 取得第n個Servo Device的Rx PDO Offset與其數量
      HAL_ECAT_GetServoRxOffset( HAL_RMB, 0, &dwHalRmbBase, &dwHalRmbRxNum);
      pwHalRmbRxBase = (WORD *)&pbyEcatPDOut[dwHalRmbBase/8];
      dwHRmbNum = (DWORD)iNum;
      }
  //      }
  ***/
  
  p_McmdInfoOut[0] = (DWORD)pbyHalRmbRxBase;
  p_McmdInfoOut[1] = dwHalRmbRxNum;
  p_McmdInfoOut[2] = (DWORD)pbyHalRmbTxBase;
  p_McmdInfoOut[3] = dwHalRmbTxNum;
  
  //  07.09.20; 
  RtPrintf("VVVVV: mot_get_HalRmb_offset()..  \n");
  RtPrintf("     :  ( %d, %08x, %08x, %lu, %lu );  \n",
             iNum, (DWORD)pbyHalRmbRxBase, (DWORD)pbyHalRmbTxBase,
             dwHalRmbRxNum, dwHalRmbTxNum );          
  //  07.10.24, 07.09.21;
  if  ( iNum > 0 )
      {   //  _hrmb_start_request( );
      //  暫時保留, RMB 已經在 HAL_ECAT 內啟用;
      wHRmbInitFlag = 0;
      dwHrmbInitCnt = 0;
      wHRmbActiveNum = 0x03;    //  ch_0 & Ch_1, DAOUT enable;
      dwHRmbDOutp[2] = 0x03;    //  for 0x70FE object;
      dwHalRmbStartF = 1;
      //  for ( i = 0; i < _MAX_RMB_AXIS_; i++)
      //      dwHRmbAuToCtrl[i] = 1;
      //  wHRmbInitFlag = 1;
      }
  else
      {
      dwHalRmbBase = 0x1002;
      }

  return( 0 );
}

//======================================================================

#define  _MAX_ACC_STEP_         2000
#define  _R3_CMD_NUM_           10

int    AccBuffSum[MAX_NUM_SLAVES], AccBuffRem[MAX_NUM_SLAVES], 
       AccBuffIdx[MAX_NUM_SLAVES], AccBuffData[MAX_NUM_SLAVES][_MAX_ACC_STEP_];
int    iAccStep[MAX_NUM_SLAVES], iCurrCmdPulse[MAX_NUM_SLAVES];
int    iR3CmdBuff[MAX_NUM_SLAVES][_R3_CMD_NUM_];
int    iR3GlbPos[MAX_NUM_SLAVES], iR3Dist2GoPos[MAX_NUM_SLAVES];

/*:: mot_r3_initial ------------------*/
void mot_r3_initial( )
{
  int  i;
  
  for ( i = 0; i < MAX_NUM_SLAVES; i++ )
      {
      AccBuffSum[i] = AccBuffRem[i] = AccBuffIdx[i] = 0;
      ZeroMemory( &AccBuffData[i][0], _MAX_ACC_STEP_*sizeof(int) );
      iAccStep[i] = 400;
      iR3Dist2GoPos[i] = 0;
      iR3GlbPos[i] = 0;  
      }
}

/*:: mot_acc_dec ---------------------*/
void mot_acc_dec(int *pPul,int *pSum,int *pRem,int *pIdx,int num,int *pBuff)
{
  int div_result, tidx;
  int SumRem, rem;

  tidx = *pIdx;
  *pSum += (*pPul - pBuff[tidx]);
  pBuff[tidx] = *pPul;
  tidx += 1;
  if  ( tidx >= num ) tidx = 0;
  *pIdx = tidx;
  
  if  ( *pSum!=0 || *pPul!=0 ) 
      {
      SumRem = *pSum;
      div_result = (int)( SumRem / num );
      rem = SumRem - (div_result*num);
      *pPul = div_result;
      *pRem = rem + (*pRem);
      if  ( *pRem >= num ) 
          {
          *pPul = *pPul + 1;
          *pRem = *pRem - num;
          }
      else if  ( *pRem <= -num ) 
          {
          *pPul = *pPul - 1;
          *pRem = *pRem + num;
          }
      }
}

/*:: mot_r3_write_cmd_all ------------*/
int  mot_r3_write_cmd_all()
{  
  int  i, j;
  
  for ( j = 0; j < _R3_CMD_NUM_; j++ )
      for ( i = 0; i < MAX_NUM_SLAVES; i++ )
          {
          oprtss_write_dda_cmd( i, iR3CmdBuff[i][j]);
          }

  return( 0 );
}

/*:: mot_r3_get_user_info ------------*/
int  mot_r3_get_user_info( )
{  
  int   i, iTmp0, iTmp;
  DWORD dwCheck;

  dwCheck = p_McmdBlkIn[499];
  if  (  dwCheck == 1 )
      {
      //  From controller; (client) 
      for (i = 0; i < MAX_NUM_SLAVES; i++)
          {
          iUsrPosCmd[i]    = (int)p_McmdBlkIn[200+i]; 
          iUsrVelCmd[i]    = (int)p_McmdBlkIn[220+i]; 
          shUsrTorqCmd[i]  = (short)p_McmdBlkIn[240+i]; 
          wUsrTorqMax[i]   = (USHORT)p_McmdBlkIn[260+i]; 
          wUsrCtrlCmd[i]   = (USHORT)p_McmdBlkIn[280+i]; 
          bUsrModeWrite[i] = (BYTE)p_McmdBlkIn[300+i];
          wUsrTouchFunc[i] = (USHORT)p_McmdBlkIn[320+i];
          //  11.12.09;  
          iUsrVffCmd[i]    = (int)p_McmdBlkIn[340+i];
          //  08.04.16;
          dwUsrReqFunFlag[i] = (DWORD)p_McmdBlkIn[360+i];
          //  dwAxUsrReqFlag |= dwUbitVal[i];
          if  ( dwUsrReqFunFlag[i] != 0 )
              dwAxUsrReqFlag |= dwUbitVal[i];
          }
      p_McmdBlkIn[499] = 0;
      dwUsrReqFlag = 1;
      }
  //  13.05.27, 13.03.06;
  dwCheck = p_McmdBlkIn[500];
  if  (  dwCheck == 1 )
      {
      for (i = 0; i < 10; i++)
          {
          iTmp0 = p_McmdBlkIn[460+i];
          if  ( iTmp0 == 1 )
              {
              iTmp = p_McmdBlkIn[380+i];
              if  ( iTmp != dwParam0[i] ) 
                  dwParam0[i] = iTmp; 
              iTmp = p_McmdBlkIn[390+i];
              if  ( iTmp != dwParam1[i] ) 
                  dwParam1[i] = iTmp; 
              iTmp = p_McmdBlkIn[400+i];
              if  ( iTmp != dwParam2[i] ) 
                  dwParam2[i] = iTmp; 
              iTmp = p_McmdBlkIn[410+i];
              if  ( iTmp != dwParam3[i] ) 
                  dwParam3[i] = iTmp; 
              iTmp = p_McmdBlkIn[420+i];
              if  ( iTmp != dwParam4[i] ) 
                  dwParam4[i] = iTmp; 
              iTmp = p_McmdBlkIn[430+i];
              if  ( iTmp != dwParam5[i] ) 
                  dwParam5[i] = iTmp; 
              p_McmdBlkIn[460+i] = 0;
              } 
          }
      p_McmdBlkIn[500] = 0;
      }
  
  return( 0 );
}

/*:: mot_r3_process ------------------*/
int  mot_r3_process()
{  //  Every 4ms.;  
  int  ax, i, iPCmd, iTmpCmd, iTmpVel;
  
  //  Trig OP Event, every 40ms.;
  if  ( (dwMotR3Count % 4) == 3 )
      RtSetEvent( hOpRtssEvent );
  //  14.07.02;
  //  if  ( p_McmdSysInfo[10] == 907 )
  //      {
  p_McmdSysInfo[0] = dwOpRtssCount;
  p_McmdSysInfo[1] = (DWORD)iMaxProcTm[0];
  p_McmdSysInfo[2] = dwMotR3Count;
  p_McmdSysInfo[3] = (DWORD)iMaxProcTm[1];
  p_McmdSysInfo[4] = (DWORD)iMaxProcTm[2];
  p_McmdSysInfo[5] = (DWORD)iMaxProcTm[3];
  //  10.10.20;
  p_McmdSysInfo[20] = dwMotAppCount;
  p_McmdSysInfo[21] = dwMotAppBusyCount;
  p_McmdSysInfo[22] = dwOpRtssCount;
  return( 1 );
  //      }
  //  09.02.26, 取消獨立測試功能;
  //  return( 2 );
  //  14.07.02, 以下不會執行, 程式碼保留;
  //
  mot_r3_get_user_info( );
  for (ax = 0; ax < (int)dwYasFoundSlaves; ax++)
      if  ( dwUsrReqFlag == 1 && iR3Dist2GoPos[ax] == 0 )
          iR3Dist2GoPos[ax] = iUsrPosCmd[ax] - iR3GlbPos[ax];
  
  for (ax = 0; ax < (int)dwYasFoundSlaves; ax++)
      {
      ZeroMemory( &iR3CmdBuff[ax][0], _R3_CMD_NUM_*sizeof(int) );
      for (i = 0; i < _R3_CMD_NUM_; i++)
          {
          iCurrCmdPulse[ax] = 0;
          
          //  Planning;
          if  ( bModeRead[ax] == 9 )
              {
              iR3GlbPos[ax] = iCurrGlbPos[ax];
              iR3Dist2GoPos[ax] = 0;
              }
          else if  ( iR3Dist2GoPos[ax] != 0 && bModeRead[ax] == 8 )
              {              
              iTmpCmd = 0;
              //  Update speed, 確認命令同向;
              iTmpVel = iUsrVelCmd[ax];
              if  ( iTmpVel < 0 ) iTmpVel = -iTmpVel;
              if  ( iR3Dist2GoPos[ax] < 0 )
                   iTmpVel = -iTmpVel;
              
              if  ( (iTmpVel > 0 && iR3Dist2GoPos[ax] >= iTmpVel) ||
                    (iTmpVel < 0 && iR3Dist2GoPos[ax] <= iTmpVel) )
                  {
                  iTmpCmd = iTmpVel; 
                  iR3Dist2GoPos[ax] -= iTmpCmd; 
                  }
              else
                  {
                  iTmpCmd = iR3Dist2GoPos[ax];
                  iR3Dist2GoPos[ax] = 0; 
                  }
              iCurrCmdPulse[ax] = iTmpCmd;
              iR3GlbPos[ax] += iTmpCmd;
              }
                               
          iPCmd = iCurrCmdPulse[ax];
          mot_acc_dec( &iPCmd, &AccBuffSum[ax], &AccBuffRem[ax], &AccBuffIdx[ax], 
                   iAccStep[ax], &AccBuffData[ax][0] );
          
          //  mot_r3_write_cmd( ax, iPCmd );
          iR3CmdBuff[ax][i] = iPCmd;
          }
      }
  
  mot_r3_write_cmd_all( );
  
  //  07.01.09;
  p_McmdSysInfo[0] = dwOpRtssCount;
  p_McmdSysInfo[1] = (DWORD)iMaxProcTm[0];
  p_McmdSysInfo[2] = dwMotR3Count;
  p_McmdSysInfo[3] = (DWORD)iMaxProcTm[1];
  p_McmdSysInfo[4] = (DWORD)iMaxProcTm[2];
  p_McmdSysInfo[5] = (DWORD)iMaxProcTm[3];
  
  return( 0 );
}

//======================================================================
#define MBX_TIMEOUT                         5000
/*:: op_sdo_read_request -------------*/
int  op_sdo_read_request( )
{
  int    ax, iRet;
  WORD   wObIndex;
  BYTE   byObSubIndex;
  DWORD  dwTmp, dwDataLen;
  //  DWORD  dwVal, dwSize;
  
  DWORD  MySlaveAddr;
  EC_T_CFG_SLAVE_INFO* pMySlave = NULL;

  ax = *(int *)p_McmdParm1;
  dwTmp = *p_McmdParm2;     //  Object index::Subindex:Length;  
  
  dwDataLen = ( dwTmp & 0x0ff );
  byObSubIndex = (BYTE)( (dwTmp >> 8 ) & 0x0ff );
  wObIndex = (WORD)( (dwTmp >> 16 ) & 0x0ffff );
  *p_McmdRetSts = 0;
 
  //  08.06.24, MAX_NUM_SLAVES -> _MAX_YAS_AXIS_;
  //  if  ( ax < 0 || ax > MAX_NUM_SLAVES )  ax = 0;
  if  ( ax < 0 || ax >= (_MAX_YAS_AXIS_+5) )  ax = 0;
  
  if  ( ax >= _MAX_YAS_AXIS_ )
      {  //  07.02.08, HalRmb;
      oprtss_ptime_start( 2 );
      iRet =  HAL_ECAT_SDOUpload( 1004, wObIndex, byObSubIndex,
                     (BYTE *)&g_dwParaVal, dwDataLen, (DWORD *)&g_dwParaSize, MBX_TIMEOUT, 0 );   
      }  
  else
      {
      pMySlave = (EC_T_CFG_SLAVE_INFO*)my_aSlaveList[ax];
      MySlaveAddr = (DWORD)pMySlave->wStationAddress;
      oprtss_ptime_start( 2 );
      //  10.08.16;
      if  ( dwDataLen > 4 && dwDataLen < 20 )
          {
          iRet =  HAL_ECAT_SDOUpload( MySlaveAddr, wObIndex, byObSubIndex,
                     &strSdoRdBuff[0], dwDataLen, (DWORD *)&g_dwParaSize, MBX_TIMEOUT, 0 );   
          g_dwParaVal = g_dwParaSize;
          //  if  ( g_dwParaSize > 4 && g_dwParaSize < 20 )
          memcpy( (BYTE *)p_McmdRetVal3, &strSdoRdBuff[0], 16 );
          }
      else
          iRet =  HAL_ECAT_SDOUpload( MySlaveAddr, wObIndex, byObSubIndex,
                     (BYTE *)&g_dwParaVal, dwDataLen, (DWORD *)&g_dwParaSize, MBX_TIMEOUT, 0 );   
      }  
  oprtss_ptime_end( 2 );

  //  10.08.16;
  if  ( dwDataLen > 4 && dwDataLen < 20 )
      *p_McmdRetSts = 17;   //  Service finish;
  else
      *p_McmdRetSts = 1;    //  Service finish;
  *p_McmdRetVal1 = g_dwParaVal;
  *p_McmdRetVal2 = (DWORD)iRet;

  RtPrintf("HAL_ECAT_SDOUploa: %x, %x, %x, %x, MBX_TIMEOUT, 0, %x, %x \n", 
					 MySlaveAddr, wObIndex, byObSubIndex, dwDataLen, *p_McmdRetVal1, *p_McmdRetVal2);
  
  return( 0 );
}

/*:: op_sdo_write_request ------------*/
int  op_sdo_write_request( )
{
  int    ax, iRet;
  WORD   wObIndex;
  BYTE   byObSubIndex;
  DWORD  dwTmp, dwDataLen;
  //  DWORD  dwVal;
  
  DWORD  MySlaveAddr;
  EC_T_CFG_SLAVE_INFO* pMySlave = NULL;

  ax = *(int *)p_McmdParm1;
  dwTmp = *p_McmdParm2;     //  Object index::Subindex:Length;  
  g_dwParaVal = *p_McmdParm3;
  
  dwDataLen = ( dwTmp & 0x0ff );
  byObSubIndex = (BYTE)( (dwTmp >> 8 ) & 0x0ff );
  wObIndex = (WORD)( (dwTmp >> 16 ) & 0x0ffff );
  *p_McmdRetSts = 0;
  //  10.08.16, 10.08.13;  
  if  ( dwDataLen > 4 && dwDataLen < 20 )
      memcpy( &strSdoWrBuff[0], (BYTE *)p_McmdParm4, 16 );
 
  //  08.06.24, MAX_NUM_SLAVES -> _MAX_YAS_AXIS_;
  //  if  ( ax < 0 || ax > MAX_NUM_SLAVES )  ax = 0;
  if  ( ax < 0 || ax >= (_MAX_YAS_AXIS_+5) )  ax = 0;

  if  ( ax >= _MAX_YAS_AXIS_ )
      {  //  07.02.08, HalRmb;
      oprtss_ptime_start( 3 );
      iRet =  HAL_ECAT_SDODownload( 1004, wObIndex, byObSubIndex,
                     (BYTE *)&g_dwParaVal, dwDataLen, MBX_TIMEOUT, 0 );   
      }  
  else
      {
      pMySlave = (EC_T_CFG_SLAVE_INFO*)my_aSlaveList[ax];
      MySlaveAddr = (DWORD)pMySlave->wStationAddress;
      oprtss_ptime_start( 3 );
      //  10.08.16, 10.08.13;
      if  ( dwDataLen > 4 && dwDataLen < 20 )
          iRet =  HAL_ECAT_SDODownload( MySlaveAddr, wObIndex, byObSubIndex,
                     &strSdoWrBuff[0], dwDataLen, MBX_TIMEOUT, 0 );   
      else
          iRet =  HAL_ECAT_SDODownload( MySlaveAddr, wObIndex, byObSubIndex,
                     (BYTE *)&g_dwParaVal, dwDataLen, MBX_TIMEOUT, 0 );   
      }
  oprtss_ptime_end( 3 );
  
  *p_McmdRetSts = 1;    //  Service finish;
  *p_McmdRetVal1 = (DWORD)iRet;
  
  return( 0 );
}

/*:: oprtss_set_info_out -------------*/
void oprtss_set_info_out( )
{
  p_McmdInfoOut[8] = (DWORD)wHRmbActiveNum;
  p_McmdInfoOut[9] = (DWORD)wHRmbInitFlag;
  //  07.07.24;
  p_McmdInfoOut[10] = dwMotEcatStatus;
  //  13.07.09;
  p_McmdInfoOut[20] = dwCncIoIsrCnt;
  p_McmdInfoOut[21] = dwCncIoSync0Cnt;
  p_McmdInfoOut[22] = dwCncIoDiffCnt;
  //
  p_McmdInfoOut[30] = dwCncIoDOut_0;
  p_McmdInfoOut[31] = dwCncIoDOut_1;
  p_McmdInfoOut[32] = dwCncIoDOut_2;
}

/*:: op_hrmb_start_request -----------*/
int  op_hrmb_start_request( )
{
  WORD  wAxNum;
   
  *p_McmdRetSts = 0;
  wAxNum = *(WORD *)p_McmdParm1;
  if  ( wAxNum == 0 || wAxNum > 5 )  wAxNum = 1;

  dwHrmbInitCnt = 0;
  wHRmbActiveNum = wAxNum;
  dwHalRmbStartF = 1;
  
  *p_McmdRetSts = 1;    //  Service finish;
    
  return( 0 );
}

/*:: op_hrmb_read_request ------------*/
int  op_hrmb_read_request( )
{
  WORD  wOffset, wReadVal, *pwTmpHalBase;
  int   iRet;
  
  pwTmpHalBase = (WORD *)pbyHalRmbTxBase;
  *p_McmdRetSts = 0;
  wOffset = *(WORD *)p_McmdParm1;
  iRet = 1;     //  Service finish NORMAL;
  if  ( dwHalRmbRxNum == 0 )
      iRet = -1;
  else if  ( wOffset >= dwHalRmbTxNum )
      iRet = -2;
  else
      wReadVal = pwTmpHalBase[wOffset];

  *p_McmdRetVal1 = (DWORD)wReadVal;
  *p_McmdRetSts  = (DWORD)iRet;

  return( 0 );
}

/*:: op_hrmb_write_request -----------*/
int  op_hrmb_write_request( )
{
  WORD  wOffset, wWriteVal, *pwTmpHalBase;
  int   iRet;
  
  *p_McmdRetSts = 0;
  pwTmpHalBase = (WORD *)pbyHalRmbRxBase;  
  wOffset   = *(WORD *)p_McmdParm1;
  wWriteVal = *(WORD *)p_McmdParm2;
  iRet = 1;     //  Service finish NORMAL;
  if  ( dwHalRmbRxNum == 0 )
      iRet = -1;
  else if  ( wOffset >= dwHalRmbRxNum )
      iRet = -2;
  else
      pwTmpHalBase[wOffset] = wWriteVal;
      
  *p_McmdRetSts = (DWORD)iRet;

  return( 0 );
}

/*:: op_probe_func_request -----------*/
int  op_probe_func_request( )
{
  int    ax;
  DWORD  dwTmp;

  ax = *(int *)p_McmdParm1;
  dwTmp = *p_McmdParm2;     
  dwTmp &= 0x1717;
    
  *p_McmdRetSts = 0;
 
  if  ( ax < 0 || ax >= (_MAX_ECAT_AXIS_+_MAX_RMB_AXIS_) )  ax = 0;

  if  ( ax >= _MAX_ECAT_AXIS_ )
      wHRmbLatFunc[ax-_MAX_ECAT_AXIS_] = (WORD)dwTmp;
  else
      wTouchFunc[ax] = (WORD)dwTmp;
    
  *p_McmdRetSts = 1;    //  Service finish;
  
  return( 0 );
}

/*:: op_dump_info_ax -----------------*/
int  op_dump_info_ax( )
{
  int   ax;
  FILE  *fout;
  char  szFileName[MAX_PATH], time_str[20];
  
  EC_T_CFG_SLAVE_INFO* pMySlave = NULL;

  ax = *(int *)p_McmdParm1;    
  *p_McmdRetSts = 0;
 
  if  ( ax < 0 || ax > MAX_NUM_SLAVES )  ax = 0;

  #if  ( _MOT_CE600_ == 1 )
    GetLocalTime( &sysHalCurrTime );   
	if (g_bNewLogDirectort)													// 2026/07/06
		strcpy( szFileName, "\\LOG\\ecat_info_" );
	else
		strcpy( szFileName, "\\HDD\\ecat_info_" );
    sprintf( time_str, "%02uX%02uX%02u",
             sysHalCurrTime.wHour, sysHalCurrTime.wMinute, sysHalCurrTime.wSecond );
    strcat( szFileName, time_str );
  #else
  strcpy( szFileName, "d:\\ecat_info_" );
  _strtime( time_str );
  time_str[2] = time_str[5] = 'x';
  strcat( szFileName, time_str );
  strcat( szFileName, "_" );
  _strdate( time_str );
  time_str[2] = time_str[5] = 'x';
  strcat( szFileName, time_str );
  #endif
  strcat( szFileName, ".log" );
  fout = fopen( szFileName, "w" );
  if  ( fout == NULL )
      {
      //  RtPrintf("op_dump_info_ax(): file open failed!!! %s \n", szFileName );
      *p_McmdRetSts = (DWORD)-1;
      return( 0 );
      }
  
  pMySlave = (EC_T_CFG_SLAVE_INFO*)my_aSlaveList[ax];

  fprintf( fout, " EC_T_CFG_SLAVE_INFO [%2u]:  size = %u Bytes ==== \n",
           (unsigned)ax, (unsigned)sizeof(EC_T_CFG_SLAVE_INFO) );
  fprintf( fout,"\n\n  =========  Data Start  ========== \n" );
  fprintf( fout," Slave ID      : %u \n", pMySlave->dwSlaveId );        // DWORD;
  fprintf( fout," Slave name    : %s \n", pMySlave->abyDeviceName );    // char [];
  fprintf( fout," HCGroupIdx    : %u \n", pMySlave->dwHCGroupIdx );     // DWORD;  
  fprintf( fout," IsPresent     : %u, %u \n",
           (DWORD)pMySlave->bIsPresent, (DWORD)pMySlave->bIsHCGroupPresent );  
  fprintf( fout," Vendor ID     : %u \n", pMySlave->dwVendorId );   
  fprintf( fout," ProductCode   : %u \n", pMySlave->dwProductCode );   
  fprintf( fout," Revision num. : %u \n", pMySlave->dwRevisionNumber );  
  fprintf( fout," Serial num.   : %u \n", pMySlave->dwSerialNumber );  
  fprintf( fout," Station addr. : %u \n", (DWORD)pMySlave->wStationAddress );  
  fprintf( fout," AutoInc addr. : %u \n", (DWORD)pMySlave->wAutoIncAddress );   
  fprintf( fout," PdIn offset   : 0x%08x \n", pMySlave->dwPdOffsIn );   
  fprintf( fout," PdIn size     : %u \n", pMySlave->dwPdSizeIn );   
  fprintf( fout," PdOut offset  : 0x%08x \n", pMySlave->dwPdOffsOut );   
  fprintf( fout," PdOut size    : %u \n", pMySlave->dwPdSizeOut );   
  fprintf( fout," PdIn2 offset  : 0x%08x \n", pMySlave->dwPdOffsIn2 );   
  fprintf( fout," PdIn2 size    : %u \n", pMySlave->dwPdSizeIn2 );   
  fprintf( fout," PdOut2 offset : 0x%08x \n", pMySlave->dwPdOffsOut2 );   
  fprintf( fout," PdOut2 size   : %u \n", pMySlave->dwPdSizeOut2 );   
  fprintf( fout," PdIn3 offset  : 0x%08x \n", pMySlave->dwPdOffsIn3 );   
  fprintf( fout," PdIn3 size    : %u \n", pMySlave->dwPdSizeIn3 );   
  fprintf( fout," PdOut3 offset : 0x%08x \n", pMySlave->dwPdOffsOut3 );   
  fprintf( fout," PdOut3 size   : %u \n", pMySlave->dwPdSizeOut3 );   
  fprintf( fout," PdIn4 offset  : 0x%08x \n", pMySlave->dwPdOffsIn4 );   
  fprintf( fout," PdIn4 size    : %u \n", pMySlave->dwPdSizeIn4 );   
  fprintf( fout," PdOut4 offset : 0x%08x \n", pMySlave->dwPdOffsOut4 );   
  fprintf( fout," PdOut4 size   : %u \n", pMySlave->dwPdSizeOut4 );   
  fprintf( fout," Mbx protocols : %u \n", pMySlave->dwMbxSupportedProtocols );   
  fprintf( fout," Mbx out_size  : %u \n", pMySlave->dwMbxOutSize );   
  fprintf( fout," Mbx in_size   : %u \n", pMySlave->dwMbxInSize );   
  fprintf( fout," Mbx out2_size : %u \n", pMySlave->dwMbxOutSize2 );   
  fprintf( fout," Mbx in2_size  : %u \n", pMySlave->dwMbxInSize2 );   
  fprintf( fout," IsDcSupport   : %u \n", (DWORD)pMySlave->bDcSupport );   
  fprintf( fout," NumProcessVarsInp  : %u \n", (DWORD)pMySlave->wNumProcessVarsInp );   
  fprintf( fout," NumProcessVarsOutp : %u \n", (DWORD)pMySlave->wNumProcessVarsOutp );   
  fprintf( fout," PrevStationAddress : %u \n", (DWORD)pMySlave->wPrevStationAddress );   
  fprintf( fout," PrevSlavePort      : %u \n", (DWORD)pMySlave->wPrevPort );   
  fprintf( fout," IdentifyAdo        : %u \n", (DWORD)pMySlave->wIdentifyAdo );   
  fprintf( fout," IdentifyData       : %u \n", (DWORD)pMySlave->wIdentifyData );   
  fprintf( fout," PortDescriptor     : %u \n", (DWORD)pMySlave->byPortDescriptor );   
  fprintf( fout," WkcStateOffsIn[]   : %u, %u, %u, %u \n",
          (DWORD)pMySlave->wWkcStateDiagOffsIn[0],
          (DWORD)pMySlave->wWkcStateDiagOffsIn[1],
          (DWORD)pMySlave->wWkcStateDiagOffsIn[2],
          (DWORD)pMySlave->wWkcStateDiagOffsIn[3] );   
  fprintf( fout," WkcStateOffsOut[]  : %u, %u, %u, %u \n",
          (DWORD)pMySlave->wWkcStateDiagOffsOut[0],
          (DWORD)pMySlave->wWkcStateDiagOffsOut[1],
          (DWORD)pMySlave->wWkcStateDiagOffsOut[2],
          (DWORD)pMySlave->wWkcStateDiagOffsOut[3] );   
  fprintf( fout, "\n\n  =========  Data end  ========== \n ");
  fflush( fout );
  fclose( fout );
    
  *p_McmdRetSts = 1;    //  Service finish;
  
  return( 0 );
}

/*:: oprtss_thread ===================*/
DWORD WINAPI oprtss_thread(LPVOID pp)
{
  USHORT    uCmd;

  while ( dwOpRtssActiveF == 1 )
      {  //  every 12ms.;
      RtWaitForSingleObject( hOpRtssEvent, INFINITE);
      uCmd = (USHORT)*p_McmdComd;
      if  ( uCmd != 0 )
          log_ecat_opcmd_info( );
      switch ( uCmd )
          {
          case _SDO_READ_RQ_ :  
              op_sdo_read_request( );
              break;
          case _SDO_WRITE_RQ_ :  
              op_sdo_write_request( );
              break;
          case _PROBE_FUNC_RQ_ :  
              op_probe_func_request( );
              break;
          case _HRMB_START_RQ_ :  
              op_hrmb_start_request( );
              break;
          case _HRMB_READ_RQ_ :  
              op_hrmb_read_request( );
              break;
          case _HRMB_WRITE_RQ_ :  
              op_hrmb_write_request( );
              break;
          case _DUMP_INFO_AX_ :  
              op_dump_info_ax( );
              break;
          case _SAVE_ECAT_INFO_ :  //  08.05.06;  
              SaveEcatOpData();
              SaveEcatCommData();
              break;
          case _OPRT_CLOSE_ :
              break;
          }  
      oprtss_set_info_out();    //  07.02.13;
      *p_McmdComd = 0;
      }

  return( 0 );
}

/*:: hrmb_initial_proc ---------------*/
int  hrmb_initial_proc()
{    //  07.02.13;
  //  int   i;
  
  if  ( dwHalRmbStartF == 0 )  return( -1 );
  if  ( dwHRmbNum == 0 )
      {
      dwHalRmbStartF = 0;
      return( -2 );
      }
    
  dwHrmbInitCnt++;
  if  ( dwHrmbInitCnt == 1 )
      {
      //  wHRmbActiveNum = 0x03;    //  07.09.21, ch_0 & Ch_1, enable;
      HAL_ECAT_SDODownload( dwHalRmbSAddr, 0x2006, 0,
                 (BYTE *)&wHRmbActiveNum, 2, MBX_TIMEOUT, 0 );   
      }

  if  ( dwHrmbInitCnt >= 10 )
      {
      //  07.09.21, wHRmbActiveNum 不是個數;
      //  for ( i = 0; i < wHRmbActiveNum; i++)
      //      dwHRmbAuToCtrl[i] = 1;      
      dwHalRmbStartF = 0;   //  Finish!!;
      wHRmbInitFlag = 1;
      }
  
  return( 0 );
}

/*:: mot_r0_thread ===================*/
DWORD WINAPI mot_r0_thread(LPVOID pp)
{
  while ( dwMotR0ActiveF == 1 )
      {
      RtWaitForSingleObject( hEcatR0Event, INFINITE);
      dwMotR0Count++;
      mot_r0_process( );
      }

  return( 0 );
}

/*:: mot_r3_thread ===================*/
//  執行週期: 10ms;
DWORD WINAPI mot_r3_thread(LPVOID pp)
{
  while ( dwMotR3ActiveF == 1 )
      {
      RtWaitForSingleObject( hEcat4msEvent, INFINITE);
      dwMotR3Count++;
      //  07.02.07;
      if  ( dwEcatMasterState == eEcatState_OP && 
            dwHalRmbBase == 0 )
          {
          mot_get_HalRmb_offset();
          }
      mot_r3_process( );
      if  ( wHRmbInitFlag != 1 )
          hrmb_initial_proc( );      
      }

  return( 0 );
}

//==========================================================
//  

#include "SlaveInfo.h"

EC_T_CFG_SLAVE_INFO      S_MotSlaveList[MAX_NUM_SLAVES];

DWORD       dwAppFoundSlaves = 0;
DWORD       dwMotAppCount = 0;
//  10.10.20;
DWORD       dwMotAppBusyCount = 0, dwMotAppBusyFlag = 0;
HANDLE      hMasterThd;   

EC_T_BYTE   YasInBuff[MAX_NUM_SLAVES][64], YasOutBuff[MAX_NUM_SLAVES][40];
EC_T_INT    iEncPos[MAX_NUM_SLAVES], iFolwErrVal[MAX_NUM_SLAVES];
EC_T_INT    iCmdPos[MAX_NUM_SLAVES], iVelCmd[MAX_NUM_SLAVES];
EC_T_SHORT  shTorqVal[MAX_NUM_SLAVES], shTorqCmd[MAX_NUM_SLAVES];
EC_T_WORD   wSlavSts[MAX_NUM_SLAVES], wCtrlCmd[MAX_NUM_SLAVES], wTorqMax[MAX_NUM_SLAVES];  
EC_T_WORD   wTouchFunc[MAX_NUM_SLAVES], wErrCode[MAX_NUM_SLAVES];  
EC_T_BYTE   bModeWrite[MAX_NUM_SLAVES], bModeRead[MAX_NUM_SLAVES];
EC_T_SHORT  shVffCmd[MAX_NUM_SLAVES];   //  11.12.09;
EC_T_INT    iVelRdVal[MAX_NUM_SLAVES];  //  13.09.03;

/*:: MotAppInit ----------------------*/
void MotAppInit( )
{
  int  i;
  
  RtPrintf("VVVVV: MotAppInit().. [%s] \n", szDateMsg);
  dwAppFoundSlaves = 0;
  dwYasFoundSlaves = 0;
  ZeroMemory( S_MotSlaveList, MAX_NUM_SLAVES*sizeof(EC_T_CFG_SLAVE_INFO));
  dwEcatMasterState = 0;
  
  pbyEcatPDIn = pbyEcatPDOut = EC_NULL;
  for ( i = 0; i < MAX_NUM_SLAVES; i++ )
      my_aSlaveList[i] = (void *)&S_MotSlaveList[i];

  #if  ( _MOT_NT400_ == 1 )
  if  ( !LoadEcatAPI() )
      {
      RtPrintf( "Error: LoadLibrary() failed!!! [LoadEcatAPI] " );
      }
  #endif

  // 2026/07/06, New Log Directory, Ulysses
  wchar_t chNewLogPath[32] = L"\\LOG";											// 2026/07/06
  g_bNewLogDirectort = IsDirectoryExist(chNewLogPath);							// 2026/07/06
      
  oprtss_init();
}

/*:: MotAppClose ---------------------*/
void MotAppClose( )
{
  RtPrintf("VVVVV: MotAppClose().. [%s] \n", szDateMsg);
  
  #if  ( _MOT_NT400_ == 1 )
  UnLoadEcatAPI();  //  07.01.11;
  #endif
  
  oprtss_close();
}

/*:: MotAppPrepare -------------------*/
void MotAppPrepare( void **pSlaveList, DWORD SlaveNum )
{
  DWORD     dwIdx, dwTmp;
  WORD      wStation, wAutoInc;
  //  int       iNum;
  
  EC_T_CFG_SLAVE_INFO* pMySlave = NULL;
  //  DWORD     dwRes;
  
  RtPrintf("VVVVV: MotAppPrepare()..  \n");

  //  Searching for: SGDV-E1;
  //  dwSlaves = emGetNumConnectedSlaves(INSTANCE_MASTER_DEFAULT);
  RtPrintf("     : dwSlaves = %lu;  \n", SlaveNum);

  dwYasFoundSlaves = SlaveNum;
  
  for (dwIdx = 0; dwIdx < MAX_NUM_SLAVES; dwIdx++)
      {
      //  my_aSlaveList[dwIdx] = pSlaveList[dwIdx];
      wSlaveType[dwIdx] = 0;
      if  ( dwIdx < SlaveNum )
          {
          my_aSlaveList[dwIdx] = pSlaveList[dwIdx];
          pMySlave = (EC_T_CFG_SLAVE_INFO *)my_aSlaveList[dwIdx];
          wStation = pMySlave->wStationAddress;
          wAutoInc = pMySlave->wAutoIncAddress;
          if  ( pMySlave->dwVendorId == ecvendor_shihlin_electric )
              {  //  13.03.06;
              if  ( pMySlave->dwProductCode == 0x010c000d )
                  {
                  wSlaveType[dwIdx] = 16;   //  SA3_EC301;
                  ec301_set_init( (int)dwIdx );
                  }
              else
                  wSlaveType[dwIdx] = 2;
              }
          else if  ( pMySlave->dwVendorId == ecvendor_hal )
              {  //  09.02.26;
              if  ( pMySlave->dwProductCode == ecprodcode_hal_rmb )
                  wSlaveType[dwIdx] = 6;        //  ecprodcode_hal_rmb;
              else
                  wSlaveType[dwIdx] = 3;        //  ecprodcode_hal_cncio;
              }
          else if  ( pMySlave->dwVendorId == ecvendor_panasonic )
              wSlaveType[dwIdx] = 4;
          else if  ( pMySlave->dwVendorId == ecvendor_maxsine )
              wSlaveType[dwIdx] = 5;        //  08.03.20;
          else if  ( pMySlave->dwVendorId == 0x2e1 )
              wSlaveType[dwIdx] = 8;        //  CDHD, Servotronix, 10.03.08;
          else if  ( pMySlave->dwVendorId == 0x6b5 )
              wSlaveType[dwIdx] = 9;        //  GSHD_EC, 固高, 10.07.15;
          else if  ( pMySlave->dwVendorId == 0x00504D43 )
              wSlaveType[dwIdx] = 10;       //  Physis, 菲仕, 11.01.26;
          else if  ( pMySlave->dwVendorId == 0x00D9 )
              wSlaveType[dwIdx] = 11;       //  Phase, 菲仕_2, 11.01.27;
          else if  ( pMySlave->dwVendorId == 0x00F9 )
              wSlaveType[dwIdx] = 12;       //  Nidec, 11.06.21;
          else if  ( pMySlave->dwVendorId == 0x09ED )
              wSlaveType[dwIdx] = 17;       //  Beijing CTB, 13.08.16;
          else if  ( pMySlave->dwVendorId == 0x116c7 )
              wSlaveType[dwIdx] = 18;       //  HCFA, 禾川, 14.12.23;
          else if  ( pMySlave->dwVendorId == 0x200000 )
              wSlaveType[dwIdx] = 19;       //  ACS880, 弘遠, 15.01.15;
		  else if  ( pMySlave->dwVendorId == 0x09CF )
              wSlaveType[dwIdx] = 20;       //  US800-C06, 眾辰, 15.06.04;
          else if  ( pMySlave->dwVendorId == 0x0800005A )
              {
              dwTmp = pMySlave->dwProductCode;
              if  ( dwTmp == 0x0389 )
                  wSlaveType[dwIdx] = 14;       //  Schneider, [ATV320], 12.07.20;
              else
                  wSlaveType[dwIdx] = 13;       //  Schneider, 施耐德, 11.11.22;
              }
          else if  ( pMySlave->dwVendorId == 0x003b )
              wSlaveType[dwIdx] = 15;       //  Lenze, 12.12.14, 13.10.18;
          else
              wSlaveType[dwIdx] = 1;
#if WOLONG_IDENTIFICATION_ENABLED
          if (pMySlave->dwVendorId == WOLONG_VENDOR_ID &&
              pMySlave->dwProductCode == WOLONG_PRODUCT_CODE)
              wSlaveType[dwIdx] = TYPE_WOLONG;
#endif
          RtPrintf("     :  ( %lu, %lu, %lu, %lu );  \n",
                   dwIdx, (DWORD)wStation, (DWORD)wAutoInc, (DWORD)wSlaveType[dwIdx] );          
          }
      }
  p_McmdSysInfo[8] = dwYasFoundSlaves;
  //  07.03.19;

  for (dwIdx = 0; dwIdx < dwYasFoundSlaves; dwIdx++)
  {
      p_McmdBlkOut[160+dwIdx] = (DWORD)wSlaveType[dwIdx];

	  if (dwIdx < (int)dwYasFoundSlaves - 2)			// 2026/07/29
		  Type20ParamService(dwIdx);					// 2026/07/29, Ulysses
  }
}

/*:: MotAppWorkpd --------------------*/
void MotAppWorkpd( BYTE *pbyPDIn, BYTE *pbyPDOut, DWORD dwState )
{
  //  int   i;
  //  WORD  wByOffset, *pwVal;
  
  //  EC_T_CFG_SLAVE_INFO* pMySlave = EC_NULL;

  if  ( dwMotAppCount == 0 )
      RtPrintf("VVVVV: MotAppWorkpd()..  \n");
  dwMotAppCount++;
  //  10.10.20, 避免[重覆進入];
  if  ( dwMotAppBusyFlag != 0 )
      {
      dwMotAppBusyCount++;
      return;
      }
  dwMotAppBusyFlag = 1;
  
  //  pwVal    = EC_NULL;
  pbyEcatPDIn  = pbyPDIn;
  pbyEcatPDOut = pbyPDOut;
  dwEcatMasterState = dwState;
  //  10.10.13, 10.10.07;
  #if  (_NO_ECAT_R0_THREAD_ == 1 )
    if  ( dwMotAppCount < 50 )
        {
        if  ( dwMotAppCount == 1 )
            hMasterThd = GetCurrentThread();
        //  iCurrThdPri = CeGetThreadPriority( GetCurrentThread() );
        iCurrThdPri = CeGetThreadPriority( hMasterThd );
        if  ( iLstThdPri != iCurrThdPri )
            {
            iPreThdPri = iLstThdPri;
            iLstThdPri = iCurrThdPri;
            } 
        //  14.07.22;
        //  if  ( dwMotAppCount == 40 )
        //      CeSetThreadPriority( hMasterThd, 1 );
        }
    mot_r0_process();
  #else  
  if  ( dwMotAppCount < 50 )
      {
      if  ( dwMotAppCount == 1 )
          hMasterThd = GetCurrentThread();
      if  ( dwMotAppCount == 40 )
          CeSetThreadPriority( hMasterThd, 1 );
      if  ( hMasterThd != NULL )
          iCurrThdPri = CeGetThreadPriority( hMasterThd );
      if  ( iLstThdPri != iCurrThdPri )
          {
          iPreThdPri = iLstThdPri;
          iLstThdPri = iCurrThdPri;
          } 
      }
  //  Trig Event;
  //  if  ( (dwMotAppCount % 4) == 3 )
  //  RtSetEvent( hEcatR3fEvent );
  RtSetEvent( hEcatR0Event );
  #endif
  //  07.10.26, _R3_CMD_NUM_ == 10;
  if  ( (dwMotAppCount % 10) == 9 )
      RtSetEvent( hEcat4msEvent );

  dwMotAppBusyFlag = 0;     //  10.10.20;
}

/*:: mot_r0_process ------------------*/
int  mot_r0_process()
{
  int   i;
  WORD  wByOffset, *pwVal;
  
  EC_T_CFG_SLAVE_INFO* pMySlave = EC_NULL;

  pwVal = EC_NULL;
  if  ( dwEcatMasterState == eEcatState_OP )
      {
      if  ( (dwYasFoundSlaves != 0) && (pbyEcatPDIn != EC_NULL) && (pbyEcatPDOut != EC_NULL) )
          {
          /* Validate all selected Wolong layouts BEFORE any image copy. */
          for (i = 0; i < (int)dwYasFoundSlaves; i++)
              {
              if (wSlaveType[i] == TYPE_WOLONG)
                  {
                  pMySlave = (EC_T_CFG_SLAVE_INFO *)my_aSlaveList[i];
                  if (!WolongLayoutMatches(pMySlave->dwPdOffsIn,
                                           pMySlave->dwPdSizeIn,
                                           pMySlave->dwPdOffsOut,
                                           pMySlave->dwPdSizeOut))
                      {
                      static int lastBadAxis = -1;
                      if (lastBadAxis != i)
                          RtPrintf("Wolong PDO layout mismatch: axis=%d, in=%lu bits, out=%lu bits\n",
                                   i, pMySlave->dwPdSizeIn, pMySlave->dwPdSizeOut);
                      lastBadAxis = i;
                      /* Abort this cycle; not a safety-rated stop mechanism. */
                      return(-2);
                      }
                  }
              }
          oprtss_ptime_start( 1 );
          for ( i = 0; i < (int)dwYasFoundSlaves; i++ )
              {
              // READ -----  Tx;
              //  pMySlave = &S_MotSlaveList[i];    
              pMySlave = (EC_T_CFG_SLAVE_INFO *)my_aSlaveList[i];    
              wByOffset = pMySlave->dwPdOffsIn/8;                
              //  08.03.20, 08.01.07, 08.01.02, 07.12.26;
              //  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 20);
              if (wSlaveType[i] == TYPE_WOLONG)
                  memcpy(&YasInBuff[i][0], &pbyEcatPDIn[wByOffset], WOLONG_TX_BYTES);
              else if  ( wSlaveType[i] == 2 )
                  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 25);   //  12.11.28, 21 -> 25;
              else if  ( wSlaveType[i] == 3 )
                  {  //  08.05.24, CNC-IO, 讀取兩軸, 54 = 27*2;
                  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 54);
                  }
              else if  ( wSlaveType[i] == 4 )
                  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 25);   //  09.04.14;
                  //  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 23);
              else if  ( wSlaveType[i] == 5 )
                  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 25);
              else if  ( wSlaveType[i] == 8 )
                  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 23);   //  10.07.29;
                  //  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 33);   //  10.03.08;
              else if  ( wSlaveType[i] == 9 )
                  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 29);   //  10.10.20;
              else if  ( wSlaveType[i] == 10 || wSlaveType[i] == 11 )
                  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 23);   //  12.11.28, 11.03.01;
              else if  ( wSlaveType[i] == 12 )
                  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 17);   //  11.06.21;
              else if  ( wSlaveType[i] == 13 )
                  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 17);   //  11.11.22;
              else if  ( wSlaveType[i] == 14 )
                  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 4);    //  12.07.20;
              else if  ( wSlaveType[i] == 15 )
                  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 6);    //  13.10.18, 12.12.14;
              else if  ( wSlaveType[i] == 16 )
                  {
                  //  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 13);   //  13.03.06;
                  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 12);   //  13.03.07, 13.03.14;
                  }
              else if  ( wSlaveType[i] == 17 )
                  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 15);   //  13.09.03;
              else if  ( wSlaveType[i] == 18 )
                  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 25);   //  14.12.23;
              else if  ( wSlaveType[i] == 19 )
                  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 26);   //  15.01.15;
              else if  ( wSlaveType[i] == 6 )
                  {  //  09.02.26, CNC-RMB, 讀取兩軸, 44 = 22*2;
                  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 44);
                  }
			  else if  ( wSlaveType[i] == 20 )
                  {  //  15.06.04, US800-C06, 眾辰
					  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 26);
			      }
              else
                  memcpy( &YasInBuff[i][0], &pbyEcatPDIn[wByOffset], 26);
              //  15.01.15, 14.12.23, 08.01.07;
              //  if  ( wSlaveType[i] == 4 )
              if  ( wSlaveType[i] == 4 || wSlaveType[i] == 18 || wSlaveType[i] == TYPE_WOLONG )
                  wSlavSts[i] = *(EC_T_WORD *)&YasInBuff[i][2];
              else if  ( wSlaveType[i] == 19 )
                  wSlavSts[i] = *(EC_T_WORD *)&YasInBuff[i][22];
              else
                  wSlavSts[i] = *(EC_T_WORD *)&pbyEcatPDIn[wByOffset];
              //  08.01.02;
              if  ( wSlaveType[i] == 3 )
                  {  //  08.05.24, CNC-IO, 先讀取兩軸, 34 = 17*2;
                  wByOffset = pMySlave->dwPdOffsOut/8;                
                  memcpy( &YasOutBuff[i][0], &pbyEcatPDOut[wByOffset], 34);
                  }              
              //  10.07.29;
              if  ( wSlaveType[i] == 8 )
                  {  //  先保留舊值;
                  wByOffset = pMySlave->dwPdOffsOut/8;                
                  memcpy( &YasOutBuff[i][0], &pbyEcatPDOut[wByOffset], 17);
                  }              
              //  10.10.20, 10.07.15, 10.03.17;
              if  ( wSlaveType[i] == 9 )
                  {  //  先保留舊值;
                  wByOffset = pMySlave->dwPdOffsOut/8;                
                  memcpy( &YasOutBuff[i][0], &pbyEcatPDOut[wByOffset], 21);
                  }              
              }
          //  09.02.26, 處理 Re-connect;
          if  ( dwMotEcatStatus == 0x8001 )
              {
              dwMotEcatRdyFlag = 0x00;;
              for ( i = 0; i < (int)dwYasFoundSlaves; i++ )
                  {
                  dwAuToCtrlCmd[i] = 1;
                  wCurrPhase[i] = _NOT_READY_SON_;
                  wCtrlCmd[i] = 0x06;
                  }
              dwMotEcatStatus = 0;
              }
          oprtss_process();
          //
          for ( i = 0; i < (int)dwYasFoundSlaves; i++ )
              {
              //  WRITE ---- Rx;
              //  pMySlave = &S_MotSlaveList[i];    
              pMySlave = (EC_T_CFG_SLAVE_INFO *)my_aSlaveList[i];    
              wByOffset = pMySlave->dwPdOffsOut/8;                
              //  13.05.27, 08.06.25, 08.05.24, 08.03.20;
              if (wSlaveType[i] == TYPE_WOLONG)
                  memcpy(&pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], WOLONG_RX_BYTES);
              else if  ( wSlaveType[i] == 3 )
                  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 13);  //  不覆寫 dwDOutVel[];
                  //  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 17);
                  //  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 34);
              else if  ( wSlaveType[i] == 4 )
                  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 15);  //  09.04.14;
                  //  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 9);
              else if  ( wSlaveType[i] == 5 )
                  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 17);
              else if  ( wSlaveType[i] == 8 )
                  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 23);  //  10.03.08, 10.07.29;
              else if  ( wSlaveType[i] == 9 )
                  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 21);  //  10.10.20;
              else if  ( wSlaveType[i] == 10 || wSlaveType[i] == 11 )
                  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 13);  //  11.01.27;
              else if  ( wSlaveType[i] == 12 )
                  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 13);  //  11.06.21;
              else if  ( wSlaveType[i] == 13 )
                  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 15);  //  11.11.22;
              else if  ( wSlaveType[i] == 14 )
                  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 4);   //  12.07.20;
              else if  ( wSlaveType[i] == 15 )
                  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 4);   //  13.10.18;
              else if  ( wSlaveType[i] == 16 )
                  {
                  //  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 33);  //  13.03.06;
                  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 10);  //  13.03.14;
                  }
              else if  ( wSlaveType[i] == 17 )
                  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 11);   //  13.09.03;
              else if  ( wSlaveType[i] == 18 )
                  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 21);   //  14.12.23;
              else if  ( wSlaveType[i] == 19 )
                  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 20);   //  15.01.15;
              else if  ( wSlaveType[i] == 6 )
                  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 16);
			  else if  ( wSlaveType[i] == 20 )	
				  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 22);   //  15.06.04, US800-C06, 眾辰, Add 60B1 VFF (4)
              else
                  memcpy( &pbyEcatPDOut[wByOffset], &YasOutBuff[i][0], 18);
              pwVal = (EC_T_WORD *)&pbyEcatPDOut[wByOffset];
              //  15.01.15;
              if  ( wSlaveType[i] != 19 )
                  *pwVal = wCtrlCmd[i];
              }
          //  07.07.24;
          if  ( dwMotEcatStatus == 0 )
              dwMotEcatStatus = 1;
          oprtss_ptime_end( 1 );
          }
      }    
  else
      {  //  07.07.24;
      if  ( dwMotEcatStatus == 1 )
          dwMotEcatStatus = 0x8001;
      }
  
  //  Trig Event;
  RtSetEvent( hEcatR3fEvent );
  
  return( 0 );
}

//==========================================================
//  

int     iEcatFOutId=0;

/*:: CheckEcatFOutId -----------------*/
void CheckEcatFOutId( )
{   
  FILE  *fout;
  int   tint1, tint2, tint3;
  char  tTmpString[110], fname[50];
  
  fout = NULL;
  #if  ( _MOT_CE600_ == 1 )
    if (g_bNewLogDirectort)													// 2026/07/06
		strcpy( fname, "\\LOG\\EcatInf00.cst" );
	else
		strcpy( fname, "\\HDD\\HURCO\\EcatInf00.cst" );
  #else
  strcpy( fname, "d:\\hurco\\EcatInf00.cst" );
  #endif
  fout = fopen( fname, "r" );

  tint1 = tint2 = tint3 = 0;
  if  ( fout != NULL )
      {
      fgets( tTmpString, 100, fout);
      sscanf( tTmpString," %d %d %d ",
              &tint1, &tint2, &tint3 );
      if  ( tint1 < 0 || tint1 >= 6 )  tint1 = 0;
      iEcatFOutId = tint1+1;
      tint3 = tint2;
      tint2 = tint1;
      tint1 = iEcatFOutId;
      fclose( fout ); 
      }
  else
      {
      tint1 = iEcatFOutId = 1;
      }
  
  fout = fopen( fname, "w" );
  fprintf( fout,"   %6d   %6d   %6d \n", 
          tint1, tint2, tint3 );
  fclose( fout ); 
}

/*:: SaveEcatOpData ------------------*/
void SaveEcatOpData( )
{    
  FILE  *fout;
  int   i, idx;
  char  fname[60], fname2[30];

  if  ( iEcatFOutId <= 0 )
      CheckEcatFOutId();
  if  ( dwEcatOpCnt <= 0 )  return;
  
  #if  ( _MOT_CE600_ == 1 )
    if (g_bNewLogDirectort)													// 2026/07/06
		strcpy( fname, "\\LOG\\" );								
	else
		strcpy( fname, "\\HDD\\HURCO\\" );
  #else
  strcpy( fname, "d:\\hurco\\" );
  #endif
  sprintf( fname2, "EcatOpDt%02u.cst", (DWORD)iEcatFOutId );
  strcat( fname, fname2 );
  fout = fopen( fname, "w" );

  fprintf( fout,"\n Total cnt.: [%10lu, %10lu,  %6lu] 0x%08x, %s  \n\n",
           dwEcatOpCnt, dwEcatOpIdx, dwMotApiLoadFlag, dwVersionMark, szDateMsg );
  idx = 0;
  if  ( dwEcatOpCnt >= _ECAT_MAX_OP_ )
      idx = dwEcatOpIdx;
  fprintf(fout,"   idx   OpCmdId    Param1    Param2    Param3    Param4   Proc_cnt \n");
  for ( i = 0; i < (int)dwEcatOpCnt; i++ )
      {
      fprintf(fout," %5d", i );
      fprintf(fout,"    0x%04x", iEcatOpBuff[idx][0] );
      fprintf(fout," %9ld", iEcatOpBuff[idx][1] );
      fprintf(fout," %9ld", iEcatOpBuff[idx][2] );
      fprintf(fout," %9ld", iEcatOpBuff[idx][3] );
      fprintf(fout," %9ld", iEcatOpBuff[idx][4] );
      fprintf(fout,"  %9ld", iEcatOpBuff[idx][5] );
      fprintf(fout, "\n" );
      idx++;
      }
      
  fprintf( fout, "\n  =========  Data end  ========== \n ");
  fclose( fout ); 
}

/*:: SaveEcatCommData ----------------*/
void SaveEcatCommData( )
{    
  FILE  *fout;
  int   i;
  char  fname[60], fname2[30];

  //  06.06.29; 
  if  ( iEcatFOutId <= 0 )
      CheckEcatFOutId();
  if  ( dwEcatCmCnt <= 0 )  return;
  
  #if  ( _MOT_CE600_ == 1 )
    if (g_bNewLogDirectort)													// 2026/07/06
		strcpy( fname, "\\LOG\\" );
	else
		strcpy( fname, "\\HDD\\HURCO\\" );
  #else
  strcpy( fname, "d:\\hurco\\" );
  #endif
  sprintf( fname2, "EcatComm%02u.cst", (DWORD)iEcatFOutId );
  strcat( fname, fname2 );
  fout = fopen( fname, "w" );

  fprintf( fout,"\n Total cnt.: %6lu [ Pri(%4d, %4d) Busy(%6d) ]  %s  \n\n", 
           dwEcatCmCnt, iCurrThdPri, iPreThdPri, dwMotAppBusyCount, szDateMsg );

  fprintf(fout,"   idx  Node CtrlCmd SlavSts   CmdPos.   EncPos.   ");
  fprintf(fout,"TqVal   ModThSts Phase  MtR0_cnt  Proc_cnt  MApp_cnt \n");
  for ( i = 0; i < (int)dwEcatCmCnt; i++ )
      {
      fprintf(fout," %5d", i );
      fprintf(fout," %5d", iEcatCommBuff[i][0] );
      fprintf(fout,"  0x%04x", iEcatCommBuff[i][1] );
      fprintf(fout,"  0x%04x", iEcatCommBuff[i][2] );
      fprintf(fout," %9ld", iEcatCommBuff[i][3] );
      fprintf(fout," %9ld", iEcatCommBuff[i][4] );
      fprintf(fout,"  %6d", iEcatCommBuff[i][5] );
      fprintf(fout," 0x%08x", iEcatCommBuff[i][6] );
      fprintf(fout,"  %4d", iEcatCommBuff[i][7] );
      fprintf(fout," %9ld", iEcatCommBuff[i][8] );
      fprintf(fout," %9ld", iEcatCommBuff[i][9] );
      fprintf(fout," %9ld", iEcatCommBuff[i][10] );
      fprintf(fout," %9ld", iEcatCommBuff[i][11] );
      fprintf(fout, "\n" );
      }
      
  fprintf( fout, "\n  =========  Data end  ========== \n ");

  DebugPrintTypeVFF( fout );			// 2026/07/31, Ulysses

  fclose( fout ); 
}

//======================================================================

void Type20UserVFFInit(int ax, int FilterTime)
{
	double dTau = (double)FilterTime * 0.00001;
	if (dTau < 0.001)
		dTau = 0.006;
	g_dType20VFFExpTau[ax] = exp( -0.001/dTau);
}


void Type20VCmdFilterInit(int ax, int FilterTime)
{
	double dTau = (double)FilterTime * 0.001;
	if (dTau < 0.001)
		dTau = 0.006;
	g_dType20VCmdFExpTau[ax] = exp( -0.001/dTau);
}

#define VFF_DEBUG_LOG	1500
float g_fDebug[5][VFF_DEBUG_LOG];
int g_nIdx = 0;
bool g_bLogStart = false;
void Type20UserVFFDebugLog(int ax, int nPos, double dVFF, double dVFFGain, int nVFFOut)
{
	if (g_nIdx >= VFF_DEBUG_LOG)
		return;
	
	g_fDebug[0][0] = g_bLogStart;
	g_fDebug[0][1] = g_dTypae20VFF[ax];
	g_fDebug[0][2] = g_nType20UserVFFGain[ax];
	g_fDebug[0][3] = dVFFGain;
	g_fDebug[0][4] = g_dType20VFFExpTau[ax];

	if (g_bLogStart)
	{
		g_fDebug[1][g_nIdx] = dwMotAppCount;
		g_fDebug[2][g_nIdx] = nPos;
		g_fDebug[3][g_nIdx] = dVFF;
		g_fDebug[4][g_nIdx] = nVFFOut;
		if (g_nIdx < VFF_DEBUG_LOG)
			g_nIdx++;
	}
}

void DebugPrintTypeVFF(FILE *fout)
{
	int i;
	if (fout)
	{
		fprintf(fout, "=========================\n");
		fprintf(fout, "%d, %f, %f, %f, %f, %f\n", 
			g_nIdx, g_fDebug[0][0], g_fDebug[0][1], g_fDebug[0][2], g_fDebug[0][3], g_fDebug[0][4]);
		for (i=0; i<g_nIdx; i++)
			fprintf(fout, "%.0f, %.3f, %.3f\n", g_fDebug[1][i], g_fDebug[2][i], g_fDebug[3][i]);
		g_nIdx = 0;
	}
}

int Type20UserVFFProcess(int ax, int nPosCmd, int nVFFGain)
{
	double dVFFGain, dVFF, nVFFOut;
	double dDelPos = (double)(nPosCmd - g_nType20PosCmd[ax]);
	if (g_nType20UserVFFEnable[ax] & 0x1 == 0 || nVFFGain == 0 || g_nType20VUnit[ax] != 0)
		return 0;

	if (nVFFGain > 0 || dDelPos > 5 || dDelPos < -5)
		g_bLogStart = true;

	dVFFGain = max((double)(nVFFGain - g_nType20UserVFFGain[ax]) * 0.01, 0);
	dVFF = g_dTypae20VFF[ax] * g_dType20VFFExpTau[ax] + dDelPos * ( 1.0 - g_dType20VFFExpTau[ax]);
	g_nType20PosCmd[ax] = nPosCmd;
	g_dTypae20VFF[ax] = dVFF;
	nVFFOut = (int)(dVFF * 1000 * dVFFGain);
	Type20UserVFFDebugLog(ax, nPosCmd, dVFF, dVFFGain, nVFFOut);

	return nVFFOut;
}

int Type20VCmdFilterProcess(int ax, int nVCmd)
{
	double dVFOut;
	
	dVFOut = g_dTypae20VCmdF[ax] * g_dType20VCmdFExpTau[ax] + (double)nVCmd * ( 1.0 - g_dType20VCmdFExpTau[ax]);
	g_nType20VCmd[ax] = nVCmd;
	g_dTypae20VCmdF[ax] = dVFOut;
	
	return (int)dVFOut;
}

//======================================================================
int Type20ParamService(int ax)
{
	int iRet0 = 0, iRet1 = 0;
	WORD   wObIndex;
	BYTE   byObSubIndex;
	DWORD  dwTmp, dwDataR, dwDataW, dwDataLen = 4, dwParamSize;

	DWORD  MySlaveAddr;
	EC_T_CFG_SLAVE_INFO* pMySlave = NULL;

	RtPrintf("[%d] Type 20 Param Service: %d\n", ax, wSlaveType[ax]);
	if (wSlaveType[ax] != 20)
		return (-1);

	RtPrintf("[%d] Type 20 Param Service: %d\n", ax, wSlaveType[ax]);

	pMySlave = (EC_T_CFG_SLAVE_INFO*)my_aSlaveList[ax];
	MySlaveAddr = (DWORD)pMySlave->wStationAddress;

	// Read Pn526: 0x3005, subindex 27, Speed Unit: 0 (PPS), 1 (PPM), 2 (%), 3 (RPM)
	wObIndex = 0x3005;
	byObSubIndex = 27;
	iRet0 =  HAL_ECAT_SDOUpload( MySlaveAddr, wObIndex, byObSubIndex,
                     (BYTE *)&dwDataR, dwDataLen, (DWORD *)&dwParamSize, MBX_TIMEOUT, 0 );  
	if ((int)dwDataR != 0)
	{
		dwDataW = 0;		// Set Speed Unit to PPS
		iRet1 =  HAL_ECAT_SDODownload( MySlaveAddr, wObIndex, byObSubIndex,
                     (BYTE *)&dwDataW, dwDataLen, MBX_TIMEOUT, 0 );  

		iRet0 =  HAL_ECAT_SDOUpload( MySlaveAddr, wObIndex, byObSubIndex,
                     (BYTE *)&dwDataR, dwDataLen, (DWORD *)&dwParamSize, MBX_TIMEOUT, 0 );
	}

	g_nType20VUnit[ax] = (int)dwDataR;
	RtPrintf("Type 20 Axis [%d, %x], Spd Unit %x [%d, %d]\n", ax, pMySlave, dwDataR, iRet0, iRet1);

	// Read Pn380: 0x3003, subindex 81, Enable / disable: Speed Cmd Offset (User VFF)
	wObIndex = 0x3003;
	byObSubIndex = 81;
	dwDataR = 0;
	iRet0 =  HAL_ECAT_SDOUpload( MySlaveAddr, wObIndex, byObSubIndex,
                     (BYTE *)&dwDataR, dwDataLen, (DWORD *)&dwParamSize, MBX_TIMEOUT, 0 );

	if (dwDataR & 0x1 != 1)
	{
		dwDataW = dwDataR | 0x1;
		iRet1 =  HAL_ECAT_SDOUpload( MySlaveAddr, wObIndex, byObSubIndex,
                     (BYTE *)&dwDataW, dwDataLen, (DWORD *)&dwParamSize, MBX_TIMEOUT, 0 );

		iRet0 =  HAL_ECAT_SDOUpload( MySlaveAddr, wObIndex, byObSubIndex,
                     (BYTE *)&dwDataR, dwDataLen, (DWORD *)&dwParamSize, MBX_TIMEOUT, 0 );
	}

	g_nType20UserVFFEnable[ax] = dwDataR; 

	RtPrintf("Type 20 Axis [%d], User VFF Enable %x [%d, %d]\n", ax, dwDataR, iRet0, iRet1);

	// Read Pn307: 0x3003, subindex 8, VFF Gain
	wObIndex = 0x3003;
	byObSubIndex = 8;
	dwDataR = 0;
	iRet0 =  HAL_ECAT_SDOUpload( MySlaveAddr, wObIndex, byObSubIndex,
                     (BYTE *)&dwDataR, dwDataLen, (DWORD *)&dwParamSize, MBX_TIMEOUT, 0 );

	g_nType20UserVFFGain[ax] = (int)dwDataR;
	RtPrintf("Type 20 Axis [%d], VFF Gain %x [%d]\n", ax, dwDataR, iRet0);

	// Read Pn308: 0x3003, subindex 9, VFF Gain
	wObIndex = 0x3003;
	byObSubIndex = 9;
	dwDataR = 0;
	iRet0 =  HAL_ECAT_SDOUpload( MySlaveAddr, wObIndex, byObSubIndex,
                     (BYTE *)&dwDataR, dwDataLen, (DWORD *)&dwParamSize, MBX_TIMEOUT, 0 );

	g_nType20UserVFFFilter[ax] = (int)dwDataR;
	RtPrintf("Type 20 Axis [%d], VFF Filter %x [%d]\n", ax, dwDataR, iRet0);

	if (g_nType20UserVFFEnable[ax] & 0x1)
		Type20UserVFFInit(ax, g_nType20UserVFFFilter[ax]);
#ifdef TYPE_20_VCMD_FILTER
	Type20VCmdFilterInit(ax, 20);		// 20 ms
#endif
	return (0);
}

//==========================================================
// Defines the entry point for the DLL application.
//
BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved
					 )
{
    return TRUE;
}

