/*
 * se_structs.h
 *
 *  Author: Zihong Ma
 ***************************************************************************
 *  OSIGHT *
 * www.osighttech.com  *
 ***************************************************************************/

#ifndef SEXXX_SE_STRUCTS_H_
#define SEXXX_SE_STRUCTS_H_

#include <stdint.h>

#define BUSPRO_OK              0   
#define BUSPRO_ERROR           -1 
#define BUSPRO_ERROR_CROSS_LINE  -2 
#define BUSPRO_ERROR_POINTER     -3 

typedef	  char                  INT8;
typedef	  short                INT16;
typedef	  int                  INT32;
typedef	  unsigned char        UINT8;
typedef	  unsigned short       UINT16;
typedef	  unsigned int         UINT32;
typedef   long long 	       INT64;
typedef	  unsigned long long   UINT64;
typedef	  char	               CHAR;
typedef	  short                SHORT;
typedef	  long                 LONG;
typedef	  unsigned char	       UCHAR;
typedef   unsigned short	   USHORT;
typedef	  unsigned int	       UINT;
typedef   unsigned long	       ULONG;
typedef   unsigned long long   ULLONG;
typedef   UINT8                BITS8;         
typedef   UINT16               BITS16;       
typedef   UINT32               BITS32;  
typedef   double               DOUBLE;
typedef   float                FLOAT;

/****************************************************************
* Lidar message Id
****************************************************************/
#define USER_PARA_SYNC_REQ                      0x02000001  
#define USER_PARA_SYNC_RSP                      0x02000002 
#define PARA_CHANGED_IND_RSP                    0x02000102  
#define PARA_PARA_CONFIG_REQ                    0x02000201 
#define PARA_PARA_CONFIG_RSP                    0x02000202 
#define PARA_ALARM_CONFIG_REQ                   0x02000301 
#define PARA_ALARM_CONFIG_RSP                   0x02000302  
#define MEASURE_TRANSMISSION_REQ                0x02000401
#define MEASURE_TRANSMISSION_RSP                0x02000402 
#define LOG_GET_REQ                             0x02000501
#define LOG_GET_RSP                             0x02000502  
#define TIME_REPORT_INF_RSP                     0x02000602
#define ACTIVE_FILTER_REQ                       0x02000701 
#define ACTIVE_FILTER_RSP                       0x02000702 
#define SET_CALIBRATION_MODE_REQ                0x02000801 
#define SET_CALIBRATION_MODE_RSP                0x02000802
#define SET_NET_MODE_REQ                        0x02000901  
#define SET_NET_MODE_RSP                        0x02000902 
#define SET_STATIC_IP_REQ                       0x02000A01
#define SET_STATIC_IP_RSP                       0x02000A02 
#define SET_TIME_REQ                            0x02000B01 
#define SET_TIME_RSP                            0x02000B02 
#define DEVELOPER_PARA_CONFIG_REQ               0x02000C01 
#define DEVELOPER_PARA_CONFIG_RSP               0x02000C02 
#define DEVELOPER_SYSTEM_FLAG_REQ               0x02000D01
#define DEVELOPER_SYSTEM_FLAG_RSP               0x02000D02 
#define DEVELOPER_CALIBRATION_PARA_REQ          0x02000E01 
#define DEVELOPER_CALIBRATION_PARA_RSP          0x02000E02  
#define DEVELOPER_REVISE_PARA_REQ               0x02000F01  
#define DEVELOPER_REVISE_PARA_RSP               0x02000F02  
#define DEVELOPER_FPGA_CODE_DISC_PARA_REQ       0x02001001 
#define DEVELOPER_FPGA_CODE_DISC_PARA_RSP       0x02001002 
#define DEVELOPER_FPGA_SUBDIVISION_PARA_REQ     0x02001101  
#define DEVELOPER_FPGA_SUBDIVISION_PARA_RSP     0x02001102 
#define MCU_REGISTER_ONLINE_RSP                 0x02001202  
#define DEVELOPER_PARA_SYNC_REQ                 0x02001301  
#define DEVELOPER_PARA_SYNC_RSP                 0x02001302  
#define DEVELOPER_CODE_ANGLE_REQ                0x02001401 
#define DEVELOPER_CODE_ANGLE_RSP                0x02001402
/****************************************************************
* 
****************************************************************/
#define UPDATE_HANDSHAKE_REQ    0x01000001  
#define UPDATE_HANDSHAKE_RSP    0x01000002 
#define UPDATE_START_DATA_REQ   0x01000101 
#define UPDATE_START_DATA_RSP   0x01000102  
#define UPDATE_DATA_REQ         0x01000201  
#define UPDATE_DATA_RSP         0x01000202  
#define UPDATE_END_DATA_REQ     0x01000301 
#define UPDATE_END_DATA_RSP     0x01000302 

#define NULL_MSGID              0x00000000
//#define NULL_MSGID                     0xA5A5A5A5
/****************************************************************
* 
****************************************************************/
#define FLASH_APP_ADDR_MCU_FLAG      0x00000000 
#define FLASH_APP_ADDR_MCU_LENGTH    0x00000001 
#define FLASH_APP_ADDR_MCU		     0x00000400 
#define FLASH_APP_ADDR_FPGA_FLAG     0x00100000
#define FLASH_APP_ADDR_FPGA_LENGTH   0x00100001 
#define FLASH_APP_ADDR_FPGA          0x00100400 
#define UPDATE_FILE_TYPE_MCU         0x01      
#define UPDATE_FILE_TYPE_FPGA        0x02
#define UPDATE_DATA_ERR_NONE         0xA0
#define UPDATE_DATA_ERR              0xA1
#define UPDATE_DATA_ERR_RELOAD       0xA2
#define UPDATE_PACKET_NO_FIRST       0x01

#define UPDATE_FPGA_SUCCESS          0x88   

/****************************************************************
* 
****************************************************************/
#define ERROR_PARA_VALID                0x0000  
#define ERR_CODE_OK                     0x0000  
#define ERR_CODE_NOT_SUPPORT_RESOLUTION 0x0001  
#define ERR_CODE_NOT_SUPPORT_SPEED      0x0002  
#define ERR_CODE_NOT_SUPPORT_INTENSITY  0x0003 
#define ERR_CODE_INVALID_RESOLUTION     0x0101  
#define ERR_CODE_INVALID_SPEED          0x0102  
#define ERR_CODE_INVALID_INTENSITY      0x0103  
#define ERR_CODE_INVALID_ALARM_ID       0x0104  
#define ERR_CODE_INVALID_ALARM_TYPE     0x0105  
#define ERR_CODE_INVALID_ALARM_PARA     0x0106 
#define ERR_CODE_INVALID_OUTLIER_FLAG   0x0107 
#define ERR_CODE_INVALID_ECHO_FLAG      0x0108 
#define ERR_CODE_INVALID_NET_MODE       0x0109  
#define ERR_CODE_INVALID_NET_IP         0x010A  



#define HTONS(x)    ((((x) & 0x00FF) << 8) | (((x) & 0xFF00) >> 8)) 
#define NTOHS(x)    ((((x) & 0x00FF) << 8) | (((x) & 0xFF00) >> 8)) 

#define HTONL(x)    ((((x) & 0x000000FF) << 24)|(((x) & 0x0000FF00) << 8)|(((x) & 0x00FF0000) >> 8)|(((x) & 0xFF000000) >> 24)) 
#define NTOHL(x)    ((((x) & 0x000000FF) << 24)|(((x) & 0x0000FF00) << 8)|(((x) & 0x00FF0000) >> 8)|(((x) & 0xFF000000) >> 24))

#define PACK_1_BYTE(pucBuff, ucData)   {*pucBuff = ucData; pucBuff++;}
#define PACK_2_BYTE(pucBuff, usData)   {*(BITS16*)pucBuff = HTONS(usData); pucBuff += sizeof(BITS16);}
#define PACK_4_BYTE(pucBuff, ulData)   {*(BITS32*)pucBuff = HTONL(ulData); pucBuff += sizeof(BITS32);}

#define UNPACK_1_BYTE(pucBuff, ucData) {ucData = *pucBuff; pucBuff++;}
#define UNPACK_2_BYTE(pucBuff, usData) {usData = NTOHS(*(BITS16*)pucBuff); pucBuff += sizeof(BITS16);}
#define UNPACK_4_BYTE(pucBuff, ulData) {ulData = NTOHL(*(BITS32*)pucBuff); pucBuff += sizeof(BITS32);}

#define VALUE    (-1)
#define VALUE_0  (0)
#define VALUE_1  (1)
#define VALUE_2  (2)
#define VALUE_3  (3)
#define VALUE_4  (4)
#define VALUE_5  (5)
#define VALUE_6  (6)
#define VALUE_9  (9)
#define VALUE_10 (10)
#define VALUE_15 (15)
#define VALUE_16 (16)

#define BIT_0            0
#define BIT_1            1
#define BIT_2            2
#define BIT_3            3
#define BIT_4            4
#define BIT_5            5
#define BIT_6            6
#define BIT_7            7

#define OFFSET_00        0
#define OFFSET_06        6
#define OFFSET_08        8
#define OFFSET_16        16
#define OFFSET_24        24
#define OFFSET_29        29
#define OFFSET_40        40

/****************************************************************
*
****************************************************************/
#define    PACKAGE_COUNT_00         0
#define    PACKAGE_COUNT_03         3
#define    PACKAGE_COUNT_05         5
#define    PACKAGE_NUM_FULL         216
#define    PACKAGE_NUM_HALF         108

/****************************************************************
*
****************************************************************/
#define    THREAD_BUF_NUM       3
#define    ALARM_AREA_NUM       16
#define    ALARM_PARA_NUM       20
#define    UDP_RX_BUFSIZE       1460
#define    UDP_TX_BUFSIZE       1460
#define    MAX_DATA_INFO_NUM    1080
#define    MAX_DATA_INFO_HALF   540
#define    MAX_LOG_DATA_NUM     1450

#define    MAX_POINT_NUM        2880 
#define    MAX_PACK_POINT_NUM   350  

#define    RX_BUFSIZE       1460
#define    TX_BUFSIZE       1000

BITS8      g_aucRxBuf[RX_BUFSIZE];  
BITS8      g_aucTxBuf[TX_BUFSIZE];  

#define    DEVICE_NUM     200


typedef union tagIntensityUinon
{
    UINT8   ucIntensity;
    UINT16  usIntensity;
}INTENSITY_U;

typedef struct tagPointType
{
    UINT32      ulDistance;   
    INTENSITY_U unIntensity;
}POINT_T;

typedef struct
{
	UCHAR  aucDevIp[4];
	ULONG  ulDevPort;
	UCHAR  aucUpperIp[4];
	ULONG  ulUpperPort;
	UCHAR  aucSubnetMask[4];
	UCHAR  aucGateway[4];
}NET_IP_T;

typedef struct tagSerialNoType
{
	ULONG  ulSerialNo;  
	ULONG  ulProductNo; 
}SERIAL_NO_T;

typedef struct
{
    BITS8        ucIndex ; 
	BITS8        ucSoftwareVersion;   
    BITS8        ucDevNum;           
	BITS8        ucLineNum;         
	BITS32       ulAngularResolution; 
	BITS8        ucIntensityStatus;   
	BITS16       usMaxDistance;      
	BITS16       usPackMeasPointNum;  
	BITS8        ucTotalPackNum;    
	BITS8        ucEcho;             
	INT32        lStartAngle;        
	BITS16       usVerticalAngle;   
}DEVICE_PARA;


typedef struct tagDateType
{
    UINT8    ucYear;
    UINT8    ucMonth;
    UINT8    ucDate;
}RTC_DATE_T;

typedef struct tagTimeType
{
    UINT8    ucHour;
    UINT8    ucMinutes;
    UINT8    ucSeconds;
    UINT32   ulMicroSeconds;
}RTC_TIME_T;

typedef struct tagTimeInfoType
{
    RTC_DATE_T stDate;
    RTC_TIME_T stTime; 
}TIME_INFO_T;


typedef struct tagMeasDataReqType
{
    UINT32  ulMsgId;
    UINT8   ucMeasureFlag;
    UINT16  usCrcCode; 
}MEAS_DATA_HEAD_REQ_T;

typedef struct tagMeasPacketInfoType
{
    UINT8   ucTotoalNum;
    UINT16  usTotoalPointNum;
    UINT16  usMaxPointNum;
    UINT16  usLastPointNum;
}MEAS_PACKET_INFO_T;


typedef struct tagMeasDataRspType
{
    UINT32          ulMsgId;                
	UINT8           ucLineNum;             
	UINT8           ucEcho;                
	UINT32          ulSerialNo;             
	UINT32          ulProductNo;           
	UINT8           ucIntensityStatus;    
	UINT8           ucDevStatus;           
	UINT32          ulTimeStamp;         
	UINT32          ulInputStatus;       
	UINT32          ulOutputStatus;       
	INT32          lStartAngle;        
	INT32          lStopAngle;          
    TIME_INFO_T     stTimeInfo;            
	UINT8           aucReserved[32];        
    UINT32          ulAngularResolution;    
    UINT16          usVerticalAngle;       
    UINT8           ucTotalPackNum;       
    UINT16          usPointMaxNum;        
    UINT8           ucPacketNo;         
    UINT16          usPackMeasPointNum;     
}MEAS_DATA_HEAD_RSP_T;

typedef struct tagMeasDataRsp
{
	MEAS_DATA_HEAD_RSP_T stMeaseDataHeadRsp;
	POINT_T	                       stPoint[350];
	UINT16                          usCrcCode; 
}MEAS_DATA_RSP_T;

typedef struct tagAlarmParaType
{
    UINT8   ucAlarmType;                   
    UINT8   aucAlarmPara[ALARM_PARA_NUM];   
}ALARM_PARA_T;

typedef struct tagFilterFlagType
{
    UINT8   ucOutLierFlag; 
    UINT8   ucEchoFlag;    
}FILTER_FLAG_T;

typedef struct tagFilterActiveReqType
{
    INT32           ulMsgId;      
    FILTER_FLAG_T   stFilterInfo;   
    UINT16          usCrcCode;    
}FILTER_ACTIVE_REQ_T;

typedef struct tagFilterActiveRspType
{
    UINT32   ulMsgId;    
    UINT16   usErrCode; 
    UINT16   usCrcCode; 
}FILTER_ACTIVE_RSP_T;


/******************************************************************
* parameter synchronous message
******************************************************************/
typedef struct
{
    ULONG  ulMsgId;
    USHORT usCrcCode;
}USER_PARA_SYNC_REQ_T;


typedef struct tagDeviceAngleType
{
    INT32  lStartAngle;      
    INT32  lStopAngle;       
}DEVICE_ANGLE_T;


typedef struct tagDeviceParaInfo
{
    UCHAR  ucCurrentSpeed;  
    UCHAR  ucIntensity;     
    ULONG  ulAngularResolution; 
}DEVICE_PARA_INFO_T;


typedef struct tagSoftwareBitType
{
    UINT16  usVersionType :1;
    UINT16  usVersion     :15;
}SOFTWARE_VERSION_BIT_T;

typedef struct tagSoftwareVerType
{
    UINT16  usMcuSoftwareVer;   
    UINT16  usFpgaSoftwareVer; 
}SOFTWARE_VERSION_T;

/******************************************************************
* User parameter message
******************************************************************/
typedef struct
{
    ULONG               ulMsgId;           
    UCHAR               aucMAC[VALUE_6];  
    SERIAL_NO_T         stSerialNoInfo;
    SOFTWARE_VERSION_T  stSoftwareVer;
    UCHAR               ucTdcType;         
    UCHAR               ucLinenum;
    DEVICE_ANGLE_T      stDeviceAngleInfo; 
    ULONG               ulPointNum; 
    ULONG               ulMaxDistance;   
    UCHAR               ucNetMode;
    NET_IP_T            stNetIp; 
    DEVICE_PARA_INFO_T  stDeviceParaInfo;   
    USHORT              usVerticalAngle;    
    FILTER_FLAG_T       stFilterInfo;    
    UINT8               ucCurrentAlarmId; 
    ALARM_PARA_T        stAlarmInfo;     
    UINT16              usCrcCode; 
}USER_PARA_SYNC_RSP_T;

typedef struct tagUserParaConfigReqType
{
    UINT32              ulMsgId;
    DEVICE_PARA_INFO_T  stDeviceParaInfo;
    UINT16              usCrcCode;  
}USER_PARA_CONFIG_REQ_T;

typedef struct tagUserParaErrCode
{
    UINT8   ucErrCode1  :1;
    UINT8   ucErrCode2  :1;
    UINT8   ucErrCode3  :1;
    UINT8   ucErrCode4  :1;
    UINT8   ucReserve   :4;
}USER_PARA_ERR_CODE_T;

typedef struct tagUserParaCogfigRspType
{
    UINT32  ulMsgId;
    UINT16  usErrCode;
    UINT16  usCrcCode;
}USER_PARA_CONFIG_RSP_T;



typedef struct tagNetIpReqType
{
    UINT32      ulMsgId;       
    NET_IP_T    stNetIpInfo; 
    UINT16      usCrcCode;   
}NET_IP_REQ_T;


typedef struct tagNetIpRspType
{
    UINT32   ulMsgId;   
    UINT16   usErrCode;
    UINT16   usCrcCode;  
}NET_IP_RSP_T;

/******************************************************************
*
******************************************************************/
typedef struct tagParaChangeType
{
    UINT32          ulMsgId;
    UINT8           ucAreaId;
    ALARM_PARA_T    stAlarmPara;
    UINT16          usCrcCode;
}PARA_CHANGED_IND_T;


/******************************************************************
* 
******************************************************************/
typedef struct
{
	BITS32  ulMsgId;
	BITS32  ulErrTdc1SmallCount;     
	BITS32  ulErrTdc1BigCount;          
	BITS32  ulErrTdc2SmallCount;  
	BITS32  ulErrTdc2BigCount;   
	BITS32  ulErrPulseNumCount;
	BITS32  aulReserve[5];
	BITS32  ulErrDistanceSmallCount;
	BITS32  ulErrDistanceBigCount;
	BITS32  ulErrIntensitySmallCount;
	BITS32  ulErrIntensityBigCount;
	BITS32  ulErrRotateSpeed;
	BITS32  ulErrCPUTemperature;
	BITS32  ulErrDeviceTemperature;
    BITS32  ulErrDeviceHumidity ;
}ERROR_REPORT_INF;

typedef struct
{
	BITS16 usAngle;	   
	BITS32 ulDistance;  
}DATA_INF;

typedef struct
{
    BITS32     ulMsgId;
    BITS8      ucPackageNO;                 
    BITS16     usMeasurePointNum;            
    DATA_INF   astDataInfo[PACKAGE_NUM_FULL]; 
}MEAS_DATA_PACKAGE;

#endif  // SEXXX_SE_STRUCTS_H_
