/*
 * ls_structs.h
 *
 *  Author: Zihong Ma
 ***************************************************************************
 *  OSIGHT *
 * www.osighttech.com  *
 ***************************************************************************/

#ifndef LSXXX_LS_STRUCTS_H_
#define LSXXX_LS_STRUCTS_H_

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


#define PARA_SYNC_REQ_ID                            0xDDDDDD01  
#define PARA_SYNC_RSP_ID                            0xFFFFFF01  

#define PARA_CHANGED_IND_ID                      0xFFFFFF02  
#define PARA_CONFIGURATION_REQ_ID          0xDDDDDD03  
#define PARA_CONFIGURATION_RSP_ID           0xFFFFFF03  
#define START_MEASURE_TRANSMISSION_ID   0xDDDDDD04   
#define MEAS_DATA_PACKAGE_ID                    0xFFFFFF05   
#define LOG_GET_REQ_ID                                0xDDDDDD06  
#define LOG_GET_RSP_ID                                0xFFFFFF06 
#define TIME_REPORT_INF_ID                         0xFFFFFF07  
#define WRITE_FACTORY_PARAM_REQ_ID       0xDDDDDD08 
#define WRITE_FACTORY_PARAM_RSP_ID       0xFFFFFF08 
#define READ_FACTORY_PARAM_REQ_ID         0xDDDDDD09   
#define READ_FACTORY_PARAM_RSP_ID         0xFFFFFF09 
#define CALIBRATION_MODE_ID                      0xDDDDDD0A  
#define NULL_MSGID                                       0x00000000

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


#define    RX_BUFSIZE       1460
#define    TX_BUFSIZE       1000

BITS8      g_aucRxBuf[RX_BUFSIZE];  
BITS8      g_aucTxBuf[TX_BUFSIZE];  

#define    DEVICE_NUM     200

typedef struct
{
	BITS8  ucAreaType;   
	BITS8  aucPara[19]; 
}ALARM_AREA_INFO;


typedef struct
{
    BITS32 ulMsgId;
    BITS16 usTransId;
}PARA_SYNC_REQ;

typedef struct
{
	BITS32            ulMsgId;           
	BITS16            usTransId;          
	BITS8             aucMAC[VALUE_6];    
	BITS32            ulSerialNum1;    
	BITS32            ulSerialNum2;
	BITS8             ucDevNum;            
	BITS8             ucSoftwareVersion;  
	BITS8             ucIndex;            
	BITS8             ucLineNum;           
	BITS32            ulStartAngle;        
	BITS16            usVerticalAngle;    
       BITS16            usMaxDistance;
	BITS32            ulPointNum;  
	BITS8             aucReserved[6];     
	BITS8             ucCurrentSpeed;      
	BITS8             ucIntensityStatus;   
	BITS8             ucCurrentAreaId;     
	BITS32            ulAngularResolution; 
	ALARM_AREA_INFO   stAlarmArea[16];    
}PARA_SYNC_RSP;

typedef struct
{
    BITS32            ulMsgId;
    BITS16            usTransId;
	BITS8             aucReserved[21];    
	BITS8             ucCurrentSpeed;     
	BITS8             ucIntensityStatus;  
	BITS8             ucCurrentAreaId;    
	BITS32            ulAngularResolution;
	ALARM_AREA_INFO   stAlarmArea[16];   
}PARA_CONFIGURATION_REQ;

typedef struct
{
    BITS32 ulMsgId;
    BITS16 usTransId;
    BITS8  ucResult;
}PARA_CONFIGURATION_RSP;


typedef struct
{
    BITS32 ulMsgId;
    BITS8   ucStart;
}START_MEASURE_TRANSMISSION_REQ;


typedef struct
{
	BITS32 ulDistance;  
}POINT0;

typedef struct
{
	BITS32 ulDistance;  
	BITS8  ucIntensity;  
}POINT1;

typedef struct
{
	BITS32  ulDistance;  
	BITS16  usIntensity;  
}POINT2;

typedef struct
{
    BITS32     ulMsgId;               
    BITS8      ucDevNum;               
	BITS8      ucSoftwareVersion;      
	BITS8      ucLineNum;           
	BITS8      ucEcho;                 
	BITS32     ulSerialNum1; 
	BITS32     ulSerialNum2;  
	BITS8      ucIntensityStatus;     
	BITS8      ucDevStatus;           
	BITS16     usScanCounter;        
	BITS32     ulTime;                
	BITS32     ulInputStatus;         
	BITS32     ulOutputStatus;         
	BITS32     ulStartAngle;         
	BITS16     usVerticalAngle;        
	BITS8       aucReserved[16];    
	BITS16     usPackMeasPointNum; 
	BITS32     ulAngularResolution;    
	BITS8      ucTotalPackNum;   
	BITS8      ucCurrentPackNO; 
       POINT0     astPoint0[350];
}MEAS_DATA_NO_INTENSITY;

typedef struct
{
    BITS32     ulMsgId;                
    BITS8      ucDevNum;               
	BITS8      ucSoftwareVersion;    
	BITS8      ucLineNum;           
	BITS8      ucEcho;              
	BITS32     ulSerialNum1; 
	BITS32     ulSerialNum2; 
	BITS8      ucIntensityStatus;   
	BITS8      ucDevStatus;         
	BITS16     usScanCounter;       
	BITS32     ulTime;              
	BITS32     ulInputStatus;  
	BITS32     ulOutputStatus;     
	BITS32     ulStartAngle;      
	BITS16     usVerticalAngle;       
	BITS8      aucReserved[16];     
	BITS16     usPackMeasPointNum;   
	BITS32     ulAngularResolution;
	BITS8      ucTotalPackNum;    
	BITS8      ucCurrentPackNO;
       POINT1     astPoint1[250];
}MEAS_DATA_HAVE_INTENSITY1;


typedef struct
{
    BITS32     ulMsgId;                
    BITS8      ucDevNum;               
	BITS8      ucSoftwareVersion;    
	BITS8      ucLineNum;           
	BITS8      ucEcho;              
	BITS32     ulSerialNum1; 
	BITS32     ulSerialNum2; 
	BITS8      ucIntensityStatus;   
	BITS8      ucDevStatus;         
	BITS16     usScanCounter;       
	BITS32     ulTime;              
	BITS32     ulInputStatus;  
	BITS32     ulOutputStatus;     
	BITS32     ulStartAngle;      
	BITS16     usVerticalAngle;       
	BITS8      aucReserved[16];     
	BITS16     usPackMeasPointNum;   
	BITS32     ulAngularResolution;
	BITS8      ucTotalPackNum;    
	BITS8      ucCurrentPackNO;
       POINT2     astPoint2[250];
}MEAS_DATA_HAVE_INTENSITY2;


#endif  // LSXXX_LS_STRUCTS_H_
