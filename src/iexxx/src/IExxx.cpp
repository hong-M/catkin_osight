/*
 * IExxx.cpp
 *
 *  Created on: 05-09-2017
 *  Author: Zihong Ma
 ***************************************************************************
 *  OSIGHT *
 * www.osighttech.com  *
 ***************************************************************************/

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>

#include "IExxx/IExxx.h"
#include "console_bridge/console.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define SERVER_PORT 5500
#define CLIENT_PORT 6500

struct sockaddr_in server_addr;
struct sockaddr_in client_addr;
int server_socket_fd;
socklen_t client_addr_length;
socklen_t server_addr_length;

extern IExxx laser;
extern sensor_msgs::LaserScan scan_msg;
extern USER_PARA_SYNC_RSP_T         g_stRealUserPara;
extern MEAS_DATA_RSP_T                    g_stMeasDataRsp;
INT32   g_lOffsetAngle=0; 

IExxx::IExxx() : connected_(false)
{
}

IExxx::~IExxx()
{
}

INT32 IExxx::connect(std::string hostPC, int portPC)
{
 /* UDP */
 /* server  */
 bzero(&server_addr, sizeof(server_addr));
 server_addr.sin_family = AF_INET;
 inet_pton(AF_INET, hostPC.c_str(), &server_addr.sin_addr);
 server_addr.sin_port = htons(portPC);

 /* client */
 //bzero(&client_addr, sizeof(client_addr));
 //client_addr.sin_family = AF_INET;
 //inet_pton(AF_INET, host.c_str(), &client_addr.sin_addr);
 //client_addr.sin_port = htons(port);

/* create socket */
server_socket_fd =-1;
 server_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
 if(server_socket_fd < 0)
 {
  perror("Create Socket Failed:");
  return 1;
 }

  /* setsockopt */
 int on = 1;
 if( setsockopt(server_socket_fd,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(int))<0 )
 {
	perror(" setsockopt error ");
	return 1;
 }

 /* bind */
 if(-1 == (bind(server_socket_fd,(struct sockaddr*)&server_addr,sizeof(server_addr))))
 {
  perror("Server Bind Failed:");
  return 1;
 }

  client_addr_length = sizeof(client_addr);
  server_addr_length = sizeof(server_addr);

  connected_ = true;
}


#if 0
void IExxx::connect(std::string host, int port)
{
  /* tcp */
  if (!connected_)
  {
    logDebug("Creating non-blocking socket.");
    socket_fd_ = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_fd_)
    {
      struct sockaddr_in stSockAddr;
      stSockAddr.sin_family = PF_INET;
      stSockAddr.sin_port = htons(port);
      inet_pton(AF_INET, host.c_str(), &stSockAddr.sin_addr);

      logDebug("Connecting socket to laser.");
      int ret = ::connect(socket_fd_, (struct sockaddr *) &stSockAddr, sizeof(stSockAddr));

      if (ret == 0)
      {
        connected_ = true;
        logDebug("Connected succeeded.");
      }
    }
  }
}
#endif

void IExxx::disconnect()
{
  if (connected_)
  {
    close(socket_fd_);
    connected_ = false;
  }
}

bool IExxx::isConnected()
{
  return connected_;
}


void IExxx::InvertUint8(unsigned char *dBuf,unsigned char *srcBuf)
{
    int i;
    unsigned char tmp[4];
    tmp[0] = 0;
    for(i=0;i< 8;i++)
    {
      if(srcBuf[0]& (1 << i))
        tmp[0]|=1<<(7-i);
    }
    dBuf[0] = tmp[0];
}
 
void IExxx::InvertUint16(unsigned short *dBuf,unsigned short *srcBuf)
{
    int i;
    unsigned short tmp[4];
    tmp[0] = 0;
    for(i=0;i< 16;i++)
    {
  if(srcBuf[0]& (1 << i))
        tmp[0]|=1<<(15 - i);
    }
    dBuf[0] = tmp[0];
}

unsigned short  IExxx::CRC16(unsigned char *puchMsg, unsigned int usDataLen)
{
  unsigned short wCRCin = 0xFFFF;
  unsigned short wCPoly = 0x8005;
  unsigned char wChar = 0;
  unsigned int  i;
  
  while (usDataLen--) 	
  {
        wChar = *(puchMsg++);
        laser.InvertUint8(&wChar,&wChar);
        wCRCin = wCRCin^(wChar << 8);
        for(i = 0;i < 8;i++)
        {
          if(wCRCin & 0x8000)
            wCRCin = (wCRCin << 1) ^ wCPoly;
          else
            wCRCin = wCRCin << 1;
        }
  }
  laser.InvertUint16(&wCRCin,&wCRCin);
  wCRCin = HTONS(wCRCin);
  
  return (wCRCin);
  
}

INT32 IExxx::UserParaSync()
{
  BITS8  ucIndex=0;
  BITS8  ucIndex1=0;
  BITS8  ucCount=30;
  INT32  ulSendLength=0;
  INT32  ulRecvLength1=0;
  INT32  ulRecvLength2=0;
  UINT16 usCrcData=0;
  BITS32 ulMsgId = NULL_MSGID;
  
  
  USER_PARA_SYNC_REQ_T    stParaSyncReq;
  USER_PARA_SYNC_RSP_T    stParaSyncRsp;
  
  stParaSyncReq.ulMsgId = USER_PARA_SYNC_REQ;

  ROS_INFO_STREAM("UserParaSync");

  memset(g_aucTxBuf,0,sizeof(g_aucTxBuf));
  ulSendLength = laser.PackUserParaSyncReq(&stParaSyncReq, g_aucTxBuf);



  if( ulSendLength <= VALUE_0  )
  {
      return BUSPRO_ERROR;
  }

  memset((BITS8*)&stParaSyncRsp,0,sizeof(USER_PARA_SYNC_RSP_T));
  memset((BITS8*)&g_stRealUserPara,0,sizeof(USER_PARA_SYNC_RSP_T));

  while(ucCount--)
   {
         laser.send_data(g_aucTxBuf, ulSendLength);
	     //ulRecvLength1 = laser.read_data(g_aucRxBuf, sizeof(stParaSyncRsp));
         ulRecvLength1 = laser.read_data(g_aucRxBuf, sizeof(g_aucRxBuf));
         if( ulRecvLength1 > VALUE_0 )
		  {
		      ulMsgId = NULL_MSGID;
			  ulMsgId = ( ((BITS32)g_aucRxBuf[BIT_0] << OFFSET_24) | ((BITS32)g_aucRxBuf[BIT_1]<<OFFSET_16)|
			                ((BITS32)g_aucRxBuf[BIT_2] << OFFSET_08) | ((BITS32)g_aucRxBuf[BIT_3]));

			if( USER_PARA_SYNC_RSP == ulMsgId )
			{
				logDebug("Parameters of synchronous response received\r\n");

                //ROS_INFO_STREAM("ulRecvLength1: "<< ulRecvLength1 << " ." );       
				ulRecvLength2 = laser.UnpackUserParaSyncRsp((BITS8 *)&g_aucRxBuf, (USER_PARA_SYNC_RSP_T *)&stParaSyncRsp, ulRecvLength1);
				//ROS_INFO_STREAM("ulRecvLength2: "<< ulRecvLength2 << " ." ); 
                if(  ulRecvLength2 > 0 )
				{
                    usCrcData = laser.CRC16((BITS8 *)g_aucRxBuf, ulRecvLength1-sizeof(UINT16) );
                    ROS_INFO_STREAM("ulRecvLength1-sizeof(UINT16): "<< ulRecvLength1-sizeof(UINT16) << " ." ); 
				    ROS_INFO_STREAM("laser.CRC16: "<< usCrcData << "   stParaSyncRsp.usCrcCode: " <<  stParaSyncRsp.usCrcCode <<"  end" );

                    if(  usCrcData == stParaSyncRsp.usCrcCode )
                    {
                        // offset angle 
                        stParaSyncRsp.stDeviceAngleInfo.lStartAngle += g_lOffsetAngle;
                        stParaSyncRsp.stDeviceAngleInfo.lStopAngle  += g_lOffsetAngle;
                        ROS_INFO_STREAM("g_lOffsetAngle: "<< g_lOffsetAngle << " ." <<"end"); 
                        //ROS_INFO_STREAM("stParaSyncRsp.stDeviceAngleInfo.lStartAngle: "<< stParaSyncRsp.stDeviceAngleInfo.lStartAngle << " ." <<"end"); 
                        //ROS_INFO_STREAM("stParaSyncRsp.stDeviceAngleInfo.lStopAngle: "<< stParaSyncRsp.stDeviceAngleInfo.lStopAngle << " ." <<"end" ); 
                        
                        memcpy((BITS8*)&g_stRealUserPara,(BITS8*)&stParaSyncRsp,sizeof(USER_PARA_SYNC_RSP_T));
                        return BUSPRO_OK;
                    }
				}
			}
			else
			{
				logDebug("invalid packet recieved\r\n");
				//return BUSPRO_ERROR;
			}
	    }
    }
    return BUSPRO_ERROR;

}

void IExxx::StartMeasureTransmission()
{
  INT32  ulSendLength=0;
  BITS32 ulMsgId = NULL_MSGID;
  MEAS_DATA_HEAD_REQ_T    stMeasParaReq;
  

  stMeasParaReq.ulMsgId = MEASURE_TRANSMISSION_REQ;
  stMeasParaReq.ucMeasureFlag = 0x01;

  ROS_INFO_STREAM("StartMeasureTransmission");

  memset(g_aucTxBuf,0,sizeof(g_aucTxBuf));
  ulSendLength = laser.PackStartMeasureTransmissionReq(&stMeasParaReq, g_aucTxBuf);

  if( ulSendLength > VALUE_0  )
  {
      laser.send_data(g_aucTxBuf, ulSendLength);
  }

}

INT32 IExxx::GetLidarMeasData()
{
  int      i=0;
  BITS32  ulIndex=0;
  BITS32  ulIndex1=0;
  BITS8  ucCount=30;
  INT32  ulRecvLength1=0;
  INT32  ulSendLength2=0;
  BITS32 ulMsgId = NULL_MSGID;

   ROS_DEBUG("GetLidarMeasData");
   while(1)
   {
             memset(g_aucRxBuf,0,sizeof(g_aucRxBuf));
		ulRecvLength1 = laser.read_data(g_aucRxBuf, sizeof(g_aucRxBuf));
             if( ulRecvLength1 > VALUE_0 )
		{
			ulMsgId = ( ((BITS32)g_aucRxBuf[BIT_0] << OFFSET_24) | ((BITS32)g_aucRxBuf[BIT_1]<<OFFSET_16)|
			                ((BITS32)g_aucRxBuf[BIT_2] << OFFSET_08) | ((BITS32)g_aucRxBuf[BIT_3]));

                    //logWarn("ulMsgId=%x",ulMsgId);
			//logWarn("MEAS_DATA_PACKAGE_ID=%x",MEAS_DATA_PACKAGE_ID);;
			if( MEASURE_TRANSMISSION_RSP == ulMsgId )
			{
				//logDebug("GetLidarMeasData received\r\n");
                //ROS_INFO_STREAM("GetLidarMeasData received");

				ulSendLength2 = laser.UnpackMeasData((BITS8 *)&g_aucRxBuf, (MEAS_DATA_RSP_T *)&g_stMeasDataRsp);

				if(  laser.CRC16(g_aucRxBuf, ulSendLength2-sizeof(UINT16) )== g_stMeasDataRsp.usCrcCode)
				{				
	 				for(ulIndex=0;ulIndex<g_stMeasDataRsp.stMeaseDataHeadRsp.usPackMeasPointNum;ulIndex++)
					{
                        scan_msg.ranges[ g_stMeasDataRsp.stMeaseDataHeadRsp.ucPacketNo * g_stMeasDataRsp.stMeaseDataHeadRsp.usPointMaxNum+ulIndex] = g_stMeasDataRsp.stPoint[ulIndex].ulDistance * 0.0001;

						if( 0x02 == g_stRealUserPara.stDeviceParaInfo.ucIntensity )
						{
							scan_msg.intensities[ g_stMeasDataRsp.stMeaseDataHeadRsp.ucPacketNo * g_stMeasDataRsp.stMeaseDataHeadRsp.usPointMaxNum + ulIndex] = g_stMeasDataRsp.stPoint[ulIndex].unIntensity.usIntensity;
						}
						else if( 0x01 == g_stRealUserPara.stDeviceParaInfo.ucIntensity )
						{
							scan_msg.intensities[ g_stMeasDataRsp.stMeaseDataHeadRsp.ucPacketNo * g_stMeasDataRsp.stMeaseDataHeadRsp.usPointMaxNum + ulIndex] = g_stMeasDataRsp.stPoint[ulIndex].unIntensity.ucIntensity;
						}
						else if( 0x00 == g_stRealUserPara.stDeviceParaInfo.ucIntensity )
						{
							//NULL;
						}
					}

					if(( g_stMeasDataRsp.stMeaseDataHeadRsp.ucPacketNo+1)==g_stMeasDataRsp.stMeaseDataHeadRsp.ucTotalPackNum )
					{
						//ROS_INFO_STREAM("g_stRealPara.ucIntensityStatus=2");
						return BUSPRO_OK;
					}
				}
				
				//return BUSPRO_OK;
			}
			else
			{
				ROS_DEBUG("invalid packet recieved,ulMsgId=%x",ulMsgId);
				//return BUSPRO_ERROR;
			}
	      }
    }
    return BUSPRO_ERROR;

}


INT32 IExxx::PackUserParaSyncReq(USER_PARA_SYNC_REQ_T *vpstUserParaSyncReq, BITS8 *vpucBuff)
{
    BITS8 *pucSendNum = vpucBuff ;
    BITS8  ucIndex=0;
    BITS8  ucIndex1=0;

    if( (NULL==vpstUserParaSyncReq)||(NULL==vpucBuff) )
    {
        logWarn("PackUserParaSyncReq:  Pointer address is invalid\n");
        return BUSPRO_ERROR_POINTER;
    }

    PACK_4_BYTE(vpucBuff, vpstUserParaSyncReq->ulMsgId);
    vpstUserParaSyncReq->usCrcCode = laser.CRC16(pucSendNum, vpucBuff-pucSendNum);
    PACK_2_BYTE(vpucBuff, vpstUserParaSyncReq->usCrcCode);

    return (vpucBuff-pucSendNum);
}

INT32 IExxx::PackStartMeasureTransmissionReq(MEAS_DATA_HEAD_REQ_T *vpstMeasParaReq, BITS8 *vpucBuff)
{
    BITS8 *pucSendNum = vpucBuff ;
    BITS8  ucIndex=0;
    BITS8  ucIndex1=0;

    if( (NULL==vpstMeasParaReq)||(NULL==vpucBuff) )
    {
        logWarn("PackStartMeasureTransmissionReq:  Pointer address is invalid\n");
        return BUSPRO_ERROR_POINTER;
    }

    PACK_4_BYTE(vpucBuff, vpstMeasParaReq->ulMsgId);
    PACK_1_BYTE(vpucBuff, vpstMeasParaReq->ucMeasureFlag);

    vpstMeasParaReq->usCrcCode = laser.CRC16(pucSendNum, vpucBuff-pucSendNum);
    PACK_2_BYTE(vpucBuff, vpstMeasParaReq->usCrcCode);

    return (vpucBuff-pucSendNum);
}


INT32 IExxx::UnpackUserParaSyncRsp(BITS8 *vpucMsg, USER_PARA_SYNC_RSP_T *vpstUserParaSyncRsp, INT32 lLength)
{
    BITS32  ulIndex=0;
    INT32   lTmpLength=0;
    BITS8   *pucRecvNum = vpucMsg ;

    if( (NULL==vpucMsg)||(NULL==vpstUserParaSyncRsp) )
    {
        logWarn("UnpackUserParaSyncRsp:  Pointer address is invalid\n");
        return BUSPRO_ERROR_POINTER;
    }

	UNPACK_4_BYTE(vpucMsg, vpstUserParaSyncRsp->ulMsgId);
    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->aucMAC[0]);
	UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->aucMAC[1]);
	UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->aucMAC[2]);
	UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->aucMAC[3]);
	UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->aucMAC[4]);
	UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->aucMAC[5]);
    
	UNPACK_4_BYTE(vpucMsg, vpstUserParaSyncRsp->stSerialNoInfo.ulSerialNo);
	UNPACK_4_BYTE(vpucMsg, vpstUserParaSyncRsp->stSerialNoInfo.ulProductNo);
    
	UNPACK_2_BYTE(vpucMsg, vpstUserParaSyncRsp->stSoftwareVer.usMcuSoftwareVer);
	UNPACK_2_BYTE(vpucMsg, vpstUserParaSyncRsp->stSoftwareVer.usFpgaSoftwareVer);
	UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->ucTdcType);
    
	UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->ucLinenum);
	UNPACK_4_BYTE(vpucMsg, vpstUserParaSyncRsp->stDeviceAngleInfo.lStartAngle);
    UNPACK_4_BYTE(vpucMsg, vpstUserParaSyncRsp->stDeviceAngleInfo.lStopAngle);
    UNPACK_4_BYTE(vpucMsg, vpstUserParaSyncRsp->ulPointNum);
    UNPACK_4_BYTE(vpucMsg, vpstUserParaSyncRsp->ulMaxDistance);


    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->ucNetMode);   
    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.aucDevIp[0]);
    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.aucDevIp[1]);
    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.aucDevIp[2]);
    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.aucDevIp[3]);
    UNPACK_4_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.ulDevPort);


    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.aucUpperIp[0]);
    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.aucUpperIp[1]);
    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.aucUpperIp[2]);
    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.aucUpperIp[3]);
    UNPACK_4_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.ulUpperPort);

    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.aucSubnetMask[0]);
    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.aucSubnetMask[1]);
    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.aucSubnetMask[2]);
    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.aucSubnetMask[3]);

    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.aucGateway[0]);
    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.aucGateway[1]);
    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.aucGateway[2]);
    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stNetIp.aucGateway[3]);

    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stDeviceParaInfo.ucCurrentSpeed);
    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stDeviceParaInfo.ucIntensity);
    UNPACK_4_BYTE(vpucMsg, vpstUserParaSyncRsp->stDeviceParaInfo.ulAngularResolution);
    UNPACK_2_BYTE(vpucMsg, vpstUserParaSyncRsp->usVerticalAngle);

    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stFilterInfo.ucOutLierFlag);
    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stFilterInfo.ucEchoFlag);

    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->ucCurrentAlarmId);
    UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stAlarmInfo.ucAlarmType);

    for(ulIndex=0;ulIndex<sizeof(vpstUserParaSyncRsp->stAlarmInfo.aucAlarmPara);ulIndex++)
    {
          UNPACK_1_BYTE(vpucMsg, vpstUserParaSyncRsp->stAlarmInfo.aucAlarmPara[ulIndex]);
    }

    //缺少哪些内容，这里地址增加上，保证后面CRC正常
    lTmpLength = ( lLength - sizeof(UINT16) ) - (vpucMsg - pucRecvNum) ;
    vpucMsg += lTmpLength;

    //CRC获得，利用最大接受数据长度来定下。lLength
    UNPACK_2_BYTE(vpucMsg, vpstUserParaSyncRsp->usCrcCode);

    //return BUSPRO_OK;
    return (vpucMsg - pucRecvNum);
}

INT32 IExxx::UnpackMeasData(BITS8 *vpucMsg, MEAS_DATA_RSP_T *vpstMeasDataRsp)
{
    BITS8 *pucRecvNum = vpucMsg ;
    BITS32  ulIndex=0;

    if( (NULL==vpucMsg)||(NULL==vpstMeasDataRsp) )
    {
        logWarn("UnpackMeasData:  Pointer address is invalid\n");
        return BUSPRO_ERROR_POINTER;
    }

	UNPACK_4_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.ulMsgId);
	UNPACK_1_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.ucLineNum);
	UNPACK_1_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.ucEcho);
	UNPACK_4_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.ulSerialNo);
	UNPACK_4_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.ulProductNo);
	UNPACK_1_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.ucIntensityStatus);
	UNPACK_1_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.ucDevStatus);
	UNPACK_4_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.ulTimeStamp);
	UNPACK_4_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.ulInputStatus);
	UNPACK_4_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.ulOutputStatus);
	UNPACK_4_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.lStartAngle);
	UNPACK_4_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.lStopAngle);
	UNPACK_1_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.stTimeInfo.stDate.ucYear);
	UNPACK_1_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.stTimeInfo.stDate.ucMonth);
	UNPACK_1_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.stTimeInfo.stDate.ucDate);
	UNPACK_1_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.stTimeInfo.stTime.ucHour);
	UNPACK_1_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.stTimeInfo.stTime.ucMinutes);
	UNPACK_1_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.stTimeInfo.stTime.ucSeconds);
	UNPACK_4_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.stTimeInfo.stTime.ulMicroSeconds);
	
	for(ulIndex=0;ulIndex<sizeof(vpstMeasDataRsp->stMeaseDataHeadRsp.aucReserved);ulIndex++)
	{
		UNPACK_1_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.aucReserved[ulIndex]);
	}

	UNPACK_4_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.ulAngularResolution);
	UNPACK_2_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.usVerticalAngle);
	UNPACK_1_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.ucTotalPackNum);
	UNPACK_2_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.usPointMaxNum);
	UNPACK_1_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.ucPacketNo);
	UNPACK_2_BYTE(vpucMsg, vpstMeasDataRsp->stMeaseDataHeadRsp.usPackMeasPointNum);

	if( 0x00==g_stRealUserPara.stDeviceParaInfo.ucIntensity )
	{
		for(ulIndex=0;ulIndex<vpstMeasDataRsp->stMeaseDataHeadRsp.usPackMeasPointNum;ulIndex++)
		{
			UNPACK_4_BYTE(vpucMsg, vpstMeasDataRsp->stPoint[ulIndex].ulDistance);
		}
	}
	else if(  0x01==g_stRealUserPara.stDeviceParaInfo.ucIntensity   )
	{
		for(ulIndex=0;ulIndex<vpstMeasDataRsp->stMeaseDataHeadRsp.usPackMeasPointNum;ulIndex++)
		{
			UNPACK_4_BYTE(vpucMsg, vpstMeasDataRsp->stPoint[ulIndex].ulDistance);
			UNPACK_1_BYTE(vpucMsg, vpstMeasDataRsp->stPoint[ulIndex].unIntensity.ucIntensity);
		}
	}
	else if(  0x02==g_stRealUserPara.stDeviceParaInfo.ucIntensity   )
	{
		for(ulIndex=0;ulIndex<vpstMeasDataRsp->stMeaseDataHeadRsp.usPackMeasPointNum;ulIndex++)
		{
			UNPACK_4_BYTE(vpucMsg, vpstMeasDataRsp->stPoint[ulIndex].ulDistance);
			UNPACK_2_BYTE(vpucMsg, vpstMeasDataRsp->stPoint[ulIndex].unIntensity.usIntensity);
		}
	}

	UNPACK_2_BYTE(vpucMsg, vpstMeasDataRsp->usCrcCode);
	   
    //return BUSPRO_OK;
    return (vpucMsg-pucRecvNum);
}


void IExxx::send_data (void* vpSrc, BITS16 usCnt)
{
	/* tcp */
	//write(socket_fd_, vpSrc, usCnt);

    	/* udp */
	sendto(server_socket_fd, vpSrc, usCnt,0,(struct sockaddr*)&client_addr,sizeof(client_addr));
}

/**********************************************************
* 函数功能: 读取数据
***********************************************************/
INT32 IExxx::read_data (void* vpSrc, BITS16 usCnt)
{
       /* tcp */
	//return read(socket_fd_, vpSrc, usCnt);

       /* udp */
	return recvfrom(server_socket_fd, vpSrc, usCnt,0,(struct sockaddr*)&client_addr, &client_addr_length);
}

