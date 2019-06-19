/*
 * IExxx.h
 *
 *  Created on: 05-09-2017
 *  Author: Zihong Ma
 ***************************************************************************
 *  OSIGHT *
 * www.osighttech.com  *
 ***************************************************************************/

#ifndef IEXXX_H_
#define IEXXX_H_

#include <IExxx/ie_buffer.h>
#include <IExxx/ie_structs.h>
#include <string>
#include <stdint.h>

typedef enum
{
  undefined = 0,
  initialisation = 1,
  configuration = 2,
  idle = 3,
  rotated = 4,
  in_preparation = 5,
  ready = 6,
  ready_for_measurement = 7
} status_t;

/*!
* @class IExxx
* @brief Class responsible for communicating with IExxx device.
*
*/

class IExxx
{
public:
  IExxx();
  virtual ~IExxx();
  INT32 connect(std::string host, int port = 6500);
  void disconnect();
  bool isConnected();

  void InvertUint8(unsigned char *dBuf,unsigned char *srcBuf);
  void InvertUint16(unsigned short *dBuf,unsigned short *srcBuf);
  unsigned short  CRC16(unsigned char *puchMsg, unsigned int usDataLen);

  INT32 UserParaSync();
  void  StartMeasureTransmission();
  INT32 GetLidarMeasData();
  INT32 PackUserParaSyncReq(USER_PARA_SYNC_REQ_T *vpstUserParaSyncReq, BITS8 *vpucBuff);
  INT32 UnpackUserParaSyncRsp(BITS8 *vpucMsg, USER_PARA_SYNC_RSP_T *vpstUserParaSyncRsp, INT32 lLength);

  INT32 PackStartMeasureTransmissionReq(MEAS_DATA_HEAD_REQ_T *vpstMeasParaReq, BITS8 *vpucBuff);
  INT32 UnpackMeasData(BITS8 *vpucMsg, MEAS_DATA_RSP_T *vpstMeasDataRsp);

  void  send_data (void* vpSrc, BITS16 usCnt);
  INT32 read_data (void* vpSrc, BITS16 usCnt);

protected:
  bool connected_;
  IEBuffer buffer_;
  int socket_fd_;
};

#endif /* IEXXX_H_ */
