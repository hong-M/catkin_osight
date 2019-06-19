/*
 * LSxxx.h
 *
 *  Created on: 05-09-2017
 *  Author: Zihong Ma
 ***************************************************************************
 *  OSIGHT *
 * www.osighttech.com  *
 ***************************************************************************/

#ifndef LSXXX_H_
#define LSXXX_H_

#include <LSxxx/ls_buffer.h>
#include <LSxxx/ls_structs.h>
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
* @class LSxxx
* @brief Class responsible for communicating with LSxxx device.
*
*/

class LSxxx
{
public:
  LSxxx();
  virtual ~LSxxx();
  INT32 connect(std::string host, int port = 6500);
  void disconnect();
  bool isConnected();
  
  INT32 ParaSync();
  INT32 ParaConfiguration();
  void   StartMeasureTransmission();
  INT32 GetLidarMeasData();
  INT32 PackParaSyncReq(PARA_SYNC_REQ *vpstParaSyncReq, BITS8 *vpucBuff);
  INT32 PackParaConfigurationReq(PARA_CONFIGURATION_REQ *vpstParaConfigurationReq, BITS8 *vpucBuff);
  INT32 PackStartMeasureTransmissionReq(START_MEASURE_TRANSMISSION_REQ *vpstStartMeasureTransmissionReq, BITS8 *vpucBuff);
  INT32 UnpackParaSyncRsp(BITS8 *vpucMsg, PARA_SYNC_RSP *vpstParaSyncRsp);
  INT32 UnpackParaConfigurationRsp(BITS8 *vpucMsg, PARA_CONFIGURATION_RSP *vpstParaConfigurationRsp);
  INT32 UnpackMeasDataNoIntensity(BITS8 *vpucMsg, MEAS_DATA_NO_INTENSITY *vpstMeasDataNoIntensity);
  INT32 UnpackMeasDataHaveIntensity1(BITS8 *vpucMsg, MEAS_DATA_HAVE_INTENSITY1 *vpstMeasDataHaveIntensity1);
  INT32 UnpackMeasDataHaveIntensity2(BITS8 *vpucMsg, MEAS_DATA_HAVE_INTENSITY2 *vpstMeasDataHaveIntensity2);


  void   send_data (void* vpSrc, BITS16 usCnt);
  INT32 read_data (void* vpSrc, BITS16 usCnt);

protected:
  bool connected_;
  LSBuffer buffer_;
  int socket_fd_;
};

#endif /* LSXXX_H_ */

