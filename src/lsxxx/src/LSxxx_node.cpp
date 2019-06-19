/*
 * LSxxx.cpp
 *
 *  Created on: 05-09-2017
 *  Author: Zihong MA
 ***************************************************************************
 *  OSIGHT *
 * www.osighttech.com  *
 ***************************************************************************/

#include <csignal>
#include <cstdio>
#include <LSxxx/LSxxx.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define DEG2RAD M_PI/180.0

LSxxx                                       laser;
sensor_msgs::LaserScan             scan_msg;
PARA_SYNC_RSP                        g_stRealPara;
MEAS_DATA_NO_INTENSITY        g_stMeasDataNoIntensity;
MEAS_DATA_HAVE_INTENSITY1   g_stMeasDataHaveIntensity1;
MEAS_DATA_HAVE_INTENSITY2   g_stMeasDataHaveIntensity2;

extern int server_socket_fd;

int main(int argc, char **argv)
{
  BITS8   err;
  
  // parameters
  std::string hostPC;
  std::string frame_id;
  int portPC;

  ros::init(argc, argv, "lsxxx");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);

  n.param<std::string>("hostPC", hostPC, "192.168.1.100");
  n.param<std::string>("frame_id", frame_id, "laser");
  n.param<int>("portPC", portPC, 5500);

  while (ros::ok())
  {
    //ROS_INFO_STREAM("Connecting to laser at " << hostPC<<"  "<<portPC );
    ROS_INFO_STREAM("Upper IP: "<< hostPC << "Port: " <<  portPC <<"Connecting to laser " );
    laser.connect(hostPC, portPC);
    if (!laser.isConnected())
    {
      ROS_WARN("Unable to connect, retrying.");
      ros::Duration(1).sleep();
      continue;
    }

    do
    {
        //ROS_INFO_STREAM("Parameters of the synchronous message.");
	 err=laser.ParaSync();
    } while( 0 !=err );

    //g_stRealPara.ucIntensityStatus = 0;
    //g_stRealPara.ucIntensityStatus = 1;
    //g_stRealPara.ucIntensityStatus = 2;
	
    //do
    //{
        //ROS_INFO_STREAM("Parameters configuration message.");
	// err = laser.ParaConfiguration();
    //} while( 0 !=err );

    ROS_INFO_STREAM("scan_msg  head");

    ROS_INFO_STREAM("CurrentSpeed:"<<g_stRealPara.ucCurrentSpeed*1.0);
    ROS_INFO_STREAM("AngularResolution:"<<g_stRealPara.ulAngularResolution/10000000.0);
    ROS_INFO_STREAM("StartAngle:"<<g_stRealPara.ulStartAngle/1000.0); 
    ROS_INFO_STREAM("PointNum:"<<g_stRealPara.ulPointNum);
	
    scan_msg.header.frame_id = frame_id;
    scan_msg.range_min = 0.01;
    //scan_msg.range_max = 500.0;   //500 meter
    scan_msg.range_max = g_stRealPara.usMaxDistance;  
    
    //scan_msg.scan_time = 100.0 / 2500;  //25HZ
    scan_msg.scan_time   =  100.0/(g_stRealPara.ucCurrentSpeed*100);
   
    //scan_msg.angle_increment = (double)2500/ 10000.0 * DEG2RAD;  //0.25 degree
    scan_msg.angle_increment = (double)(g_stRealPara.ulAngularResolution/1000.0)/10000.0*DEG2RAD;
    
    //scan_msg.angle_min = (double)0xFFF92230 / 10000.0 * DEG2RAD - M_PI / 2;   //-450000
    scan_msg.angle_min =  (double)(g_stRealPara.ulStartAngle*10)/ 10000.0 * DEG2RAD - M_PI / 2;
   
    //scan_msg.angle_max = (double)0x225510 / 10000.0 * DEG2RAD - M_PI / 2;    //2250000
    scan_msg.angle_max = (double)(g_stRealPara.ulStartAngle*10+(g_stRealPara.ulAngularResolution*g_stRealPara.ulPointNum/1000.0)) / 10000.0 * DEG2RAD - M_PI / 2;
    
    //ROS_DEBUG_STREAM("Device resolution is " << (double)2500 / 10000.0 << " degrees.");  //0.25 degree
    ROS_INFO_STREAM("Device resolution is " << (double)(g_stRealPara.ulAngularResolution/1000)/ 10000.0 << " degrees.");

    //ROS_DEBUG_STREAM("Device frequency is " <<(double) 2500 / 100.0 << " Hz");   //25HZ
    ROS_INFO_STREAM("Device frequency is " <<(double) (g_stRealPara.ucCurrentSpeed*100)/ 100.0 << " Hz");
  
    scan_msg.ranges.resize(g_stRealPara.ulPointNum);
    scan_msg.intensities.resize(g_stRealPara.ulPointNum);

    //scan_msg.time_increment =(2500 / 10000.0) / 360.0  / (2500/ 100.0);   //  0.25 degree ,  25HZ
    scan_msg.time_increment = ((g_stRealPara.ulAngularResolution/1000.0)/10000.0)/360.0/((g_stRealPara.ucCurrentSpeed*100)/100.0);

    ROS_DEBUG_STREAM("Time increment is " << static_cast<int>(scan_msg.time_increment * 1000000) << " microseconds");

    ROS_DEBUG("Starting measurements.");
    laser.StartMeasureTransmission();

    while (ros::ok())
    {
      ros::Time start = ros::Time::now();

      scan_msg.header.stamp = start;
      ++scan_msg.header.seq;

      err = laser.GetLidarMeasData();
      if(0==err)
      	{
        //ROS_DEBUG("Publishing scan data.");
        scan_pub.publish(scan_msg);
	}else
      {
        ROS_DEBUG("Laser timed out on delivering scan, attempting to reinitialize.");
        //break;
      }

      ros::spinOnce();
    }

     ROS_INFO_STREAM("hong-close(server_socket_fd)");
     close(server_socket_fd);	
	 
  }
  
  return 0;
}
