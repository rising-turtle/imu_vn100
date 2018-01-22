/*
 * Jan. 20 2018, He Zhang, hxzhang1@ualr.edu 
 *
 * IMU interface 
 *
 * */

#include "imu_reader.h"
#include <mutex>

namespace{

std::mutex g_buf_mutex; 
IMUData g_imu_data; 
bool g_new_data = false; 

/* Change the connection settings to your configuration. */
// const char* const COM_PORT = "//dev//ttyUSB0";
const char* const COM_PORT = "//dev//ttyIMU";
const int BAUD_RATE = 115200;

void asyncDataListener(void* sender, VnDeviceCompositeData* data)
{
  IMUData imu(data->ypr.roll, data->ypr.pitch, data->ypr.yaw, 
              data->acceleration.c0, data->acceleration.c1, data->acceleration.c2); 

  g_buf_mutex.lock();
    g_imu_data = imu; 
  g_buf_mutex.unlock(); 
  g_new_data = true; 
}

}

CIMUReader::CIMUReader()
{
  init(); 
}
CIMUReader::~CIMUReader()
{
  uninit(); 
}

IMUData CIMUReader::getNewIMUData()
{
  IMUData ret; 
  if(!mbOK)
  {
    printf("imu_reader: failed to connect to IMU!"); 
    return ret; 
  }
  
  while(!g_new_data) usleep(100); 
  g_new_data = false; 
  g_buf_mutex.lock(); 
    ret = g_imu_data;
  g_buf_mutex.unlock(); 
  return ret; 
}

void CIMUReader::init()
{
  VN_ERROR_CODE errorCode = vn100_connect(&mVn100, COM_PORT, BAUD_RATE); 
  /* Make sure the user has permission to use the COM port. */
  if (errorCode == VNERR_PERMISSION_DENIED) 
  {
    printf("Current user does not have permission to open the COM port.\n");
    printf("Try running again using 'sudo'.\n");
    mbOK = false; 
  }
  else if (errorCode != VNERR_NO_ERROR)
  {
    printf("Error encountered when trying to connect to the sensor.\n");
    mbOK = false; 
  }else{
  
    printf("imu_reader: succeed to connect to vn100"); 
    mbOK = true; 
  }
  if(!mbOK) 
    return ; 

  /* Disable ASCII asynchronous messages since we want to demonstrate the
     the binary asynchronous messages. */
  errorCode = vn100_setAsynchronousDataOutputType(
      &mVn100,
      VNASYNC_OFF, // VNASYNC_VNYBA
      true);

  /* Now configure the binary messages output. Notice how the configuration
     flags can be joined using the binary OR. */
  errorCode = vn100_setBinaryOutput1Configuration(
      &mVn100,
      BINARY_ASYNC_MODE_SERIAL_1,		/* Data will be output on serial port 1. This should be the one we are connected to now. */
      4,							/* Outputting binary data at 4 Hz (800 Hz on-board filter / 200 = 4 Hz). */
      // BG1_YPR | BG1_ANGULAR_RATE | BG1_ACCEL | BG1_TIME_STARTUP | BG1_TIME_GPS | BG1_TIME_SYNC_IN, 
      BG1_YPR | BG1_ANGULAR_RATE | BG1_ACCEL | BG1_TIME_STARTUP,
      BG3_NONE,
      BG5_NONE,
      true);
  /* Now register to receive notifications when a new binary asynchronous
     packet is received. */
  errorCode = vn100_registerAsyncDataReceivedListener(&mVn100, &asyncDataListener);
  return ; 
}

void CIMUReader::uninit()
{
  VN_ERROR_CODE errorCode;
  errorCode = vn100_unregisterAsyncDataReceivedListener(&mVn100, &asyncDataListener);
  errorCode = vn100_disconnect(&mVn100);

  if (errorCode != VNERR_NO_ERROR)
  {
    printf("Error encountered when trying to disconnect from the sensor.\n");
  }

  return ; 
}
