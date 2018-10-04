#include <stdio.h>
#include <unistd.h>
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>
#include "vectornav.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Imu.h"

using namespace std; 

#define D2R(d) ((d*M_PI)/180.)

/* Change the connection settings to your configuration. */
const char* const COM_PORT[2] = {"//dev//ttyUSB0", "//dev//ttyUSB1"};
const int BAUD_RATE = 115200;
static int cnt = 0; 

void asyncDataListener(
	void* sender,
	VnDeviceCompositeData* data);

int imu_record();

string fname = "./imu_vn100.log";
ofstream* pf = 0; 
ros::Publisher euler_pub; 
ros::Publisher imu_msg_pub; 
bool b_save_imu = true; 
bool b_publish_rpy = false ;
bool b_publish_imu_msg = false; 
bool b_print_out = true; 

int usb01 = 0; 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "imu_recorder"); 
  ros::NodeHandle n; 

  // init parameters 
  ros::NodeHandle np("~"); 
  np.param("imu_record_file", fname, fname); 
  np.param("publish_rpy", b_publish_rpy, b_publish_rpy); 
  np.param("publish_imu_msg", b_publish_imu_msg, b_publish_imu_msg); 
  np.param("save_imu", b_save_imu, b_save_imu); 
  np.param("ttyUSB_id", usb01, usb01); 
  np.param("print_out", b_print_out, b_print_out); 

  if(b_save_imu && fname != "")
  {
    pf = new ofstream(fname.c_str()); 
    if(!pf->is_open())
    {
      ROS_ERROR("%s failed to open file %s, exist", __FILE__, fname.c_str());
      pf = 0; 
    }else{
      ROS_WARN("%s try to record imu data in file %s", __FILE__, fname.c_str());
    }
  }
  if(b_publish_rpy)
  {
    ROS_INFO("publish msg '/euler_msg' ");
    euler_pub = n.advertise<std_msgs::Float32MultiArray>("/euler_msg", 10); 
  }
  
  if(b_publish_imu_msg)
  {
    ROS_INFO("imu_vn100: publish imu msg /imu0");
    imu_msg_pub = n.advertise<sensor_msgs::Imu>("/imu0", 1000);  
  }


  imu_record(); 
  
  if(pf!=0) 
  {
    pf->close(); 
    delete pf;
  }

  return 0; 
}

int imu_record()
{
	VN_ERROR_CODE errorCode;
	Vn100 vn100;

	errorCode = vn100_connect(
		&vn100,
		COM_PORT[usb01],
		BAUD_RATE);

	/* Make sure the user has permission to use the COM port. */
	if (errorCode == VNERR_PERMISSION_DENIED) {

		printf("Current user does not have permission to open the COM port.\n");
		printf("Try running again using 'sudo'.\n");

		return 0;
	}
	else if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to connect to the sensor.\n");

		return 0;
	}

	/* Disable ASCII asynchronous messages since we want to demonstrate the
	   the binary asynchronous messages. */
	errorCode = vn100_setAsynchronousDataOutputType(
        &vn100,
        VNASYNC_OFF, // VNASYNC_VNYBA
        true);

	/* Now configure the binary messages output. Notice how the configuration
	   flags can be joined using the binary OR. */
	errorCode = vn100_setBinaryOutput1Configuration(
		&vn100,
		BINARY_ASYNC_MODE_SERIAL_1,		/* Data will be output on serial port 1. This should be the one we are connected to now. */
		4,							/* Outputting binary data at 4 Hz (800 Hz on-board filter / 200 = 4 Hz). */
		// BG1_YPR | BG1_ANGULAR_RATE | BG1_ACCEL | BG1_TIME_STARTUP | BG1_TIME_GPS | BG1_TIME_SYNC_IN, 
                BG1_YPR | BG1_ANGULAR_RATE | BG1_ACCEL | BG1_TIME_STARTUP,
		BG3_NONE,
		BG5_NONE,
		true);

	printf("Yaw, Pitch, Roll\n");

	/* Now register to receive notifications when a new binary asynchronous
	   packet is received. */
	errorCode = vn100_registerAsyncDataReceivedListener(&vn100, &asyncDataListener);

	/* Sleep for 10 seconds. Data will be received by the asycDataListener
	   during this time. */
        while(ros::ok())
        {
          ros::spinOnce(); 
          usleep(1000*100);  
	    // sleep(10);
        }

        // printf("fps: cnt %f = %d / 10 \n", (float)cnt/10., cnt);

	errorCode = vn100_unregisterAsyncDataReceivedListener(&vn100, &asyncDataListener);
	
	errorCode = vn100_disconnect(&vn100);
	
	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to disconnect from the sensor.\n");
		
		return 0;
	}

	return 0;

}

// double last_t = 0; 
// double last_internal_t = 0;

void asyncDataListener(void* sender, VnDeviceCompositeData* data)
{
        static ros::Time t = ros::Time::now();  // used for synchronization 
        static unsigned long last_t = data->timeStartup; 
        unsigned long elaps_s = data->timeStartup - last_t; 

	if(b_print_out)
	{
	printf(" %i startUp %li timeelaspsed: %f ms data: %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f\n", ++cnt, 
		// t.toSec()*1000 - last_t,
                // data->timeStartup* 1e-6 - last_internal_t,
                data->timeStartup,
                elaps_s*1e-6, 
                data->ypr.yaw,
		data->ypr.pitch,
		data->ypr.roll, 
                data->acceleration.c0,
                data->acceleration.c1, 
                data->acceleration.c2, 
                data->angularRate.c0, 
                data->angularRate.c1, 
                data->angularRate.c2);
	}
        // printf("befor add elaps_s: %lu\n", t.toNSec());
        // t.fromSec(t.toSec() + elaps_s); 
        t += ros::Duration(0, elaps_s);
        // printf("after add elaps_s: %lu\n", t.toNSec());
        // ros::Time rt = ros::Time::fromSec(elaps_s); 
        // t = t + rt; // ros::Time::fromSec(elaps_s); 
        
        // last_t = t.toSec()*1000; 
        last_t = data->timeStartup ; 
        /*printf(" timeStartup: %ld timeGps: %ld timeSyncIn: %ld gpsToSec: %lf gpsToNs: %ld gpsWeek: %d\n", 
                data->timeStartup, 
                data->timeGps, 
                data->timeSyncIn, 
                data->gpsTowSec,
                data->gpsTowNs, 
                data->gpsWeek); */
        if(b_publish_rpy)
        {
          std_msgs::Float32MultiArray msg; 
          msg.data.resize(3); 
          msg.data[0] = D2R(data->ypr.roll); 
          msg.data[1] = D2R(data->ypr.pitch); 
          msg.data[2] = D2R(data->ypr.yaw); 
          euler_pub.publish(msg); 
          ros::spinOnce(); 
        }

        if(pf != 0)
        {
          stringstream ss; 
          ss << t;
          // ss << t.sec<<"."<<t.nsec;
          // (*pf) << std::fixed<< t.toSec()<<"\t"<< 
            (*pf) << std::fixed<< ss.str()<<"\t"<< 
                data->acceleration.c0 << "\t"<<
                data->acceleration.c1 << "\t"<<
                data->acceleration.c2 << "\t"<<
                data->angularRate.c0 << "\t"<<
                data->angularRate.c1 << "\t"<<
                data->angularRate.c2 <<"\t" <<  
                data->ypr.yaw << "\t"<<
		data->ypr.pitch << "\t"<<
		data->ypr.roll<< "\t"<< endl; 
        }
        
        if(b_publish_imu_msg)
	{
	  sensor_msgs::Imu imu_msg; 
	  imu_msg.header.stamp = t; 
	  imu_msg.linear_acceleration.x = data->acceleration.c0; 
	  imu_msg.linear_acceleration.y = data->acceleration.c1; 
          imu_msg.linear_acceleration.z = data->acceleration.c2; 
          imu_msg.angular_velocity.x = data->angularRate.c0; 
	  imu_msg.angular_velocity.y = data->angularRate.c1; 
	  imu_msg.angular_velocity.z = data->angularRate.c2; 
	  imu_msg_pub.publish(imu_msg); 
	  ros::spinOnce(); 
	}

}
/*
void asyncDataListener(void* sender, VnDeviceCompositeData* data)
{
	printf(" %i data: %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f\n", ++cnt, 
		data->ypr.yaw,
		data->ypr.pitch,
		data->ypr.roll, 
                data->acceleration.c0,
                data->acceleration.c1, 
                data->acceleration.c2, 
                data->angularRate.c0, 
                data->angularRate.c1, 
                data->angularRate.c2);
}*/
