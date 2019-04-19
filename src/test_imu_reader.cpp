/*
 *  Jan. 20 2018, He Zhang, hxzhang1@ualr.edu 
 *
 *  test imu_reader print the returned IMU data 
 *
 * */

#include "imu_reader.h"

int main(int argc, char* argv[])
{
  CIMUReader imu_reader; 
  
  for(int i=0; i<=400; i++)
  {
    IMUData d = imu_reader.getNewIMUData(); 
    printf(" %d elapsed time = %lf data:  %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f\n", i, d.melapsed_time, d.mroll, 
          d.mpitch, d.myaw, d.max, d.may, d.maz);
  }

  return 0; 
}

