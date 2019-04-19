/*
 * Jan. 20 2018, He Zhang, hxzhang1@ualr.edu 
 *
 * IMU interface 
 *
 * */

#pragma once 

#include <stdio.h>
#include <unistd.h>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>
#include "vectornav.h"
using namespace std; 

#define D2R(d) ((d*M_PI)/180.)

class IMUData
{
  public:
    double max, may, maz;
    double mroll, mpitch, myaw; 
    double melapsed_time; // ms 
    IMUData(){}
    IMUData(double r, double p, double yaw, double ax, double ay, double az):
      max(ax), may(ay), maz(az), mroll(r), mpitch(p), myaw(yaw){}
};

class CIMUReader
{
public:
    CIMUReader();
    virtual ~CIMUReader(); 
    void init(); 
    void uninit(); 
    IMUData getNewIMUData(); // return the newest IMU reading

    Vn100 mVn100; // IMU 
    bool mbOK; // whether connect to imu

};
