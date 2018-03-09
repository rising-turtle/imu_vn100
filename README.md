# Wrapper to read data from IMU

## src folder contains warpper to read data from VN100
https://www.vectornav.com/products/vn-100

**imu_vn100.cpp**: A simple example to read data from VN100  
**imu_reader.h/cpp**: An interface to read data from VN100 

## add_ADIS_imu
contains an example to digest the data obtained by ADIS 

**99-usb-serial.rules** are used to dinstinguish the IMU serial port when multiple serial connections exist, the idea can be found in the following link  
http://hintshop.ludvig.co.nz/show/persistent-names-usb-serial-devices/

