
project(imu_recorder)

####################
## vn100 library ##
###################
add_library(vn100 vn100.c vncp_services.c vndevice.c)
target_link_libraries(vn100 pthread rt)

add_executable(imu_vn100_recorder imu_vn100.cpp)
target_link_libraries(imu_vn100_recorder vn100 ${catkin_LIBRARIES})
