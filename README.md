# wit_imu_ros

ROS package for WIT inclinometer devices (http://www.wit-motion.com/english.php?m=text&a=index&classify_id=46396)

## Supported devices
- WIT901B
- WIT901C (Yet to test)

## How to compile
- It is assumed that catkin workspace is available already.
  - cd <your_path>/catkin_ws/src
  - clone this respository
  - cd ..
  - catkin_make
  - source devel/setup.bash

## How to launch
There are 3 options to launch the package.
1) Launch without IMU madgwick filter, magnetic data fusion and IMU transformer
  - roslaunch wit_imu_ros wit901_imu.launch
  
2) Launch with IMU madgwick filter, magnetic data fusion and IMU transformer
  - roslaunch wit_imu_ros wit901_filter.launch
  
  Please note ros-melodic-imu-filter-madgwick, ros-melodic-imu-transformer packages to be installed.

3) Launch with IMU madgwick filter, magnetic data fusion and IMU transformer and visualize through Rviz
  - roslaunch wit_imu_ros wit901_imu_test.launch
  
  Please note ros-melodic-imu-tools and other package mentioned in filter launch to be installed.
  
## Published topics
- /wit/imu_data         (sensor_msgs/Imu Message)
- /wit/mag_data         (sensor_msgs/MagneticField)
- /wit/temparture_data  (sensor_msgs/Temperature)
- /wit/pressure_data    (sensor_msgs/FluidPressure)
