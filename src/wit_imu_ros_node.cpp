/*
Copyright (c) 2020 Balamurugan Kandan
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <serial/serial.h>
#include "wit_imu_parser.h"

boost::array<double, 9ul> setCovariance(XmlRpc::XmlRpcValue rpc) {
    boost::array<double, 9ul> output = { 0.0 };

    ROS_ASSERT(rpc.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for(int i = 0; i < 9; i++){
        ROS_ASSERT(rpc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        output[i] = (double)rpc[i];
    }
    return output;
}

int main(int argc, char *argv[]) {
    std::string port;
    int baudrate;
    int pub_rate;
    //bool pub_mag;
    //bool pub_temperature;
    //bool pub_pressure;
    std::string imu_topic;
    std::string mag_topic;
    std::string temperature_topic;
    std::string pressure_topic;
    std::string frame_id;
    bool tf_ned_to_enu;
    bool frame_based_enu;
    serial::Serial serial;
    boost::array<double, 9ul> linear_accel_covariance = { 1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6 };
    boost::array<double, 9ul> angular_vel_covariance = { 1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6 };
    boost::array<double, 9ul> orientation_covariance = { 1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6 };
    boost::array<double, 9ul> magnetic_covariance = { 1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6 };
    XmlRpc::XmlRpcValue rpc_temp;
    ros::Publisher pubIMU, pubMag, pubTemp, pubPres;
    int num = 0; int count = 0;

    ros::init(argc, argv, "wit_imu_ros_node");
    ROS_INFO("wit_imu_ros_node initilized");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.param<std::string>("serial_port", port, "/dev/ttyUSB0");
    nh_private.param<int>("serial_baud", baudrate, 115200);
    nh_private.param<int>("pub_rate", pub_rate, 20);
    nh_private.param<std::string>("frame_id", frame_id, "wit_imu_link");
    nh_private.param<bool>("tf_ned_to_enu", tf_ned_to_enu, false);
    nh_private.param<bool>("frame_based_enu", frame_based_enu, false);
    nh_private.param<std::string>("imu_topic", imu_topic, "imu_data");
    nh_private.param<std::string>("mag_topic", mag_topic, "mag_data");
    nh_private.param<std::string>("temperature_topic", temperature_topic, "temperature_data");
    nh_private.param<std::string>("pressure_topic", pressure_topic, "pressure_data");
    //
    // TODO: Does enable / disable topic functionality useful???
    //
    //nh_private.param<bool>("publish_mag", pub_mag, true);
    //nh_private.param<bool>("publish_temperature", pub_temperature, true);
    //nh_private.param<bool>("publish_pressure", pub_pressure, true);

    //Call to set covariances
    if(nh_private.getParam("linear_accel_covariance",rpc_temp)) {
        linear_accel_covariance = setCovariance(rpc_temp);
    }
    if(nh_private.getParam("angular_vel_covariance",rpc_temp)) {
        angular_vel_covariance = setCovariance(rpc_temp);
    }
    if(nh_private.getParam("orientation_covariance",rpc_temp)) {
        orientation_covariance = setCovariance(rpc_temp);
    }
    if(nh_private.getParam("magnetic_covariance",rpc_temp)) {
        magnetic_covariance = setCovariance(rpc_temp);
    }

    CJY901 imu = CJY901();
    try {
        serial.setPort(port);
        serial.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(500);
        serial.setTimeout(to);
        serial.open();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Serial port exception occured! Please check the connection");
        return -1;
    }
    if (serial.isOpen()) {
        ROS_INFO_STREAM("Serial Port opened");
    } else {
        return -1;
    }
    ros::Rate loop_rate(pub_rate);
    while (ros::ok()) {
        count = serial.available();
        num = 0;
        if (count != 0) {
            ROS_INFO_ONCE("Data received from serial port");
            unsigned char read_buf[count];
            num = serial.read(read_buf, count);
            imu.FetchData(read_buf, num);
            sensor_msgs::Imu imu_data;

            imu_data.header.stamp = ros::Time::now();
            imu_data.header.frame_id = frame_id;
            //Quaternion message comes in as a Yaw (z) pitch (y) Roll (x) format
            if (tf_ned_to_enu) {
                // If we want the orientation to be based on the reference label on the imu
                tf2::Quaternion tf2_quat(imu.stcQuat.x,imu.stcQuat.y,imu.stcQuat.z,imu.stcQuat.w);
                geometry_msgs::Quaternion quat_msg;

                if(frame_based_enu) {
                    // Create a rotation from NED -> ENU
                    tf2::Quaternion q_rotate;
                    q_rotate.setRPY (M_PI, 0.0, M_PI/2);
                    // Apply the NED to ENU rotation such that the coordinate frame matches
                    tf2_quat = q_rotate*tf2_quat;
                    quat_msg = tf2::toMsg(tf2_quat);

                    // Since everything is in the normal frame, no flipping required
                    imu_data.angular_velocity.x = imu.stcGyro.w[0];
                    imu_data.angular_velocity.y = imu.stcGyro.w[1];
                    imu_data.angular_velocity.z = imu.stcGyro.w[2];
                    imu_data.linear_acceleration.x = imu.stcAcc.a[0];
                    imu_data.linear_acceleration.y = imu.stcAcc.a[1];
                    imu_data.linear_acceleration.z = imu.stcAcc.a[2];
                } else {
                    // put into ENU - swap X/Y, invert Z
                    quat_msg.x = imu.stcQuat.y;
                    quat_msg.y = imu.stcQuat.x;
                    quat_msg.z = -(imu.stcQuat.z);
                    quat_msg.w = imu.stcQuat.w;

                    // Flip x and y then invert z
                    imu_data.angular_velocity.x = imu.stcGyro.w[1];
                    imu_data.angular_velocity.y = imu.stcGyro.w[0];
                    imu_data.angular_velocity.z = -(imu.stcGyro.w[2]);
                    // Flip x and y then invert z
                    imu_data.linear_acceleration.x = imu.stcAcc.a[1];
                    imu_data.linear_acceleration.y = imu.stcAcc.a[0];
                    imu_data.linear_acceleration.z = -(imu.stcAcc.a[2]);
                }

                imu_data.orientation = quat_msg;
            } else {
                imu_data.orientation.x = imu.stcQuat.x;
                imu_data.orientation.y = imu.stcQuat.y;
                imu_data.orientation.z = imu.stcQuat.z;
                imu_data.orientation.w = imu.stcQuat.w;

                imu_data.angular_velocity.x = imu.stcGyro.w[0];
                imu_data.angular_velocity.y = imu.stcGyro.w[1];
                imu_data.angular_velocity.z = imu.stcGyro.w[2];
                imu_data.linear_acceleration.x = imu.stcAcc.a[0];
                imu_data.linear_acceleration.y = imu.stcAcc.a[1];
                imu_data.linear_acceleration.z = imu.stcAcc.a[2];
            }
            // Covariances pulled from parameters
            imu_data.orientation_covariance = orientation_covariance;
            imu_data.angular_velocity_covariance = angular_vel_covariance;
            imu_data.linear_acceleration_covariance = linear_accel_covariance;
            //
            // Publish imu topic
            //
            pubIMU = nh.advertise<sensor_msgs::Imu>(imu_topic, 1000);
            pubIMU.publish(imu_data);
            //
            // Publish temperature topic
            //
            pubTemp = nh.advertise<sensor_msgs::Temperature>(temperature_topic, 1000);
            sensor_msgs::Temperature msgTemp;
            msgTemp.header.stamp = imu_data.header.stamp;
            msgTemp.header.frame_id = imu_data.header.frame_id;
            msgTemp.temperature = imu.stcAcc.T;
            pubTemp.publish(msgTemp);
            // Magnatometer
            if (imu.isMagDataAvailable()) {
                pubMag = nh.advertise<sensor_msgs::MagneticField>(mag_topic, 1000);
                sensor_msgs::MagneticField mag_data;
                mag_data.header.stamp = imu_data.header.stamp;
                mag_data.header.frame_id = imu_data.header.frame_id;
                mag_data.magnetic_field.x = imu.stcMag.h[0];
                mag_data.magnetic_field.y = imu.stcMag.h[1];
                mag_data.magnetic_field.z = imu.stcMag.h[2];
                mag_data.magnetic_field_covariance = magnetic_covariance;
                pubMag.publish(mag_data);
            }
            // Barometer
            if (imu.isPressureDataAvailable()) {
                pubPres = nh.advertise<sensor_msgs::FluidPressure>(pressure_topic, 1000);
                sensor_msgs::FluidPressure msgPres;
                msgPres.header.stamp = imu_data.header.stamp;
                msgPres.header.frame_id = imu_data.header.frame_id;
                msgPres.fluid_pressure = imu.stcPress.lPressure;
                pubPres.publish(msgPres);
            }
        }

        loop_rate.sleep();
    }

    serial.close();
	return 0;
}