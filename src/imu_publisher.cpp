
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <boost/bind.hpp>
#include "beaglebone/IMU.h"
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

#include <MPU6000IMU.h>
#define PI 3.14159265359

//using namespace DemconRobot;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "IMU_publisher");
    ros::NodeHandle n;
    ros::NodeHandle priv_nh("");
    std::string imu_id;
    priv_nh.param("imu_id", imu_id, std::string("/base_imu"));

    MPU6000IMU* imuSensor = new MPU6000IMU(2, 0x68);
    sensor_msgs::Imu imu;
    imu.header.frame_id = imu_id;

    imuSensor->initModule();

    short ACCx, ACCy, ACCz, GYRx, GYRy, GYRz, temp;


    ros::Publisher Imu_pub = n.advertise<sensor_msgs::Imu>("/imu_data", 1);
    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        imuSensor->getData(&ACCx, &ACCy, &ACCz, &GYRx, &GYRy, &GYRz, &temp);
        imu.header.stamp = ros::Time::now();
        imu.angular_velocity.x = GYRx * (PI/(131*180));
        imu.angular_velocity.y = GYRy * (PI/(131*180));
        imu.angular_velocity.z = GYRz * (PI/(131*180));

        imu.orientation = tf::createQuaternionMsgFromYaw(ACCz * (PI/(131*180)));

        imu.linear_acceleration.x = ACCx * (9.81/16834);
        imu.linear_acceleration.y = ACCy * (9.81/16834);
        imu.linear_acceleration.z = 0;

        Imu_pub.publish(imu);
        loop_rate.sleep();
        ros::spinOnce();
    }
}
