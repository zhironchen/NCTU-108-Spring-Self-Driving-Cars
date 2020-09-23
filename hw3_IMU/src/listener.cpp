#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "listener.h"
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Publisher line = n.advertise<visualization_msgs::Marker>("draw_imu_path", 100);
  ImuIntegrator *imu_integrator = new ImuIntegrator(line);
  ros::Subscriber sub = n.subscribe("imu/data", 1000, &ImuIntegrator::Imu_Callback, imu_integrator);
  ros::spin();
  return 0;
}


ImuIntegrator::ImuIntegrator(const ros::Publisher &pub)
{
    Eigen::Vector3d zero(0, 0, 0);
    pose.pos = zero;
    pose.orien = Eigen::Matrix3d::Identity();
    velocity = zero;
    line_pub = pub;  
    first_frame = true;

    // let line is blue
    path.color.b = 1.0;
    path.color.a = 1.0;

    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.header.frame_id = "/global";   
    path.action = visualization_msgs::Marker::ADD;
    path.scale.x = 0.3;
    path.pose.orientation.w = 1.0;
    geometry_msgs::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    path.points.push_back(p);  
}
void ImuIntegrator::Imu_Callback(const sensor_msgs::Imu &msg)
{
    //recording first frame
    if (first_frame) {
        time = msg.header.stamp;
        deltaT = 0;
        setGravity(msg.linear_acceleration);
        first_frame = false;
    }
    else {
        deltaT = (msg.header.stamp - time).toSec();
        time = msg.header.stamp;
         (msg.angular_velocity);
        calcPosition(msg.linear_acceleration);
        updatePath(pose.pos);
        publishMessage();
    }
    std::cout << pose.pos << std::endl;
}
void ImuIntegrator::setGravity(const geometry_msgs::Vector3 &msg)
{
    gravity[0] = msg.x;
    gravity[1] = msg.y;
    gravity[2] = msg.z;
}
void ImuIntegrator::updatePath(const Eigen::Vector3d &msg)
{
    geometry_msgs::Point p;
    p.x = msg[0];
    p.y = msg[1];
    p.z = msg[2];
    path.points.push_back(p);
}
void ImuIntegrator::publishMessage()
{
    line_pub.publish(path);
}
void ImuIntegrator::calcOrientation(const geometry_msgs::Vector3 &msg)
{
    Eigen::Matrix3d B;
    B << 0,                -msg.z * deltaT,     msg.y * deltaT,
        msg.z * deltaT,    0,                  -msg.x * deltaT,
        -msg.y * deltaT,    msg.x * deltaT,     0;
    double sigma = std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2) + std::pow(msg.z, 2)) * deltaT;
    //std::cout << "sigma: " << sigma << std::endl << Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B << std::endl << pose.orien << std::endl;
    pose.orien = pose.orien * (Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B - ((1 - std::cos(sigma)) / std::pow(sigma, 2)) * B * B);
}
void ImuIntegrator::calcPosition(const geometry_msgs::Vector3 &msg)
{
    Eigen::Vector3d acc_l(msg.x, msg.y, msg.z);
    Eigen::Vector3d acc_g = pose.orien * acc_l;
    //Eigen::Vector3d acc(msg.x - gravity[0], msg.y - gravity[1], msg.z - gravity[2]);
    velocity = velocity + deltaT * (acc_g - gravity);
    pose.pos = pose.pos + deltaT * velocity;
}