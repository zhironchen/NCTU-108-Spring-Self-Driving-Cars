/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#ifndef __ODOM_ESTIMATION_NODE__
#define __ODOM_ESTIMATION_NODE__

// ros stuff
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "odom_estimation.h"
#include <robot_pose_ekf/GetStatus.h>

// messages
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <boost/thread/mutex.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
// log files
#include <fstream>
#include <visualization_msgs/Marker.h>
#include <cmath>
namespace estimation
{

/** \mainpage
 *  \htmlinclude manifest.html
 * 
 * <b>Package Summary</b>
 * This package provides two main classes: 
 *  1) OdomEstimation performs all sensor fusion operations, and 
 *  2) OdomEstimationNode provides a ROS wrapper around OdomEstimation
*/

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
typedef boost::shared_ptr<sensor_msgs::Imu const> ImuConstPtr;
typedef boost::shared_ptr<nav_msgs::Odometry const> VoConstPtr;
typedef boost::shared_ptr<nav_msgs::Odometry const> GpsConstPtr;
typedef boost::shared_ptr<geometry_msgs::Twist const> VelConstPtr;

struct Pose
{
    /*
    Orientation orien;
    Position pos;
    */
    Eigen::Vector3d pos;
    Eigen::Matrix3d orien;
};

class ImuIntegrator
{
private:
    Pose pose;
    ros::Time time;
    Eigen::Vector3d gravity;
    Eigen::Vector3d velocity;
    visualization_msgs::Marker path;
    ros::Publisher line_pub;
    double deltaT;
    bool first_frame;
public:
    //! Constructor.
    ImuIntegrator(const ros::Publisher&);
    //! Destructor.
    ~ImuIntegrator();

    //! Callback function for dynamic reconfigure server.
    //void configCallback(node_example::node_example_paramsConfig &config, uint32_t level);

    //! Publish the message.
    void publishMessage();

    //! Callback function for subscriber.
    void Imu_Callback(const sensor_msgs::Imu &msg);

    void setGravity(const geometry_msgs::Vector3 &msg);
    void updatePath(const Eigen::Vector3d &msg);
    void calcPosition(const geometry_msgs::Vector3 &msg);
    void calcOrientation(const geometry_msgs::Vector3 &msg);
};

class OdomEstimationNode
{
public:
  /// constructor
  OdomEstimationNode();

  /// destructor
  virtual ~OdomEstimationNode();

private:
  /// the mail filter loop that will be called periodically
  void spin(const ros::TimerEvent& e);

  /// callback function for odo data
  void odomCallback(const OdomConstPtr& odom);

  /// callback function for imu data
  void imuCallback(const ImuConstPtr& imu);

  /// callback function for vo data
  void voCallback(const VoConstPtr& vo);

  /// callback function for vo data
  void gpsCallback(const GpsConstPtr& gps);


  /// get the status of the filter
  bool getStatus(robot_pose_ekf::GetStatus::Request& req, robot_pose_ekf::GetStatus::Response& resp);

  ros::NodeHandle node_;
  ros::Timer timer_;
  ros::Publisher pose_pub_, line;
  ros::Subscriber odom_sub_, imu_sub_, vo_sub_,gps_sub_, imu_data_sub;
  ros::ServiceServer state_srv_;

  // ekf filter
  OdomEstimation my_filter_;

  // estimated robot pose message to send
  geometry_msgs::PoseWithCovarianceStamped  output_; 

  // robot state
  tf::TransformListener    robot_state_;
  tf::TransformBroadcaster odom_broadcaster_;

  // vectors
  tf::Transform odom_meas_, imu_meas_, vo_meas_, gps_meas_;
  tf::Transform base_vo_init_;
  tf::Transform base_gps_init_;
  tf::StampedTransform camera_base_;
  ros::Time odom_time_, imu_time_, vo_time_, gps_time_;
  ros::Time odom_stamp_, imu_stamp_, vo_stamp_, gps_stamp_, filter_stamp_;
  ros::Time odom_init_stamp_, imu_init_stamp_, vo_init_stamp_, gps_init_stamp_;
  bool odom_active_, imu_active_, vo_active_, gps_active_;
  bool odom_used_, imu_used_, vo_used_, gps_used_;
  bool odom_initializing_, imu_initializing_, vo_initializing_, gps_initializing_;
  double timeout_;
  MatrixWrapper::SymmetricMatrix odom_covariance_, imu_covariance_, vo_covariance_, gps_covariance_;
  bool debug_, self_diagnose_;
  std::string output_frame_, base_footprint_frame_, tf_prefix_, imu_frame_;

  // log files for debugging
  std::ofstream odom_file_, imu_file_, vo_file_, gps_file_, corr_file_, time_file_, extra_file_;

  // counters
  unsigned int odom_callback_counter_, imu_callback_counter_, vo_callback_counter_,gps_callback_counter_, ekf_sent_counter_;

}; // class

}; // namespace

#endif
