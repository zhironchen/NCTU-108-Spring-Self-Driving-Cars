#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2_msgs/TFMessage.h"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include "fstream"
#include "sstream"
struct Transform{
    double x, y, z, roll, yaw, pitch;
};
struct Current_pose{
    double x, y, z, roll, yaw, pitch;
};
class ICP_Cloud_Matcher{
public: 
    ros::Time scan_stamp;
    pcl::PCLPointCloud2 cloud_blob;
    pcl::PCLPointCloud2 cloud_temp;
    pcl::PCLPointCloud2 cloud_pcd_combined;
    double current_score, current_score_temp;
    int k, ittr;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    Current_pose current_pose;
    Transform transformed;
    std::fstream file;
    tf::Transform transform_scan2car;
    tf::Transform transform, transform_record;
    tf::TransformBroadcaster br;
    int initial_flag, gps_flag, imu_flag, tf_flag, first_frame_flag, read_file_flag;
    Eigen::Matrix4f initial_guess, gps_matrix, icp_transformed, icp_transformed_temp;
    Eigen::Matrix3d rotation_matrix;
    //Eigen::Quaterniond imu_eq;
    geometry_msgs::Quaternion quat_msg;
    //Eigen::Quaterniond imu_q;
    tf::Matrix3x3 mat_local, rotation_tf_matrix;
    ros::Publisher transformed_pt_pub, map_pub, scan_pub;
    void Imu_Callback(const sensor_msgs::Imu &imu_msg);
    void Point_Cloud_Callback(const sensor_msgs::PointCloud2ConstPtr &pc2_msg);
    void GPS_Callback(const geometry_msgs::PointStamped &gps_msg);
    void TF_Callback(const tf2_msgs::TFMessage &tf);
    void Setting_Initial_Guess();
    void tf_broadcast();
    void write_file();
    void Load_PCD();
};