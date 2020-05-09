#include "icp_cloud_match.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include <iostream>
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_msgs/TFMessage.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/package.h>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "fstream"
#include "sstream"
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;


int main(int argc, char **argv){
    ros::init(argc, argv, "icp_cloud_match");
    ros::NodeHandle n;
    ICP_Cloud_Matcher *pt_cloud_integrator = new ICP_Cloud_Matcher();
    
    pt_cloud_integrator->transformed_pt_pub = n.advertise<sensor_msgs::PointCloud2>("transformed_pc", 100);
    pt_cloud_integrator->map_pub = n.advertise<sensor_msgs::PointCloud2>("map_c", 100);
    pt_cloud_integrator->scan_pub = n.advertise<sensor_msgs::PointCloud2>("scan_c", 100);
    pt_cloud_integrator->initial_guess = Eigen::Matrix4f::Zero();
    pt_cloud_integrator->initial_flag = 0;
    pt_cloud_integrator->gps_flag = 0;
    pt_cloud_integrator->imu_flag = 0;
    pt_cloud_integrator->tf_flag = 0;
    pt_cloud_integrator->read_file_flag = 0;
    pt_cloud_integrator->first_frame_flag = 0;
    pt_cloud_integrator->k = 0;
    pt_cloud_integrator->current_score_temp = 50;
    ros::Subscriber sub_imu = n.subscribe("/imu/data", 10000, &ICP_Cloud_Matcher::Imu_Callback, pt_cloud_integrator);
    ros::Subscriber sub_pc  = n.subscribe("/lidar_points", 10000, &ICP_Cloud_Matcher::Point_Cloud_Callback, pt_cloud_integrator);
    ros::Subscriber sub_gps = n.subscribe("/fix", 10000, &ICP_Cloud_Matcher::GPS_Callback, pt_cloud_integrator);
    ros::Subscriber sub_tf  = n.subscribe("/tf", 10000, &ICP_Cloud_Matcher::TF_Callback, pt_cloud_integrator);
    ros::spin();    
    return 0;
}

void ICP_Cloud_Matcher::TF_Callback(const tf2_msgs::TFMessage &tf){
   if(tf_flag == 0){
    
    //std::cout << "Got TF" << std::endl;
    }
}

void ICP_Cloud_Matcher::Imu_Callback(const sensor_msgs::Imu &imu_msg){ //
  if(imu_flag == 0){
      //tf::Quaternion myQuaternion;
      //myQuaternion.setRPY( 0.0244197710602082, -0.0387098077352205, -2.19931039711146);
      /*
      quat_msg = imu_msg.orientation;
      tf::Quaternion imu_q(quat_msg.x,quat_msg.y,quat_msg.z,quat_msg.w);
      imu_matrix.setRotation(imu_q); //myQuaternion
      tf::matrixTFToEigen(imu_matrix, imu_eigen);
      std::cout << imu_eigen << std::endl;
     
      std::cout << "Got IMU" << imu_flag << std::endl;
      */
      imu_flag = 1;
    }
}

void ICP_Cloud_Matcher::GPS_Callback(const geometry_msgs::PointStamped &gps_msg){//setorigin
  if(gps_flag == 0){
      gps_matrix = Eigen::Matrix4f::Zero();
      gps_matrix(0,3) = gps_msg.point.x;
      gps_matrix(1,3) = gps_msg.point.y;
      gps_matrix(2,3) = gps_msg.point.z;
      gps_matrix(3,3) = 1;
      gps_flag = 1;
      std::cout << gps_matrix(0,3) << std::endl;
      std::cout << "Got GPS" << gps_flag << std::endl;
  }
}

void ICP_Cloud_Matcher::Point_Cloud_Callback(const sensor_msgs::PointCloud2ConstPtr &pc2_msg){

//Declare Point Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut_new(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::PCLPointCloud2 cloud_blob2;
  pcl::PCLPointCloud2 cloud_blob3;
  pcl::PCLPointCloud2 cloud_blob4;
  pcl::PCLPointCloud2 cloud_blob5;
  pcl::PCLPointCloud2 cloud_blob6;
  pcl::PCLPointCloud2 cloud_combined;
  scan_stamp = pc2_msg->header.stamp;
// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::fromROSMsg (*pc2_msg, *cloud_in);
  std::cout << "scan in" << std::endl;

  
  
//Load map from pcd file
/*
if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/toby/catkin_ws/src/inf_icp/src/map.pcd", *cloud2_in) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file map.pcd \n");
  }
  pcl::io::loadPCDFile("/home/toby/catkin_ws/src/inf_icp/src/map.pcd", cloud_blob);
*/
  //Load_PCD();
//if(read_file_flag == 0){

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/toby/catkin_ws/src/inf_icp/src/map_nu/map/map_900_1200.pcd", *cloud2_in) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file map.pcd \n");
  }
  pcl::io::loadPCDFile("/home/toby/catkin_ws/src/inf_icp/src/map_nu/map/map_900_1200.pcd", cloud_blob);
  
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/toby/catkin_ws/src/inf_icp/src/map_nu/map/map_900_1300.pcd", *cloud2_in) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file map.pcd \n");
  }
  pcl::io::loadPCDFile("/home/toby/catkin_ws/src/inf_icp/src/map_nu/map/map_900_1300.pcd", cloud_blob2);
  pcl::concatenatePointCloud(cloud_blob, cloud_blob2, cloud_blob3);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/toby/catkin_ws/src/inf_icp/src/map_nu/map/map_800_1300.pcd", *cloud2_in) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file map.pcd \n");
  }
  pcl::io::loadPCDFile("/home/toby/catkin_ws/src/inf_icp/src/map_nu/map/map_800_1300.pcd", cloud_blob4);
  pcl::concatenatePointCloud(cloud_blob3, cloud_blob4, cloud_blob5);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/toby/catkin_ws/src/inf_icp/src/map_nu/map/map_1000_1200.pcd", *cloud2_in) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file map.pcd \n");
  }
  pcl::io::loadPCDFile("/home/toby/catkin_ws/src/inf_icp/src/map_nu/map/map_1000_1200.pcd", cloud_blob6);
  pcl::concatenatePointCloud(cloud_blob5, cloud_blob6, cloud_combined);
  //cloud2_in->header.frame_id = "base_link";
  //read_file_flag = 1;
//}

  pcl::fromPCLPointCloud2(cloud_combined, *cloud2_in);

//Setting initial guess
/*
  pcl::VoxelGrid<pcl::PointXYZ> sor_map;
  sor_map.setInputCloud (cloud2_in);
  sor_map.setLeafSize (0.4, 0.4 ,0.4);
  sor_map.filter (*filtered_map_ptr);

  pcl::VoxelGrid<pcl::PointXYZ> sor_scan;
  sor_scan.setInputCloud (cloud_in);
  sor_scan.setLeafSize (1, 1 ,1);
  sor_scan.filter (*filtered_scan_ptr);
  */
  //Perform ICP
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud2_in);

  icp.setMaxCorrespondenceDistance(1);
  icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.001);
  icp.setMaximumIterations (1000);

  if(initial_flag == 0){ 
  Setting_Initial_Guess();
  tf_broadcast();
    if(initial_flag == 1)
      { std::cout << "first frame recorded" << std::endl;
        write_file();
      }
  }else if(initial_flag == 1)
  {
  std::cout << "ICP start" << std::endl; 

  for(ittr = 0; ittr <6; ittr++){
    icp.getFitnessScore(1);
    icp.align(*cloudOut_new, icp_transformed);
    icp_transformed = icp.getFinalTransformation();
  }
  tf_broadcast();

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  
  // Convert the pcl/PointCloud to sensor_msgs/PointCloud2
  sensor_msgs::PointCloud2 map_publish;
  sensor_msgs::PointCloud2 scan_publish;
  pcl::toROSMsg(*cloud_in, scan_publish);//scan
  pcl::toROSMsg(*cloud2_in, map_publish);//map
  map_publish.header.frame_id = "map";
  scan_publish.header.frame_id = "velodyne";
  scan_pub.publish(scan_publish);
  map_pub.publish(map_publish);
  write_file();
  
  }else{
    std::cout << "Something not prepared" << std::endl;
  }
}
void ICP_Cloud_Matcher::Setting_Initial_Guess()
{  
   Eigen::Matrix4f initial_guess_temp;
   initial_guess_temp = Eigen::Matrix4f::Identity();
   initial_guess = Eigen::Matrix4f::Identity();
   int itt;
   double init_score, init_score_temp = 10;
   double i, j, z,angle_bag2, best_yaw, best_z, best_score = 10;
   if(gps_flag == 1){
    for(i = 3.2; i>-3.2; i = i-0.1)
    {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut1(new pcl::PointCloud<pcl::PointXYZ>);
    tf::Quaternion myQuaternion1;
    myQuaternion1.setRPY(0, 0, i);
    std::cout << "for yaw : " << i << std::endl;
    rotation_tf_matrix.setRotation(myQuaternion1); //myQuaternion
    tf::matrixTFToEigen(rotation_tf_matrix, rotation_matrix);
    initial_guess_temp(0,0) = rotation_matrix(0,0);
    initial_guess_temp(0,1) = rotation_matrix(0,1);
    initial_guess_temp(0,2) = rotation_matrix(0,2);
    initial_guess_temp(1,0) = rotation_matrix(1,0);
    initial_guess_temp(1,1) = rotation_matrix(1,1);
    initial_guess_temp(1,2) = rotation_matrix(1,2);
    initial_guess_temp(2,0) = rotation_matrix(2,0);
    initial_guess_temp(2,1) = rotation_matrix(2,1);
    initial_guess_temp(2,2) = rotation_matrix(2,2);
    initial_guess_temp(0,3) = gps_matrix(0,3);
    initial_guess_temp(1,3) = gps_matrix(1,3);
    initial_guess_temp(2,3) = gps_matrix(2,3);
    initial_guess_temp(3,3) = 1;
    //for(z=0;z<4;z++)
    //{
      //std::cout << "Z : " << z << std::endl;
      //initial_guess_temp(2,3) = gps_matrix(2,3)+z;
      icp.align(*cloudOut1, initial_guess_temp); 
      j = icp.getFitnessScore(1);
      std::cout << j << std::endl;
      std::cout << initial_guess_temp << std::endl;
      if(j < best_score)
      {
        best_yaw = i;
        best_score = j;
        //best_z = z;
        initial_guess = initial_guess_temp;
      }
    //}
  }
    icp_transformed = initial_guess;
    initial_flag = 1;
    tf_flag = 1; 
    std::cout << "Initial guess set up finished." << std::endl;
    std::cout << "Best Score : " <<best_score << std::endl;
    std::cout << "Best Yaw : " << best_yaw << std::endl;
   //std::cout << "Best Z : " << best_z << std::endl;
    //std::cout << "initial guess : " << initial_guess << std::endl;
    }
}

void ICP_Cloud_Matcher::tf_broadcast(){
  //TF setting
  transform_scan2car.setOrigin(tf::Vector3(0.46, 0, 3.46));
  tf::Quaternion tf_q(-0.0051505, 0.018102, -0.019207, 0.99964);
  transform_scan2car.setRotation(tf_q);
  //
  current_pose.x = icp_transformed(0,3);
  current_pose.y = icp_transformed(1,3);
  current_pose.z = icp_transformed(2,3);
  mat_local.setValue(static_cast<double>(icp_transformed(0,0)), static_cast<double>(icp_transformed(0,1)), static_cast<double>(icp_transformed(0,2)),
                       static_cast<double>(icp_transformed(1,0)), static_cast<double>(icp_transformed(1,1)), static_cast<double>(icp_transformed(2,2)),
                       static_cast<double>(icp_transformed(2,0)), static_cast<double>(icp_transformed(2,1)), static_cast<double>(icp_transformed(2,2)));
  mat_local.getRPY(current_pose.roll, current_pose.pitch, current_pose.yaw, 1);
  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  tf::Quaternion q;
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "velodyne"));
}

void ICP_Cloud_Matcher::write_file(){
  transform_record = transform * transform_scan2car.inverse();
  //writefile
  file.open("/home/toby/catkin_ws/src/inf_icp/src/7_medium_1.csv", std::fstream::app);
  tf::Quaternion q_record(transform_record.getRotation().x(), transform_record.getRotation().y(), transform_record.getRotation().z(), transform_record.getRotation().w());
  tf::Matrix3x3 m(q_record);
  m.getRPY(transformed.roll, transformed.pitch, transformed.yaw);
  file << scan_stamp << "," << transform_record.getOrigin().x() << "," << transform_record.getOrigin().y() << "," << transform_record.getOrigin().z() << ","
   << transformed.yaw << "," << transformed.pitch << "," << transformed.roll <<  std::endl;
  //transform_record.getOrigin().z()
  file.close();
}
/*
void ICP_Cloud_Matcher::Load_PCD(){
  struct dirent *entry;
  DIR *dir = opendir("/home/toby/catkin_ws/src/inf_icp/src/map_nu/map");
  int read_file = 0;
  while ((entry = readdir(dir)) != NULL && pcd_combined_flag == 0) {
   if(read_file < 2){
     read_file++;
   }else{
    std::cout << "first pcd" << std::endl;
    std::cout << entry->d_name << std::endl;
    std::string file_path = "/home/toby/catkin_ws/src/inf_icp/src/map_nu/map/" + std::string(entry->d_name);
    std::cout << file_path << std::endl;
    pcl::io::loadPCDFile(file_path, cloud_blob);
    if(pcd_flag = 0){
      pcd_flag = 1;
      std::cout << "first pcd" << std::endl;
      cloud_temp = cloud_blob;
    }else{
      pcl::concatenatePointCloud(cloud_temp, cloud_blob, cloud_pcd_combined);
      cloud_temp = cloud_pcd_combined;
    }
    std::cout << entry->d_name << std::endl;
  }
  }
  closedir(dir);
  pcd_combined_flag = 1;  
}
*/