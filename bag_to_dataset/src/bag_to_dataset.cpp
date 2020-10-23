#include <iostream>
#include <fstream>
#include <sys/stat.h> 　
#include <sys/types.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <opencv2/opencv.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
using namespace std;

rosbag::Bag bag;
string dataset_dir;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bag_to_dataset");
  ros::NodeHandle nh("~");
  if(argc != 2){
        cout << "Usage: rosrun    package_name    node_name   bagfilename" << endl;
        return -1;
    }
  string bagfilename=argv[1];
  bag.open(bagfilename, rosbag::bagmode::Read);
  dataset_dir=bagfilename.substr(0,bagfilename.size()-4);

  string img0_dir=dataset_dir+"/img0";
  string img0_data=img0_dir+"/data";

  string img1_dir=dataset_dir+"/img1";
  string img1_data=img1_dir+"/data"; 

  string laser_dir=dataset_dir+"/laser";
  string laser_data=laser_dir+"/data";
  
  int isCreate;

  isCreate = mkdir(dataset_dir.c_str(),0755);
  isCreate = mkdir(img0_dir.c_str(),0755);
  isCreate = mkdir(img1_dir.c_str(),0755);  
  isCreate = mkdir(laser_dir.c_str(),0755);

  isCreate = mkdir(img0_data.c_str(),0755);
  isCreate = mkdir(img1_data.c_str(),0755);
  isCreate = mkdir(laser_data.c_str(),0755);

  ofstream ofs_img0_stamp(img0_dir+"/timestamps.txt");
  ofstream ofs_img1_stamp(img1_dir+"/timestamps.txt");
  ofstream ofs_laser_stamp(laser_dir+"/timestamps.txt");

  ofs_img0_stamp<<"            time(ns)                                   data_file"<<endl;
   ofs_img1_stamp<<"            time(ns)                                   data_file"<<endl;
    ofs_laser_stamp<<"            time(ns)                                   data_file"<<endl;
 // ofstream ofs_imu(dataset_dir+"/imu.txt");
  //ofstream ofs_mag(dataset_dir+"/mag.txt");
 // ofstream ofs_gps(dataset_dir+"/gps.txt");
  //ofstream ofs_gps_vel(dataset_dir+"/gps_vel.txt");


// cout<<"reading imu"<<endl;
// //rosbag::View view_imu(bag, rosbag::TopicQuery("/imu/data"));
// rosbag::View view_imu(bag, rosbag::TopicQuery("/mynteye/imu/data_raw"));
// for(rosbag::MessageInstance const m : view_imu)
// {
//   /*
//     std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
//     if (i != NULL)
//         std::cout << i->data << std::endl;
//     */
//     sensor_msgs::Imu::ConstPtr imu=m.instantiate<sensor_msgs::Imu>();
//     if(imu!=NULL)
//       ofs_imu<<(long long)(imu->header.stamp.toSec()*1e9)<<" "
//       <<imu->angular_velocity.x<<" "
//       <<imu->angular_velocity.y<<" "
//       <<imu->angular_velocity.z<<" "
//       <<imu->linear_acceleration.x<<" "
//       <<imu->linear_acceleration.y<<" "
//       <<imu->linear_acceleration.z<<" "
//       <<endl;
// }

// cout<<"reading gps"<<endl;

// rosbag::View view_gps(bag, rosbag::TopicQuery("/ublox_gps/fix"));
// for(rosbag::MessageInstance const m : view_gps)
// {
//   /*
//     std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
//     if (i != NULL)
//         std::cout << i->data << std::endl;
//     */
//     sensor_msgs::NavSatFixConstPtr gps=m.instantiate<sensor_msgs::NavSatFix>();
//     if(gps!=NULL)
//     {
//       ofs_gps<<(long long)(gps->header.stamp.toSec()*1e9)<<" "
//       <<setprecision(15)<<gps->latitude<<" "
//       <<setprecision(15)<<gps->longitude<<" "
//       <<setprecision(6)<<gps->altitude<<" ";
//       for(int i=0;i<9;i++)
//         ofs_gps<<setw(10)<<gps->position_covariance[i]<<" ";
//       ofs_gps<<endl;
//     }    
// }
// cout<<"reading gps_vel"<<endl;

// rosbag::View view_gps_vel(bag, rosbag::TopicQuery("/ublox_gps/fix_velocity"));
// for(rosbag::MessageInstance const m : view_gps_vel)
// {
//   /*
//     std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
//     if (i != NULL)
//         std::cout << i->data << std::endl;
//     */
//     geometry_msgs::TwistWithCovarianceStampedConstPtr gps_vel
//     =m.instantiate<geometry_msgs::TwistWithCovarianceStamped>();
//     if(gps_vel!=NULL)
//     {
//       ofs_gps_vel<<(long long)(gps_vel->header.stamp.toSec()*1e9)<<" "
//       <<setprecision(6)<<gps_vel->twist.twist.linear.x<<" "
//       <<setprecision(6)<<gps_vel->twist.twist.linear.y<<" "
//       <<setprecision(6)<<gps_vel->twist.twist.linear.z<<" ";
      
//       ofs_gps_vel<<setw(10)<<gps_vel->twist.covariance[0]<<" "
//       <<setw(10)<<gps_vel->twist.covariance[1]<<" "
//       <<setw(10)<<gps_vel->twist.covariance[2]<<" "
//       <<setw(10)<<gps_vel->twist.covariance[6]<<" "
//       <<setw(10)<<gps_vel->twist.covariance[7]<<" "
//       <<setw(10)<<gps_vel->twist.covariance[8]<<" "
//       <<setw(10)<<gps_vel->twist.covariance[12]<<" "
//       <<setw(10)<<gps_vel->twist.covariance[13]<<" "
//       <<setw(10)<<gps_vel->twist.covariance[14]<<" ";
//       ofs_gps_vel<<endl;
//     }    
// }

// cout<<"reading mag"<<endl;

// rosbag::View view_mag(bag, rosbag::TopicQuery("/imu/mag"));
// for(rosbag::MessageInstance const m : view_mag)
// {
//     sensor_msgs::MagneticField::ConstPtr mag=m.instantiate<sensor_msgs::MagneticField>();
//     if(mag!=NULL)
//       ofs_mag<<(long long)(mag->header.stamp.toSec()*1e9)<<" "
//       <<mag->magnetic_field.x<<" "
//       <<mag->magnetic_field.y<<" "
//       <<mag->magnetic_field.z<<" "
//       <<endl;
// }

cout<<"reading img0"<<endl;

int img_count=0;
int sync_count=0;
vector<long long> img0_time;
int cur_img0_index=0;

rosbag::View view_img0(bag, rosbag::TopicQuery("/flir/left/image_raw"));
for(rosbag::MessageInstance const m : view_img0)
{
    sensor_msgs::Image::ConstPtr img=m.instantiate<sensor_msgs::Image>();
    if(img!=NULL)
    {
      long long time;
      img_count++;

      time=(long long)(img->header.stamp.toSec()*1e9);
      img0_time.push_back(time);

      ofs_img0_stamp<<time<<","
      <<time<<".jpg"<<endl;
      cout<<"cam0:  "<<sync_count<<"/"<<img_count<<endl;

      auto img_ptr = cv_bridge::toCvShare(img,sensor_msgs::image_encodings::MONO8);
      const cv::Mat &img_mat = img_ptr->image;
      cv::imwrite(img0_data+"/"+to_string(time)+".jpg",img_mat);
    }
}

cout<<"reading img1"<<endl;

rosbag::View view_img1(bag, rosbag::TopicQuery("/flir/right/image_raw"));
for(rosbag::MessageInstance const m : view_img1)
{
    sensor_msgs::Image::ConstPtr img=m.instantiate<sensor_msgs::Image>();
    if(img!=NULL)
    {
      
      long long time;
      bool is_find = false;
      while(true)
      {
      if(abs((double)img0_time[cur_img0_index]/1e9-img->header.stamp.toSec())<0.005)
        {
          time=img0_time[cur_img0_index];
          sync_count++;
          cur_img0_index++;
          is_find = true;
          break;
        }
      else if(((double)img0_time[cur_img0_index]/1e9-img->header.stamp.toSec())<=-0.005)
      {
        cur_img0_index++;
        
      }
      else break;
      }
      if(is_find==false) continue;
      ofs_img1_stamp<<time<<","
      <<time<<".jpg"<<endl;
      auto img_ptr = cv_bridge::toCvShare(img,sensor_msgs::image_encodings::MONO8);
      const cv::Mat &img_mat = img_ptr->image;
      //cout<<img1_dir+"/"+to_string(time)+".jpg"<<endl;
      cout<<"cam1:  "<<cur_img0_index<<" "<<sync_count<<"/"<<img_count<<endl;
      cv::imwrite(img1_data+"/"+to_string(time)+".jpg",img_mat);
    }
}

cout<<"reading laser"<<endl;
int count=0;
rosbag::View view_laser(bag, rosbag::TopicQuery("/velodyne_points"));
for(rosbag::MessageInstance const m : view_laser)
{
    sensor_msgs::PointCloud2::ConstPtr laser=m.instantiate<sensor_msgs::PointCloud2>();
    if(laser!=NULL)
    {
      long long time;

      time=(long long)(laser->header.stamp.toSec()*1e9);

      ofs_laser_stamp<<time<<","
      <<time<<".bin"<<endl;

      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(*laser, *cloud_);

      std::string file_name = laser_data+"/"+ std::to_string(time) + ".bin";
      std::ofstream bin_file(file_name, std::ios::out|std::ios::binary|std::ios::app);
    

    if(!bin_file.good()){
        cout << "Error Opening: "<< file_name << endl;
    }

    for (size_t i = 0; i < cloud_->points.size(); ++i)
    {
        bin_file.write((char*)&cloud_->points[i].x,3*sizeof(float)); 
        bin_file.write((char*)&cloud_->points[i].intensity,sizeof(float));
    }
  	
    bin_file.close();
    ++count;
    cout<<"lidar:  "<<count<<endl;
    }
}

    return 0;


}
