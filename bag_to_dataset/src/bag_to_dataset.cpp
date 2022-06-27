#include "utility.h"
using namespace std;

#define UNIX_GPS_DIF 315964800
#define LEAPSEC 18
#define TIME_ZONE_8 28800

rosbag::Bag bag;
string dataset_dir;

using PointXYZIRT = VelodynePointXYZIRT;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bag_to_dataset");
  ros::NodeHandle nh;

  string bagfilename, lidartopic, image0topic, image1topic;
  bool uselidar, useimage0, useimage1;
  int lidartype;

  nh.param<std::string>("bag_path", bagfilename, "hahaha");

  nh.param<bool>("use_lidar", uselidar, true);
  nh.param<bool>("use_image0", useimage0, false);
  nh.param<bool>("use_image1", useimage1, false);

  nh.param<std::string>("lidar_topic", lidartopic, "/velodyne_points");
  nh.param<std::string>("image0_topic", image0topic, "/flir/left/image_raw");
  nh.param<std::string>("image1_topic", image1topic, "/flir/right/image_raw");

  nh.param<int>("lidar_type", lidartype, 1);

  cout<<"bag:"<<bagfilename<<endl;

  SensorType sensor;
  if (lidartype == 1)
  {
    sensor = SensorType::VELODYNE;
  }
  else if (lidartype == 2)
  {
    sensor = SensorType::OUSTER;
  }
  else if (lidartype == 3)
  {
    sensor = SensorType::LIVOX;
  }

  bag.open(bagfilename, rosbag::bagmode::Read);
  dataset_dir = bagfilename.substr(0, bagfilename.size() - 4);

  string img0_dir = dataset_dir + "/img0";
  string img0_data = img0_dir + "/data";

  string img1_dir = dataset_dir + "/img1";
  string img1_data = img1_dir + "/data";

  string laser_dir = dataset_dir + "/laser";
  string laser_data = laser_dir + "/data";

  int isCreate;

  isCreate = mkdir(dataset_dir.c_str(), 0755);

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
  //       <<setprecision(6)<<gps_vel->twist.twist.lineisCreate = mkdir(img1_dir.c_str(), 0755);ar.z<<" ";

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
  vector<long long> img0_time;
  int cur_img0_index = 0;
  int img_count = 0;
  int sync_count = 0;
  if (useimage0)
  {
    isCreate = mkdir(img0_dir.c_str(), 0755);
    isCreate = mkdir(img0_data.c_str(), 0755);
    ofstream ofs_img0_stamp(img0_dir + "/timestamps.txt");
    ofs_img0_stamp << "#     time(gps second)                               data_file" << endl;

    cout << "reading img0" << endl;

    rosbag::View view_img0(bag, rosbag::TopicQuery(image0topic));
    for (rosbag::MessageInstance const m : view_img0)
    {
      sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
      if (img != NULL)
      {
        long long time;

        img_count++;

        time = (long long)(img->header.stamp.toSec() * 1e9);
        img0_time.push_back(time);

        double GPS_time = time / 1e9 - UNIX_GPS_DIF + LEAPSEC;
        double GPS_inweek = fmod(GPS_time, 604800);

        ofs_img0_stamp << fixed << setprecision(4) << GPS_inweek << ","
                       << time << ".jpg" << endl;

        cout << "cam0:  " << sync_count << "/" << img_count << endl;

        auto img_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::MONO8);
        const cv::Mat &img_mat = img_ptr->image;
        cv::imwrite(img0_data + "/" + to_string(time) + ".jpg", img_mat);
      }
    }
  }
  if (useimage1)
  {
    isCreate = mkdir(img1_dir.c_str(), 0755);
    isCreate = mkdir(img1_data.c_str(), 0755);
    ofstream ofs_img1_stamp(img1_dir + "/timestamps.txt");
    ofs_img1_stamp << "#     time(gps second)                               data_file" << endl;

    cout << "reading img1" << endl;

    rosbag::View view_img1(bag, rosbag::TopicQuery(image1topic));
    for (rosbag::MessageInstance const m : view_img1)
    {
      sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
      if (img != NULL)
      {

        long long time;
        bool is_find = false;
        while (true)
        {
          if (abs((double)img0_time[cur_img0_index] / 1e9 - img->header.stamp.toSec()) < 0.005)
          {
            time = img0_time[cur_img0_index];
            sync_count++;
            cur_img0_index++;
            is_find = true;
            break;
          }
          else if (((double)img0_time[cur_img0_index] / 1e9 - img->header.stamp.toSec()) <= -0.005)
          {
            cur_img0_index++;
          }
          else
            break;
        }
        if (is_find == false)
          continue;

        double GPS_time = time / 1e9 - UNIX_GPS_DIF + LEAPSEC;
        double GPS_inweek = fmod(GPS_time, 604800);

        ofs_img1_stamp << fixed << setprecision(4) << GPS_inweek << ","
                       << time << ".jpg" << endl;

        auto img_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::MONO8);
        const cv::Mat &img_mat = img_ptr->image;
        //cout<<img1_dir+"/"+to_string(time)+".jpg"<<endl;
        cout << "cam1:  " << cur_img0_index << " " << sync_count << "/" << img_count << endl;
        cv::imwrite(img1_data + "/" + to_string(time) + ".jpg", img_mat);
      }
    }
  }

  if (uselidar)
  {
    isCreate = mkdir(laser_dir.c_str(), 0755);
    isCreate = mkdir(laser_data.c_str(), 0755);
    ofstream ofs_laser_stamp(laser_dir + "/timestamps.txt");
    ofs_laser_stamp << "#     time(gps second)                               data_file" << endl;

    cout << "reading laser" << endl;
    int count = 0;
    rosbag::View view_laser(bag, rosbag::TopicQuery(lidartopic));
    for (rosbag::MessageInstance const m : view_laser)
    {
      sensor_msgs::PointCloud2::ConstPtr laser = m.instantiate<sensor_msgs::PointCloud2>();
      
      if (laser != NULL)
      {
        long long time;

        time = (long long)(laser->header.stamp.toSec() * 1e9);
        double GPS_time = time / 1e9 - UNIX_GPS_DIF + LEAPSEC;
        double GPS_inweek = fmod(GPS_time, 604800);

        ofs_laser_stamp << fixed << setprecision(4) << GPS_inweek << ","
                        << time << ".bin" << endl;

        //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZI>);
        //pcl::fromROSMsg(*laser, *cloud_);
        pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX)
        {
          pcl::fromROSMsg(*laser, *laserCloudIn);
        }
        else if (sensor == SensorType::OUSTER)
        {
          // Convert to Velodyne format
          pcl::fromROSMsg(*laser, *tmpOusterCloudIn);
          laserCloudIn->points.resize(tmpOusterCloudIn->size());
          laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
          for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
          {
            auto &src = tmpOusterCloudIn->points[i];
            auto &dst = laserCloudIn->points[i];
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.intensity;
            dst.ring = src.ring;
            dst.time = src.t * 1e-9f;
          }
        }

        std::string file_name = laser_data + "/" + std::to_string(time) + ".bin";
        std::ofstream bin_file(file_name, std::ios::out | std::ios::binary | std::ios::app);

        if (!bin_file.good())
        {
          cout << "Error Opening: " << file_name << endl;
        }

        for (size_t i = 0; i < laserCloudIn->points.size(); ++i)
        {
          bin_file.write((char *)&laserCloudIn->points[i].x, 3 * sizeof(float));
          bin_file.write((char *)&laserCloudIn->points[i].intensity, sizeof(float));
          bin_file.write((char *)&laserCloudIn->points[i].ring, sizeof(uint16_t));
          bin_file.write((char *)&laserCloudIn->points[i].time, sizeof(float));
        }

        bin_file.close();
        ++count;
        cout << "lidar:  " << count << endl;
      }
    }
  }

  return 0;
}
