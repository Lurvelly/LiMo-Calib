
#include <iostream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <deque>
#include "motor_lidar_calib/CustomMsg.h"
#include "lidardata.h"
#include <std_msgs/String.h>

#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>

struct ParaConfig
{
  std::string lidar_topic;
  std::string radar_topic;
  std::string rotor_topic;

  std::string bagFilePath;
  

  double bag_start;
  double bag_durr;

  std::string rotlidarFilePath;

  // calibration
  Eigen::Matrix3d R_rot_li;
  Eigen::Matrix3d R_rot_ra;
};

ros::Publisher pub_livox_points;
ros::Publisher pub_radar_points;

ParaConfig config_;
std::deque<motor_lidar_calib::CustomMsgPtr> livox_lidar_messages_;
std::deque<sensor_msgs::PointCloudPtr> radar_messages_;
std::deque<std::pair<double,double>> rotor_messages_;

void loadConfig(ros::NodeHandle& nh)
{
  std::string config_name;

  nh.param<std::string>("lidar_topic", config_.lidar_topic, "");
  nh.param<std::string>("radar_topic", config_.radar_topic, "");
  nh.param<std::string>("rotor_topic", config_.rotor_topic, "");

  nh.param<double>("bag_start", config_.bag_start, 0);
  nh.param<double>("bag_durr", config_.bag_durr, -1);

  nh.param<std::string>("bagFilePath", config_.bagFilePath, "");
  ROS_INFO("bag file path: %s\n",config_.bagFilePath.c_str());

  nh.param<std::string>("rotlidarFilePath", config_.rotlidarFilePath, "");

  // calibration
  config_.R_rot_li = Eigen::AngleAxisd(-M_PI/3.,Eigen::Vector3d::UnitY());
  config_.R_rot_ra = Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ());
}

void getValidRotorMeasures(double begTime, double endTime, std::vector<std::pair<double,double>>& validMeasures)
{
  for (auto& measure: rotor_messages_)
  {
    // std::cout<<"tmp time: " <<measure.first <<"\n";
    if(measure.first < begTime)
    {
      // rotor_messages_.pop_front();
    }
    if(measure.first > begTime && measure.first < endTime)
    {
      validMeasures.push_back(measure);
    }
    if(measure.first > endTime)
    {
      validMeasures.push_back(measure);
      break;
    }
  }
}


void SpinOffline()
{
  rosbag::Bag bag;
  bag.open(config_.bagFilePath, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(config_.lidar_topic);
  topics.push_back(config_.radar_topic);
  topics.push_back(config_.rotor_topic);
  

  rosbag::View view_;
  rosbag::View view_full;
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  time_init += ros::Duration(config_.bag_start);
  ros::Time time_finish = (config_.bag_durr < 0)
                              ? view_full.getEndTime()
                              : time_init + ros::Duration(config_.bag_durr);
  view_.addQuery(bag, rosbag::TopicQuery(topics), time_init, time_finish);

  if (view_.size() == 0) {
    ROS_ERROR("No messages to play on specified topics.  Exiting.");
    ros::shutdown();
    return;
  }

  double latest_rotor_time = -1;
  for (const rosbag::MessageInstance& m : view_) {
    ros::Time ros_bag_time = m.getTime();
    if (m.getTopic() == config_.lidar_topic)
    {
    //   ROS_INFO("lidar_topic");
      auto lidar_msg = m.instantiate<motor_lidar_calib::CustomMsg>();
      livox_lidar_messages_.push_back(lidar_msg);
    }
    else if (m.getTopic() == config_.radar_topic)
    {
    //   ROS_INFO("radar_topic");
      auto radar_msg = m.instantiate<sensor_msgs::PointCloud>();
      radar_messages_.push_back(radar_msg);

    }
    else if (m.getTopic() == config_.rotor_topic)
    {
      std_msgs::String::Ptr point_msg = m.instantiate<std_msgs::String>();
      double stamp = m.getTime().toSec();
      //double stamp = point_msg->header.stamp.toSec();
      double position_value; // 用于存储解析的 Position 值

      // 检查消息是否有效
      if (point_msg) {
        if (sscanf(point_msg->data.c_str(), "Position:%lf,", &position_value) == 1) {
            double angular = -position_value / 180 * M_PI;
            rotor_messages_.push_back(std::make_pair(stamp, angular));
            latest_rotor_time = stamp;
        } else {
            ROS_WARN("Failed to parse Position from rotor message: %s", point_msg->data.c_str());
            }
      }
    }
  }
}

void saveLiDARRotorFile(std::string filename)
{

  FILE* file = fopen(filename.c_str(),"w");
  for (motor_lidar_calib::CustomMsgPtr lidar_msg: livox_lidar_messages_)
  {
    // ROS_INFO("%lf",msg->header.stamp.toSec());
    RTPointCloud::Ptr raw_cloud(new RTPointCloud);
    raw_cloud->header.stamp = lidar_msg->header.stamp.toNSec();
    // raw_cloud_->clear();
    for(const auto& p : lidar_msg->points){
      RTPoint point;
      int line_num = (int)p.line;
      
      if(p.reflectivity < 1) continue;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      point.intensity = p.reflectivity;
      point.time = p.offset_time/1e9;
      point.ring = (line_num);

      raw_cloud->push_back(point);
    }

    // find the corresponding rotor angulars
    double beg_lidar_time = raw_cloud->points[0].time + raw_cloud->header.stamp/1e9;
    double end_lidar_time = raw_cloud->points.back().time + raw_cloud->header.stamp/1e9;

    std::vector<std::pair<double,double>> validMeasures;
    getValidRotorMeasures(beg_lidar_time-0.1, end_lidar_time+0.1, validMeasures);

    ROS_INFO("validMeasures.size():%d",validMeasures.size());

    if(validMeasures.size() < 20)
    {
      return;
    }

    // transform to body frame
    RTPointCloud::Ptr transformed_cloud(new RTPointCloud);
    for (size_t i = 0; i < raw_cloud->size(); i++)
    {
      RTPoint point = raw_cloud->points[i];
      RTPoint pt_trans = point;

      Eigen::Vector3d pt_e;
      pt_e[0] = point.x;
      pt_e[1] = point.y;
      pt_e[2] = point.z;
      double tmpTime = raw_cloud->header.stamp/1e9 + point.time;
      double rotAngular = 0;
      bool valid = false;
      for (size_t j = 0; j < validMeasures.size()-1; j++)
      {
        if(tmpTime > validMeasures[j].first && tmpTime < validMeasures[j+1].first)
        {
          double anglediff = validMeasures[j+1].second-validMeasures[j].second;

          if(anglediff > M_PI)
          {
            anglediff-=2*M_PI;
          }
          if(anglediff < -M_PI)
          {
            anglediff+=2*M_PI;
          }
          rotAngular = (tmpTime-validMeasures[j].first)/(validMeasures[j+1].first-validMeasures[j].first)*anglediff + validMeasures[j].second;
          valid = 1;
          break;
        }
      }
      if(!valid)
      {
        continue;
      }
      Eigen::Vector3d pt_e_trans = Eigen::AngleAxisd(rotAngular,Eigen::Vector3d::UnitZ()) * config_.R_rot_li*pt_e;
      pt_trans.x = pt_e_trans[0];
      pt_trans.y = pt_e_trans[1];
      pt_trans.z = pt_e_trans[2];

      transformed_cloud->push_back(pt_trans);

      // save to a data log
      // Initial (x y z) Raw (x y z) intensity angle time
      fprintf(file,"%lf %lf %lf %lf %lf %lf %d %lf %lf \n", pt_trans.x, pt_trans.y, pt_trans.z, point.x, point.y, point.z, rotAngular, tmpTime);
    }
  }

  fclose(file);
  
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "calibtration");
  ros::NodeHandle nh("~");
  loadConfig(nh);

  // load all data
  SpinOffline();
  
  // save to LiDAR Rotor file
  saveLiDARRotorFile(config_.rotlidarFilePath);



  return 0;
}