#pragma once

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_ros/point_cloud.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "ros/subscriber.h"
#include "ros/publisher.h"
#include <nav_msgs/Odometry.h>

#include "pcl_conversions/pcl_conversions.h"
#include "lodestar_odometry/statistics.h"
#include "lodestar_odometry/pointnormal.h"
#include <algorithm>

#include <Eigen/Eigen>
#include "eigen_conversions/eigen_msg.h"
#include <tf_conversions/tf_eigen.h>
#include "tf/message_filter.h"
#include <tf/transform_listener.h>
#include "tf/transform_broadcaster.h"


namespace lodestar_odom  {
using std::cout;
using std::endl;
using std::cerr;


class lodestar
{
public:
  class Parameters
  {
  public:
    Parameters() {}
    float z_min = 60; // min power
    float range_res = 0.05;
    int azimuths = 400;
    int nb_guard_cells = 20, window_size = 10;
    float false_alarm_rate = 0.01;
    float min_distance = 2.5, max_distance = 200;
    std::string radar_frameid = "sensor_est", topic_filtered = "/marine/Filtered";
    std::string dataset = "marine";
    std::string radar_topic = "/radar_data";
    int contour_threshold = 50;
    int k_nearest = 20;

    void GetParametersFromRos( ros::NodeHandle& param_nh){

      param_nh.param<float>("range_res", range_res, 0.0438);
      param_nh.param<float>("z_min", z_min, 60);
      param_nh.param<float>("min_distance", min_distance, 2.5);
      param_nh.param<float>("max_distance", max_distance, 130);
      param_nh.param<std::string>("topic_filtered", topic_filtered, "/marine/Filtered");
      param_nh.param<std::string>("radar_frameid", radar_frameid, "sensor_est");
      param_nh.param<std::string>("dataset", dataset, "marine");
      param_nh.param<std::string>("radar_topic_name", radar_topic, "/radar_data");
      param_nh.param<int>("contour_threshold", contour_threshold, 50);
      param_nh.param<int>("k_nearest", k_nearest, 20);
    }
    std::string ToString(){
      std::ostringstream stringStream;
      //stringStream << "lodestar::Parameters:"<<endl;
      stringStream << "range res, "<<range_res<<endl;
      stringStream << "z min, "<<z_min<<endl;
      stringStream << "min distance, "<<min_distance<<endl;
      stringStream << "max distance, "<<max_distance<<endl;
      stringStream << "topic_filtered, "<<topic_filtered<<endl;
      stringStream << "radar_frameid, "<<radar_frameid<<endl;
      stringStream << "dataset, "<<dataset<<endl;
      stringStream << "nb guard cells, "<<nb_guard_cells<<endl;
      stringStream << "window size, "<<window_size<<endl;
      stringStream << "false alarm rate, "<<false_alarm_rate<<endl;

      return stringStream.str();
    }

  };

  lodestar(const Parameters& pars, bool disable_callback = false);

  ~lodestar(){}

  void CallbackOffline(const sensor_msgs::ImageConstPtr &marine_radar_img, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, Eigen::Affine3d &T, Eigen::Vector3d &v);
  

  

private:

  void InitAngles();

  void Callback(const sensor_msgs::ImageConstPtr &marine_radar_img);

  void CallbackOxford(const sensor_msgs::ImageConstPtr &marine_radar_img);
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr toPointCloud(const cv::Mat& radar_img, const double &range_resolution);
  
  cv::Mat polar_transform(const cv::Mat& cart_img);
  cv::Mat polarToNearPol(const cv::Mat& polar_img, const int& k);
  double rotationCorrection(const cv::Mat& img1, const cv::Mat& img2);
  double crossCorrelation(const std::vector<double>& a, const std::vector<double>& b);
//  std::vector<std::complex<double>> crosscorr(const std::vector<double>& x, const std::vector<double>& y);
  

  Parameters par;
  std::vector<float> sin_values;
  std::vector<float> cos_values;
  float max_distance_sqrd, min_distance_sqrd;
  ros::NodeHandle nh_;
  ros::Subscriber sub;
  ros::Publisher FilteredPublisher,ExperimentalPublisher, UnfilteredPublisher, imgPublisher, rotPublisher;
  image_transport::Publisher pub;
  image_transport::ImageTransport it;

    int initialize = 0;
    int count_d = 0;
    double acc_rots = 0.0;
    cv::Mat prev_img;
    cv::Mat curr_img;
    std::vector<cv::Mat> window_list, dense_list;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered;
  Eigen::Affine3d Trot;
  Eigen::Vector3d dense_trans;

};

}
