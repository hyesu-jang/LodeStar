#pragma once


#include <iostream>
#include <string>

#include <ros/ros.h>


#include "nav_msgs/Odometry.h"
#include <std_msgs/Float32.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "boost/filesystem.hpp"
#include <sensor_msgs/Image.h>
#include "ros/time.h"
#include "vector"
#include "eigen3/Eigen/Eigen"
#include "eigen3/Eigen/StdVector"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "eigen3/Eigen/SVD"
#include "tf/transform_broadcaster.h"
#include "eigen_conversions/eigen_msg.h"
#include "tf_conversions/tf_eigen.h"
#include "numeric"
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/common/transforms.h"
#include "nav_msgs/Path.h"
#include "ros/publisher.h"
#include "geometry_msgs/PoseStamped.h"
#include "pcl/2d/convolution.h"
#include "pcl/filters/random_sample.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/filters/radius_outlier_removal.h"

namespace lodestar_odom {


using std::string;
using std::cout;
using std::cerr;
using std::endl;

using namespace message_filters;
using namespace sensor_msgs;

typedef std::pair<Eigen::Affine3d, ros::Time> poseStamped;
typedef std::vector<poseStamped, Eigen::aligned_allocator<poseStamped>> poseStampedVector;
typedef sync_policies::ApproximateTime<std_msgs::Float32, nav_msgs::Odometry> double_odom;


class EvalTrajectory
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  class Parameters
  {
  public:
    Parameters() {}
    std::string est_output_dir = "";
    std::string sequence = "", method = "";
    std::string odom_est_topic = "";
    int job_nr = -1;
    bool save_pcd = false;
    bool synced_callback = true;

    void GetParametersFromRos( ros::NodeHandle& param_nh){
      param_nh.param<std::string>("est_topic", odom_est_topic, "/lidar_odom");
      param_nh.param<std::string>("est_output_dir", est_output_dir, "");
      param_nh.param<std::string>("bag_name",  sequence, "");
      param_nh.param<std::string>("method",  method, "");
      param_nh.param<bool>("save_pcd",  save_pcd, false);
      param_nh.param<bool>("synced_callback",  synced_callback, true);
    }

    std::string ToString(){
      std::ostringstream stringStream;
      //stringStream << "EvaluateTrajectory::Parameters:"<<endl;
      stringStream << "odom_est_topic, "<<odom_est_topic<<endl;
      stringStream << "est_output_dir, "<<est_output_dir<<endl;
      stringStream << "sequence, "<<sequence<<endl;
      stringStream << "job nr, "<<job_nr<<endl;
      stringStream << "save pcd, "<<save_pcd<<endl;
      stringStream << "method, "<<method<<endl;
      return stringStream.str();
    }
  };

  EvalTrajectory(const EvalTrajectory::Parameters& pars, bool disable_callback = false);

  std::string DatasetToSequence(const std::string& dataset);

  void CallbackEst(const nav_msgs::Odometry::ConstPtr &msg);

  void CallbackRot(const std_msgs::Float32::ConstPtr &msg);

  void Save();

  void CallbackESTEigen(const poseStamped& Test);

  void CallbackESTEigen(const poseStamped& Test, const pcl::PointCloud<pcl::PointXYZI>& cld);
  
  size_t GetSize(){return est_vek.size();}


private:

  void SavePCD(const std::string& folder);

  void PublishTrajectory(poseStampedVector& vek, ros::Publisher& pub);

  void Write(const std::string& path, const poseStampedVector& v,const poseStampedVector& r);


  poseStampedVector::iterator FindElement(const ros::Time& t);

  Parameters par;

  ros::NodeHandle nh_;
  ros::Subscriber sub_est, sub_rot_est;
  ros::Publisher pub_est, pub_cloud;
  tf::TransformBroadcaster br;
  Subscriber<nav_msgs::Odometry> *pose_sub_est = NULL;
  Subscriber<std_msgs::Float32> *rot_sub_est = NULL;
  Synchronizer<double_odom> *sync = NULL;

  poseStampedVector est_vek, rot_vek;
  Eigen::Affine3d rot_mat, est_mat;
  //std::vector<float> rot_vek;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> clouds;
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled;



};


}
