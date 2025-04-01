
#include "ros/ros.h"
#include "ros/node_handle.h"
#include "vector"
#include "stdio.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Image.h"

#include "boost/foreach.hpp"
#include "rosbag/view.h"
#include "rosbag/bag.h"

#include "lodestar_odometry/lodestar.h"
#include "lodestar_odometry/odometrykeyframefuser.h"

#include <mutex>
#include <condition_variable>
#include "ros/time.h"
#include "eigen_conversions/eigen_msg.h"
#include "lodestar_odometry/eval_trajectory.h"
#include <boost/program_options.hpp>
#include <iostream>

#define foreach BOOST_FOREACH
#define MAX_SIZE 3
using namespace lodestar_odom;
namespace po = boost::program_options;

/** \brief Original code is based on CFEAR odometry by Daniel Adolfsson.
 * This is the revised version for estiating maritime radar odometry with lodestar descriptor.
 */

typedef struct eval_parameters_{
  std::string bag_file_path ="";
  bool save_pcds = false;
}eval_parameters;


class radarReader
{

private:

  ros::NodeHandle nh_;
  ros::Publisher pub_odom;
  EvalTrajectory eval;
  lodestar driver;
  OdometryKeyframeFuser fuser;
  bool save = true;
  Eigen::Affine3d Toffset = Eigen::Affine3d::Identity();


public:


  radarReader(const OdometryKeyframeFuser::Parameters& odom_pars,
              const lodestar::Parameters& rad_pars,
              const EvalTrajectory::Parameters& eval_par,
              const eval_parameters& p) : nh_("~"), driver(rad_pars,true), fuser(odom_pars, true), eval(eval_par,true) {

    cout<<"Loading bag from: "<<p.bag_file_path<<endl;
    rosbag::Bag bag;
    bag.open(p.bag_file_path, rosbag::bagmode::Read);

    // std::vector<std::string> topics = {"radar_image_inrange"};//You can revise the radar topic name here
    std::vector<std::string> topics = {rad_pars.radar_topic};//You can revise the radar topic name here
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int frame = 0;
    ros::Time tinit;

/////////////////////////////////   Bagfile Start //////////////////////////////////////////////////////////////
    foreach(rosbag::MessageInstance const m, view)
    {

      if(!ros::ok())
        break;

      sensor_msgs::ImageConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
      if(image_msg != NULL) {
        tinit = ros::Time::now();
        pcl::PointCloud<pcl::PointXYZI>::Ptr lodestar_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        Eigen::Affine3d Trot;
        Eigen::Vector3d dense_trans;
        driver.CallbackOffline(image_msg, lodestar_cloud, Trot, dense_trans);// Operating LodeStar based pointcloud generation
        Eigen::Matrix3d rotMat = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();// To initialize offset
        Trot.rotate(rotMat);// To initialize offset
        
        lodestar_odom::timing.Document("Filtered points",lodestar_cloud->size());
        Eigen::Affine3d Tcurrent;
        fuser.pointcloudCallback(lodestar_cloud, Tcurrent, Trot);
        //cout<<Tcurrent.translation().transpose()<<endl;
        
        Eigen::Affine3d T_final= Tcurrent*Trot;//Tcurrent = Matrix from CFEAR, Trot = rotation from LodeStar
        
        const ros::Time t = image_msg->header.stamp;
        if(eval_par.save_pcd)
          eval.CallbackESTEigen(std::make_pair(T_final, t),*lodestar_cloud);
        else
          eval.CallbackESTEigen(std::make_pair(T_final, t));
        ros::Time tnow = ros::Time::now();
        ros::Duration d = ros::Duration(tnow-tinit);
        static ros::Duration tot(0);
        tot +=d;
        //usleep(100*1000);

        cout<<"Frame: "<<frame<<", Odom duration: "<<d<<"sec, avg: "<<++frame/tot.toSec()<<" Hz "<<endl;
        cout <<"oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo" << endl;
      }
    }
    cout<<fuser.GetStatus()<<endl;
    bag.close();
    //lodestar_odom::timing.PresentStatistics();

    return;
  }

  void Save(){
    eval.Save();
    return;
  }

  ~radarReader(){
    return;
  }
  size_t GetSize(){return eval.GetSize();}
};


void ReadOptions(const int argc, char**argv, OdometryKeyframeFuser::Parameters& par, lodestar::Parameters& rad_par, lodestar_odom::EvalTrajectory::Parameters& eval_par, eval_parameters& p){

    po::options_description desc{"Options"};
    desc.add_options()
        ("help,h", "Help screen")
        ("res", po::value<double>()->default_value(3.5), "res")
        ("range-res", po::value<double>()->default_value(0.0438), "range resolution")
        ("min_distance", po::value<double>()->default_value(0.5), "min sensor distance")
        ("max_distance", po::value<double>()->default_value(2000), "mib sensor distance ")
        ("submap_scan_size", po::value<int>()->default_value(3), "submap_scan_size")
        ("weight_intensity", po::value<bool>()->default_value(true),"weight_intensity")
        ("job_nr", po::value<int>()->default_value(-1), "jobnr")
        ("registered_min_keyframe_dist", po::value<double>()->default_value(1.5), "registered_min_keyframe_dist")
        ("z-min", po::value<double>()->default_value(65), "zmin intensity, expected noise level")
        ("soft_constraint", po::value<bool>()->default_value(false),"soft_constraint")
        ("savepcd", "save_pcd_files")
        ("disable_compensate", po::value<bool>()->default_value(false),"disable_compensate")
        ("cost_type", po::value<std::string>()->default_value("P2L"), "P2L")
        ("loss_type", po::value<std::string>()->default_value("Huber"), "robust loss function eg. Huber Caunchy, None")
        ("loss_limit", po::value<double>()->default_value(0.1), "loss limit")
        ("covar_scale", po::value<double>()->default_value(1), "covar scale")// Please fix combined parameter
        ("regularization", po::value<double>()->default_value(1), "regularization")
        ("est_directory", po::value<std::string>()->default_value(""), "output folder of estimated trajectory")
        ("sequence", po::value<std::string>()->default_value("2019-01-10-12-32-52-radar-oxford-10k"), "sequence contrained in \"bagfile\" to evaluate e.g. 2019-01-10-12-32-52-radar-oxford-10k")
        ("dataset", po::value<std::string>()->default_value("oxford"), "name of dataset, take special actions depending on radar file format etc")
        ("radar_topic", po::value<std::string>()->default_value("/radar_data"), "Radar topic name")
        ("k_nearest", po::value<int>()->default_value(20), "k_nearest points from the radar center")
        ("contour_threshold", po::value<int>()->default_value(50), "Threshold to extract the contour")
        ("method_name", po::value<std::string>()->default_value("method"), "method name")
        ("weight_option", po::value<int>()->default_value(0), "how to weight residuals")
        ("false-alarm-rate", po::value<float>()->default_value(0.01), "CA-CFAR false alarm rate")
        ("nb-guard-cells", po::value<int>()->default_value(10), "CA-CFAR nr guard cells")
        ("nb-window-cells", po::value<int>()->default_value(10), "CA-CFAR nr guard cells")
        ("bag_path", po::value<std::string>()->default_value("/home/daniel/rosbag/oxford-eval-sequences/2019-01-10-12-32-52-radar-oxford-10k/radar/2019-01-10-12-32-52-radar-oxford-10k.bag"), "bag file to open");

    po::variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help"))
      std::cout << desc << '\n';
    if (vm.count("res"))
      par.res = vm["res"].as<double>();
    if (vm.count("min_distance"))
      rad_par.min_distance = vm["min_distance"].as<double>();
    if (vm.count("max_distance"))
      rad_par.max_distance = vm["max_distance"].as<double>();
    if (vm.count("job_nr"))
      eval_par.job_nr = vm["job_nr"].as<int>();
    if (vm.count("cost_type"))
      par.cost_type = vm["cost_type"].as<std::string>();
    if (vm.count("loss_type"))
      par.loss_type_ = vm["loss_type"].as<std::string>();
    if (vm.count("loss_limit"))
      par.loss_limit_ = vm["loss_limit"].as<double>();
    if (vm.count("covar_scale"))
      par.covar_scale_ = vm["covar_scale"].as<double>();
    if (vm.count("regularization"))
      par.regularization_ = vm["regularization"].as<double>();
    if (vm.count("submap_scan_size"))
      par.submap_scan_size = vm["submap_scan_size"].as<int>();
    if (vm.count("registered_min_keyframe_dist"))
      par.min_keyframe_dist_= vm["registered_min_keyframe_dist"].as<double>();
    if (vm.count("est_directory"))
      eval_par.est_output_dir= vm["est_directory"].as<std::string>();
    if (vm.count("method_name"))
      eval_par.method = vm["method_name"].as<std::string>();
    if (vm.count("bag_path"))
      p.bag_file_path = vm["bag_path"].as<std::string>();
    if (vm.count("sequence"))
      eval_par.sequence = vm["sequence"].as<std::string>();
    if (vm.count("z-min"))
      rad_par.z_min = vm["z-min"].as<double>();
    if (vm.count("dataset"))
      rad_par.dataset = vm["dataset"].as<std::string>();
    if (vm.count("range-res"))
      rad_par.range_res = vm["range-res"].as<double>();
    if (vm.count("savepcd"))
      eval_par.save_pcd = true;
    if (vm.count("weight_option"))
      par.weight_opt = static_cast<weightoption>(vm["weight_option"].as<int>());
    if (vm.count("regularization"))
      rad_par.false_alarm_rate = vm["regularization"].as<double>();
    if (vm.count("covar_scale"))
      rad_par.window_size = vm["covar_scale"].as<double>();
    if (vm.count("radar_topic"))
      rad_par.radar_topic = vm["radar_topic"].as<std::string>();
    if (vm.count("k_nearest"))
      rad_par.k_nearest = vm["k_nearest"].as<int>();
    if (vm.count("contour_threshold"))
      rad_par.contour_threshold = vm["contour_threshold"].as<int>();



    par.weight_intensity_ = vm["weight_intensity"].as<bool>();;
    par.compensate = !vm["disable_compensate"].as<bool>();
    par.use_guess = true; //vm["soft_constraint"].as<bool>();
    par.soft_constraint = false; // soft constraint is rarely useful, this is changed for testing of initi // vm["soft_constraint"].as<bool>();

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lodestar_odom_node");
  OdometryKeyframeFuser::Parameters odom_pars;
  lodestar::Parameters rad_pars;
  EvalTrajectory::Parameters eval_pars;


  eval_parameters eval_p;
  ReadOptions(argc, argv, odom_pars, rad_pars, eval_pars, eval_p);


  std::ofstream ofs_before(eval_pars.est_output_dir+std::string("../pars.txt")); // Write
  std::string par_str_before = rad_pars.ToString()+odom_pars.ToString()+eval_pars.ToString()+"nr_frames, "+std::to_string(0)+"\n"+lodestar_odom::timing.GetStatistics();
  cout<<"Odometry parameters:\n" << par_str_before<<endl;
  ofs_before<<par_str_before<<endl;
  ofs_before.close();

  radarReader reader(odom_pars, rad_pars, eval_pars, eval_p);
  reader.Save();

  std::ofstream ofs(eval_pars.est_output_dir+std::string("../pars.txt")); // Write
  std::string par_str = rad_pars.ToString()+odom_pars.ToString()+eval_pars.ToString()+"\nnr_frames, "+std::to_string(reader.GetSize())+"\n"+lodestar_odom::timing.GetStatistics();
  ofs<<par_str<<endl;
  ofs.close();

  return 0;
}



