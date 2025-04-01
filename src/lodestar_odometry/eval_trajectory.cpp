#include "lodestar_odometry/eval_trajectory.h"

namespace lodestar_odom {


EvalTrajectory::EvalTrajectory(const EvalTrajectory::Parameters& pars, bool disable_callback) :par(pars), nh_("~"),downsampled(new pcl::PointCloud<pcl::PointXYZI>()){

  if(!disable_callback){
    assert(!par.odom_est_topic.empty());
    if(par.synced_callback){
      //pose_sub_est  = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, par.odom_est_topic, 100);
      //rot_sub_est  = new message_filters::Subscriber<std_msgs::Float32>(nh_, "/rot_lodestar", 100);
      //sync = new Synchronizer<double_odom>(double_odom(100), *rot_sub_est, *pose_sub_est);
      //sync->registerCallback(boost::bind(&EvalTrajectory::CallbackSynchronized,this, _1, _2));
    }
    else{
      sub_rot_est = nh_.subscribe("/rot_lodestar", 1000, &EvalTrajectory::CallbackRot, this);
      sub_est = nh_.subscribe(par.odom_est_topic, 1000, &EvalTrajectory::CallbackEst, this);
    }
  }
  pub_est = nh_.advertise<nav_msgs::Path>("path_est", 10);
  pub_cloud = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("map_cloud", 10);

}


void EvalTrajectory::CallbackESTEigen(const poseStamped& Test){
  est_vek.push_back(Test);
}
void EvalTrajectory::CallbackESTEigen(const poseStamped& Test, const pcl::PointCloud<pcl::PointXYZI>& cld){
  clouds.push_back(cld);
  CallbackESTEigen(Test);
}

void EvalTrajectory::CallbackEst(const nav_msgs::Odometry::ConstPtr &msg){
  Eigen::Affine3d T;
  tf::poseMsgToEigen(msg->pose.pose, T);
  //T = T*rot_vek;
  ros::Time t = msg->header.stamp;
  est_vek.push_back(std::make_pair(T, t));
  //est_mat = T;
}

void EvalTrajectory::CallbackRot(const std_msgs::Float32::ConstPtr &msg){
  //rot_vek.push_back(msg_rot->data);
  cout << "callback" << endl;
  Eigen::Matrix3d rotationMatrix;
    rotationMatrix =
        Eigen::AngleAxisd(msg->data*M_PI/180, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // Now convert it into an Affine3d transformation.
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.rotate(rotationMatrix);
    ros::Time t = ros::Time::now();
  rot_vek.push_back(std::make_pair(transform, t));
  //rot_mat = transform;
}


void EvalTrajectory::Write(const std::string& path, const poseStampedVector& v,const poseStampedVector& r){
  std::ofstream evalfile;
  cout<<"Saving: "<<v.size()<<" poses to file: "<<path<<endl;
  //cout<<"With: "<<r.size()<<" rotations "<<endl;
  evalfile.open(path);
  for(size_t i=0;i<v.size();i++){
    Eigen::MatrixXd m(v[i].first.matrix());
    evalfile<< std::fixed << std::showpoint;
    assert(m.rows()== 4 && m.cols()==4);
//(2778m/2048px)/(0.05m/px)*0.75 = 20.346679687 // seadronix
//(1654.8/2048px)/0.05 = 16.16015625 // pohang
    evalfile<< m(0,0) <<" "<< m(0,1) <<" "<< m(0,2) <<" "<< m(0,3)*22.73 <<" "<<
               m(1,0) <<" "<< m(1,1) <<" "<< m(1,2) <<" "<< m(1,3)*22.73 <<" "<<
               m(2,0) <<" "<< m(2,1) <<" "<< m(2,2) <<" "<< m(2,3) <<std::endl;
  }
  evalfile.close();
  return;
}



std::string EvalTrajectory::DatasetToSequence(const std::string& dataset){
  return "01.txt";
}

void EvalTrajectory::PublishTrajectory(poseStampedVector& vek, ros::Publisher& pub){
  nav_msgs::Path path;
  path.header.frame_id="world";
  path.header.stamp = ros::Time::now();

  std::vector<tf::StampedTransform> trans_vek;
  for (int i=0;i<vek.size();i++) {
    Eigen::Affine3d T = vek[i].first;
    geometry_msgs::PoseStamped Tstamped;
    tf::poseEigenToMsg(T,Tstamped.pose);
    path.poses.push_back(Tstamped);
  }
  pub.publish(path);
}

void EvalTrajectory::SavePCD(const std::string& folder){
  for (size_t i=0;i<clouds.size();i++) {
    pcl::PointCloud<pcl::PointXYZI> cld_transformed;
    pcl::transformPointCloud(clouds[i], cld_transformed, est_vek[i].first);
    pcl::io::savePCDFileBinary(folder+"cloud_"+std::to_string(i)+std::string(".pcd"), cld_transformed);
  }
}

void EvalTrajectory::Save(){
  cout << "Saving, outpout: " << par.est_output_dir << std::endl;
  if(est_vek.empty()){
    cout<<"Nothing estimated"<<endl;
    cerr<<"array size error. est_vek.size()="<<est_vek.size()<<endl;
    exit(0);
  }
  else{
    boost::filesystem::create_directories(par.est_output_dir);
    std::string est_path = par.est_output_dir+DatasetToSequence(par.sequence);
    cout<<"Saving estimated "<<est_vek.size()<<" poses"<<endl;
    cout<<"To path: "<<est_path<<endl;
    if(par.save_pcd)
      SavePCD(par.est_output_dir);
    Write(est_path, est_vek, rot_vek);
    cout<<"Trajectoy saved"<<endl;
    return;
    
  }
  return;
}




}
