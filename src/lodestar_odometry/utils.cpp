#include "lodestar_odometry/utils.h"
namespace lodestar_odom {

typedef std::tuple<double, double, pcl::PointXYZI> PointAIXYZ;
bool sortGreater(const PointAIXYZ& a,
                 const PointAIXYZ& b)
{
  return (std::get<0>(a) > std::get<0>(b));
}




void Compensate(pcl::PointCloud<pcl::PointXYZI>& cloud, const std::vector<double>& mot){
  for ( int i=0; i<cloud.size();i++) {
    pcl::PointXYZI p = cloud.points[i];
    double d = GetRelTimeStamp(p.x, p.y);
    Eigen::Vector2d peig(p.x,p.y);
    Eigen::Matrix2d R = getScaledRotationMatrix(mot, d).block<2,2>(0,0);
    Eigen::Vector2d t = getScaledTranslationVector(mot, d).block<2,1>(0,0);
    Eigen::Vector2d peig_transformed = R*peig + t;
    cloud.points[i].x = peig_transformed(0,0);
    cloud.points[i].y = peig_transformed(1,0);
  }
}

void Compensate(pcl::PointCloud<pcl::PointXYZI>& cloud, const Eigen::Affine3d& Tmotion){
  cout << "Compenstaion??" << endl;
  std::vector<double> mot;
  lodestar_odom::Affine3dToVectorXYeZ(Tmotion, mot);
  Compensate(cloud, mot);
}

void Affine3dToVectorXYeZ(const Eigen::Affine3d& T, std::vector<double>& par) {
  if(par.size()!=3)
    par.resize(3,0);
  par[0] = T.translation()(0);
  par[1] = T.translation()(1);
  Eigen::Vector3d eul = T.linear().eulerAngles(0,1,2);
  par[2] = eul(2);
}

void Affine3dToEigVectorXYeZ(const Eigen::Affine3d& T, Eigen::Vector3d& par) {
  Eigen::Vector3d eul = T.linear().eulerAngles(0,1,2);
  par << T.translation()(0), T.translation()(1), eul(2);
}


Eigen::Matrix3d getScaledRotationMatrix(const std::vector<double>& parameters, double factor)
{
  Eigen::Matrix3d rotation_matrix;
  const double s_1 = ceres::sin(factor*parameters[2]);
  const double c_1 = ceres::cos(factor*parameters[2]);
  rotation_matrix <<
      c_1,     -s_1,     0.0,
      s_1,     c_1,      0.0,
      0.0,     0.0,      1.0;
  return rotation_matrix;
}

Eigen::Vector3d getScaledTranslationVector(const std::vector<double>& parameters, double factor){
  Eigen::Vector3d vek;
  vek << factor*parameters[0], factor*parameters[1], 0.0;
  return vek;
}






}
