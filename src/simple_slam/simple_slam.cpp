#include <iostream>
#include <unistd.h>

#include "slam_debug/simple_slam/simple_slam.hpp"
#include "slam_debug/simple_slam/sission_data.hpp"

#include <gtsam/slam/BetweenFactor.h> //bifactors defined edges between nodes in the factor graph

using namespace std;

App::App(CommandLineConfig &cl_cfg):
    cl_cfg_(cl_cfg){

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;

  isam_ = new ISAM2(parameters);

//    prior_info: [1.e+2, 1.e+2, 1.e+2, 1.e+2, 1.e+2, 1.e+2]
//    odometry_info: [1.e+6, 1.e+6, 1.e+6, 1.e+6, 1.e+6, 1.e+6] # order: roll, pitch, yaw, x, y, z

  Vector6 prior_info;
  prior_info << 1.e+2, 1.e+2, 1.e+2, 1.e+2, 1.e+2, 1.e+2;
  prior_noise_ = noiseModel::Diagonal::Precisions(prior_info);

  Vector6 odometry_info;
  odometry_info << 1.e+6, 1.e+6, 1.e+6, 1.e+6, 1.e+6, 1.e+6;
  odometry_noise_ = noiseModel::Diagonal::Precisions(odometry_info);

}

void quat_to_euler(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2.0*(q0*q1+q2*q3), 1.0-2.0*(q1*q1+q2*q2));
  pitch = asin(2.0*(q0*q2-q3*q1));
  yaw = atan2(2.0*(q0*q3+q1*q2), 1.0-2.0*(q2*q2+q3*q3));
}

Eigen::Isometry3d gtsamPoseToIso(const Pose3 &pose){
  Eigen::Isometry3d pose_eigen = Eigen::Isometry3d::Identity();
  pose_eigen.translate(pose.translation());
  Eigen::Quaterniond quat = pose.rotation().toQuaternion();

  //std::cerr << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z() << " eigen\n";
  //gtsam::Quaternion quat_sam = pose.rotation().toQuaternion();
  //std::cerr << quat_sam.w() << " " << quat_sam.x() << " " << quat_sam.y() << " " << quat_sam.z() << " gtsam\n\n";

  pose_eigen.rotate(quat);
  return pose_eigen;
}

void quickPrint(const Eigen::Isometry3d &pose, const std::string &label){
  double rpy[3];
  quat_to_euler(Eigen::Quaterniond(pose.rotation()), rpy[0], rpy[1], rpy[2]);
  std::cerr << pose.translation().transpose() << " " << rpy[2]*180/M_PI << " " << label << "\n";;
}


uint64_t App::addPriorFactor(uint64_t keyID,
                                const Eigen::Isometry3d &initial_pose_eigen){

  NonlinearFactorGraph factor_graph;
  Values initial_guess;
  Pose3 initial_pose(Rot3(initial_pose_eigen.rotation()),Point3(initial_pose_eigen.translation()));
  factor_graph.addPrior<Pose3>(keyID, initial_pose, prior_noise_);
  //pose_times_.push_back(key_time);
  initial_guess.insert(keyID, initial_pose);
  const ISAM2Result result = isam_->update(factor_graph, initial_guess);

  // isam_->update(); // unnecessary ... removed
  solution_ = isam_->calculateEstimate();

  return static_cast<uint64_t>(result.newFactorsIndices[0]);
}


void App::addOdometryFactorInner(uint64_t tailID, uint64_t headID,
                                    const Eigen::Isometry3d &relative_trans,
                                    const Eigen::Isometry3d &head_initial_guess,
                                    gtsam::noiseModel::Base::shared_ptr relative_noise) {

  NonlinearFactorGraph factor_graph;
  Values initial_guess;
  Pose3 relative_trans_gtsam(Rot3(relative_trans.rotation()), Point3(relative_trans.translation()));
  factor_graph.emplace_shared<BetweenFactor<Pose3>>(tailID, headID, relative_trans_gtsam, relative_noise);


  Pose3 initial_guess_pose(Rot3(head_initial_guess.rotation()), Point3(head_initial_guess.translation()));
  initial_guess.insert(headID, initial_guess_pose);
  //std::cerr << "a\n";
//  factor_graph.print("Serialized factor graph: ");
  isam_->update(factor_graph, initial_guess);
  //std::cerr << "b\n";
  isam_->update();
  //std::cerr << "c\n";

  solution_ = isam_->calculateEstimate();

}



void App::getResult(std::vector<Eigen::Isometry3d> &vertices,
                  std::vector<std::pair<uint64_t, uint64_t> > &edges){


 std::map<Key, Pose3> poses = solution_.extract<Pose3>();
 for (const auto& key_value: poses){
   // dont count landmarks
   const Symbol sym(key_value.first);
   if ((sym.chr() == 'l') || (sym.chr() == 'i')){
     continue;
   }
   const gtsam::Pose3 pose = key_value.second;

    //std::cerr << pose << " gtsam pose. getResult\n";
    Eigen::Isometry3d pose_eigen = gtsamPoseToIso(pose);
    //quickPrint(pose_eigen, "pose_eigen");

    vertices.push_back(pose_eigen);

    //std::cerr << key_value.key << ": " << pose.translation().transpose() << ", "
    //  << quat.w() << ", " << quat.x() << ", " << quat.y() << ", " << quat.z() << "\n";

    last_pose_ = pose;
  }

}


std::string getEnvVar( std::string const & key )
{
    char * val = getenv( key.c_str() );
    return val == NULL ? std::string("") : std::string(val);
}

void App::runTest(){
  std::cerr << "starting runTest\n";
  std::string file_path = getEnvVar("HOME") + "/git/slam_debug/data/";
  
  std::cerr << "About to read:\n";
  std::cerr << string(file_path + "/slam_pose_graph.slam") << "\n";

  SissionData mission = SissionData(string(file_path + "/slam_pose_graph.slam"),0,0);
  std::cout << mission.poses.size() << " is mission poses size\n";


  Pose initial_p = mission.poses[0];

  Eigen::Isometry3d initial_pose = initial_p.pose;
  addPriorFactor(0, initial_pose);


  Eigen::Isometry3d last_running_pose = Eigen::Isometry3d::Identity();

  int n_pose_counter=0;
  Eigen::Isometry3d running_pose = initial_p.pose;

  Eigen::Affine3f running_pose_af = Eigen::Affine3f (initial_p.pose.matrix().cast<float>() );
  for (size_t i = 0; i < mission.edges.size(); i++){
    const Edge e = mission.edges[i];
    std::cerr << "\n\n";
    std::cerr << i << " is i\n";
    if (e.i + 1 == e.j){
      n_pose_counter++;

      last_running_pose = running_pose;
      running_pose = running_pose * e.delta;

      std::vector<Eigen::Isometry3d> last_vertices;
      std::vector<std::pair<uint64_t, uint64_t>> last_edges;
      getResult(last_vertices, last_edges);

      Eigen::Isometry3d initial_guess = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d this_relative_trans = last_running_pose.inverse() * running_pose;

      // Initial Guess is last SLAM pose composed with the relative trans (the 'odometry measurement')
      initial_guess = last_vertices.back() * this_relative_trans;
      

      //std::cerr << "These should be identical:\n";
      //Eigen::Isometry3d last_pose_eigen = gtsamPoseToIso(last_pose_);
      //quickPrint(last_pose_eigen, "last_pose_gtsam" );
      //quickPrint(last_vertices.back(), "last_pose_eigen" );

      //initial_guess = last_vertices[i] * this_relative_trans;
      //initial_guess = last_pose_eigen * this_relative_trans;

      std::cerr << i << " and " << i+1 << " - tailID and headID\n";
      addOdometryFactorInner(i,i+1, e.delta, initial_guess, odometry_noise_);
      //addOdometryFactorInner(i,i+1, e.delta, running_pose, odometry_noise_); // THICK IS A HACK - FEEDS ODOMETRY AS INITIAL GUESS - NOT LAST SLAM RESULT!


      std::vector<Eigen::Isometry3d> vertices;
      std::vector<std::pair<uint64_t, uint64_t>> edges;
      getResult(vertices, edges);



      std::cout << vertices.size() << " vertices end\n";
      std::cerr << "These should be identical:\n";
      quickPrint(initial_guess, "initial_guess" );
      quickPrint(vertices.back(), "vertices" );

    }else{
      std::cerr << "no loops in this mode\n";
      exit(-1);
    }
  }

  bool write_output=false;
  if(write_output){

    std::vector<Eigen::Isometry3d> vertices;
    std::vector<std::pair<uint64_t, uint64_t>> edges;
    getResult(vertices, edges);

    std::cerr << vertices.size() << " is size of vertices\n";
    for (size_t i=0; i < vertices.size(); i++){
      quickPrint(vertices[i], "vertices");
    }
  }

  std::cout << "Finished runTest\n";
}


int main( int argc, char** argv ){

  CommandLineConfig cl_cfg;
  cl_cfg.odom_frame = "odom_vilens";
  cl_cfg.map_path = "path-to-raw-data";

  App app(cl_cfg);
  app.runTest();

  return 0;
}

