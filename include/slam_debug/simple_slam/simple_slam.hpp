#pragma once

#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h> // for the marginal covariance of the variables

using namespace gtsam;




struct CommandLineConfig
{
  std::string odom_frame;
  std::string map_path;
};

class App{
  public:
    App(CommandLineConfig &cl_cfg);
    
    ~App(){
    }    

    void getResult(std::vector<Eigen::Isometry3d> &vertices,
                  std::vector<std::pair<uint64_t, uint64_t> > &edges);

    void readOdometryData();

    uint64_t addPriorFactor(uint64_t keyID,
                                const Eigen::Isometry3d &initial_pose_eigen);

    void addOdometryFactorInner(uint64_t tailID, uint64_t headID,
                                    const Eigen::Isometry3d &relative_trans,
                                    const Eigen::Isometry3d &head_initial_guess,
                                    gtsam::noiseModel::Base::shared_ptr relative_noise);

  private:
    CommandLineConfig cl_cfg_;

    gtsam::ISAM2 *isam_;


    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_;

  gtsam::Values solution_;

    gtsam::Pose3 last_pose_;

};
