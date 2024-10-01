/**
 * @brief  Holds a Vilens SLAM mission data.
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <string>
#include <vector>
#include <map>

struct Pose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int id;
  uint32_t sec;
  uint32_t nsec;
  Eigen::Isometry3d pose;
};

struct Edge {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int i, j;
  Eigen::Isometry3d delta;
  bool switchable;
  bool maxMix;
  Eigen::MatrixXd info; // 6 by 6 information matrix
  double weight;
};

// Currently used by April Tag
struct Landmark {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int i;
  int tag_id;
  Eigen::Isometry3d relative_pose;
  uint32_t sec;
  uint32_t nsec;
};

typedef std::vector<Pose, Eigen::aligned_allocator<Pose>> PoseVector;
typedef std::vector<Edge, Eigen::aligned_allocator<Edge>> EdgeVector;
typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark>> LandmarkVector;

// Minimal parser for just the TAG_DETECTION files
bool parseDatasetAprilTag(const std::string &mission_filename,
                          LandmarkVector &landmarks);

// Reads in a full SLAM problem
bool parseDataset(const std::string &mission_filename, PoseVector &poses,
                  EdgeVector &edges, LandmarkVector &landmarks,
                  PoseVector &image_poses, LandmarkVector &image_landmarks,
                  std::multimap<int, int> &poseToEdges, int id_offset);

/* Reads in a list of manual loop closure proposal pairs
 PAIR: pose id pairs are used like geometric LC proposals e.g. given pair 5
      and 100 get the geometric prior ... use it as the initial offset before
      doing ICP this only works if the graph is within 1-2m of correct
 PAIR_PRIOR: <currently> adds an additional offset on the geometric prior to
      large drift. This really a shaky idea - because subsequent loop closures
      are dependent on the first.
 DO NOT USE PAIR_PRIOR! Instead need to have a good relative initial offset guess.
*/
bool parseDatasetPair(const std::string &mission_filename, EdgeVector &edges);


class SissionData
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:

  SissionData();

  SissionData(const std::string &mission_filename, int id_offset_in,
              int mission_id_in);

  ~SissionData(){
  }

  Pose getPoseFromId(int input_id) const;

  Pose getPoseFromTimestamp(uint32_t input_sec, uint32_t input_nsec) const;

  void setOdomPoses();

  PoseVector poses;
  EdgeVector edges;
  LandmarkVector landmarks;
  LandmarkVector image_landmarks;
  std::multimap<int, int> poseToEdges;

  // Secondary Data - computed from input data
  PoseVector image_poses;

  PoseVector odom_poses; // raw original odometry poses. NB: id and nsec/sec not set

  int mission_id;
};
