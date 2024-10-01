#include "slam_debug/simple_slam/sission_data.hpp"

#include <fstream>
#include <iostream>

Landmark makeLandmarkFromStream(std::ifstream &inFile, int id_offset) {
  Landmark l;
  int in_id;
  double in_x, in_y, in_z;
  double in_qx, in_qy, in_qz, in_qw;
  inFile >> in_id >> l.tag_id >> in_x >> in_y >> in_z >> in_qx >> in_qy >>
      in_qz >> in_qw >> l.sec >> l.nsec;
  l.i = in_id + id_offset;
  l.relative_pose = Eigen::Isometry3d::Identity();
  l.relative_pose.translation() << in_x, in_y, in_z;
  l.relative_pose.rotate(Eigen::Quaterniond(in_qw, in_qx, in_qy, in_qz));

  return l;
}

// TODO: get rid of this function and use the same parsing as makeLandmarkFromStream
// TOOD: first would need to store time in the .slam file
Landmark makeImageLandmarkFromStream(std::ifstream &inFile, int id_offset) {
  Landmark l;
  int in_id;
  double in_x, in_y, in_z;
  double in_qx, in_qy, in_qz, in_qw;
  inFile >> in_id >> l.tag_id >> in_x >> in_y >> in_z >> in_qx >> in_qy >>
      in_qz >> in_qw; //
  l.sec = 0;  // not stored currently. TODO: fix
  l.nsec = 0; // not stored currently. TODO: fix
  l.i = in_id + id_offset;
  l.relative_pose = Eigen::Isometry3d::Identity();
  l.relative_pose.translation() << in_x, in_y, in_z;
  l.relative_pose.rotate(Eigen::Quaterniond(in_qw, in_qx, in_qy, in_qz));

  return l;
}

Pose makePoseFromStream(std::ifstream &inFile, int id_offset) {
  double in_x, in_y, in_z;
  double in_qx, in_qy, in_qz, in_qw;
  int in_id;
  uint32_t in_sec, in_nsec;
  inFile >> in_id >> in_x >> in_y >> in_z >> in_qx >> in_qy >> in_qz >>
          in_qw >> in_sec >> in_nsec;
  Pose p;
  p.id = in_id + id_offset;
  p.sec = in_sec;
  p.nsec = in_nsec;
  p.pose = Eigen::Isometry3d::Identity();
  p.pose.translation() << in_x, in_y, in_z;
  p.pose.rotate(Eigen::Quaterniond(in_qw, in_qx, in_qy, in_qz));

  return p;
}


bool parseDatasetAprilTag(const std::string &mission_filename,
                          LandmarkVector &landmarks) {
  int id_offset = 0;  // hardcoded to be zero

  // open the file
  std::ifstream inFile(mission_filename.c_str());
  if (!inFile) {
    std::cerr << "Error opening dataset file " << mission_filename << std::endl;
    return false;
  }

  // go through the dataset file
  while (!inFile.eof()) {
    // depending on the type of vertex or edge, read the data
    std::string type;
    inFile >> type;
    if (type == "#") {  // if line begins '#', ignore remainder
      getline(inFile, type);
    } else if (type == "TAG_DETECTION") {
      landmarks.push_back(makeLandmarkFromStream(inFile, id_offset));
    }
  }
  return true;
}

bool parseDataset(const std::string &mission_filename, PoseVector &poses,
                  EdgeVector &edges, LandmarkVector &landmarks,
                  PoseVector &image_poses, LandmarkVector &image_landmarks,
                  std::multimap<int, int> &poseToEdges, int id_offset) {
  std::cout << std::endl
            << "Parsing dataset " << mission_filename << " ... " << std::endl;

  // open the file
  std::ifstream inFile(mission_filename.c_str());
  if (!inFile) {
    std::cerr << "Error opening dataset file " << mission_filename << std::endl;
    return false;
  }

  // go through the dataset file
  while (!inFile.eof()) {
    // depending on the type of vertex or edge, read the data
    std::string type;
    inFile >> type;
    if (type == "#") {  // if line begins '#', ignore remainder
      getline(inFile, type);
    } else if (type == "VERTEX_SE3:QUAT_TIME") {
      poses.push_back(makePoseFromStream(inFile, id_offset));
    } else if (type == "EDGE_SE3_SWITCHABLE" || type == "EDGE_SE3:QUAT" ||
               type == "EDGE_SE3_MAXMIX") {
      double dummy;
      Edge e;

      // read pose IDs
      inFile >> e.i >> e.j;
      e.i = e.i + id_offset;
      e.j = e.j + id_offset;

      // 30June22: The following could cause crypic errors. I dont know why it
      // exists It has been commented out but left here in case it existed for a
      // reason
      // if (e.i>e.j) {
      //  std::swap(e.i,e.j);
      //  std::cout << "Fix this if it ever happens\n";
      //  exit(-1);
      //}

      // read the switch variable ID (only needed in g2o, we dont need it here
      // in the gtsam world)
      if (type == "EDGE_SE3_SWITCHABLE") inFile >> dummy;
      if (type == "EDGE_SE3_MAXMIX") inFile >> e.weight;

      // read odometry measurement
      double in_x, in_y, in_z;
      double in_qx, in_qy, in_qz, in_qw;
      inFile >> in_x >> in_y >> in_z >> in_qx >> in_qy >> in_qz >> in_qw;
      e.delta = Eigen::Isometry3d::Identity();
      e.delta.translation() << in_x, in_y, in_z;
      e.delta.rotate(Eigen::Quaterniond(in_qw, in_qx, in_qy, in_qz));

      // g2o's EDGE_SE3:QUAT stores covariance in t,R order, unlike GTSAM (R,t)
      // 1. read information matrix in the order (t, R) ...
      double info[21];
      inFile >> info[0] >> info[1] >> info[2] >> info[3] >> info[4] >>
          info[5] >> info[6] >> info[7] >> info[8] >> info[9] >> info[10] >>
          info[11] >> info[12] >> info[13] >> info[14] >> info[15] >>
          info[16] >> info[17] >> info[18] >> info[19] >> info[20];
      Eigen::MatrixXd infoMatrix_g2o = Eigen::MatrixXd::Identity(6, 6);
      infoMatrix_g2o(0, 0) = info[0];
      infoMatrix_g2o(1, 1) = info[6];
      infoMatrix_g2o(2, 2) = info[11];
      infoMatrix_g2o(3, 3) = info[15];
      infoMatrix_g2o(4, 4) = info[18];
      infoMatrix_g2o(5, 5) = info[20];

      infoMatrix_g2o(0, 1) = infoMatrix_g2o(1, 0) = info[1];
      infoMatrix_g2o(0, 2) = infoMatrix_g2o(2, 0) = info[2];
      infoMatrix_g2o(0, 3) = infoMatrix_g2o(3, 0) = info[3];
      infoMatrix_g2o(0, 4) = infoMatrix_g2o(4, 0) = info[4];
      infoMatrix_g2o(0, 5) = infoMatrix_g2o(5, 0) = info[5];

      infoMatrix_g2o(1, 2) = infoMatrix_g2o(2, 1) = info[7];
      infoMatrix_g2o(1, 3) = infoMatrix_g2o(3, 1) = info[8];
      infoMatrix_g2o(1, 4) = infoMatrix_g2o(4, 1) = info[9];
      infoMatrix_g2o(1, 5) = infoMatrix_g2o(5, 1) = info[10];

      infoMatrix_g2o(2, 3) = infoMatrix_g2o(3, 2) = info[12];
      infoMatrix_g2o(2, 4) = infoMatrix_g2o(4, 2) = info[13];
      infoMatrix_g2o(2, 5) = infoMatrix_g2o(5, 2) = info[14];

      infoMatrix_g2o(3, 4) = infoMatrix_g2o(4, 3) = info[16];
      infoMatrix_g2o(3, 5) = infoMatrix_g2o(5, 3) = info[17];

      infoMatrix_g2o(4, 5) = infoMatrix_g2o(5, 4) = info[19];

      // Reorder to GTSAM format which is (R,t)
      Eigen::MatrixXd infoMatrix = Eigen::MatrixXd::Identity(6, 6);
      infoMatrix.block<3, 3>(0, 0) =
          infoMatrix_g2o.block<3, 3>(3, 3);  // swap cov rotation ul -> lr
      infoMatrix.block<3, 3>(3, 3) =
          infoMatrix_g2o.block<3, 3>(0, 0);  // swap cov translation lr -> ul
      infoMatrix.block<3, 3>(3, 0) =
          infoMatrix_g2o.block<3, 3>(0, 3);  // swap off diagonal ur -> ll
      infoMatrix.block<3, 3>(0, 3) =
          infoMatrix_g2o.block<3, 3>(3, 0);  // swap off diagonal ll -> ur
      // std::cout << "gtsam info matrix is:\n" << infoMatrix << std::endl; //
      // gtsam information matrix is correct

      e.info = infoMatrix;

      if (type == "EDGE_SE3_SWITCHABLE") {
        e.switchable = true;
        e.maxMix = false;
      } else if (type == "EDGE_SE3_MAXMIX") {
        e.switchable = false;
        e.maxMix = true;
      } else {
        e.switchable = false;
        e.maxMix = false;
      }

      edges.push_back(e);
      int id = edges.size() - 1;
      poseToEdges.insert(std::pair<int, int>(e.j, id));
    } else if (type == "TAG_DETECTION") {
      landmarks.push_back(makeLandmarkFromStream(inFile, id_offset));
    } else if (type == "IMAGE_POSE") {
      image_poses.push_back(makePoseFromStream(inFile, id_offset));
    } else if (type == "EDGE_POSE_TO_IMAGE") {
      image_landmarks.push_back(makeImageLandmarkFromStream(inFile, id_offset));
    }
  }
  return true;
}

bool parseDatasetPair(const std::string &mission_filename, EdgeVector &edges) {
  std::cout << std::endl
            << "Parsing pair dataset " << mission_filename << " ... "
            << std::endl;

  // open the file
  std::ifstream inFile(mission_filename.c_str());
  if (!inFile) {
    std::cerr << "Error opening pair dataset file " << mission_filename
              << std::endl;
    return false;
  }

  // go through the dataset file
  while (!inFile.eof()) {
    std::string type;
    inFile >> type;
    if (type == "#") {  // if line begins '#', ignore remainder
      getline(inFile, type);
    } else if (type == "PAIR") {
      Edge e;
      inFile >> e.i >> e.j;
      e.delta = Eigen::Isometry3d::Identity();
      edges.push_back(e);

    } else if (type == "PAIR_PRIOR") {
      // This is intepreted as an adjustment to the current estimate of T_AB
      // (from the slam system) But we would like it to be the transform from
      // P_A which is the full estimate of T_AB As a result this is only really
      // useful for the first pose DONT USE THIS! TODO: make this more useful
      std::cout << "PAIR_PRIOR - this mode is disabled - needs better "
                   "implementation\n";
      exit(-1);
      Edge e;
      inFile >> e.i >> e.j;
      // e.i = e.i + id_offset;
      // e.j = e.j + id_offset;

      // read odometry measurement
      double in_x, in_y, in_z;
      double in_qx, in_qy, in_qz, in_qw;
      inFile >> in_x >> in_y >> in_z >> in_qx >> in_qy >> in_qz >> in_qw;
      e.delta = Eigen::Isometry3d::Identity();
      e.delta.translation() << in_x, in_y, in_z;
      e.delta.rotate(Eigen::Quaterniond(in_qw, in_qx, in_qy, in_qz));
      // Eigen::MatrixXd informationMatrix = Eigen::MatrixXd::Identity(6,6);
      // e.covariance = informationMatrix.inverse();
      edges.push_back(e);
    }
  }
  return true;
}

SissionData::SissionData() {}

SissionData::SissionData(const std::string &mission_filename, int id_offset_in,
                         int mission_id_in) {
  parseDataset(mission_filename, poses, edges, landmarks, image_poses, image_landmarks, poseToEdges,
               id_offset_in);
  mission_id = mission_id_in;

  setOdomPoses();
}

Pose SissionData::getPoseFromId(int input_id) const {
  Pose p;
  p.id = -1;
  for (size_t i = 0; i < poses.size(); i++) {
    // if (e.j == mission->poses[i].id){
    if (input_id == poses[i].id) {
      // std::cout << i << ": found the match " << e.j << " | " <<
      // mission->poses[i].id << "\n";
      std::cout << i << ": found the match " << input_id << " | " << poses[i].id
                << "\n";
      std::cout << poses[i].sec << " and " << poses[i].nsec << "\n";
      p = poses[i];
      return p;
    }
  }
  p.sec = 0;
  p.nsec = 0;
  p.pose = Eigen::Isometry3d::Identity();
  return p;
}

Pose SissionData::getPoseFromTimestamp(uint32_t input_sec,
                                       uint32_t input_nsec) const {
  Pose p;
  p.id = -1;
  for (size_t i = 0; i < poses.size(); i++) {
    // if (e.j == mission->poses[i].id){
    if (input_sec == poses[i].sec) {
      if (input_nsec == poses[i].nsec) {
        // std::cout << i << ": found the match " << e.j << " | " <<
        // mission->poses[i].id << "\n"; std::cout << i << ": found the match "
        // << poses[i].id << "\n"; std::cout << poses[i].sec << " and " <<
        // poses[i].nsec << "\n";
        p = poses[i];
        return p;
      }
    }
  }
  p.sec = 0;
  p.nsec = 0;
  p.pose = Eigen::Isometry3d::Identity();
  return p;
}

void SissionData::setOdomPoses() {
  if (poses.size() == 0){
    return;
  }

  odom_poses.clear();

  // Accumulate the raw odometry
  Pose odom_pose;
  odom_pose = poses[0];
  odom_poses.push_back(odom_pose);
  for (const Edge &e : edges) {
    if (e.i + 1 != e.j) {  // TODO: make this better. it's quite fragile
      continue;            // not an odom pose
    }
    odom_pose.pose = odom_pose.pose * e.delta;
    odom_poses.push_back(odom_pose);
  }
}
