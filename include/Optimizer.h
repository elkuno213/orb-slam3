/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel
 * and Juan D. Tardós, University of Zaragoza. Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M.
 * Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <map>
#include <set>
#include <vector>
#include <Eigen/Core>
#include <g2o/g2o/types/sim3.h>
#include "LoopClosing.h"

namespace ORB_SLAM3 {

class Frame;
class KeyFrame;
class Map;
class MapPoint;

class Optimizer {
public:
  /// Statistics returned by local bundle adjustment methods
  struct BAStats {
    int num_fixed_kf   = 0; ///< Number of fixed keyframes
    int num_opt_kf     = 0; ///< Number of optimized keyframes
    int num_map_points = 0; ///< Number of map points
    int num_edges      = 0; ///< Number of edges in the optimization graph
  };

  /// Result of full inertial optimization (gravity, scale, biases)
  struct InertialOptResult {
    Eigen::Matrix3d Rwg;         ///< Rotation from world to gravity-aligned frame
    double          scale = 1.0; ///< Scale factor
    Eigen::Vector3d bg;          ///< Gyroscope bias
    Eigen::Vector3d ba;          ///< Accelerometer bias
    Eigen::MatrixXd covInertial; ///< Inertial covariance (TODO: implement covariance propagation)
  };

  /// Result of bias-only inertial optimization
  struct BiasOptResult {
    Eigen::Vector3d bg; ///< Gyroscope bias
    Eigen::Vector3d ba; ///< Accelerometer bias
  };

  /// Result of gravity/scale inertial optimization
  struct GravityScaleResult {
    Eigen::Matrix3d Rwg;         ///< Rotation from world to gravity-aligned frame
    double          scale = 1.0; ///< Scale factor
  };

  static void BundleAdjustment(
    const std::vector<KeyFrame*>& vpKF,
    const std::vector<MapPoint*>& vpMP,
    int                           nIterations = 5,
    bool*                         pbStopFlag  = nullptr,
    unsigned long                 nLoopKF     = 0,
    bool                          bRobust     = true
  );
  static void GlobalBundleAdjustment(
    Map*          pMap,
    int           nIterations = 5,
    bool*         pbStopFlag  = nullptr,
    unsigned long nLoopKF     = 0,
    bool          bRobust     = true
  );
  static void FullInertialBA(
    Map*             pMap,
    int              its,
    bool             bFixLocal  = false,
    unsigned long    nLoopId    = 0,
    bool*            pbStopFlag = nullptr,
    bool             bInit      = false,
    float            priorG     = 1e2,
    float            priorA     = 1e6,
    Eigen::VectorXd* vSingVal   = nullptr,
    bool*            bHess      = nullptr
  );

  [[nodiscard]] static BAStats LocalBundleAdjustment(KeyFrame* pKF, bool* pbStopFlag, Map* pMap);

  static int PoseOptimization(Frame* pFrame);
  static int PoseInertialOptimizationLastKeyFrame(Frame* pFrame, bool bRecInit = false);
  static int PoseInertialOptimizationLastFrame(Frame* pFrame, bool bRecInit = false);

  // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
  static void OptimizeEssentialGraph(
    Map*                                            pMap,
    KeyFrame*                                       pLoopKF,
    KeyFrame*                                       pCurKF,
    const LoopClosing::KeyFrameAndPose&             NonCorrectedSim3,
    const LoopClosing::KeyFrameAndPose&             CorrectedSim3,
    const std::map<KeyFrame*, std::set<KeyFrame*>>& LoopConnections,
    bool                                            bFixScale
  );
  static void OptimizeEssentialGraph(
    KeyFrame*               pCurKF,
    std::vector<KeyFrame*>& vpFixedKFs,
    std::vector<KeyFrame*>& vpFixedCorrectedKFs,
    std::vector<KeyFrame*>& vpNonFixedKFs,
    std::vector<MapPoint*>& vpNonCorrectedMPs
  );

  // For inertial loopclosing
  static void OptimizeEssentialGraph4DoF(
    Map*                                            pMap,
    KeyFrame*                                       pLoopKF,
    KeyFrame*                                       pCurKF,
    const LoopClosing::KeyFrameAndPose&             NonCorrectedSim3,
    const LoopClosing::KeyFrameAndPose&             CorrectedSim3,
    const std::map<KeyFrame*, std::set<KeyFrame*>>& LoopConnections
  );

  // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono) (NEW)
  static int OptimizeSim3(
    KeyFrame*                    pKF1,
    KeyFrame*                    pKF2,
    std::vector<MapPoint*>&      vpMatches1,
    g2o::Sim3&                   g2oS12,
    float                        th2,
    bool                         bFixScale,
    Eigen::Matrix<double, 7, 7>& mAcumHessian,
    bool                         bAllPoints = false
  );

  // For inertial systems

  [[nodiscard]] static BAStats LocalInertialBA(
    KeyFrame* pKF, bool* pbStopFlag, Map* pMap, bool bLarge = false, bool bRecInit = false
  );
  static void MergeInertialBA(
    KeyFrame*                     pCurrKF,
    KeyFrame*                     pMergeKF,
    bool*                         pbStopFlag,
    Map*                          pMap,
    LoopClosing::KeyFrameAndPose& corrPoses
  );

  // Local BA in welding area when two maps are merged
  static void LocalBundleAdjustment(
    KeyFrame*                     pMainKF,
    std::vector<KeyFrame*>        vpAdjustKF,
    const std::vector<KeyFrame*>& vpFixedKF,
    bool*                         pbStopFlag
  );

  // Marginalize block element (start:end,start:end). Perform Schur complement.
  // Marginalized elements are filled with zeros.
  static Eigen::MatrixXd Marginalize(const Eigen::MatrixXd& H, int start, int end);

  // Inertial pose-graph

  /// Full inertial optimization: optimizes gravity direction, scale, and biases
  [[nodiscard]] static InertialOptResult InertialOptimization(
    Map*                   pMap,
    const Eigen::Matrix3d& initial_Rwg,
    double                 initial_Scale,
    const Eigen::Vector3d& initial_Bg,
    const Eigen::Vector3d& initial_Ba,
    bool                   bMono,
    bool                   bFixedVel = false,
    bool                   bGauss    = false,
    float                  priorG    = 1e2,
    float                  priorA    = 1e6
  );

  /// Bias-only inertial optimization: optimizes gyroscope and accelerometer biases
  [[nodiscard]] static BiasOptResult InertialOptimization(
    Map* pMap, float priorG = 1e2, float priorA = 1e6
  );

  /// Gravity/scale inertial optimization: optimizes gravity direction and scale
  [[nodiscard]] static GravityScaleResult InertialOptimization(
    Map* pMap, const Eigen::Matrix3d& initial_Rwg, double initial_Scale
  );

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

} // namespace ORB_SLAM3
