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

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <spdlog/logger.h>
#include "Types.h"

namespace ORB_SLAM3 {

class GeometricCamera;
class KeyFrame;
class MapPoint;

/// Result of Sim3Solver::find()
struct Sim3Result {
  Eigen::Matrix4f   T;               ///< Transformation matrix (Identity if failed)
  std::vector<bool> inliers;         ///< Inlier flags for each match
  int               num_inliers = 0; ///< Number of inliers found
};

/// Result of Sim3Solver::iterate()
struct IterateResult {
  Eigen::Matrix4f   T;               ///< Transformation matrix (Identity if failed/not converged)
  bool              no_more = false; ///< True if max iterations reached
  std::vector<bool> inliers;         ///< Inlier flags for each match
  int               num_inliers = 0; ///< Number of inliers found
  bool              converged   = false; ///< True if solution converged
};

class Sim3Solver {
public:
  struct CentroidResult {
    Eigen::Matrix3f relative_coords; ///< Relative coordinates to centroid
    Eigen::Vector3f centroid;
  };

  Sim3Solver(
    KeyFrame*                     pKF1,
    KeyFrame*                     pKF2,
    const std::vector<MapPoint*>& vpMatched12,
    bool                          bFixScale           = true,
    std::vector<KeyFrame*>        vpKeyFrameMatchedMP = std::vector<KeyFrame*>()
  );

  void SetRansacParameters(double probability = 0.99, int minInliers = 6, int maxIterations = 300);

  /// Run RANSAC until convergence or max iterations
  [[nodiscard]] Sim3Result find();

  /// Run a fixed number of RANSAC iterations
  [[nodiscard]] IterateResult iterate(int nIterations);

  [[nodiscard]] Eigen::Matrix4f GetEstimatedTransformation();
  [[nodiscard]] Eigen::Matrix3f GetEstimatedRotation();
  [[nodiscard]] Eigen::Vector3f GetEstimatedTranslation();
  [[nodiscard]] float           GetEstimatedScale() const;

protected:
  [[nodiscard]] static CentroidResult ComputeCentroid(const Eigen::Matrix3f& P);

  void ComputeSim3(Eigen::Matrix3f& P1, Eigen::Matrix3f& P2);

  void CheckInliers();

  [[nodiscard]] static std::vector<Eigen::Vector2f> Project(
    const std::vector<Eigen::Vector3f>& vP3Dw, const Eigen::Matrix4f& Tcw, GeometricCamera* pCamera
  );
  [[nodiscard]] static std::vector<Eigen::Vector2f> FromCameraToImage(
    const std::vector<Eigen::Vector3f>& vP3Dc, GeometricCamera* pCamera
  );

  // KeyFrames and matches
  KeyFrame* mpKF1;
  KeyFrame* mpKF2;

  std::vector<Eigen::Vector3f> mvX3Dc1;
  std::vector<Eigen::Vector3f> mvX3Dc2;
  std::vector<MapPoint*>       mvpMapPoints1;
  std::vector<MapPoint*>       mvpMapPoints2;
  std::vector<MapPoint*>       mvpMatches12;
  std::vector<std::size_t>     mvnIndices1;
  std::vector<std::size_t>     mvSigmaSquare1;
  std::vector<std::size_t>     mvSigmaSquare2;
  std::vector<std::size_t>     mvnMaxError1;
  std::vector<std::size_t>     mvnMaxError2;

  int N;
  int mN1;

  // Current Estimation
  Eigen::Matrix3f   mR12i;
  Eigen::Vector3f   mt12i;
  float             ms12i;
  Eigen::Matrix4f   mT12i;
  Eigen::Matrix4f   mT21i;
  std::vector<bool> mvbInliersi;
  int               mnInliersi;

  // Current Ransac State
  int               mnIterations{0};
  std::vector<bool> mvbBestInliers;
  int               mnBestInliers{0};
  Eigen::Matrix4f   mBestT12;
  Eigen::Matrix3f   mBestRotation;
  Eigen::Vector3f   mBestTranslation;
  float             mBestScale;

  // Scale is fixed to 1 in the stereo/RGBD case
  bool mbFixScale;

  // Indices for random selection
  std::vector<std::size_t> mvAllIndices;

  // Projections
  std::vector<Eigen::Vector2f> mvP1im1;
  std::vector<Eigen::Vector2f> mvP2im2;

  // RANSAC probability
  double mRansacProb;

  // RANSAC min inliers
  int mRansacMinInliers;

  // RANSAC max iterations
  int mRansacMaxIts;

  float mTh;
  float mSigma2;

  // Calibration
  // cv::Mat mK1;
  // cv::Mat mK2;

  GeometricCamera *pCamera1, *pCamera2;

  std::shared_ptr<spdlog::logger> _logger;
};

} // namespace ORB_SLAM3
