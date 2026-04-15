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

#include <Eigen/Core>
#include <g2o/g2o/types/se3quat.h>
#include <g2o/g2o/types/sim3.h>
#include <opencv2/core.hpp>
#include <sophus/geometry.hpp>
#include <sophus/sim3.hpp>

namespace ORB_SLAM3 {

class Converter {
public:
  static std::vector<cv::Mat> toDescriptorVector(const cv::Mat& Descriptors);

  static cv::Mat toCvMat(const Eigen::Matrix3f& m);

  static Eigen::Vector3f toVector3f(const cv::Mat& cvVector);
  static Eigen::Matrix3f toMatrix3f(const cv::Mat& cvMat3);

  static Sophus::SE3f  toSophus(const cv::Mat& T);
  static Sophus::Sim3f toSophus(const g2o::Sim3& S);
};

} // namespace ORB_SLAM3
