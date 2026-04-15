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

#include "Converter.h"
#include <vector>
#include <sophus/rxso3.hpp>

namespace ORB_SLAM3 {

std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat& Descriptors) {
  std::vector<cv::Mat> vDesc;
  vDesc.reserve(Descriptors.rows);
  for (int j = 0; j < Descriptors.rows; j++) {
    vDesc.push_back(Descriptors.row(j));
  }

  return vDesc;
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3f& m) {
  cv::Mat cvMat(3, 3, CV_32F);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      cvMat.at<float>(i, j) = m(i, j);
    }
  }

  return cvMat.clone();
}

Eigen::Vector3f Converter::toVector3f(const cv::Mat& cvVector) {
  Eigen::Vector3f v;
  v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

  return v;
}

Eigen::Matrix3f Converter::toMatrix3f(const cv::Mat& cvMat3) {
  Eigen::Matrix3f M;

  M << cvMat3.at<float>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2),
    cvMat3.at<float>(1, 0), cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2), cvMat3.at<float>(2, 0),
    cvMat3.at<float>(2, 1), cvMat3.at<float>(2, 2);

  return M;
}

Sophus::SE3f Converter::toSophus(const cv::Mat& T) {
  const Eigen::Matrix3f    eigMat = toMatrix3f(T.rowRange(0, 3).colRange(0, 3));
  const Eigen::Quaternionf q(eigMat);
  const Eigen::Vector3f    t = toVector3f(T.rowRange(0, 3).col(3));
  return {q, t};
}

Sophus::Sim3f Converter::toSophus(const g2o::Sim3& S) {
  return {
    Sophus::RxSO3d(static_cast<float>(S.scale()), S.rotation().matrix()).cast<float>(),
    S.translation().cast<float>()
  };
}

} // namespace ORB_SLAM3
