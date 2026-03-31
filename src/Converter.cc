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

g2o::SE3Quat Converter::toSE3Quat(const cv::Mat& cvT) {
  Eigen::Matrix<double, 3, 3> R;
  R << cvT.at<float>(0, 0), cvT.at<float>(0, 1), cvT.at<float>(0, 2), cvT.at<float>(1, 0),
    cvT.at<float>(1, 1), cvT.at<float>(1, 2), cvT.at<float>(2, 0), cvT.at<float>(2, 1),
    cvT.at<float>(2, 2);

  const Eigen::Matrix<double, 3, 1> t(cvT.at<float>(0, 3), cvT.at<float>(1, 3), cvT.at<float>(2, 3));

  return {R, t};
}

g2o::SE3Quat Converter::toSE3Quat(const Sophus::SE3f& T) {
  return {T.unit_quaternion().cast<double>(), T.translation().cast<double>()};
}

Eigen::Vector3f Converter::toVector3f(const cv::Mat& cvVector) {
  Eigen::Vector3f v;
  v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);
  return v;
}

Eigen::Matrix3f Converter::toMatrix3f(const cv::Mat& cvMat3) {
  Eigen::Matrix3f M;
  M << cvMat3.at<float>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2),
    cvMat3.at<float>(1, 0), cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2),
    cvMat3.at<float>(2, 0), cvMat3.at<float>(2, 1), cvMat3.at<float>(2, 2);
  return M;
}

Sophus::SE3<float> Converter::toSophus(const cv::Mat& T) {
  Eigen::Matrix<double, 3, 3> eigR;
  eigR << T.at<float>(0, 0), T.at<float>(0, 1), T.at<float>(0, 2),
    T.at<float>(1, 0), T.at<float>(1, 1), T.at<float>(1, 2),
    T.at<float>(2, 0), T.at<float>(2, 1), T.at<float>(2, 2);
  const Eigen::Quaternionf q(eigR.cast<float>());

  const Eigen::Matrix<float, 3, 1> t(T.at<float>(0, 3), T.at<float>(1, 3), T.at<float>(2, 3));

  return {q, t};
}

Sophus::Sim3f Converter::toSophus(const g2o::Sim3& S) {
  return {
    Sophus::RxSO3d(static_cast<float>(S.scale()), S.rotation().matrix()).cast<float>(),
    S.translation().cast<float>()
  };
}

} // namespace ORB_SLAM3
