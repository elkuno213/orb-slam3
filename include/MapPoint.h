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
#include <memory>
#include <mutex>
#include <set>
#include <tuple>
#include <Eigen/Core>
#include <boost/serialization/access.hpp>
#include <boost/serialization/array_wrapper.hpp>
#include <boost/serialization/map.hpp>
#include <opencv2/core.hpp>
#include <spdlog/logger.h>
#include "SerializationUtils.h"
#include "Types.h"

namespace ORB_SLAM3 {

class Frame;
class KeyFrame;
class Map;

class MapPoint {
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & mnId;
    ar & mnFirstKFid;
    ar & mnFirstFrame;
    ar & nObs;
    // Protected variables
    ar& boost::serialization::make_array(mWorldPos.data(), mWorldPos.size());
    ar& boost::serialization::make_array(mNormalVector.data(), mNormalVector.size());
    ar & mBackupObservationsId1;
    ar & mBackupObservationsId2;
    serializeMatrix(ar, mDescriptor, version);
    ar & mBackupRefKFId;
    ar & mbBad;
    ar & mBackupReplacedId;

    ar & mfMinDistance;
    ar & mfMaxDistance;
  }

public:
  MapPoint();

  MapPoint(const Eigen::Vector3f& Pos, KeyFrame* pRefKF, Map* pMap);
  MapPoint(double invDepth, cv::Point2f uv_init, KeyFrame* pRefKF, KeyFrame* pHostKF, Map* pMap);
  MapPoint(const Eigen::Vector3f& Pos, Map* pMap, Frame* pFrame, int idxF);

  void                         SetWorldPos(const Eigen::Vector3f& Pos);
  [[nodiscard]] Eigen::Vector3f GetWorldPos();

  [[nodiscard]] Eigen::Vector3f GetNormal();
  void                          SetNormalVector(const Eigen::Vector3f& normal);

  [[nodiscard]] KeyFrame* GetReferenceKeyFrame();

  [[nodiscard]] std::map<KeyFrame*, std::tuple<int, int>> GetObservations();
  [[nodiscard]] int                                       Observations();

  void AddObservation(KeyFrame* pKF, int idx);
  void EraseObservation(KeyFrame* pKF);

  [[nodiscard]] std::tuple<int, int> GetIndexInKeyFrame(KeyFrame* pKF);
  [[nodiscard]] bool                 IsInKeyFrame(KeyFrame* pKF);

  void                SetBadFlag();
  [[nodiscard]] bool  isBad();

  void                   Replace(MapPoint* pMP);
  [[nodiscard]] MapPoint* GetReplaced();

  void              IncreaseVisible(int n = 1);
  void              IncreaseFound(int n = 1);
  float             GetFoundRatio();
  void ComputeDistinctiveDescriptors();

  [[nodiscard]] cv::Mat GetDescriptor();

  void UpdateNormalAndDepth();

  [[nodiscard]] float GetMinDistanceInvariance();
  [[nodiscard]] float GetMaxDistanceInvariance();
  [[nodiscard]] int   PredictScale(float currentDist, KeyFrame* pKF);
  [[nodiscard]] int   PredictScale(float currentDist, Frame* pF);

  [[nodiscard]] Map* GetMap();
  void UpdateMap(Map* pMap);

  void PreSave(std::set<KeyFrame*>& spKF, std::set<MapPoint*>& spMP);
  void PostLoad(IdKeyFrameMap& mpKFid, IdMapPointMap& mpMPid);

  MapPointId        mnId;
  static MapPointId nNextId;
  long int                 mnFirstKFid;
  long int                 mnFirstFrame;
  int                      nObs;

  // Variables used by the tracking
  float             mTrackProjX;
  float             mTrackProjY;
  float             mTrackDepth;
  float             mTrackDepthR;
  float             mTrackProjXR;
  float             mTrackProjYR;
  bool              mbTrackInView, mbTrackInViewR;
  int               mnTrackScaleLevel, mnTrackScaleLevelR;
  float             mTrackViewCos, mTrackViewCosR;
  FrameId mnTrackReferenceForFrame;
  FrameId mnLastFrameSeen;

  // Variables used by local mapping
  FrameId mnBALocalForKF;
  FrameId mnFuseCandidateForKF;

  // Variables used by loop closing
  FrameId         mnLoopPointForKF;
  FrameId         mnCorrectedByKF;
  FrameId         mnCorrectedReference;
  Eigen::Vector3f mPosGBA;
  FrameId         mnBAGlobalForKF;
  FrameId         mnBALocalForMerge;

  // Variable used by merging
  Eigen::Vector3f mPosMerge;
  Eigen::Vector3f mNormalVectorMerge;

  // Fopr inverse depth optimization
  double    mInvDepth;
  double    mInitU;
  double    mInitV;
  KeyFrame* mpHostKF;

  static std::mutex mGlobalMutex;

  unsigned int mnOriginMapId;

protected:
  // Position in absolute coordinates
  Eigen::Vector3f mWorldPos;

  // Keyframes observing the point and associated index in keyframe
  std::map<KeyFrame*, std::tuple<int, int>> mObservations;
  // For save relation without pointer, this is necessary for save/load function
  std::map<FrameId, int> mBackupObservationsId1;
  std::map<FrameId, int> mBackupObservationsId2;

  // Mean viewing direction
  Eigen::Vector3f mNormalVector;

  // Best descriptor to fast matching
  cv::Mat mDescriptor;

  // Reference KeyFrame
  KeyFrame*         mpRefKF;
  FrameId mBackupRefKFId;

  // Tracking counters
  int mnVisible;
  int mnFound;

  // Bad flag (we do not currently erase MapPoint from memory)
  bool      mbBad;
  MapPoint* mpReplaced;
  // For save relation without pointer, this is necessary for save/load function
  long long int mBackupReplacedId;

  // Scale invariance distances
  float mfMinDistance;
  float mfMaxDistance;

  Map* mpMap;

  // Mutex
  std::mutex mMutexPos;
  std::mutex mMutexFeatures;
  std::mutex mMutexMap;

  std::shared_ptr<spdlog::logger> _logger;
};

} // namespace ORB_SLAM3
