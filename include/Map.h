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

#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <Eigen/Core>
#include <boost/serialization/access.hpp>
#include <boost/serialization/vector.hpp>
#include <sophus/se3.hpp>
#include <spdlog/logger.h>
#include "ORBVocabulary.h"
#include "Types.h"

namespace ORB_SLAM3 {

class Atlas;
class GeometricCamera;
class KeyFrame;
class KeyFrameDatabase;
class MapPoint;

class Map {
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /* version */) {
    ar & mnId;
    ar & mnInitKFid;
    ar & mnMaxKFid;
    ar & mnBigChangeIdx;

    // Save/load a set structure, the set structure is broken in libboost 1.58 for ubuntu 16.04, a
    // vector is serializated
    ar & mvpBackupKeyFrames;
    ar & mvpBackupMapPoints;

    ar & mvBackupKeyFrameOriginsId;

    ar & mnBackupKFinitialID;
    ar & mnBackupKFlowerID;

    ar & mbImuInitialized;
    ar & mbIsInertial;
    ar & mbIMU_BA1;
    ar & mbIMU_BA2;
  }

public:
  Map();
  explicit Map(int initKFid);
  ~Map();

  void AddKeyFrame(KeyFrame* pKF);
  void AddMapPoint(MapPoint* pMP);
  void EraseMapPoint(MapPoint* pMP);
  void EraseKeyFrame(KeyFrame* pKF);
  void SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs);
  void InformNewBigChange();
  int  GetLastBigChangeIdx();

  std::vector<KeyFrame*> GetAllKeyFrames();
  std::vector<MapPoint*> GetAllMapPoints();
  std::vector<MapPoint*> GetReferenceMapPoints();

  unsigned long MapPointsInMap();
  unsigned long KeyFramesInMap();

  unsigned long GetId() const;

  FrameId GetInitKFid();
  void    SetInitKFid(FrameId initKFif);
  FrameId GetMaxKFid();

  KeyFrame* GetOriginKF();

  void SetCurrentMap();
  void SetStoredMap();

  [[nodiscard]] bool IsInUse() const;

  void SetBad();
  [[nodiscard]] bool IsBad() const;

  void clear();

  int  GetMapChangeIndex();
  void IncreaseChangeIndex();
  int  GetLastMapChange();
  void SetLastMapChange(int currentChangeId);

  void SetImuInitialized();
  bool isImuInitialized();

  void ApplyScaledRotation(const Sophus::SE3f& T, float s, bool bScaledVel = false);

  void SetInertialSensor();
  bool IsInertial();
  void SetIniertialBA1();
  void SetIniertialBA2();
  bool GetIniertialBA1();
  bool GetIniertialBA2();

  void ChangeId(unsigned long nId);

  unsigned int GetLowerKFID();

  void PreSave(std::set<GeometricCamera*>& spCams);
  void PostLoad(
    KeyFrameDatabase* pKFDB,
    ORBVocabulary*    pORBVoc /*, IdKeyFrameMap& mpKeyFrameId*/,
    std::map<unsigned int, GeometricCamera*>& mpCams
  );

  std::vector<KeyFrame*>         mvpKeyFrameOrigins;
  std::vector<FrameId> mvBackupKeyFrameOriginsId;
  KeyFrame*                      mpFirstRegionKF;
  std::mutex                     mMutexMapUpdate;

  // This avoid that two points are created simultaneously in separate threads (id conflict)
  std::mutex mMutexPointCreation;

  static unsigned long nNextId;

  // DEBUG: show KFs which are used in LBA
  std::set<FrameId> msOptKFs;
  std::set<FrameId> msFixedKFs;

protected:
  unsigned long mnId;

  std::set<MapPoint*> mspMapPoints;
  std::set<KeyFrame*> mspKeyFrames;

  // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is
  // serializated
  std::vector<MapPoint*> mvpBackupMapPoints;
  std::vector<KeyFrame*> mvpBackupKeyFrames;

  KeyFrame* mpKFinitial;
  KeyFrame* mpKFlowerID;

  FrameId mnBackupKFinitialID;
  FrameId mnBackupKFlowerID;

  std::vector<MapPoint*> mvpReferenceMapPoints;

  bool mbImuInitialized;

  int mnMapChange;
  int mnMapChangeNotified;

  FrameId mnInitKFid;
  FrameId mnMaxKFid;

  // Index related to a big change in the map (loop closure, global BA)
  int mnBigChangeIdx;

  bool mIsInUse;
  bool mHasTumbnail;
  bool mbBad = false;

  bool mbIsInertial;
  bool mbIMU_BA1;
  bool mbIMU_BA2;

  // Mutex
  std::mutex mMutexMap;

  std::shared_ptr<spdlog::logger> _logger;
};

} // namespace ORB_SLAM3
