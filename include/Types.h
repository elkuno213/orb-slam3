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

#include <cstdint>

namespace ORB_SLAM3 {

// Input sensor type
enum class Sensor : uint8_t {
  Mono           = 0,
  Stereo         = 1,
  Rgbd           = 2,
  InertialMono   = 3,
  InertialStereo = 4,
  InertialRgbd   = 5,
};

// Tracking state enum (extracted to avoid circular dependency)
enum class TrackingState : int8_t {
  SystemNotReady = -1,
  NoImagesYet    = 0,
  NotInitialized = 1,
  Ok             = 2,
  RecentlyLost   = 3,
  Lost           = 4,
  OkKlt          = 5
};

} // namespace ORB_SLAM3
