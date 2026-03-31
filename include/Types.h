#pragma once

#include <list>
#include <map>
#include <set>
#include <vector>

namespace ORB_SLAM3 {

// Forward declarations
class KeyFrame;
class MapPoint;
class Map;

// ---------------------------------------------------------------------------
// ID types
// ---------------------------------------------------------------------------
using FrameId    = unsigned long;
using MapPointId = unsigned long;

// ---------------------------------------------------------------------------
// Collection aliases — {Element}{Container} pattern
// ---------------------------------------------------------------------------

// KeyFrame collections
using KeyFrameVec  = std::vector<KeyFrame*>;
using KeyFrameSet  = std::set<KeyFrame*>;
using KeyFrameList = std::list<KeyFrame*>;

// MapPoint collections
using MapPointVec  = std::vector<MapPoint*>;
using MapPointSet  = std::set<MapPoint*>;
using MapPointList = std::list<MapPoint*>;

// Map collections
using MapVec = std::vector<Map*>;
using MapSet = std::set<Map*>;

// ---------------------------------------------------------------------------
// Map aliases — {Key}{Value}Map pattern
// ---------------------------------------------------------------------------
using KeyFrameWeightMap     = std::map<KeyFrame*, int>;
using IdKeyFrameMap         = std::map<FrameId, KeyFrame*>;
using IdMapPointMap         = std::map<FrameId, MapPoint*>;
using KeyFrameObservationMap = std::map<KeyFrame*, std::tuple<int, int>>;

// ---------------------------------------------------------------------------
// Chi-squared thresholds for outlier rejection
// ---------------------------------------------------------------------------
// Inverse CDF of chi-squared distribution at 95% confidence
inline constexpr float kChi2Mono   = 5.991F;  // 2 DOF (monocular reprojection)
inline constexpr float kChi2Stereo = 7.815F;  // 3 DOF (stereo reprojection)
inline constexpr float kChi2OneDof = 3.841F;  // 1 DOF

} // namespace ORB_SLAM3
