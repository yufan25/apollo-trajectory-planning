/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>

#include "vehicle_config.pb.h"
#include "perception_obstacle.pb.h"
// #include "modules/planning/proto/decision.pb.h"
// #include "modules/planning/proto/sl_boundary.pb.h"
// #include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "box2d.h"
#include "vec2d.h"
#include "polygon2d.h"
#include "math_utils.h"
// #include "modules/planning/common/indexed_list.h"
// #include "modules/planning/common/speed/st_boundary.h"

#include "pnc_point.pb.h"
#include "planning_gflags.h"

namespace apollo {
namespace planning {

using namespace apollo::common::math;
/**
 * @class Obstacle
 * @brief This is the class that associates an Obstacle with its path
 * properties. An obstacle's path properties relative to a path.
 * The `s` and `l` values are examples of path properties.
 * The decision of an obstacle is also associated with a path.
 *
 * The decisions have two categories: lateral decision and longitudinal
 * decision.
 * Lateral decision includes: nudge, ignore.
 * Lateral decision safety priority: nudge > ignore.
 * Longitudinal decision includes: stop, yield, follow, overtake, ignore.
 * Decision safety priorities order: stop > yield >= follow > overtake > ignore
 *
 * Ignore decision belongs to both lateral decision and longitudinal decision,
 * and it has the lowest priority.
 */
class Obstacle {
 public:
  Obstacle() = default;
  explicit Obstacle(
      const std::string& id,
      const struct apollo::perception::PerceptionObstacle& perception_obstacle,
      //const prediction::ObstaclePriority::Priority& obstacle_priority,
      const bool is_static) : id_(id),
      perception_id_(perception_obstacle.id),
      perception_obstacle_(perception_obstacle),
      perception_bounding_box_({perception_obstacle_.position.x,
                                perception_obstacle_.position.y},
                               perception_obstacle_.theta,
                               perception_obstacle_.length,
                               perception_obstacle_.width)
  {
    // is_caution_level_obstacle_ = (obstacle_priority == ObstaclePriority::CAUTION);
    std::vector<common::math::Vec2d> polygon_points;
    if (FLAGS_use_navigation_mode ||
        perception_obstacle.polygon_point.size() <= 2) {
      perception_bounding_box_.GetAllCorners(&polygon_points);
    } else {
      // CHECK(perception_obstacle.polygon_point_size() > 2)
      //     << "object " << id << "has less than 3 polygon points";
      for (const auto& point : perception_obstacle.polygon_point) {
        polygon_points.emplace_back(point.x, point.y);
      }
    }
    bool pd = ComputeConvexHull(polygon_points, &perception_polygon_);

    // is_static_ = (is_static || obstacle_priority == ObstaclePriority::IGNORE);
    is_static_ = true;
    // is_virtual_ = (perception_obstacle.id() < 0);
    is_virtual_ = false;
    speed_ = std::hypot(perception_obstacle.velocity.x,
                        perception_obstacle.velocity.y);
  }
  // explicit Obstacle(
  //     const std::string& id,
  //     const perception::PerceptionObstacle& perception_obstacle,
  //     const prediction::Trajectory& trajectory,
  //     const prediction::ObstaclePriority::Priority& obstacle_priority,
  //     const bool is_static);

  const std::string& Id() const { return id_; }
  void SetId(const std::string& id) { id_ = id; }

  // double speed() const { return speed_; }

bool ComputeConvexHull(const std::vector<Vec2d> &points,
                                  Polygon2d *const polygon) {
  // CHECK_NOTNULL(polygon);
  const int n = static_cast<int>(points.size());
  if (n < 3) {
    return false;
  }
  std::vector<int> sorted_indices(n);
  for (int i = 0; i < n; ++i) {
    sorted_indices[i] = i;
  }
  std::sort(sorted_indices.begin(), sorted_indices.end(),
            [&](const int idx1, const int idx2) {
              const Vec2d &pt1 = points[idx1];
              const Vec2d &pt2 = points[idx2];
              const double dx = pt1.x() - pt2.x();
              if (std::abs(dx) > kMathEpsilon) {
                return dx < 0.0;
              }
              return pt1.y() < pt2.y();
            });
  int count = 0;
  std::vector<int> results;
  results.reserve(n);
  int last_count = 1;
  for (int i = 0; i < n + n; ++i) {
    if (i == n) {
      last_count = count;
    }
    const int idx = sorted_indices[(i < n) ? i : (n + n - 1 - i)];
    const Vec2d &pt = points[idx];
    while (count > last_count &&
           CrossProd(points[results[count - 2]], points[results[count - 1]],
                     pt) <= kMathEpsilon) {
      results.pop_back();
      --count;
    }
    results.push_back(idx);
    ++count;
  }
  --count;
  if (count < 3) {
    return false;
  }
  std::vector<Vec2d> result_points;
  result_points.reserve(count);
  for (int i = 0; i < count; ++i) {
    result_points.push_back(points[results[i]]);
  }
  *polygon = Polygon2d(result_points);
  return true;
}  

  // int32_t PerceptionId() const { return perception_id_; }

  // bool IsStatic() const { return is_static_; }
  bool IsVirtual() const { return is_virtual_; }

  TrajectoryPoint GetPointAtTime(const double time) const
  {
    const auto& points = trajectory_.trajectory_point;
    // if (points.size() < 2) {
      TrajectoryPoint point;
      point.path_point.set_x(perception_obstacle_.position.x);
      point.path_point.set_y(perception_obstacle_.position.y);
      point.path_point.set_z(perception_obstacle_.position.z);
      point.path_point.set_theta(perception_obstacle_.theta);
      point.path_point.set_s(0.0);
      point.path_point.set_kappa(0.0);
      point.path_point.set_dkappa(0.0);
      point.path_point.set_ddkappa(0.0);
      point.set_v(0.0);
      point.set_a(0.0);
      point.set_relative_time(0.0);
      return point;
    // } 
    // else {
    //   auto comp = [](const TrajectoryPoint p, const double time) {
    //     return p.relative_time < time;
    //   };

    //   auto it_lower =
    //       std::lower_bound(points.begin(), points.end(), time, comp);

    //   if (it_lower == points.begin()) {
    //     return *points.begin();
    //   } else if (it_lower == points.end()) {
    //     return *points.rbegin();
    //   }
    //   return common::math::InterpolateUsingLinearApproximation(
    //       *(it_lower - 1), *it_lower, time);
    // }
  }

  common::math::Box2d GetBoundingBox(
      const TrajectoryPoint& point) const
  {
    return common::math::Box2d({point.path_point.x, point.path_point.y},
                             point.path_point.theta,
                             perception_obstacle_.length,
                             perception_obstacle_.width); 
  }

  // const common::math::Box2d& PerceptionBoundingBox() const {
  //   return perception_bounding_box_;
  // }
  const common::math::Polygon2d& PerceptionPolygon() const {
    return perception_polygon_;
  }
  const struct Trajectory& Trajectory() const { return trajectory_; }
  // common::TrajectoryPoint* AddTrajectoryPoint() {
  //   return trajectory_.add_trajectory_point();
  // }
  bool HasTrajectory() const {
    return !(trajectory_.trajectory_point.empty());
  }

  // const perception::PerceptionObstacle& Perception() const {
  //   return perception_obstacle_;
  // }

  /**
   * @brief This is a helper function that can create obstacles from prediction
   * data.  The original prediction may have multiple trajectories for each
   * obstacle. But this function will create one obstacle for each trajectory.
   * @param predictions The prediction results
   * @return obstacles The output obstacles saved in a list of unique_ptr.
   */
  // static std::list<std::unique_ptr<Obstacle>> CreateObstacles(
  //     const prediction::PredictionObstacles& predictions);

  // static std::unique_ptr<Obstacle> CreateStaticVirtualObstacles(
  //     const std::string& id, const common::math::Box2d& obstacle_box);

  // static bool IsValidPerceptionObstacle(
  //     const perception::PerceptionObstacle& obstacle);

  // static bool IsValidTrajectoryPoint(const common::TrajectoryPoint& point);

  // inline bool IsCautionLevelObstacle() const {
  //   return is_caution_level_obstacle_;
  // }

  // const Obstacle* obstacle() const;

  /**
   * return the merged lateral decision
   * Lateral decision is one of {Nudge, Ignore}
   **/
//   const ObjectDecisionType& LateralDecision() const;

//   /**
//    * @brief return the merged longitudinal decision
//    * Longitudinal decision is one of {Stop, Yield, Follow, Overtake, Ignore}
//    **/
//   const ObjectDecisionType& LongitudinalDecision() const;

//   const std::string DebugString() const;

//   const SLBoundary& PerceptionSLBoundary() const;

//   const STBoundary& reference_line_st_boundary() const;

//   const STBoundary& path_st_boundary() const;

//   const std::vector<std::string>& decider_tags() const;

//   const std::vector<ObjectDecisionType>& decisions() const;

//   void AddLongitudinalDecision(const std::string& decider_tag,
//                                const ObjectDecisionType& decision);

//   void AddLateralDecision(const std::string& decider_tag,
//                           const ObjectDecisionType& decision);
//   bool HasLateralDecision() const;

//   void set_path_st_boundary(const STBoundary& boundary);

//   void SetStBoundaryType(const STBoundary::BoundaryType type);

//   void EraseStBoundary();

//   void SetReferenceLineStBoundary(const STBoundary& boundary);

//   void SetReferenceLineStBoundaryType(const STBoundary::BoundaryType type);

//   void EraseReferenceLineStBoundary();

//   bool HasLongitudinalDecision() const;

//   bool HasNonIgnoreDecision() const;

//   /**
//    * @brief Calculate stop distance with the obstacle using the ADC's minimum
//    * turning radius
//    */
//   double MinRadiusStopDistance(const common::VehicleParam& vehicle_param) const;

//   /**
//    * @brief Check if this object can be safely ignored.
//    * The object will be ignored if the lateral decision is ignore and the
//    * longitudinal decision is ignore
//    *  return longitudinal_decision_ == ignore && lateral_decision == ignore.
//    */
//   bool IsIgnore() const;
//   bool IsLongitudinalIgnore() const;
//   bool IsLateralIgnore() const;

//   void BuildReferenceLineStBoundary(const ReferenceLine& reference_line,
//                                     const double adc_start_s);

//   void SetPerceptionSlBoundary(const SLBoundary& sl_boundary);

//   /**
//    * @brief check if an ObjectDecisionType is a longitudinal decision.
//    **/
//   static bool IsLongitudinalDecision(const ObjectDecisionType& decision);

//   /**
//    * @brief check if an ObjectDecisionType is a lateral decision.
//    **/
//   static bool IsLateralDecision(const ObjectDecisionType& decision);

//   void SetBlockingObstacle(bool blocking) { is_blocking_obstacle_ = blocking; }
//   bool IsBlockingObstacle() const { return is_blocking_obstacle_; }

//   /*
//    * @brief IsLaneBlocking is only meaningful when IsStatic() == true.
//    */
//   bool IsLaneBlocking() const { return is_lane_blocking_; }
//   void CheckLaneBlocking(const ReferenceLine& reference_line);
//   bool IsLaneChangeBlocking() const { return is_lane_change_blocking_; }
//   void SetLaneChangeBlocking(const bool is_distance_clear);

//  private:
//   FRIEND_TEST(MergeLongitudinalDecision, AllDecisions);
//   static ObjectDecisionType MergeLongitudinalDecision(
//       const ObjectDecisionType& lhs, const ObjectDecisionType& rhs);
//   FRIEND_TEST(MergeLateralDecision, AllDecisions);
//   static ObjectDecisionType MergeLateralDecision(const ObjectDecisionType& lhs,
//                                                  const ObjectDecisionType& rhs);

//   bool BuildTrajectoryStBoundary(const ReferenceLine& reference_line,
//                                  const double adc_start_s,
//                                  STBoundary* const st_boundary);
//   bool IsValidObstacle(
//       const perception::PerceptionObstacle& perception_obstacle);

 private:
  std::string id_ ="zero";
  int32_t perception_id_ = 0;
  bool is_static_ = false;
  bool is_virtual_ = false;
  double speed_ = 0.0;
  struct Trajectory trajectory_; //如果是dynamic的障碍物的话是有trajectory_point的，静态的没有
  // std::vector<TrajectoryPoint> trajectory_;
  apollo::perception::PerceptionObstacle perception_obstacle_;
  common::math::Box2d perception_bounding_box_;//构造函数赋值， 来源是perception_obstacle_
  common::math::Polygon2d perception_polygon_; //由Polygon2d::ComputeConvexHull赋值，来源是perception_obstacle_

  // std::vector<ObjectDecisionType> decisions_;
  // std::vector<std::string> decider_tags_;
  // SLBoundary sl_boundary_;

  // STBoundary reference_line_st_boundary_;
  // STBoundary path_st_boundary_;

  // ObjectDecisionType lateral_decision_;
  // ObjectDecisionType longitudinal_decision_;

  // for keep_clear usage only
  bool is_blocking_obstacle_ = false;

  bool is_lane_blocking_ = false;

  bool is_lane_change_blocking_ = false;

  bool is_caution_level_obstacle_ = false;

  double min_radius_stop_distance_ = -1.0;

  // struct ObjectTagCaseHash {
  //   size_t operator()(
  //       const planning::ObjectDecisionType::ObjectTagCase tag) const {
  //     return static_cast<size_t>(tag);
  //   }
  // };

  // static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
  //                                 ObjectTagCaseHash>
  //     s_lateral_decision_safety_sorter_;
  // static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
  //                                 ObjectTagCaseHash>
  //     s_longitudinal_decision_safety_sorter_;
};

// typedef IndexedList<std::string, Obstacle> IndexedObstacles;
// typedef ThreadSafeIndexedList<std::string, Obstacle> ThreadSafeIndexedObstacles;

}  // namespace planning
}  // namespace apollo
