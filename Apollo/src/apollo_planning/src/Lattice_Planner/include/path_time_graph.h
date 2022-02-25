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

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// #include "modules/common/proto/geometry.pb.h"
// 
#include "polygon2d.h"
// #include "modules/planning/common/frame.h"
#include "obstacle.h"
#include "reference_line_info.h"
#include "reference_line.h"
#include "vehicle_config.pb.h"
#include "planning_gflags.h"
#include "linear_interpolation.h"
#include "path_matcher.h"


#include "st_boundary.h"
#include "st_point.h"

namespace apollo {
namespace planning {
using namespace apollo::planning;
using namespace apollo::common::math;
// using apollo::common::math::lerp;
// using namespace apollo::common::math;

class PathTimeGraph {
 public:
  PathTimeGraph(const std::vector<const Obstacle*>& obstacles,
                const std::vector<PathPoint>& discretized_ref_points,
                ReferenceLineInfo* ptr_reference_line_info,
                const double s_start, const double s_end, const double t_start,
                const double t_end, const std::array<double, 3>& init_d)
    {
        path_range_.first = s_start;
        path_range_.second = s_end;
        time_range_.first = t_start;
        time_range_.second = t_end;
        ptr_reference_line_info_ = ptr_reference_line_info;
        init_d_ = init_d;

        SetupObstacles(obstacles, discretized_ref_points);

    }

  const std::vector<STBoundary>& GetPathTimeObstacles() const  {
  return path_time_obstacles_;
}

//   bool GetPathTimeObstacle(const std::string& obstacle_id,
//                            STBoundary* path_time_obstacle);

  std::vector<std::pair<double, double>> GetPathBlockingIntervals(
      const double t) const
{
    // CHECK(time_range_.first <= t && t <= time_range_.second);
  std::vector<std::pair<double, double>> intervals;
  for (const auto& pt_obstacle : path_time_obstacles_) {
    if (t > pt_obstacle.max_t() || t < pt_obstacle.min_t()) {
      continue;
    }
    double s_upper = lerp(pt_obstacle.upper_left_point().s(),
                          pt_obstacle.upper_left_point().t(),
                          pt_obstacle.upper_right_point().s(),
                          pt_obstacle.upper_right_point().t(), t);

    double s_lower = lerp(pt_obstacle.bottom_left_point().s(),
                          pt_obstacle.bottom_left_point().t(),
                          pt_obstacle.bottom_right_point().s(),
                          pt_obstacle.bottom_right_point().t(), t);

    intervals.emplace_back(s_lower, s_upper);
  }
  return intervals;
}

  std::vector<std::vector<std::pair<double, double>>> GetPathBlockingIntervals(
      const double t_start, const double t_end, const double t_resolution)
{
    std::vector<std::vector<std::pair<double, double>>> intervals;
    for (double t = t_start; t <= t_end; t += t_resolution) {
    intervals.push_back(GetPathBlockingIntervals(t));
     }
     return intervals;
}

double lerp( const double a0,  const double t0,  const double a1, const double t1,
             const double t) const
{
  double d = a1-a0;
  double r = (t-t0) / (t1-t0);
  return (a0 + r*d);
}

//   std::pair<double, double> get_path_range() const;

//   std::pair<double, double> get_time_range() const;

  std::vector<STPoint> GetObstacleSurroundingPoints(
      const std::string& obstacle_id, const double s_dist,
      const double t_density) const {
      std::vector<STPoint> pt_pairs;
        if (path_time_obstacle_map_.find(obstacle_id) ==
            path_time_obstacle_map_.end()) {
            return pt_pairs;
        }

        const auto& pt_obstacle = path_time_obstacle_map_.at(obstacle_id);

        double s0 = 0.0;
        double s1 = 0.0;

        double t0 = 0.0;
        double t1 = 0.0;
        if (s_dist > 0.0) {
            s0 = pt_obstacle.upper_left_point().s();
            s1 = pt_obstacle.upper_right_point().s();

            t0 = pt_obstacle.upper_left_point().t();
            t1 = pt_obstacle.upper_right_point().t();
        } else {
            s0 = pt_obstacle.bottom_left_point().s();
            s1 = pt_obstacle.bottom_right_point().s();

            t0 = pt_obstacle.bottom_left_point().t();
            t1 = pt_obstacle.bottom_right_point().t();
        }

        double time_gap = t1 - t0;
        //   CHECK(time_gap > -FLAGS_numerical_epsilon);
        time_gap = std::fabs(time_gap);

        size_t num_sections = static_cast<size_t>(time_gap / t_density + 1);
        double t_interval = time_gap / static_cast<double>(num_sections);

        for (size_t i = 0; i <= num_sections; ++i) {
            double t = t_interval * static_cast<double>(i) + t0;
            //double s = lerp(s0, t0, s1, t1, t) + s_dist;
            double s = (s1-s0)*(t-t0)/(t1-t0) + s0 + s_dist;

            STPoint ptt;
            ptt.set_t(t);
            ptt.set_s(s);
            pt_pairs.push_back(std::move(ptt));
        }

        return pt_pairs;

    }

  bool IsObstacleInGraph(const std::string& obstacle_id)
  {
    return path_time_obstacle_map_.find(obstacle_id) !=
        path_time_obstacle_map_.end();
  }

  std::vector<std::pair<double, double>> GetLateralBounds(
      const double s_start, const double s_end, const double s_resolution)
    {
        std::vector<std::pair<double, double>> bounds;
        std::vector<double> discretized_path;
        double s_range = s_end - s_start;
        double s_curr = s_start;
        size_t num_bound = static_cast<size_t>(s_range / s_resolution);

        // const auto& vehicle_config =
        //     common::VehicleConfigHelper::Instance()->GetConfig();
        apollo::common::VehicleConfig vehicle_config; //重新写的
        double ego_width = vehicle_config.vehicle_param.width;

        // Initialize bounds by reference line width
        for (size_t i = 0; i < num_bound; ++i) {
            double left_width = FLAGS_default_reference_line_width / 2.0;
            double right_width = FLAGS_default_reference_line_width / 2.0;
            ptr_reference_line_info_->reference_line().GetLaneWidth(s_curr, &left_width,
                                                                    &right_width);
            double ego_d_lower = init_d_[0] - ego_width / 2.0;
            double ego_d_upper = init_d_[0] + ego_width / 2.0;
            bounds.emplace_back(
                std::min(-right_width, ego_d_lower - FLAGS_bound_buffer),
                std::max(left_width, ego_d_upper + FLAGS_bound_buffer));
            discretized_path.push_back(s_curr);
            s_curr += s_resolution;
        }
        /* 引入障碍物的时候需要激活
        for (const SLBoundary& static_sl_boundary : static_obs_sl_boundaries_) {
            UpdateLateralBoundsByObstacle(static_sl_boundary, discretized_path, s_start,
                                        s_end, &bounds);
        }
        */

        for (size_t i = 0; i < bounds.size(); ++i) {
            bounds[i].first += ego_width / 2.0;
            bounds[i].second -= ego_width / 2.0;
            if (bounds[i].first >= bounds[i].second) {
            bounds[i].first = 0.0;
            bounds[i].second = 0.0;
            }
        }
        return bounds;

    }

 private:
  // void SetupObstacles(
  //     const std::vector<const Obstacle*>& obstacles,
  //     const std::vector<PathPoint>& discretized_ref_points)
  //   {
  //       for (const Obstacle* obstacle : obstacles) {
  //           // if (obstacle->IsVirtual()) {
  //           // continue;
  //           // }
  //           // if (!obstacle->HasTrajectory()) 
  //           // {
  //           // SetStaticObstacle(obstacle, discretized_ref_points);
  //           // } 
  //           // else {
  //           // SetDynamicObstacle(obstacle, discretized_ref_points);
  //           // }
  //           continue;
  //       }

  //   }

  void SetupObstacles(
    const std::vector<const Obstacle*>& obstacles,
    const std::vector<PathPoint>& discretized_ref_points) 
  {
    for (const Obstacle* obstacle : obstacles) {
      if (obstacle->IsVirtual()) {
        continue;
      }
      // if (!obstacle->HasTrajectory()) {
      //   SetStaticObstacle(obstacle, discretized_ref_points);
      // } 
      // else 
      // {
      //   SetDynamicObstacle(obstacle, discretized_ref_points);
      // }
      SetStaticObstacle(obstacle, discretized_ref_points);
    }

    std::sort(static_obs_sl_boundaries_.begin(), static_obs_sl_boundaries_.end(),
              [](const SLBoundary& sl0, const SLBoundary& sl1) {
                return sl0.start_s < sl1.start_s;
              });

    for (auto& path_time_obstacle : path_time_obstacle_map_) {
      path_time_obstacles_.push_back(path_time_obstacle.second);
    }
  }

  SLBoundary ComputeObstacleBoundary(
      const std::vector<common::math::Vec2d>& vertices,
      const std::vector<PathPoint>& discretized_ref_points) const
  {
    double start_s(std::numeric_limits<double>::max());
    double end_s(std::numeric_limits<double>::lowest());
    double start_l(std::numeric_limits<double>::max());
    double end_l(std::numeric_limits<double>::lowest());

    for (const auto& point : vertices) {
      auto sl_point = PathMatcher::GetPathFrenetCoordinate(discretized_ref_points,
                                                          point.x(), point.y());
      start_s = std::fmin(start_s, sl_point.first);
      end_s = std::fmax(end_s, sl_point.first);
      start_l = std::fmin(start_l, sl_point.second);
      end_l = std::fmax(end_l, sl_point.second);
    }

    SLBoundary sl_boundary;
    sl_boundary.set_start_s(start_s);
    sl_boundary.set_end_s(end_s);
    sl_boundary.set_start_l(start_l);
    sl_boundary.set_end_l(end_l);

    return sl_boundary;  
  }

  STPoint SetPathTimePoint(const std::string& obstacle_id, const double s,
                           const double t) const
  {
    STPoint path_time_point(s, t);
    return path_time_point;
  }

  void SetStaticObstacle(
      const Obstacle* obstacle,
      const std::vector<PathPoint>& discretized_ref_points)
  {
      const Polygon2d& polygon = obstacle->PerceptionPolygon();

  std::string obstacle_id = obstacle->Id();
  SLBoundary sl_boundary = //障碍物的边界以及边界内部的点
      ComputeObstacleBoundary(polygon.GetAllVertices(), discretized_ref_points);

  double left_width = FLAGS_default_reference_line_width * 0.5;
  double right_width = FLAGS_default_reference_line_width * 0.5;
  ptr_reference_line_info_->reference_line().GetLaneWidth(
      sl_boundary.start_s, &left_width, &right_width);
  if (sl_boundary.start_s > path_range_.second ||
      sl_boundary.end_s < path_range_.first ||
      sl_boundary.start_l > left_width ||
      sl_boundary.end_l < -right_width) {
    // ADEBUG << "Obstacle [" << obstacle_id << "] is out of range.";
    return;
  }

  path_time_obstacle_map_[obstacle_id].set_id(obstacle_id);
  path_time_obstacle_map_[obstacle_id].set_bottom_left_point(
      SetPathTimePoint(obstacle_id, sl_boundary.start_s, 0.0));
  path_time_obstacle_map_[obstacle_id].set_bottom_right_point(SetPathTimePoint(
      obstacle_id, sl_boundary.start_s, FLAGS_trajectory_time_length));
  path_time_obstacle_map_[obstacle_id].set_upper_left_point(
      SetPathTimePoint(obstacle_id, sl_boundary.end_s, 0.0));
  path_time_obstacle_map_[obstacle_id].set_upper_right_point(SetPathTimePoint(
      obstacle_id, sl_boundary.end_s, FLAGS_trajectory_time_length));
  static_obs_sl_boundaries_.push_back(std::move(sl_boundary));
  // ADEBUG << "ST-Graph mapping static obstacle: " << obstacle_id
  //        << ", start_s : " << sl_boundary.start_s()
  //        << ", end_s : " << sl_boundary.end_s()
  //        << ", start_l : " << sl_boundary.start_l()
  //        << ", end_l : " << sl_boundary.end_l();
  }

  void SetDynamicObstacle(
      const Obstacle* obstacle,
      const std::vector<PathPoint>& discretized_ref_points);

//   void UpdateLateralBoundsByObstacle(
//       const SLBoundary& sl_boundary,
//       const std::vector<double>& discretized_path, const double s_start,
//       const double s_end, std::vector<std::pair<double, double>>* const bounds);

 private:
  std::pair<double, double> time_range_;
  std::pair<double, double> path_range_;
  ReferenceLineInfo* ptr_reference_line_info_;
  std::array<double, 3> init_d_;

  std::unordered_map<std::string, STBoundary> path_time_obstacle_map_;//有机会看看
  std::vector<STBoundary> path_time_obstacles_;
  std::vector<SLBoundary> static_obs_sl_boundaries_;
};

}  // namespace planning
}  // namespace apolloh
