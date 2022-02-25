/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <utility>
#include <vector>

#include "pnc_point.pb.h"

namespace apollo {
namespace common {
namespace math {

class PathMatcher {
 public:
  PathMatcher() = delete;

  static PathPoint MatchToPath(const std::vector<PathPoint>& reference_line,
                               const double x, const double y);

  static std::pair<double, double> GetPathFrenetCoordinate(
      const std::vector<PathPoint>& reference_line, const double x,
      const double y);

  static PathPoint MatchToPath(const std::vector<PathPoint>& reference_line,
                               const double s);

    static PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const double s) {

  double s0 = p0.s;
  double s1 = p1.s;

  PathPoint path_point;
  double weight = (s - s0) / (s1 - s0);
  double x = (1 - weight) * p0.x + weight * p1.x;
  double y = (1 - weight) * p0.y + weight * p1.y;
  //double theta = slerp(p0.theta, p0.s, p1.theta, p1.s, s);
  double theta = ((p1.theta-p0.theta)*(s-s0)/(s1-s0)+p0.theta);
  double kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
  double dkappa = (1 - weight) * p0.dkappa + weight * p1.dkappa;
  double ddkappa = (1 - weight) * p0.ddkappa + weight * p1.ddkappa;
  path_point.set_x(x);
  path_point.set_y(y);
  path_point.set_theta(theta);
  path_point.set_kappa(kappa);
  path_point.set_dkappa(dkappa);
  path_point.set_ddkappa(ddkappa);
  path_point.set_s(s);
  return path_point;
  }

 private:
  static PathPoint FindProjectionPoint(const PathPoint& p0, const PathPoint& p1,
                                       const double x, const double y);
};

}  // namespace math
}  // namespace common
}  // namespace apollo
