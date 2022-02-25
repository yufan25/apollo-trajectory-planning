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
 * @brief Linear interpolation functions.
 */

#pragma once



#include <cmath>

//#include "cyber/common/log.h"
#include "pnc_point.pb.h"
#include "math_utils.h"

/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace apollo {
namespace common {
namespace math {

/**
 * @brief Linear interpolation between two points of type T.
 * @param x0 The coordinate of the first point.
 * @param t0 The interpolation parameter of the first point.
 * @param x1 The coordinate of the second point.
 * @param t1 The interpolation parameter of the second point.
 * @param t The interpolation parameter for interpolation.
 * @param x The coordinate of the interpolated point.
 * @return Interpolated point.
 */
// template <typename T>
// T lerp(const T &x0, const double t0, const T &x1, const double t1,
//        const double t) {
//   if (std::abs(t1 - t0) <= 1.0e-6) {
//     AERROR << "input time difference is too small";
//     return x0;
//   }
//   const double r = (t - t0) / (t1 - t0);
//   const T x = x0 + r * (x1 - x0);
//   return x;
// }

/**
 * @brief Spherical linear interpolation between two angles.
 *        The two angles are within range [-M_PI, M_PI).
 * @param a0 The value of the first angle.
 * @param t0 The interpolation parameter of the first angle.
 * @param a1 The value of the second angle.
 * @param t1 The interpolation parameter of the second angle.
 * @param t The interpolation parameter for interpolation.
 * @param a The value of the spherically interpolated angle.
 * @return Interpolated angle.
 */
// double slerp(const double a0, const double t0, const double a1, const double t1,
//              const double t);

// SLPoint InterpolateUsingLinearApproximation(const SLPoint &p0,
//                                             const SLPoint &p1, const double w);
/*
PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
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
*/
// TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &tp0,
//                                                     const TrajectoryPoint &tp1,
//                                                     const double t);
/*

double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t)
{
    if (std::abs(t1 - t0) <= kMathEpsilon) {
    // ADEBUG << "input time difference is too small";
    return NormalizeAngle(a0);
  }
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return NormalizeAngle(a);
}


//lerp wrote by yufan
double lerp(const double a0, const double t0, const double a1, const double t1,
             const double t)
{
  double d = a1-a0;
  double r= (t-t0) / (t1-t0);
  return a0 + r*d;
}*/

}  // namespace math
}  // namespace common
}  // namespace apollo

// #endif
