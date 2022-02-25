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

#include <array>
#include <cmath>

#include "math_utils.h"

//#include "modules/common/math/vec2d.h"

namespace apollo {
namespace common {
namespace math {

// Notations:
// s_condition = [s, s_dot, s_ddot]
// s: longitudinal coordinate w.r.t reference line.
// s_dot: ds / dt
// s_ddot: d(s_dot) / dt
// d_condition = [d, d_prime, d_pprime]
// d: lateral coordinate w.r.t. reference line
// d_prime: dd / ds
// d_pprime: d(d_prime) / ds
// l: the same as d.
class CartesianFrenetConverter {
 public:
  CartesianFrenetConverter() = delete;
  /**
   * Convert a vehicle state in Cartesian frame to Frenet frame.
   * Decouple a 2d movement to two independent 1d movement w.r.t. reference
   * line.
   * The lateral movement is a function of longitudinal accumulated distance s
   * to achieve better satisfaction of nonholonomic constraints.
   */
  static void cartesian_to_frenet(const double rs, const double rx,
                                  const double ry, const double rtheta,
                                  const double rkappa, const double rdkappa,
                                  const double x, const double y,
                                  const double v, const double a,
                                  const double theta, const double kappa,
                                  std::array<double, 3>* const ptr_s_condition,
                                  std::array<double, 3>* const ptr_d_condition)
    {
        const double dx = x - rx;
        const double dy = y - ry;

        const double cos_theta_r = std::cos(rtheta);
        const double sin_theta_r = std::sin(rtheta);

        const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
        ptr_d_condition->at(0) =
            std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);

        const double delta_theta = theta - rtheta;
        const double tan_delta_theta = std::tan(delta_theta);
        const double cos_delta_theta = std::cos(delta_theta);

        const double one_minus_kappa_r_d = 1 - rkappa * ptr_d_condition->at(0);
        ptr_d_condition->at(1) = one_minus_kappa_r_d * tan_delta_theta;

        const double kappa_r_d_prime =
            rdkappa * ptr_d_condition->at(0) + rkappa * ptr_d_condition->at(1);

        ptr_d_condition->at(2) =
            -kappa_r_d_prime * tan_delta_theta +
            one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
                (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa);

        ptr_s_condition->at(0) = rs;

        ptr_s_condition->at(1) = v * cos_delta_theta / one_minus_kappa_r_d;

        const double delta_theta_prime =
            one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa;
        ptr_s_condition->at(2) =
            (a * cos_delta_theta -
            ptr_s_condition->at(1) * ptr_s_condition->at(1) *
                (ptr_d_condition->at(1) * delta_theta_prime - kappa_r_d_prime)) /
            one_minus_kappa_r_d;
        return;

    }

  static void cartesian_to_frenet(const double rs, const double rx,
                                  const double ry, const double rtheta,
                                  const double x, const double y, double* ptr_s,
                                  double* ptr_d);

  /**
   * Convert a vehicle state in Frenet frame to Cartesian frame.
   * Combine two independent 1d movement w.r.t. reference line to a 2d movement.
   */
  static void frenet_to_cartesian(const double rs, const double rx,
                                  const double ry, const double rtheta,
                                  const double rkappa, const double rdkappa,
                                  const std::array<double, 3>& s_condition,
                                  const std::array<double, 3>& d_condition,
                                  double* const ptr_x, double* const ptr_y,
                                  double* const ptr_theta,
                                  double* const ptr_kappa, double* const ptr_v,
                                  double* const ptr_a)
{
    // if (std::abs(rs - s_condition[0]) < 1.0e-6)
    //    std::cout << "The reference point s and s_condition[0] don't match"<<endl;

    const double cos_theta_r = std::cos(rtheta);
    const double sin_theta_r = std::sin(rtheta);

    *ptr_x = rx - sin_theta_r * d_condition[0];
    *ptr_y = ry + cos_theta_r * d_condition[0];

    const double one_minus_kappa_r_d = 1 - rkappa * d_condition[0];

    const double tan_delta_theta = d_condition[1] / one_minus_kappa_r_d;
    const double delta_theta = std::atan2(d_condition[1], one_minus_kappa_r_d);
    const double cos_delta_theta = std::cos(delta_theta);

    *ptr_theta = NormalizeAngle(delta_theta + rtheta);

    const double kappa_r_d_prime =
        rdkappa * d_condition[0] + rkappa * d_condition[1];
    *ptr_kappa = (((d_condition[2] + kappa_r_d_prime * tan_delta_theta) *
                    cos_delta_theta * cos_delta_theta) /
                        (one_minus_kappa_r_d) +
                    rkappa) *
                cos_delta_theta / (one_minus_kappa_r_d);

    const double d_dot = d_condition[1] * s_condition[1];
    *ptr_v = std::sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d *
                            s_condition[1] * s_condition[1] +
                        d_dot * d_dot);

    const double delta_theta_prime =
        one_minus_kappa_r_d / cos_delta_theta * (*ptr_kappa) - rkappa;

    *ptr_a = s_condition[2] * one_minus_kappa_r_d / cos_delta_theta +
            s_condition[1] * s_condition[1] / cos_delta_theta *
                (d_condition[1] * delta_theta_prime - kappa_r_d_prime);
}

    // given sl point extract x, y, theta, kappa
    static double CalculateTheta(const double rtheta, const double rkappa,
                                const double l, const double dl);

    static double CalculateKappa(const double rkappa, const double rdkappa,
                                const double l, const double dl,
                                const double ddl);


//   static Vec2d CalculateCartesianPoint(const double rtheta, const Vec2d& rpoint,
//                                        const double l);
  /**
   * @brief: given sl, theta, and road's theta, kappa, extract derivative l,
   *second order derivative l:
   */
  static double CalculateLateralDerivative(const double theta_ref,
                                           const double theta, const double l,
                                           const double kappa_ref);

  // given sl, theta, and road's theta, kappa, extract second order derivative
  static double CalculateSecondOrderLateralDerivative(
      const double theta_ref, const double theta, const double kappa_ref,
      const double kappa, const double dkappa_ref, const double l);
};

}  // namespace math
}  // namespace common
}  // namespace apollo
