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
 * @file reference_point.h
 **/

#pragma once

#include <string>
#include <vector>

#include "pnc_point.pb.h"

#include "path.h"

namespace apollo {
namespace planning {

class ReferencePoint {//: public hdmap::MapPathPoint {
 public:
  ReferencePoint() = default;

  // ReferencePoint(const MapPathPoint& map_path_point, const double kappa,
  //                const double dkappa);
  ReferencePoint(const double x, const double y, const double kappaheading,
                const double kappa, const double dkappa);

  PathPoint ToPathPoint(double s) const;

  double kappa() const{ return kappa_; }
  double dkappa() const{ return dkappa_; }
  double x() const{ return x_; }
  double y() const{ return y_; }
  double heading() const{ return heading_; }

  void set_x(double inp) {x_=inp;}
  void set_y(double inp) {y_=inp;}
  void set_heading(double inp) {heading_=inp;}
  void set_kappa(double inp) {kappa_=inp;}
  void set_dkappa(double inp) {dkappa_=inp;}

  std::string DebugString() const;

  static void RemoveDuplicates(std::vector<ReferencePoint>* points);

 private:
  double x_ = 0.0;
  double y_ = 0.0;
  double heading_ = 0.0;
  double kappa_ = 0.0;
  double dkappa_ = 0.0;
};

}  // namespace planning
}  // namespace apollo
