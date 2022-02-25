

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "reference_point.h"
// #include "modules/common/proto/pnc_point.pb.h"
// #include "modules/map/proto/map_geometry.pb.h"
// #include "modules/planning/proto/sl_boundary.pb.h"
// #include "modules/routing/proto/routing.pb.h"

// #include "modules/common/math/vec2d.h"
// #include "modules/map/pnc_map/path.h"
// #include "modules/planning/reference_line/reference_point.h"

namespace apollo {
namespace planning {

class ReferenceLine {
 public:
  ReferenceLine() = default;
  explicit ReferenceLine(const ReferenceLine& reference_line) = default;
  // template <typename Iterator>
  // explicit ReferenceLine(const Iterator begin, const Iterator end)
  //     : reference_points_(begin, end),
  //       map_path_(std::move(std::vector<hdmap::MapPathPoint>(begin, end))) {}
  // explicit ReferenceLine(const std::vector<ReferencePoint>& reference_points);
  //explicit ReferenceLine(const hdmap::Path& hdmap_path);
  const std::vector<ReferencePoint>& reference_points() const {return reference_points_;}

  void set_reference_points(std::vector<ReferencePoint>& points) {reference_points_ = points;}

  double GetSpeedLimitFromS(const double s) const;

  bool GetLaneWidth(const double s, double* const lane_left_width,
                                 double* const lane_right_width) {
  // if (map_path_.path_points().empty()) {
  //   return false;
  // }

  // if (!map_path_.GetLaneWidth(s, lane_left_width, lane_right_width)) {
  //   return false;
  // }
  s_curr = s;
  left_width = *lane_left_width;
  right_width = *lane_right_width;

  return true;
}

 private:
  struct SpeedLimit {
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0;  // unit m/s

    SpeedLimit() = default;
    SpeedLimit(double _start_s, double _end_s, double _speed_limit)
        : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
  };
  /**
   * This speed limit overrides the lane speed limit
   **/
  std::vector<SpeedLimit> speed_limit_;
  std::vector<ReferencePoint> reference_points_;
  //hdmap::Path map_path_;
  double s_curr=0; //yufan增加，替代map_path
  double left_width=0; //yufan增加，替代map_path
  double right_width=0; //yufan增加，替代map_path
  uint32_t priority_ = 0;
};

}  // namespace planning
}  // namespace apollo
