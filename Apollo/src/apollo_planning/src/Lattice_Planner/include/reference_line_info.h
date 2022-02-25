#pragma once

#include <limits>
#include <list>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "pnc_point.pb.h"
#include "reference_line.h"
#include "lattice_structure.pb.h"
#include "discretized_trajectory.h"


// #include "modules/common/proto/drive_state.pb.h"
// #include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
// #include "modules/planning/proto/lattice_structure.pb.h"
// #include "modules/planning/proto/planning.pb.h"

// #include "modules/map/hdmap/hdmap_common.h"
// #include "modules/map/pnc_map/pnc_map.h"
// #include "modules/planning/common/path/path_data.h"
// #include "modules/planning/common/path_boundary.h"
// #include "modules/planning/common/path_decision.h"
// #include "modules/planning/common/speed/speed_data.h"
// #include "modules/planning/common/st_graph_data.h"
// #include "modules/planning/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {

/**
 * @class ReferenceLineInfo
 * @brief ReferenceLineInfo holds all data for one reference line.
 */
class ReferenceLineInfo {
 public:
  enum class LaneType { LeftForward, LeftReverse, RightForward, RightReverse };
  ReferenceLineInfo() = default;

  const PlanningTarget& planning_target() const { return planning_target_; }

  //const ReferenceLine& reference_line() const {return reference_line_;}
  ReferenceLine& reference_line() {return reference_line_;}

  DiscretizedTrajectory& discretized_trajectory () {return discretized_trajectory_;}

  void set_is_on_reference_line() { is_on_reference_line_ = true; }

  void SetCruiseSpeed(double speed)
  {
      planning_target_.set_cruise_speed(speed);
    }

  void SetTrajectory(const DiscretizedTrajectory& trajectory){discretized_trajectory_ = trajectory;}

  void SetTry(double input){try_y.ini=input;};

  double ShowTry(){return try_y.ini;}

  void SetDrivable(bool drivable) { is_drivable_ = drivable; }

  double PriorityCost() const{ return priority_cost_;}

  void SetCost(double cost) { cost_ = cost; }

 private:

  PlanningTarget planning_target_;
  bool is_drivable_ = true;
  bool is_on_reference_line_ = false;
  ReferenceLine reference_line_;
  DiscretizedTrajectory discretized_trajectory_;

  double priority_cost_ = 0.0;
  double cost_ = 0.0;

 tryy try_y;


};

}  // namespace planning
}  // namespace apollo
