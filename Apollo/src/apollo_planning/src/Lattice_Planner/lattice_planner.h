#pragma once

#include"include/pnc_point.pb.h"
#include"include/frame.h"
#include"include/reference_line_info.h"

namespace apollo
{
namespace planning
{

class LatticePlanner {
    public:
    LatticePlanner() = default;
    virtual ~LatticePlanner() = default;

    void PlanOnReferenceLine(
      const TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info);
    
    
/*
    void PlanOnReferenceLine(
      const TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) override;
      */
};

std::vector<PathPoint> ToDiscretizedReferenceLine(
    const std::vector<ReferencePoint>& ref_points);

void ComputeInitFrenetState(const PathPoint& matched_point,
                            const TrajectoryPoint& cartesian_state,
                            std::array<double, 3>* ptr_s,
                            std::array<double, 3>* ptr_d);


}   //namespace planning
}   //namespace Apollo