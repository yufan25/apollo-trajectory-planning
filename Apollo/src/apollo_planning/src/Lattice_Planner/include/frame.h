#pragma once

#include<iostream>
#include<vector>

#include"obstacle.h"
using namespace std;

namespace apollo {
namespace planning {

class Frame {
  public:


  //const std::vector<const Obstacle *> obstacles() const;
  const std::vector<const Obstacle*> obstacles() const {return obstacles_;}

  const TrajectoryPoint &PlanningStartPoint() const;

//   const std::vector<const Obstacle *> obstacles() const {
//   return obstacles_.Items();
// }

  void add_obstacle( Obstacle* obstacle)
  {
    // obstacles_p.push_back(obstacle);
    obstacles_.push_back(obstacle);
  }


  private:

  std::vector<const Obstacle*> obstacles_;
  // std::vector<const Obstacle> obstacles_p;
  
};

}
}