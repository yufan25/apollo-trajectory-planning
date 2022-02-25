#include<iostream>
#include <ctime>
#include <math.h>
#include "lattice_planner.h"
#include "include/path_matcher.h"
#include "include/cartesian_frenet_conversion.h"
#include "include/path_time_graph.h"
#include "include/prediction_querier.h"
#include "include/trajectory1d_generator.h"
#include "include/trajectory_evaluator.h"
#include "include/trajectory_combiner.h"
#include "include/collision_checker.h"
#include "include/constraint_checker.h"



using namespace std;
using namespace apollo;
using namespace apollo::common::math;
// = using apollo::common::math::PathMatcher;

namespace apollo
{
namespace planning
{

//LatticePlanner::LatticePlanner(){};

void LatticePlanner::PlanOnReferenceLine(
        const TrajectoryPoint& planning_init_point, Frame* frame,
        ReferenceLineInfo* reference_line_info)
{
    static size_t num_planning_cycles = 0;
    static size_t num_planning_succeeded_cycles = 0;

    clock_t start_time = clock();
    clock_t current_time = start_time;

    ++num_planning_cycles;

    reference_line_info->set_is_on_reference_line();

    //-------------------------------------------------------------------
    //1. obtain a reference line and transform it to the PathPoint format.
    //vector<PathPoint> ptr = ToDiscretizedReferenceLine(
    //    reference_line_info->reference_line().reference_points());

    //auto ptr_reference_line = std::make_shared<std::vector<PathPoint>>(ptr);
    auto ptr_reference_line = std::make_shared<std::vector<PathPoint>>(ToDiscretizedReferenceLine(
        reference_line_info->reference_line().reference_points()));
    //测试指针转成实体
     vector<PathPoint> ptr = *ptr_reference_line;
    // double tt = ptr[120].s;
    // cout<<"haha"<<endl<<tt<<endl<<"hgh"<<endl;
    // cout<<"1--参考线配置成功，一共"<<ptr.size()<<"个点，距离为"<<ptr.back().s<<"米"<<endl;

    //-----------------------------------------------------------------
    //2. compute the matched point of the init planning point on the reference
    // line.
    PathPoint matched_point = PathMatcher::MatchToPath(  //PathMatcher类的MatchToPath函数必须是静态函数
      *ptr_reference_line, planning_init_point.path_point.x,
      planning_init_point.path_point.y);

    //cout<<matched_point.x<<" "<<matched_point.y<<endl;
    // cout<<"2--规划起始点在参考线中完成对应，对应点为: ("<<matched_point.x<<", "<<matched_point.y<<")"<<endl;

    //-----------------------------------------------------------------
    // 3. according to the matched point, compute the init state in Frenet frame.
    std::array<double, 3> init_s;
    std::array<double, 3> init_d;
    ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);

    // ADEBUG << "ReferenceLine and Frenet Conversion Time = "
    //      << (Clock::NowInSeconds() - current_time) * 1000;
    current_time = clock();
    // cout<<"3--SL坐标下起始点数据换算完成。"<<endl<<
    //     "   S方向数据: ("<<init_s[0]<<", "<<init_s[1]<<", "<<init_s[2]<<")"<<endl<<
    //     "   L方向数据: ("<<init_d[0]<<", "<<init_d[1]<<", "<<init_d[2]<<")"<<endl;
    
    //-----------------------------------------------------------------
    // 4. parse the decision and get the planning target.

    //4.1将frame中障碍物的信息传递给ptr_prediction_querier
    auto ptr_prediction_querier = std::make_shared<PredictionQuerier>(
    frame->obstacles(), ptr_reference_line);

    // double FLAGS_speed_lon_decision_horizon = init_s[1]*5; //规划的路径长度
    // double FLAGS_trajectory_time_length = 5;               //规划的时间长度

    //4.2 将障碍物信息，参考线，规划起始点，目标规划的路径以及时间长度传递给ptr_path_time_graph
    auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(
        ptr_prediction_querier->GetObstacles(), *ptr_reference_line,
        reference_line_info, init_s[0],
        init_s[0] + FLAGS_speed_lon_decision_horizon, 0.0,
        FLAGS_trajectory_time_length, init_d);

    // double speed_limit =
    //     reference_line_info->reference_line().GetSpeedLimitFromS(init_s[0]);

    //4.3设置目标车速
    double speed_limit = 5; //限速？？
    reference_line_info->SetCruiseSpeed(speed_limit);

    //4.4将规划的目标传递给planning_target，包括目标车速，停车指示牌
    PlanningTarget planning_target = reference_line_info->planning_target();

    // cout<<"4--规划参数设定完毕。规划路径长度以及规划时间长度为:"<<FLAGS_speed_lon_decision_horizon
    //   <<", "<<FLAGS_trajectory_time_length<<endl
    //   <<"   路径上障碍物数量为: "<<ptr_prediction_querier->GetObstacles().size()<<endl;
    // if (planning_target.stop_point.s >=0) {
    //     cout << "Planning target stop s: " << planning_target.stop_point.s
    //         << "Current ego s: " << init_s[0];
    // }
    // else
    // {
    //   cout<<"   无停车指示牌"<<endl;

    // }

    // ADEBUG << "Decision_Time = " << (Clock::NowInSeconds() - current_time) * 1000;
    current_time = clock();

    //-----------------------------------------------------------------
    // 5. generate 1d trajectory bundle for longitudinal and lateral respectively.
    Trajectory1dGenerator trajectory1d_generator(
        init_s, init_d, ptr_path_time_graph, ptr_prediction_querier);
    std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
    std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;
    trajectory1d_generator.GenerateTrajectoryBundles(
        planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle);

    // ADEBUG << "Trajectory_Generation_Time = "
    //         << (Clock::NowInSeconds() - current_time) * 1000;
    current_time = clock();

    // cout<<"5--轨迹簇生成完毕"<<endl<<"   纵向轨迹数量: "<<lon_trajectory1d_bundle.size()
    //       <<endl<<"   横向轨迹数量: "<<lat_trajectory1d_bundle.size()<<endl;
    

    //-----------------------------------------------------------------
    // 6. first, evaluate the feasibility of the 1d trajectories according to
    // dynamic constraints.
    //   second, evaluate the feasible longitudinal and lateral trajectory pairs
    //   and sort them according to the cost.
    TrajectoryEvaluator trajectory_evaluator(
        init_s, planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle,
        ptr_path_time_graph, ptr_reference_line);
    
    // std::cout<<"6--轨迹排序完成"<<endl;

    // std::cout <<"   Trajectory_Evaluator_Construction_Time = "
    //       << (clock() - current_time)<<endl;
    // current_time = clock();

    // std::cout <<"   number of trajectory pairs = "
    //       << trajectory_evaluator.num_of_trajectory_pairs()<<endl
    //       <<"   number_lon_traj = " << lon_trajectory1d_bundle.size()<<endl
    //       <<"   number_lat_traj = " << lat_trajectory1d_bundle.size()<<endl;
    
    auto fir = trajectory_evaluator.next_top_trajectory_pair(); 
    auto lon = fir.first;
    auto lat = fir.second;

    int i=0;

    // Get instance of collision checker and constraint checker
    CollisionChecker collision_checker(frame->obstacles(), init_s[0], init_d[0],
                                      *ptr_reference_line, reference_line_info,
                                      ptr_path_time_graph);

    // 7. always get the best pair of trajectories to combine; return the first
    // collision-free trajectory.
    // size_t constraint_failure_count = 0;
    size_t collision_failure_count = 0;
    size_t combined_constraint_failure_count = 0;

    size_t lon_vel_failure_count = 0;
    size_t lon_acc_failure_count = 0;
    size_t lon_jerk_failure_count = 0;
    size_t curvature_failure_count = 0;
    size_t lat_acc_failure_count = 0;
    size_t lat_jerk_failure_count = 0;

    size_t num_lattice_traj = 0;

    while (trajectory_evaluator.has_more_trajectory_pairs()) {
      double trajectory_pair_cost =
          trajectory_evaluator.top_trajectory_pair_cost();
      auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();

      //!!! combine two 1d trajectories to one 2d trajectory
      auto combined_trajectory = TrajectoryCombiner::Combine(
          *ptr_reference_line, *trajectory_pair.first, *trajectory_pair.second,
          planning_init_point.relative_time);

      // check longitudinal and lateral acceleration
      // considering trajectory curvatures
      auto result = ConstraintChecker::ValidTrajectory(combined_trajectory);
      if (result != ConstraintChecker::Result::VALID) {
        ++combined_constraint_failure_count;

        switch (result) {
          case ConstraintChecker::Result::LON_VELOCITY_OUT_OF_BOUND:
            lon_vel_failure_count += 1;
            break;
          case ConstraintChecker::Result::LON_ACCELERATION_OUT_OF_BOUND:
            lon_acc_failure_count += 1;
            break;
          case ConstraintChecker::Result::LON_JERK_OUT_OF_BOUND:
            lon_jerk_failure_count += 1;
            break;
          case ConstraintChecker::Result::CURVATURE_OUT_OF_BOUND:
            curvature_failure_count += 1;
            break;
          case ConstraintChecker::Result::LAT_ACCELERATION_OUT_OF_BOUND:
            lat_acc_failure_count += 1;
            break;
          case ConstraintChecker::Result::LAT_JERK_OUT_OF_BOUND:
            lat_jerk_failure_count += 1;
            break;
          case ConstraintChecker::Result::VALID:
          default:
            // Intentional empty
            break;
        }
        continue;
      }

      // check collision with other obstacles
      if (collision_checker.InCollision(combined_trajectory)) {
        ++collision_failure_count;
        continue;
      }

      // put combine trajectory into debug data
      const auto& combined_trajectory_points = combined_trajectory;
      num_lattice_traj += 1;
      reference_line_info->SetTrajectory(combined_trajectory);
      reference_line_info->SetCost(reference_line_info->PriorityCost() +
                                  trajectory_pair_cost);
      reference_line_info->SetDrivable(true);

      // Print the chosen end condition and start condition
      // ADEBUG << "Starting Lon. State: s = " << init_s[0] << " ds = " << init_s[1]
      //       << " dds = " << init_s[2];
      // cast
      auto lattice_traj_ptr =
          std::dynamic_pointer_cast<LatticeTrajectory1d>(trajectory_pair.first);
      if (!lattice_traj_ptr) {
        // ADEBUG << "Dynamically casting trajectory1d ptr. failed.";
      }

      if (lattice_traj_ptr->has_target_position()) {
        // ADEBUG << "Ending Lon. State s = " << lattice_traj_ptr->target_position()
        //       << " ds = " << lattice_traj_ptr->target_velocity()
        //       << " t = " << lattice_traj_ptr->target_time();
      }

      // ADEBUG << "InputPose";
      // ADEBUG << "XY: " << planning_init_point.ShortDebugString();
      // ADEBUG << "S: (" << init_s[0] << ", " << init_s[1] << "," << init_s[2]
      //       << ")";
      // ADEBUG << "L: (" << init_d[0] << ", " << init_d[1] << "," << init_d[2]
      //       << ")";

      // ADEBUG << "Reference_line_priority_cost = "
      //       << reference_line_info->PriorityCost();
      // ADEBUG << "Total_Trajectory_Cost = " << trajectory_pair_cost;
      // ADEBUG << "OutputTrajectory";
      for (unsigned int i = 0; i < 10; ++i) {
        // ADEBUG << combined_trajectory_points[i].ShortDebugString();
      }

      break;
      /*
      auto combined_trajectory_path =
          ptr_debug->mutable_planning_data()->add_trajectory_path();
      for (uint i = 0; i < combined_trajectory_points.size(); ++i) {
        combined_trajectory_path->add_trajectory_point()->CopyFrom(
            combined_trajectory_points[i]);
      }
      combined_trajectory_path->set_lattice_trajectory_cost(trajectory_pair_cost);
      */
    }

    // std::cout << "Trajectory_Evaluation_Time = "
    //       << double(clock() - current_time)/CLOCKS_PER_SEC<<endl;

    // std::cout << "Step CombineTrajectory Succeeded";

    // ADEBUG << "1d trajectory not valid for constraint ["
    //       << constraint_failure_count << "] times";
    // ADEBUG << "Combined trajectory not valid for ["
    //       << combined_constraint_failure_count << "] times";
    // ADEBUG << "Trajectory not valid for collision [" << collision_failure_count
    //       << "] times";
    std::cout << "Total_Lattice_Planning_Frame_Time = "
          << double(clock() - start_time)/CLOCKS_PER_SEC<<endl;
    std::cout<<reference_line_info->discretized_trajectory()[0].path_point.x<<"  "<<reference_line_info->discretized_trajectory()[0].path_point.y
    <<"  "<<reference_line_info->discretized_trajectory()[0].path_point.theta<<  endl;

    // if (num_lattice_traj > 0) {
    //   cout << "Planning succeeded";
    //   num_planning_succeeded_cycles += 1;
    //   reference_line_info->SetDrivable(true);
    //   return Status::OK();
    // } else {
    //   cout << "Planning failed";
    //   if (FLAGS_enable_backup_trajectory) {
    //     cout << "Use backup trajectory";
    //     BackupTrajectoryGenerator backup_trajectory_generator(
    //         init_s, init_d, planning_init_point.relative_time(),
    //         std::make_shared<CollisionChecker>(collision_checker),
    //         &trajectory1d_generator);
    //     DiscretizedTrajectory trajectory =
    //         backup_trajectory_generator.GenerateTrajectory(*ptr_reference_line);

    //     reference_line_info->AddCost(FLAGS_backup_trajectory_cost);
    //     reference_line_info->SetTrajectory(trajectory);
    //     reference_line_info->SetDrivable(true);
    //     return Status::OK();

    //   } else {
    //     reference_line_info->SetCost(std::numeric_limits<double>::infinity());
    //   }
    //   return Status(ErrorCode::PLANNING_ERROR, "No feasible trajectories");
    // }

}

std::vector<PathPoint> ToDiscretizedReferenceLine(
    const std::vector<ReferencePoint>& ref_points) {
  double s = 0.0;
  std::vector<PathPoint> path_points;
  for (const auto& ref_point : ref_points) {
    PathPoint path_point;
    path_point.set_x(ref_point.x());
    path_point.set_y(ref_point.y());
    path_point.set_theta(ref_point.heading());
    path_point.set_kappa(ref_point.kappa());
    path_point.set_dkappa(ref_point.dkappa());

    if (!path_points.empty()) {
      double dx = path_point.x - path_points.back().x;
      double dy = path_point.y - path_points.back().y;
      s += std::sqrt(dx * dx + dy * dy);
    }
    path_point.set_s(s);
    path_points.push_back(std::move(path_point));
  }
  return path_points;
}

void ComputeInitFrenetState(const PathPoint& matched_point,
                            const TrajectoryPoint& cartesian_state,
                            std::array<double, 3>* ptr_s,
                            std::array<double, 3>* ptr_d) {
  CartesianFrenetConverter::cartesian_to_frenet(
      matched_point.s, matched_point.x, matched_point.y,
      matched_point.theta, matched_point.kappa, matched_point.dkappa,
      cartesian_state.path_point.x, cartesian_state.path_point.y,
      cartesian_state.v, cartesian_state.a,
      cartesian_state.path_point.theta,
      cartesian_state.path_point.kappa, ptr_s, ptr_d);
}


}
}

