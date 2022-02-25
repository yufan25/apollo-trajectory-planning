#include <ctime>
// #include <windows.h>
#include <cmath>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


#include"lattice_planner.h"
#include"include/pnc_point.pb.h"
#include"include/frame.h"
#include"include/reference_line_info.h"

using namespace std;
using namespace apollo::planning;

void ini(TrajectoryPoint& planning_start_point, Frame* frame,
        ReferenceLineInfo* reference_line_info)
{
    //initialise start point
    planning_start_point.v = 5;
    planning_start_point.a = 0;
    planning_start_point.path_point.x = 1;
    planning_start_point.path_point.y = -32.5;
    planning_start_point.path_point.theta = 0;
    planning_start_point.path_point.kappa = 0;


    //initialise reference_line_info
    std::vector<ReferencePoint> points;


    for (int32_t i =0;i<60;i++)
    {
      ReferencePoint p;
      p.set_x (2+i*2);
      p.set_y (-32);
      p.set_heading (0.0);
      p.set_kappa (0.0);
      p.set_dkappa (0.0);

      points.push_back(p);
    }

    reference_line_info->reference_line().set_reference_points(points);

}

int main( int argc, char** argv ){

  ros::init(argc, argv, "lattice_planner");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("Obj_Pub", 10);
  ros::Rate r(10);


  visualization_msgs::Marker car, line_plan;
  car.header.frame_id =line_plan.header.frame_id =   "/my_frame";
  car.header.stamp = line_plan.header.stamp  = ros::Time::now();
  car.ns =line_plan.ns = "PNC_car_line";
  car.action =  line_plan.action = visualization_msgs::Marker::ADD;
  // car.pose.orientation.w = 1.0;
  // car.pose.position.x= 50;
  // car.pose.position.y= -32;
  // car.pose.position.z=0.9;

  car.id = 1;
  line_plan.id =3;

  car.type = visualization_msgs::Marker::CUBE;
  line_plan.type = visualization_msgs::Marker::POINTS;

  car.scale.x = 2.5;
  car.scale.y = 1.2;
  car.scale.z= 0.8;

  line_plan.scale.x = 0.5;
   line_plan.scale.y= 0.5;

  car.color.g= 1.0;
  car.color.a = 1.0;  
  line_plan.color.r= 1.0;
  line_plan.color.a = 1.0;
  

  TrajectoryPoint planning_start_point;
  
  Frame frame_;
  Frame* frame = &frame_;

  ReferenceLineInfo reference_line_info_;
  ReferenceLineInfo* reference_line_info = &reference_line_info_;

  LatticePlanner lattice_planner;

  ini(planning_start_point, frame, reference_line_info);

  string id = "erster";
  struct apollo::perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.id = 1;
  perception_obstacle.position.x = 45;
  perception_obstacle.position.y = -32;
  perception_obstacle.theta = 0;
  perception_obstacle.length = 2;
  perception_obstacle.width = 1;

  perception_obstacle.polygon_point.push_back({46,-31.5,0});
  perception_obstacle.polygon_point.push_back({46,-32.5,0});
  perception_obstacle.polygon_point.push_back({44,-31.5,0});
  perception_obstacle.polygon_point.push_back({44,-32.5,0});

  // perception_obstacle.polygon_point.push_back({49,-31,2});
  // perception_obstacle.polygon_point.push_back({49,-33,2});

  bool is_static = true;

  Obstacle obstacle(id, perception_obstacle, is_static);

  frame->add_obstacle(&obstacle);

  while(ros::ok()&&car.pose.position.x<110)
  {
    //lattice planner
    lattice_planner.PlanOnReferenceLine(planning_start_point, frame, reference_line_info);

    car.pose.orientation.x = cos(reference_line_info->discretized_trajectory()[0].path_point.theta );
    car.pose.orientation.y = sin(reference_line_info->discretized_trajectory()[0].path_point.theta );
    // car.pose.orientation.w = cos(reference_line_info->discretized_trajectory()[0].path_point.theta * M_PI/180);

    car.pose.position.x= reference_line_info->discretized_trajectory()[0].path_point.x;
    car.pose.position.y= reference_line_info->discretized_trajectory()[0].path_point.y ;
    car.pose.orientation.w =0;
    car.pose.orientation.z=0;
    // car.pose.orientation.y =0;
    // car.pose.orientation.x =0;

    // car.pose.position.x = 5.2;
    // car.pose.position.y = -30;

    planning_start_point.v = reference_line_info->discretized_trajectory()[1].v;
    planning_start_point.a = reference_line_info->discretized_trajectory()[1].a;
    planning_start_point.path_point.x = reference_line_info->discretized_trajectory()[1].path_point.x;
    planning_start_point.path_point.y = reference_line_info->discretized_trajectory()[1].path_point.y;
    planning_start_point.path_point.theta = reference_line_info->discretized_trajectory()[1].path_point.theta;
    planning_start_point.path_point.kappa = reference_line_info->discretized_trajectory()[1].path_point.kappa;


    for(auto iter : reference_line_info->discretized_trajectory())
    {
      geometry_msgs::Point p; 
      p.x = iter.path_point.x;
      p.y = iter.path_point.y;
      p.z = 0;

      line_plan.points.push_back(p);
    }

    marker_pub.publish(car);
    marker_pub.publish(line_plan);
    r.sleep();

    line_plan.points.resize(0);


  }


  return 0;


}