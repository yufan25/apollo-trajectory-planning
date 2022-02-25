
#pragma once

#include<iostream>
#include <vector>
using namespace std;

struct PathPoint {
  // coordinates
    double x = -256;
    double y = -256;
    double z = 0;

  // direction on the x-y plane
    double theta = 0;
  // curvature on the x-y planning
    double kappa = 0;
  // accumulated distance from beginning of the path
    double s = 0;

  // derivative of kappa w.r.t s.
    double dkappa = 0;
  // derivative of derivative of kappa w.r.t s.
    double ddkappa = 0;
  // The lane ID where the path point is on
    string lane_id = "0";

  // derivative of x and y w.r.t parametric parameter t in CosThetareferenceline5
    double x_derivative = 0;
    double y_derivative = 0;

    void set_x(double inp) {x=inp;}
    void set_y(double inp) {y=inp;}
    void set_z(double inp) {z=inp;}
    void set_s(double inp) {s=inp;}
    void set_theta(double inp) {theta=inp;}
    void set_kappa(double inp) {kappa=inp;}
    void set_dkappa(double inp) {dkappa=inp;}
    void set_ddkappa(double inp) {ddkappa=inp;}
};


 struct TrajectoryPoint {
  // path point
    PathPoint path_point;
  // linear velocity
    double v = 5;  // in [m/s]
  // linear acceleration
    double a = 0;
  // relative time from beginning of the trajectory
    double relative_time = 0;
  // longitudinal jerk
    double da = 0;
  // The angle between vehicle front wheel and vehicle longitudinal axis
    double steer = 0;
    double probability = 1;

    void set_v(double inp) {v=inp;}
    void set_a(double inp) {a=inp;}
    void set_relative_time(double inp) {relative_time=inp;}

};

struct FrenetFramePoint {
   double s = 1;
   double l = 2;
   double dl = 3;
   double ddl = 4;
};

struct SLPoint {
  double s = 1;
  double l = 2;
};

struct SLBoundary {
  double start_s = 1;
  double end_s = 2;
  double start_l = 3;
  double end_l = 4;
  std::vector<SLPoint> boundary_point;

  void set_start_s(double inp) {start_s=inp;}
  void set_end_s(double inp) {end_s=inp;}
  void set_start_l(double inp) {start_l=inp;}
  void set_end_l(double inp) {end_l=inp;}
};

struct Trajectory {
  double probability = 1;

  std::vector<TrajectoryPoint> trajectory_point;
};

struct SpeedPoint {
  double s = 1;
  double t = 2;
  // speed (m/s)
  double v = 5;
  // acceleration (m/s^2)
  double a = 4;
  // jerk (m/s^3)
  double da = 5;
};
