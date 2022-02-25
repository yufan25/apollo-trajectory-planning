
#pragma once

#include<iostream>
using namespace std;

struct StopPoint {
   double s = -10;
  // enum Type {
  //   HARD = 0;
  //   SOFT = 1;
  // }
  //  Type type = 2 [default = HARD];
};

struct PlanningTarget {
   StopPoint stop_point;
   double cruise_speed = 5;

   void set_cruise_speed(double inp) {cruise_speed = inp;}
};

struct tryy {
  double ini=1;
};
