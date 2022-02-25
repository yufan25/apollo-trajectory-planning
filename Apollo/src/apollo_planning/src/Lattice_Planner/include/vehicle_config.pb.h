#pragma once

#include <iostream>
#include <vector>

namespace apollo
{
namespace common
{

using namespace std;


struct Point3D {
    double x = 1;
    double y = 2;
    double z = 3;
};

struct Quaternion {
    double qx = 1;
    double qy = 2;
    double qz = 3;
    double qw = 4;
};

struct Transform {
    uint8_t source_frame = 1;  // Also known as "frame_id."

    uint8_t target_frame = 2;  // Also known as "child_frame_id."

  // Position of target in the source frame.
    Point3D translation;

  // Activate rotation from the source frame to the target frame.
    Quaternion rotation;
};

struct Extrinsics {
  // repeated Transform tansforms = 1;
  std::vector<Transform> tansforms;
};

// Vehicle parameters shared among several modules.
// By default, all are measured with the SI units (meters, meters per second,
// etc.).

// struct VehicleBrand {
//   LINCOLN_MKZ = 0;
//   GEM = 1;
//   LEXUS = 2;
//   TRANSIT = 3;
//   GE3 = 4;
//   WEY = 5;
//   ZHONGYUN = 6;
//   CH = 7;
// };

struct VehicleID {
    string vin;
    string plate;
    string other_unique_id;
};

struct VehicleParam {
    uint8_t brand = 1;
  // Car center point is car reference point, i.e., center of rear axle.
    VehicleID vehicle_id;
    double front_edge_to_center = 3.89;
    double back_edge_to_center = 1.043;
    double left_edge_to_center = 1.055;
    double right_edge_to_center = 1.055;

    double length = 4.933;
    double width = 2.11;
    double height = 1.48;

    double min_turn_radius = 5.05386147161;
    double max_acceleration = 2.0;
    double max_deceleration = -6.0;

  // The following items are used to compute trajectory constraints in
  // planning/control/canbus,
  // vehicle max steer angle
    double max_steer_angle = 8.20304748437;
  // vehicle max steer rate; how fast can the steering wheel turn.
    double max_steer_angle_rate = 8.55211;
  // vehicle min steer rate;
    double min_steer_angle_rate = 0;
  // ratio between the turn of steering wheel and the turn of wheels
    double steer_ratio = 16;
  // the distance between the front and back wheels
    double wheel_base = 2.8448;
  // Tire effective rolling radius (vertical distance between the wheel center
  // and the ground).
    double wheel_rolling_radius = 0.335;

  // minimum differentiable vehicle speed, in m/s
    float max_abs_speed_when_stopped = 0.2;

  // minimum value get from chassis.brake, in percentage
    double brake_deadzone = 15.5;
  // minimum value get from chassis.throttle, in percentage
    double throttle_deadzone = 18.0;
};

struct Header {
  // Message publishing time in seconds. It is recommended to obtain
  // timestamp_sec from ros::Time::now(), right before calling
  // SerializeToString() and publish().
    double timestamp_sec = 1;

  // Module name.
    string module_name;

  // Sequence number for each message. Each module maintains its own counter for
  // sequence_num, always starting from 1 on boot.
    uint32_t sequence_num = 3;

  // Lidar Sensor timestamp for nano-second.
    uint64_t lidar_timestamp = 4;

  // Camera Sensor timestamp for nano-second.
    uint64_t camera_timestamp = 5;

  // Radar Sensor timestamp for nano-second.
    uint64_t radar_timestamp = 6;

  // data version
    uint32_t version = 7;

    // StatusPb status = 8;

    string frame_id;
};

struct VehicleConfig {

  Header header;
  VehicleParam vehicle_param ;
  Extrinsics extrinsics;
};



}  //namespace common
}  //namespace apollo