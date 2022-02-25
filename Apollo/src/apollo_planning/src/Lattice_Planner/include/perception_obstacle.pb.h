// syntax = "proto2";
#pragma once

#include <vector>

using namespace std;
namespace apollo{
namespace perception{

// import "modules/common/proto/error_code.proto";
// import "modules/common/proto/geometry.proto";
// import "modules/common/proto/header.proto";
// import "modules/map/proto/map_lane.proto";

struct BBox2D {
   double xmin = 1;  // in pixels.
   double ymin = 2;  // in pixels.
   double xmax = 3;  // in pixels.
   double ymax = 4;  // in pixels.
};
struct Point3D {
  double x = 1;// [default = nan];
  double y = 2;// [default = nan];
  double z = 3;// [default = nan];
};

// message LightStatus {
//    double brake_visible = 1;
//    double brake_switch_on = 2;
//    double left_turn_visible = 3;
//    double left_turn_switch_on = 4;
//    double right_turn_visible = 5;
//    double right_turn_switch_on = 6;
// }

// message SensorMeasurement {
//    string sensor_id = 1;
//    int32 id = 2;

//    common.Point3D position = 3;
//    double theta = 4;
//    double length = 5;
//    double width = 6;
//    double height = 7;

//    common.Point3D velocity = 8;

//    PerceptionObstacle.Type type = 9;
//    PerceptionObstacle.SubType sub_type = 10;
//    double timestamp = 11;
//    BBox2D box = 12;  // only for camera measurements
// }

struct PerceptionObstacle {
   int id = 1;  // obstacle ID.

  // obstacle position in the world coordinate system.
   Point3D position = {1,2,3};

   double theta = 3;  // heading in the world coordinate system.
   Point3D velocity = {0,0,0};  // obstacle velocity.

  // Size of obstacle bounding box.
   double length = 5;  // obstacle length.
   double width = 6;   // obstacle width.
   double height = 7;  // obstacle height.

  std::vector<Point3D> polygon_point;  // obstacle corner points.

  // duration of an obstacle since detection in s.
   double tracking_time = 9;

  // enum Type {
  //   UNKNOWN = 0;
  //   UNKNOWN_MOVABLE = 1;
  //   UNKNOWN_UNMOVABLE = 2;
  //   PEDESTRIAN = 3;  // Pedestrian, usually determined by moving behavior.
  //   BICYCLE = 4;     // bike, motor bike
  //   VEHICLE = 5;     // Passenger car or truck.
  // };
  //  Type type = 10;         // obstacle type
   double timestamp = 11;  // GPS time in seconds.

  // Just for offline debugging, will not fill this field on board.
  // Format: [x0, y0, z0, x1, y1, z1...]
  vector <double> point_cloud;

   double confidence = 13;// [deprecated = true];
  // enum ConfidenceType {
  //   CONFIDENCE_UNKNOWN
  //   CONFIDENCE_CNN
  //   CONFIDENCE_RADAR
  // };
  //  ConfidenceType confidence_type = 14 [deprecated = true];
  // trajectory of object.
  vector<Point3D> drops;// = 15 [deprecated = true];

  // The following fields are new added in Apollo 4.0
  Point3D acceleration;  // obstacle acceleration

  // a stable obstacle point in the world coordinate system
  // position defined above is the obstacle bounding box ground center
   Point3D anchor_point;
   BBox2D bbox2d;

  // enum SubType {
  //   ST_UNKNOWN = 0;
  //   ST_UNKNOWN_MOVABLE = 1;
  //   ST_UNKNOWN_UNMOVABLE = 2;
  //   ST_CAR = 3;
  //   ST_VAN = 4;
  //   ST_TRUCK = 5;
  //   ST_BUS = 6;
  //   ST_CYCLIST = 7;
  //   ST_MOTORCYCLIST = 8;
  //   ST_TRICYCLIST = 9;
  //   ST_PEDESTRIAN = 10;
  //   ST_TRAFFICCONE = 11;
  // };
  //  SubType sub_type = 19;  // obstacle sub_type

  // repeated SensorMeasurement measurements = 20;  // sensor measurements

  // orthogonal distance between obstacle lowest point and ground plane
   double height_above_ground = 21;// [default = nan];

  // position covariance which is a row-majored 3x3 matrix
  // repeated double position_covariance = 22 [packed = true];
  // // velocity covariance which is a row-majored 3x3 matrix
  // repeated double velocity_covariance = 23 [packed = true];
  // // acceleration covariance which is a row-majored 3x3 matrix
  // repeated double acceleration_covariance = 24 [packed = true];

  // lights of vehicles
  //  LightStatus light_status = 25;
};

// message LaneMarker {
//    apollo.hdmap.LaneBoundaryType.Type lane_type = 1;
//    double quality = 2;  // range = [0,1]; 1 = the best quality
//    int32 model_degree = 3;

//   // equation X = c3 * Z^3 + c2 * Z^2 + c1 * Z + c0
//    double c0_position = 4;
//    double c1_heading_angle = 5;
//    double c2_curvature = 6;
//    double c3_curvature_derivative = 7;
//    double view_range = 8;
//    double longitude_start = 9;
//    double longitude_end = 10;
// }

// message LaneMarkers {
//    LaneMarker left_lane_marker = 1;
//    LaneMarker right_lane_marker = 2;
//   repeated LaneMarker next_left_lane_marker = 3;
//   repeated LaneMarker next_right_lane_marker = 4;
// }

// message CIPVInfo {
//    int32 cipv_id = 1;
//   repeated int32 potential_cipv_id = 2;
// }

// message PerceptionObstacles {
//   repeated PerceptionObstacle perception_obstacle = 1;  // An array of obstacles
//    common.Header header = 2;                    // Header
//    common.ErrorCode error_code = 3 [default = OK];
//    LaneMarkers lane_marker = 4;
//    CIPVInfo cipv_info = 5;  // Closest In Path Vehicle (CIPV)
// }

} //namespace perception
} //namespace apollo
