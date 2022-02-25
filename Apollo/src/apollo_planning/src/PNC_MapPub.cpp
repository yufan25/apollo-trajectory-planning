#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "PNC_MapPub");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("Map_Pub", 10);

  ros::Rate r(1);

  float f = 0.0;
  int pp= 0;


    //创建一个 visualization_msgs/Marker消息，并且初始化所有数据。
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "PNC_MapPub";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;


    points.id = pp;
    //pp++;
    line_strip.id = 1;
    line_list.id = 2;


    //设置marker类型到 POINTS,  LINE_LIST
    points.type = visualization_msgs::Marker::POINTS;
   // line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.type = visualization_msgs::Marker::LINE_LIST;

    points.scale.x = 0.5;
    points.scale.y = 0.5;

    line_strip.scale.x = 0.5;
    line_list.scale.x = 0.1;


    points.color.g = 1.0;
    points.color.a = 1.0;

    line_strip.color.b = 2;
    line_strip.color.a = 2;

    while (ros::ok())
    {

      // The line list needs two points for each line
    //   line_list.points.push_back(p);
    //   p.z += 1.0;
    //   line_list.points.push_back(p);
    for (int32_t i =0;i<60;i++)
    {
      geometry_msgs::Point p;
      p.x = 2+i*2;
      p.y = -32;
      p.z = 0;

      points.points.push_back(p);
      line_strip.points.push_back(p);
    }
 

  

    //发布各个markers
   // marker_pub.publish(points);
    marker_pub.publish(line_strip);

    r.sleep();

   // f += 0.04;
  }
}
