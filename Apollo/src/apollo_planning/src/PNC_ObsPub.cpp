#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "PNC_Obs");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("Obs_Pub", 10);

  ros::Rate r(1);


    //创建一个 visualization_msgs/Marker消息
    visualization_msgs::Marker obs;
    obs.header.frame_id =  "/my_frame";
    obs.header.stamp  = ros::Time::now();
    obs.ns = "PNC_Obs";
    obs.action =  visualization_msgs::Marker::ADD;
    obs.pose.orientation.w = 1.0;
    obs.pose.position.x= 45;
    obs.pose.position.y= -32;
    obs.pose.position.z=1;
    // obs.pose.orientation.z=0.5;


    //分配id
   obs.id = 1;

    obs.type = visualization_msgs::Marker::CUBE;

    obs.scale.x = 2;
    obs.scale.y = 1;
    obs.scale.z= 2;

    obs.color.r = 1.0;
    obs.color.a = 1.0;


    while (ros::ok())
    {
    

    //发布markers
    marker_pub.publish(obs);

    r.sleep();

  }
}
