#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>

class Mimic
{
  public:
    Mimic();

  private:
    void poseCallback(const geometry_msgs::PoseConstPtr& pose);
    ros::Publisher marker_pub_;
    ros::Subscriber pose_sub_;
};

Mimic::Mimic()
{
  ros::NodeHandle input_nh("input");
  ros::NodeHandle output_nh("output");
  marker_pub_ = output_nh.advertise<visualization_msgs::Marker> ("/optimize/pose/marker", 10);
  pose_sub_ = input_nh.subscribe<geometry_msgs::Pose> ("/ud/optimize/pose", 10, &Mimic::poseCallback, this);
}

void Mimic::poseCallback(const geometry_msgs::PoseConstPtr& pose)
{

  visualization_msgs::Marker marker;

  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "optimize marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = pose->position.x;
  marker.pose.position.y = pose->position.y;
  marker.pose.position.z = 0.3;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.7f;
  marker.color.b = 0.5f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(5);

  marker_pub_.publish(marker);

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "optimize_ud_point_marker_rviz");

  Mimic mimic;

  ros::spin();

}
