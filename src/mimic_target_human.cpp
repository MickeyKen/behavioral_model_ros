#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
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
  marker_pub_ = output_nh.advertise<visualization_msgs::Marker> ("/target_human/marker", 1);
  pose_sub_ = input_nh.subscribe<geometry_msgs::Pose> ("/target_human/pose", 1, &Mimic::poseCallback, this);
}

void Mimic::poseCallback(const geometry_msgs::PoseConstPtr& pose)
{
  visualization_msgs::Marker marker_body;
  // visualization_msgs::Marker marker_head;

  marker_body.header.frame_id = "/map";
  marker_body.header.stamp = ros::Time::now();

  marker_body.ns = "basic_shapes";
  marker_body.id = 0;
  marker_body.type = 3;
  marker_body.action = visualization_msgs::Marker::ADD;
  marker_body.pose.position.x = pose->position.x;
  marker_body.pose.position.y = pose->position.y;
  marker_body.pose.position.z = 0.85;
  marker_body.pose.orientation.x = 0.0;
  marker_body.pose.orientation.y = 0.0;
  marker_body.pose.orientation.z = 0.0;
  marker_body.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker_body.scale.x = 0.5;
  marker_body.scale.y = 0.5;
  marker_body.scale.z = 1.7;

  // Set the color -- be sure to set alpha to something non-zero!
  marker_body.color.r = 1.0f;
  marker_body.color.g = 0.0f;
  marker_body.color.b = 0.0f;
  marker_body.color.a = 1.0;

  marker_body.lifetime = ros::Duration(0.5);

  marker_pub_.publish(marker_body);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_human_marker_rviz");

  Mimic mimic;

  ros::spin();

}
