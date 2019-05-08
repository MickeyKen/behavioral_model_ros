#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <people_msgs/PositionMeasurement.h>

class Mimic
{
  public:
    Mimic();

  private:
    void poseCallback(const people_msgs::PositionMeasurementArrayConstPtr& pose);
    ros::Publisher marker_pub_;
    ros::Subscriber pose_sub_;
};

Mimic::Mimic()
{
  ros::NodeHandle input_nh("input");
  ros::NodeHandle output_nh("output");
  marker_pub_ = output_nh.advertise<visualization_msgs::MarkerArray> ("/filter/target_human/marker", 10);
  pose_sub_ = input_nh.subscribe<people_msgs::PositionMeasurementArray> ("/filter/people_tracker_measurement", 10, &Mimic::poseCallback, this);
}

void Mimic::poseCallback(const people_msgs::PositionMeasurementArrayConstPtr& pose)
{
  int num = pose->people.size();
  // printf ("%d", num);

  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker_body;
  visualization_msgs::Marker marker_head;

  for (int i = 0; i < num; i++) {

    marker_body.header.frame_id = "/map";
    marker_body.header.stamp = ros::Time::now();

    marker_body.ns = "basic_shapes";
    marker_body.id = (i * 2) + 1;
    marker_body.type = 3;
    marker_body.action = visualization_msgs::Marker::ADD;
    marker_body.pose.position.x = pose->people[i].pos.x;
    marker_body.pose.position.y = pose->people[i].pos.y;
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
    marker_body.color.r = 0.0f;
    marker_body.color.g = 0.0f;
    marker_body.color.b = 1.0f;
    marker_body.color.a = 1.0;

    marker_body.lifetime = ros::Duration(0.5);
    markers.markers.push_back(marker_body);

    marker_head.header.frame_id = "/map";
    marker_head.header.stamp = ros::Time::now();

    marker_head.ns = "basic_shapes";
    marker_head.id = (i * 2) + 2;
    marker_head.type = 2;
    marker_head.action = visualization_msgs::Marker::ADD;
    marker_head.pose.position.x = pose->people[i].pos.x;
    marker_head.pose.position.y = pose->people[i].pos.y;
    marker_head.pose.position.z = 1.95;
    marker_head.pose.orientation.x = 0.0;
    marker_head.pose.orientation.y = 0.0;
    marker_head.pose.orientation.z = 0.0;
    marker_head.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker_head.scale.x = 0.5;
    marker_head.scale.y = 0.5;
    marker_head.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker_head.color.r = 0.0f;
    marker_head.color.g = 0.0f;
    marker_head.color.b = 1.0f;
    marker_head.color.a = 1.0;

    marker_head.lifetime = ros::Duration(0.5);
    markers.markers.push_back(marker_head);
  }

  marker_pub_.publish(markers);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_fileterd_human_marker_rviz");

  Mimic mimic;

  ros::spin();

}
