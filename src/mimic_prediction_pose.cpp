#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

class Mimic
{
  public:
    Mimic();

  private:
    void posearrayCallback(const geometry_msgs::PoseArrayConstPtr& poses);
    ros::Publisher markerarray_pub_;
    ros::Subscriber posearray_sub_;
};

Mimic::Mimic()
{
  ros::NodeHandle input_nh("input");
  ros::NodeHandle output_nh("output");
  markerarray_pub_ = output_nh.advertise<visualization_msgs::MarkerArray> ("/target_human/prediction/marker", 10);
  posearray_sub_ = input_nh.subscribe<geometry_msgs::PoseArray> ("/target_human/prediction/pose", 10, &Mimic::posearrayCallback, this);
}

void Mimic::posearrayCallback(const geometry_msgs::PoseArrayConstPtr& poses)
{
  visualization_msgs::MarkerArray markers;
  for (int i = 0; i < 5; i++) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "prediction marker";
    marker.id = i+1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = poses->poses[i].position.x;
    marker.pose.position.y = poses->poses[i].position.y;
    marker.pose.position.z = 0.85;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.7f;
    marker.color.g = 0.7f;
    marker.color.b = 0.9f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(5);

    markers.markers.push_back(marker);

  }
  markerarray_pub_.publish(markers);

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "prediction_human_marker_rviz");

  Mimic mimic;

  ros::spin();

}
