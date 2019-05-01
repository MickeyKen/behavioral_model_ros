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
  markerarray_pub_ = output_nh.advertise<visualization_msgs::MarkerArray> ("/target_human/prediction/marker", 1);
  posearray_sub_ = input_nh.subscribe<geometry_msgs::PoseArray> ("/target_human/prediction/pose", 1, &Mimic::posearrayCallback, this);
}

void Mimic::posearrayCallback(const geometry_msgs::PoseArrayConstPtr& poses)
{
  visualization_msgs::MarkerArray markers;
  for (int i = 0; i < 5; i++) {
    // visualization_msgs::Marker marker_head;

    markers.markers[i].header.frame_id = "/map";
    markers.markers[i].header.stamp = ros::Time::now();

    markers.markers[i].ns = "basic_shapes";
    markers.markers[i].id = 1;
    markers.markers[i].type = 3;
    markers.markers[i].action = visualization_msgs::Marker::ADD;
    markers.markers[i].pose.position.x = poses->poses[i].position.x;
    markers.markers[i].pose.position.y = poses->poses[i].position.y;
    markers.markers[i].pose.position.z = 0.85;
    markers.markers[i].pose.orientation.x = 0.0;
    markers.markers[i].pose.orientation.y = 0.0;
    markers.markers[i].pose.orientation.z = 0.0;
    markers.markers[i].pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    markers.markers[i].scale.x = 0.2;
    markers.markers[i].scale.y = 0.2;
    markers.markers[i].scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    markers.markers[i].color.r = 1.0f;
    markers.markers[i].color.g = 0.0f;
    markers.markers[i].color.b = 1.0f;
    markers.markers[i].color.a = 1.0;

    markers.markers[i].lifetime = ros::Duration();

  }
  markerarray_pub_.publish(markers);

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "prediction_human_marker_rviz");

  Mimic mimic;

  ros::spin();

}
