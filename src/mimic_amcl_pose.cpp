#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>


class Mimic
{
  public:
    Mimic();

  private:
    void amclCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& poses);
    ros::Publisher marker_pub_;
    ros::Subscriber amcl_sub_;
};

Mimic::Mimic()
{
  ros::NodeHandle input_nh("input");
  ros::NodeHandle output_nh("output");
  marker_pub_ = output_nh.advertise<visualization_msgs::Marker> ("/search_area/marker", 10);
  amcl_sub_ = input_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("/amcl_pose", 10, &Mimic::amclCallback, this);
}

void Mimic::amclCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& poses)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "prediction marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = poses->pose.pose.position.x;
    marker.pose.position.y = poses->pose.pose.position.y;
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

    marker.lifetime = ros::Duration(0.5);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "amcl_pose_marker_rviz");

  Mimic mimic;

  ros::spin();

}
