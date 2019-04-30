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
  
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_human_marker_rviz");

  Mimic mimic;

  ros::spin();

}
