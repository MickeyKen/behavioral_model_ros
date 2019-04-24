#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "behavioral_model/AddPoseRetStr.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"

class Server {
public:
  Server();
  ~Server() {};

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& target_pose);

  bool PatrolService(behavioral_model::AddPoseRetStr::Request  &req,
                    behavioral_model::AddPoseRetStr::Response &res);
  void loop();

private:
  ros::Publisher nav_pub;
  ros::Subscriber human_sub;
  ros::NodeHandle nh;
  ros::ServiceServer service;

  geometry_msgs::PoseStamped a_pose, b_pose, c_pose, d_pose;

  const static double a_x = 0.1;

  std_msgs::String ret_str;
  // ret_str = "success";
  std::stringstream ss;

};

Server::Server()
{
  ROS_INFO("Ready to patrol");

  service = nh.advertiseService("/patrol/2012",&Server::PatrolService,this);

  nav_pub= nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

  human_sub = nh.subscribe("/human_pose", 10, &Server::poseCallback, this);

}

bool Server::PatrolService(behavioral_model::AddPoseRetStr::Request  &req,
                          behavioral_model::AddPoseRetStr::Response &res)
{
  a_pose.header.frame_id = "map";
  a_pose.pose.position.x =  1.0;
  a_pose.pose.position.y =  0.0;
  b_pose.header.frame_id = "map";
  b_pose.pose.position.x =  -1.0;
  b_pose.pose.position.y =  0.0;
  c_pose.header.frame_id = "map";
  c_pose.pose.position.x =  -1.0;
  c_pose.pose.position.y =  2.0;
  d_pose.header.frame_id = "map";
  d_pose.pose.position.x =  1.0;
  d_pose.pose.position.y =  2.0;
  for (int i = 0; i < 4; i++) {
    if (i == 0){
      ros::Time time = ros::Time::now();
      a_pose.header.stamp = time;
    }
  }
  // ret_str = "result";
  ss << "hello world ";
  res.result.data = ss.str();
  // req.result = ret_str;
  return true;
}


void Server::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  ss << "hello world ";
}

void Server::loop()
{
  ros::Rate loop_rate(30);

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "Search_and_Wander_program");

  Server svr;

  svr.loop();

  return 0;
}
