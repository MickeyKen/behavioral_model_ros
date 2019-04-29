#include "ros/ros.h"
#include <stdio.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "behavioral_model/AddPoseRetStr.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
// #include <people_msgs/PositionMeasurementArray.h>
// #include <people_msgs/PositionMeasurement.h>

class Server {
public:
  Server();
  ~Server() {};

  // void poseCallback(const people_msgs::PositionMeasurementArray::ConstPtr& pose);
  void poseCallback(const std_msgs::String::ConstPtr& pose);

  bool PatrolService(behavioral_model::AddPoseRetStr::Request  &req,
                    behavioral_model::AddPoseRetStr::Response &res);
  void loop();

private:
  ros::Publisher nav_pub;
  ros::Subscriber human_sub;
  ros::NodeHandle nh;
  ros::ServiceServer service;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  geometry_msgs::PoseStamped a_pose, b_pose, c_pose, d_pose, ex_pose;

  const static double a_x = 0.1;

  std_msgs::String ret_str;
  // ret_str = "success";
  std::string ss;

  int flag;

  const static double offset_x = 0.5;
  const static double offset_y = 0.5;
  const static double offset_w = 0.05;

};

Server::Server()
{
  ROS_INFO("Ready to patrol");

  service = nh.advertiseService("/search/target_human",&Server::PatrolService,this);

  nav_pub= nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

  // human_sub = nh.subscribe("/people_tracker_measurements", 10, &Server::poseCallback, this);
  human_sub = nh.subscribe("/ptm/server/result", 10, &Server::poseCallback, this);

}

bool Server::PatrolService(behavioral_model::AddPoseRetStr::Request  &req,
                          behavioral_model::AddPoseRetStr::Response &res)
{
  a_pose.header.frame_id = "map";
  a_pose.pose.position.x =  1.0;
  a_pose.pose.position.y =  0.0;
  a_pose.pose.orientation.w = 1.0;
  b_pose.header.frame_id = "map";
  b_pose.pose.position.x =  -1.0;
  b_pose.pose.position.y =  0.0;
  b_pose.pose.orientation.w = 1.0;
  c_pose.header.frame_id = "map";
  c_pose.pose.position.x =  -1.0;
  c_pose.pose.position.y =  2.0;
  c_pose.pose.orientation.w = 1.0;
  d_pose.header.frame_id = "map";
  d_pose.pose.position.x =  1.0;
  d_pose.pose.position.y =  1.0;
  d_pose.pose.orientation.w = 1.0;

  double target_x = 0.0;
  double target_y = 0.0;

  for (int i = 0; i < 4; i++) {
    // printf ("pass");
    ros::Time time = ros::Time::now();
    if (i == 0){
      target_x = a_pose.pose.position.x;
      target_y = a_pose.pose.position.y;
      a_pose.header.stamp = time;
      nav_pub.publish(a_pose);
    } else if (i == 1){
      target_x = b_pose.pose.position.x;
      target_y = b_pose.pose.position.y;
      b_pose.header.stamp = time;
      nav_pub.publish(b_pose);
    } else if (i == 2){
      target_x = c_pose.pose.position.x;
      target_y = c_pose.pose.position.y;
      c_pose.header.stamp = time;
      nav_pub.publish(c_pose);
    } else{
      target_x = d_pose.pose.position.x;
      target_y = d_pose.pose.position.y;
      d_pose.header.stamp = time;
      nav_pub.publish(d_pose);
    }

    while (1) {
      if (flag == 1) {
        // ss = "human";
        res.result.data = ss;
        return true;

      } else {

        try {
          listener.waitForTransform("/map","/base_link", ros::Time(0), ros::Duration(3.0));
          listener.lookupTransform("/map","/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }

        if (transform.getOrigin().x() > (-offset_x + target_x) && transform.getOrigin().x() < (offset_x + target_x) &&
            transform.getOrigin().y() > (-offset_y + target_y) && transform.getOrigin().y() < (offset_y + target_y))
            break;
      }
    }
  }

  ss = "nohuman";

  res.result.data = ss;

  return true;
}


// void Server::poseCallback(const people_msgs::PositionMeasurementArray::ConstPtr& pose)
void Server::poseCallback(const std_msgs::String::ConstPtr& pose)
{
  if (pose->data == "nohuman") {
    ss = "nohuman";
    flag = 0;
  }
  else {
    ss = pose->data;
    flag = 1;
  }
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
