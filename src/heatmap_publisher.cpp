#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <people_msgs/PositionMeasurement.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/highgui/OccupancyGrid.h>
#include <chrono>
#include <thread>

int flag = 0;

void Callback(const people_msgs::PositionMeasurementArray& msg)
{
  //std::cout << msg.data << std::endl;
  data_base=msg.data;
  ros::NodeHandle n;

  float cell_x, cell_y = 0.0;
  int occ, occ_x, occ_y = 0;
  if (flag == 0) {

  } else {

  }

  n.getParam("/exp_num", exp_num);
  n.setParam("exp_miki_img/switch", 1);

}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "exp_9101112");

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("static_map");
  get_map = client.call();
  

  ros::Subscriber sub = n.subscribe("/filter/people_tracker_measurement", 1000, &Callback);

  ros::Rate rate(20);

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
    }


  return 0;
}
