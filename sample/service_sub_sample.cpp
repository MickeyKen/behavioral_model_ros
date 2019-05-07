#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include "my_service/Wheel.h"

class Server {
public:
  Server();
  ~Server() {};
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  // サーバーのコールバック関数（サービスの本体）
  bool wheelService(my_service::Wheel::Request  &req,
                    my_service::Wheel::Response &res);
  void loop(); // ループ関数

private:
  ros::Publisher cmd_pub;     // パブリッシャ
  ros::Subscriber odom_sub;   // サブスクライバ
  ros::NodeHandle nh;         // ノードハンドルの宣言
  ros::ServiceServer service; // サービス
  double tmp_linear_velocity; // 現在の並進速度
  double tmp_angluar_velocity; // 現在の回転速度
  const static double max_linear_velocity =  0.7; // 最大並進速度
  const static double min_linear_velocity = -0.7; // 最小並進速度
  const static double max_angular_velocity =  1.2; // 最大回転速度
  const static double min_angular_velocity = -1.2; // 最小回転速度
  geometry_msgs::Twist target_vel, tmp_vel; // 目標速度、現在の速度
};

// コンストラクタ
Server::Server()
{
  ROS_INFO("Ready to wheel");

  //サービスの設定
  service = nh.advertiseService("wheel",&Server::wheelService,this);

  // パブリッシャ(配信)の設定:
  cmd_pub= nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

  // サブスクライバ(購読)の設定
  // /odomトピックはロボットの速度情報を持っている
  odom_sub = nh.subscribe("/odom", 10, &Server::odomCallback, this);

  // 速度の初期化
  target_vel.linear.x  = 0.0;
  target_vel.linear.y  = 0.0;
  target_vel.linear.z  = 0.0;
  target_vel.angular.x = 0.0;
  target_vel.angular.y = 0.0;
  target_vel.angular.z = 0.0;
  tmp_vel.linear.x  = 0.0;
  tmp_vel.linear.y  = 0.0;
  tmp_vel.linear.z  = 0.0;
  tmp_vel.angular.x = 0.0;
  tmp_vel.angular.y = 0.0;
  tmp_vel.angular.z = 0.0;
}

// サーバーのコールバック関数
bool Server::wheelService(my_service::Wheel::Request  &req,
                          my_service::Wheel::Response &res)
{
  ROS_INFO("Set velocity: linear=%.2f angular=%.2f",
           req.target_linear_velocity, req.target_angular_velocity);
  // 指令速度が最小、最大速度内かチェック
  if (min_linear_velocity  > req.target_linear_velocity ||
      max_linear_velocity  < req.target_linear_velocity)  return false;
  if (min_angular_velocity > req.target_angular_velocity ||
      max_angular_velocity < req.target_angular_velocity) return false;
  // クライアントからの速度をロボットの速度指令とする
  target_vel.linear.x  = req.target_linear_velocity;
  target_vel.angular.z = req.target_angular_velocity;
  // /odomトピックからサブスクライブした速度情報をクライアントに返す
  res.current_linear_velocity  = tmp_vel.linear.x;
  res.current_angular_velocity = tmp_vel.angular.z;
  return true;
}
// オドメトリのコールバック関数（現在速度を知る）
void Server::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  tmp_vel = odom->twist.twist; // 現在の速度ゲット
}

// ループ関数
void Server::loop()
{
  ros::Rate loop_rate(30); // Hz

  while(ros::ok()) {
    // ロボットを動かすため目標速度をパブリッシュ
    cmd_pub.publish(target_vel);
    // コールバック関数を呼ぶ
    ros::spinOnce();
    // 決められた周期でループするため寝て待つ
    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{
  //ROSの初期化 wheel_serverはノード名
  ros::init(argc, argv, "wheel_server");

  Server svr;
  svr.loop();
  return 0;
}
