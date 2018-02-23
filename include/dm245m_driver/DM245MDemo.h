#pragma once
#include <dm245m_driver/DM245M.h>
#include <geometry_msgs/Twist.h>

class DM245MDemo {

private:
  DM245MDriver dm245m_throttle_;
  bool positive;
  int turn_count_;
  int throttle_speed_;
  int steer_turn_;

private:
  void cmdVelCallBack(const geometry_msgs::Twist::ConstPtr& cmd_vel_);

public:
  DM245MDemo(ros::NodeHandle& nh_, ros::NodeHandle& priv_nh_);
  ~DM245MDemo();
  void run();

};
