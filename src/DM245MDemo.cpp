#include <dm245m_driver/DM245MDemo.h>

DM245MDemo::DM245MDemo(ros::NodeHandle &nh_, ros::NodeHandle &priv_nh_):
  dm245m_throttle_(nh_, priv_nh_),
  positive       (true),
  turn_count_     (0),
  throttle_speed_ (1),
  steer_turn_     (1)
{

}

DM245MDemo::~DM245MDemo()
{

}

void DM245MDemo::cmdVelCallBack(const geometry_msgs::Twist::ConstPtr& cmd_vel_)
{
  throttle_speed_ = 0;
  if(cmd_vel_->linear.x > 0)
  {
    positive = true;
    throttle_speed_ = (int)cmd_vel_->linear.x;

  } else if ( cmd_vel_->linear.x < 0)
  {
    positive = false;
    throttle_speed_ = (int)cmd_vel_->linear.x;
    throttle_speed_ = -throttle_speed_;
  }
}

void DM245MDemo::run()
{
  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    if(positive)
    {
      ROS_WARN("Moving forward count: %d", throttle_speed_);
      if(throttle_speed_ != 0)
        dm245m_throttle_.turn_motor(false,throttle_speed_);
    } else {
      ROS_ERROR("Moving Backwards count: %d", throttle_speed_);
      if(throttle_speed_ != 0)
        dm245m_throttle_.turn_motor(true,throttle_speed_);

    }
    loop_rate.sleep();
    ros::spinOnce();
  }
  dm245m_throttle_.stop_motor(false);
  ros::shutdown();
}
