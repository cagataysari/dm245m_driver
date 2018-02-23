#include <dm245m_driver/DM245MDemo.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dm245m_demo_node");
  ROS_INFO("Starting Simple demo for DM245M");
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  DM245MDemo demo_(nh_, nh_priv_);
  demo_.run();
}
