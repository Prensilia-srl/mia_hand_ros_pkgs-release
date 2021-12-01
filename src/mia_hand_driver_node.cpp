#include "mia_hand_driver/ros_driver.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mia_hand");

  ros::NodeHandle mia_hand_nh;
  ros::NodeHandle mia_hand_nh_priv("~");

  mia_hand::ROSDriver mia_hand(mia_hand_nh, mia_hand_nh_priv);

	ros::spin();
	
 	return 0;
}
