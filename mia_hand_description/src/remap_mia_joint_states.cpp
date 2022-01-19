#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <string>
#include <cmath>
#include <urdf/model.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include "mia_hand_description/mia_thumb_opp_passivejoints.h"


/**
 * Class to remap Mia JointState
 */
class myRemap
{

    public:

    /**
     * Constructor
    */
    myRemap()
    {
      pub = n.advertise<sensor_msgs::JointState>("miaviz_joint_states", 1000);
      sub = n.subscribe("joint_states", 1000,  &myRemap::chattercallback, this);

      const bool load = true;
      MyTh_opp_passiveJoint.init(load);
      MyTh_opp_passiveJoint.robot_description_ = "robot_description"; // default
      j_index_name = MyTh_opp_passiveJoint.j_index_name;
      j_thumb_name = MyTh_opp_passiveJoint.j_thumb_name;

      //Get Server IP address
      if (ros::param::has("~remap_MIA_th_opp"))
      {
          ros::param::get("~remap_MIA_th_opp", remap_th_opp);
      }
      else
      {
          remap_th_opp = false; // default value
          ros::param::set("~remap_MIA_th_opp", remap_th_opp);

      }


    }

    private:

   /**
    * Bool True to remap also the thumb opp position
    */
    bool remap_th_opp;

    /**
     * Ros node varibale
     */
    ros::NodeHandle n;

    /**
     * Publisher to publish remapped miajoint
     */
    ros::Publisher pub;

    /**
     * Subscriber to publish listen to raw mia joint states
     */
    ros::Subscriber sub;

    /**
     * Name of the index flex joint as declared in the URDF
     */
    std::string j_index_name ;

    /**
     * Name of the thumb opp joint as declared in the URDF
     */
    std::string j_thumb_name ;

    /**
     * Name of parameter link to the URDF
     */
    std::string robot_description_ ;

    /**
     * Class describing the thumb_opp and index flex passive joint
     */
    mia_hand::thumb_opp_passive_joint MyTh_opp_passiveJoint;


   /**
    * Callback of the subscriber
    * @see sub()
    */
   void chattercallback(const sensor_msgs::JointState::ConstPtr& msg)
   {
     int nJoints = msg->name.size();

     sensor_msgs::JointState sensor_msgs = *msg;

     int j_index = -1;
     int j_th = -1;

     for(unsigned int j=0; j < nJoints; j++)
     {
     	const std::string jname = sensor_msgs.name[j];
  	if (jname == j_index_name) {j_index = j;}
  	if (jname == j_thumb_name) {j_th = j;}
  	if(j_index > -1 && j_th > -1) {break;}
     }

     // Remap


     if (j_index > -1 && j_th >-1 && remap_th_opp == true)
     {
        double th_fle_pos = MyTh_opp_passiveJoint.GetThumbOppPosition(sensor_msgs.position[j_index]);
        sensor_msgs.position[j_th] = th_fle_pos;
     	sensor_msgs.position[j_index] = std::abs(sensor_msgs.position[j_index]);

     }
     else if (j_index > -1 && (j_th == -1 || remap_th_opp == false))
     {
         sensor_msgs.position[j_index] = std::abs(sensor_msgs.position[j_index]);
     }

     pub.publish(sensor_msgs);

   }


};



/**
 * Main of the ROS node using the Remap class
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "remap_mia_joint_states");

  myRemap myInstance;


  ros::spin();

  return 0;
}
