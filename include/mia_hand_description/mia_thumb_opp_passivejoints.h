/*********************************************************************************************************
 thumb_opp_passive_joints class.
 Author: Prensilia srl
  version: 1.0
**********************************************************************************************************/
#ifndef MIA_THUMB_OPP_PASSIVEJOINTS_H
#define MIA_THUMB_OPP_PASSIVEJOINTS_H

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

/**
 * namespace containing thumb_opp_passive_joints class.
 */
namespace mia_hand
{

/**
 * thumb_opp_passive_joints class.
 * This class can be used to evaluate the state of the thumb opposition (passive
 * joint) of the Mia Hand. using the state of the index_flex joint.
 */
class thumb_opp_passive_joint
{
public:

    /**
     * Class constructor.
     */
    thumb_opp_passive_joint();  // Constructor, parameters with default values.

    /**
     * Class destructor.
     */
    ~thumb_opp_passive_joint();


    /**
     * Initialize class.
     * @param LoadURDFInfo a bool argument. True if the class has to load joint limits info from the URDF.
     */
    void init(const bool LoadURDFInfo_ = false);

    /**
     * Evaluate the position of the thumb opposition based on index flexion state.
     * A method taking the position of the index_fle joint and returning the position of the thumb_opp joint.
     * @param j_index_flex_pos a double argument describing the position of the index_fle joint.
     * @ŗeturn double describing the position of the thumb_opp joint.
     */
     double GetThumbOppPosition(double j_index_flex_pos);

    /**
     * Load Info from loaded URDF.
     * A method that uses the class attributes to extract the upper and lower limit of the thumb joint from the URDF.
     * @see j_index_name()
     * @see j_thumb_name()
     * @see robot_description_()
     * @see ThMinPos()
     * @see ThMaxPos()
     * @return updates var ThMinPos and ThMaxPos.
     */
     bool LoadURDFInfo();


    /**
     * Name of the index_fle joint specified in the URDF.
     *
     */
    std::string j_index_name ;

    /**
     * Name of the thumb_opp joint specified in the URDF.
     *
     */
    std::string j_thumb_name ;

    /**
     * Name of parameter describing the robot description loaded in the ROS parameter server.
     *
     */
    std::string robot_description_ ;

    /**
     * Lower Limit of the thumb_opp joint.
     *
     */
    double ThMinPos;

    /**
     * Upper Limit of the thumb_opp joint.
     *
     */
    double ThMaxPos;

private:
    /**
     * A ROS node variable.
     *
     */
    ros::NodeHandle n;

  /**
		* Get the name of the URDF.
		* A method taking the ros parameter urdf generic name and return the specific
		* of the loaded URDF.
		* @param param_name generic name of the param to look for.
		* @return the specific name of the parameter linking to the robot URDF
		*/
    std::string getURDF(std::string param_name) const;


};
}  // namespace

#endif  // MIA_THUMB_OPP_PASSIVEJOINTS_H
