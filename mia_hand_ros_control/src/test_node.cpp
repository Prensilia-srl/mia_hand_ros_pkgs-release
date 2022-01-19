#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <vector>
#include <sstream>

#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/simple_transmission.h>
#include "mia_hand_ros_control/mia_transmission_interface.h"
#include <mia_hand_ros_control/mia_mrl_transmission.h>
#include <mia_hand_ros_control/mia_index_transmission.h>
#include <mia_hand_ros_control/mia_thfle_transmission.h>


#include <iostream> // to be deleted

int main(int argc, char **argv)
{


  ros::init(argc, argv, "test_node");


  ros::NodeHandle n;


  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);


  int count = 0;


   // Transmission MRL
  // Raw data
  double a_mrl_pos;
  double j_mrl_pos;

  double a_mrl_spe;
  double j_mrl_spe;


   // Transmission MRL
   transmission_interface::MiaThfleTransmission ThfleTrans;
   std::vector<transmission_interface::Transmission*> mio;
   mio.resize(3);
   mio[0] = & ThfleTrans;

  transmission_interface::MiaMrlTransmission MrlTrans;



   // Transmission


  // Wrap raw data MRL
  transmission_interface::ActuatorData a_mrl_data;
  a_mrl_data.position.push_back(&a_mrl_pos);
  a_mrl_data.velocity.push_back(&a_mrl_spe);

  transmission_interface::JointData j_mrl_data;
  j_mrl_data.position.push_back(&j_mrl_pos);
  j_mrl_data.velocity.push_back(&j_mrl_spe);



   // Transmission interface
  transmission_interface::ActuatorToJointPositionInterface act_to_jnt_mrl_pos;
  act_to_jnt_mrl_pos.registerHandle(transmission_interface::ActuatorToJointPositionHandle("MrlTrans", &MrlTrans, a_mrl_data, j_mrl_data));

  transmission_interface::JointToActuatorPositionInterface jnt_to_act_mrl_pos;
  jnt_to_act_mrl_pos.registerHandle(transmission_interface::JointToActuatorPositionHandle("MrlTrans", &MrlTrans, a_mrl_data, j_mrl_data));

  transmission_interface::ActuatorToJointVelocityInterface act_to_jnt_mrl_spe;
  act_to_jnt_mrl_spe.registerHandle(transmission_interface::ActuatorToJointVelocityHandle("MrlTrans", &MrlTrans, a_mrl_data, j_mrl_data));

  transmission_interface::JointToActuatorVelocityInterface jnt_to_act_mrl_spe;
  jnt_to_act_mrl_spe.registerHandle(transmission_interface::JointToActuatorVelocityHandle("MrlTrans", &MrlTrans, a_mrl_data, j_mrl_data));



    // Transmission INDEX
     // Raw data
  double a_index_pos; // state
  double j_index_pos;

  double a_index_spe; // state
  double j_index_spe;

   double a_index_pos_cmd; // cmd
   double a_index_spe_cmd; // cmd


  transmission_interface::MiaIndexTransmission IndexTrans; // index trasmission


  // Wrap raw data MRL
  transmission_interface::ActuatorData a_index_data;
  a_index_data.position.push_back(&a_index_pos); // state
  a_index_data.velocity.push_back(&a_index_spe);

  transmission_interface::ActuatorData a_index_cmd;
  a_index_cmd.position.push_back(&a_index_pos_cmd); // cmd
  a_index_cmd.velocity.push_back(&a_index_spe_cmd);

  transmission_interface::JointData j_index_data;
  j_index_data.position.push_back(&j_index_pos);
  j_index_data.velocity.push_back(&j_index_spe);



   // Transmission interface
  transmission_interface::MiaActuatorToJointPositionInterface act_to_jnt_index_pos;
  act_to_jnt_index_pos.registerHandle(transmission_interface::MiaActuatorToJointPositionHandle("IndexTrans", &IndexTrans, a_index_data, j_index_data));

  transmission_interface::MiaJointToActuatorPositionInterface jnt_to_act_index_pos;
  jnt_to_act_index_pos.registerHandle(transmission_interface::MiaJointToActuatorPositionHandle("IndexTrans", &IndexTrans, a_index_cmd, j_index_data,a_index_data));

  transmission_interface::MiaActuatorToJointVelocityInterface act_to_jnt_index_spe;
  act_to_jnt_index_spe.registerHandle(transmission_interface::MiaActuatorToJointVelocityHandle("IndexTrans", &IndexTrans, a_index_data, j_index_data));

  transmission_interface::MiaJointToActuatorVelocityInterface jnt_to_act_index_spe;
  jnt_to_act_index_spe.registerHandle(transmission_interface::MiaJointToActuatorVelocityHandle("IndexTrans", &IndexTrans, a_index_cmd, j_index_data ,a_index_data));


  while (ros::ok())
  {

   //******* MRL ****** //

    std::cout<< "Give me MRL actuator pos data:\n";
    std::cin>> a_mrl_pos;

    // Propagate actuator position to joint space
    act_to_jnt_mrl_pos.propagate();
    std::cout<< "j_mrl_angle: " << j_mrl_pos;
    std::cout<< std::endl;


     std::cout<< "Give me MRL joint positin data:\n";
     std::cin>> j_mrl_pos;
     std::cout<< std::endl;

    // Propagate actuator position to joint space
    jnt_to_act_mrl_pos.propagate();
    std::cout<< "j_mrl_pos counts: " << a_mrl_pos ;
    std::cout<< std::endl;

    // Velocity
     std::cout<< "Give me MRL actuator speed data:\n";
     std::cin>> a_mrl_spe;
     std::cout<< std::endl;

    // Propagate actuator position to joint space
    act_to_jnt_mrl_spe.propagate();
    std::cout<< "j_mrl_angular_speed: " << j_mrl_spe ;
    std::cout<< std::endl;

     std::cout<< "Give me MRL joint angular velocity:\n";
     std::cin>> j_mrl_spe ;
     std::cout<< std::endl;

    // Propagate actuator position to joint space
    jnt_to_act_mrl_spe.propagate();
    std::cout<< "j_mrl_spe counts: " << a_mrl_spe << std::endl;


    //******* Index ****** //
    /*
    std::cout<< "Give me Index actuator pos data:\n";
    std::cin>> a_index_pos;

    // Propagate actuator position to joint space
    act_to_jnt_index_pos.propagate();
    std::cout<< "j_index_angle: " << j_index_pos;
    std::cout<< std::endl<< std::endl;


     std::cout<< "Give me index joint positin data:\n";
     std::cin>> j_index_pos;

    // Propagate actuator position to joint space
    jnt_to_act_index_pos.propagate();
    std::cout<< "j_index_pos counts: " << a_index_pos_cmd ;
    std::cout<< std::endl<< std::endl;

    // Velocity
     std::cout<< "Give me index actuator speed data:\n";
     std::cin>> a_index_spe;


    // Propagate actuator position to joint space
    act_to_jnt_index_spe.propagate();
    std::cout<< "j_index_angular_speed: " << j_index_spe <<" at pos: " <<  a_index_pos ;
    std::cout<< std::endl<< std::endl;

     std::cout<< "Give me index joint angular velocity:\n";
     std::cin>> j_index_spe ;


    // Propagate actuator position to joint space
    jnt_to_act_index_spe.propagate();
    std::cout<< "j_index_spe counts: " << a_index_spe_cmd <<" at pos: " <<  a_index_pos << std::endl<< std::endl;

    */



    std_msgs::String msg;

    std::stringstream ss;
    ss << "\n hello world " << count;
    msg.data = ss.str();



    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
