 /*********************************************************************************************************
 Modifyed by Author: Prensilia srl
 Desc:   Transmission interfae of the MIA thumbe flexion finger

 version 1.0

**********************************************************************************************************/

#include "mia_hand_ros_control/mia_thfle_transmission.h" 
#include <math.h>

using transmission_interface::ActuatorData;
using transmission_interface::JointData;

namespace transmission_interface
{

//////////////////////////////////////////////////////////////////////////////////////////////
//MiaThfleTransmission : constructor
//////////////////////////////////////////////////////////////////////////////////////////////

 MiaThfleTransmission::MiaThfleTransmission()
  : Transmission()
{

   reduction_ = 64.0 * 40.0;


  if (0.0 == reduction_)
  {
    throw TransmissionInterfaceException("Transmission reduction ratio cannot be zero.");
  }
}


 //////////////////////////////////////////////////////////////////////////////////////////////
// Simple functions needed to evaluate trasmission ratio
/////////////////////////////////////////////////////////////////////////////////////////////

// mu = h_thfle(pos)
inline double  MiaThfleTransmission::h_thfle(const double pos)
{
  double Scale  = 11.3895;
  double Offset = 0.0;
  double mu = Scale * pos + Offset;
  return mu;
}

// pos = h_thfle_inv(mu)
double  MiaThfleTransmission::h_thfle_inv(const double mu)
{
  double Scale  = 0.0878;
  double Offset = 0;
  double pos = (double) round( Scale * mu + Offset); // cmd
  return pos;
}

// omega_m = dh(spe)
double  MiaThfleTransmission::dh (const double spe)
{
  double Scale  = 65.4167;
  double Offset = 0;
  double omega_m = Scale * spe + Offset;
  return omega_m;
}

// spe = dh_i_inv(omega_m)
double  MiaThfleTransmission::dh_inv(const double omega_m)
{
  double Scale  = 0.015287;
  double Offset = 0;
  double spe = (double) round( Scale * omega_m + Offset); // cmd
  return spe;
}




//////////////////////////////////////////////////////////////////////////////////////////////
// Macro functions of for the ros trasmission
/////////////////////////////////////////////////////////////////////////////////////////////

void MiaThfleTransmission::actuatorToJointVelocity(const ActuatorData& act_data,
                                                               JointData&    jnt_data)
{

  double spe = *act_data.velocity[0];
  double omega_m = dh(spe);
  double omega_f = omega_m / reduction_;

  *jnt_data.velocity[0] = omega_f;
}

void MiaThfleTransmission::actuatorToJointPosition(const ActuatorData& act_data,
                                                               JointData&    jnt_data)
{
  double pos = *act_data.position[0];
  double mu = h_thfle(pos);
  double fi = mu / reduction_;

  *jnt_data.position[0] = fi;
}


void MiaThfleTransmission::jointToActuatorVelocity(const JointData&    jnt_data,
                                                               ActuatorData& act_data)
{

  double omega_f = *jnt_data.velocity[0];
  double omega_m = omega_f * reduction_;
  double spe = dh_inv(omega_m);

  *act_data.velocity[0] = spe;
}

void MiaThfleTransmission::jointToActuatorPosition(const JointData&    jnt_data,
                                                               ActuatorData& act_data)
{

  double fi = *jnt_data.position[0];
  double mu = fi * reduction_;
  double pos = h_thfle_inv(mu);

  *act_data.position[0] = pos;
}


//////////////////////////////////////////////////////////////////////////////////
// NOT TO BE USED

void MiaThfleTransmission::actuatorToJointEffort(const ActuatorData& act_data,
                                                             JointData&    jnt_data)
{

   *jnt_data.effort[0] = *act_data.effort[0] * 1;
}

void MiaThfleTransmission::jointToActuatorEffort(const JointData&    jnt_data,
                                                             ActuatorData& act_data)
{

   *act_data.effort[0] = *jnt_data.effort[0] / 1;
}

///////////////////////////////////////////////////////////////////////////////////////////

}// namespace mia_transmission_interface
