/*
* Copyright (C) 2021 Prensilia s.r.l.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "mia_hand_ros_control/mia_mrl_transmission.h"
#include <math.h>
#include <ros/console.h>

using transmission_interface::ActuatorData;
using transmission_interface::JointData;

namespace transmission_interface
{

//////////////////////////////////////////////////////////////////////////////////////////////
//MiaMrlTransmission : constructor
//////////////////////////////////////////////////////////////////////////////////////////////

 MiaMrlTransmission::MiaMrlTransmission()
  : Transmission()
{

   reduction_ = 64.0 * 20.0;


  if (0.0 == reduction_)
  {
    throw TransmissionInterfaceException("Transmission reduction ratio cannot be zero.");
  }
}


 //////////////////////////////////////////////////////////////////////////////////////////////
// Simple functions needed to evaluate trasmission ratio
/////////////////////////////////////////////////////////////////////////////////////////////

// mu = h_mrl(pos)
inline double  MiaMrlTransmission::h_mrl(const double pos)
{
  double Scale  = 7.0074;
  double Offset = 0.0;
  double mu = Scale * pos + Offset;
  return mu;
}

// pos = h_mrl_inv(mu)
double  MiaMrlTransmission::h_mrl_inv(const double mu)
{
  double Scale  = 0.14271;
  double Offset = 0;
  double pos = (double) round( Scale * mu + Offset); // cmd
  return pos;
}

// omega_m = dh(spe)
double  MiaMrlTransmission::dh (const double spe)
{
  double Scale  = 65.4167;
  double Offset = 0;
  double omega_m = Scale * spe + Offset;
  return omega_m;
}

// spe = dh_i_inv(omega_m)
double  MiaMrlTransmission::dh_inv(const double omega_m)
{
  double Scale  = 0.015287;
  double Offset = 0;
  double spe = (double) round( Scale * omega_m + Offset); // cmd
  return spe;
}




//////////////////////////////////////////////////////////////////////////////////////////////
// Macro functions of for the ros trasmission
/////////////////////////////////////////////////////////////////////////////////////////////

void MiaMrlTransmission::actuatorToJointVelocity(const ActuatorData& act_data,
                                                               JointData&    jnt_data)
{

  double spe = *act_data.velocity[0];
  double omega_m = dh(spe);
  double omega_f = omega_m / reduction_;

  *jnt_data.velocity[0] = omega_f;
}

void MiaMrlTransmission::actuatorToJointPosition(const ActuatorData& act_data,
                                                               JointData&    jnt_data)
{
  double pos = *act_data.position[0];
  double mu = h_mrl(pos);
  double fi = mu / reduction_;

  *jnt_data.position[0] = fi;
}


void MiaMrlTransmission::jointToActuatorVelocity(const JointData&    jnt_data,
                                                               ActuatorData& act_data)
{

  double omega_f = *jnt_data.velocity[0];
  double omega_m = omega_f * reduction_;
  double spe = dh_inv(omega_m);

  *act_data.velocity[0] = spe;
}

void MiaMrlTransmission::jointToActuatorPosition(const JointData&    jnt_data,
                                                               ActuatorData& act_data)
{

  double fi = *jnt_data.position[0];
  double mu = fi * reduction_;
  double pos = h_mrl_inv(mu);

  *act_data.position[0] = pos;
}


//////////////////////////////////////////////////////////////////////////////////
// NOT TO BE USED

void MiaMrlTransmission::actuatorToJointEffort(const ActuatorData& act_data,
                                                             JointData&    jnt_data)
{

   *jnt_data.effort[0] = *act_data.effort[0] * 1;
}

void MiaMrlTransmission::jointToActuatorEffort(const JointData&    jnt_data,
                                                             ActuatorData& act_data)
{

   *act_data.effort[0] = *jnt_data.effort[0] / 1;
}

///////////////////////////////////////////////////////////////////////////////////////////

}// namespace mia_transmission_interface
