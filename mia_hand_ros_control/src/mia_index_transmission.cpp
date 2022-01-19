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

#include "mia_hand_ros_control/mia_index_transmission.h"
#include <math.h>

using transmission_interface::ActuatorData;
using transmission_interface::JointData;

namespace transmission_interface
{

  //////////////////////////////////////////////////////////////////////////////////////////////
  // Initialize broken lines fucntons imlementing mia index transmisiion
  /////////////////////////////////////////////////////////////////////////////////////////////
  MiaIndexTransmission::MiaIndexTransmission ()
   : Transmission()
  {
    // linear trasmission section
    linear_reduction_ = 64.0 * 20.0; // TR of the linear segment of the trasmission

    // non linear trasmission section
    x_intervals_f     = {-0.85, -0.60, -0.28, -0.09,  0.04,  0.17,  0.58, 0.88, 1.16, 1.49, 1.88, 2.20}; // N = 12
    x_intervals_f_inv = {-1.07, -0.70, -0.38, -0.17, -0.05, -0.01,  0.00, 0.01, 0.05, 0.15, 0.33, 0.80, 1.16, 1.33 ,1.5}; //N = 15
    x_intervals_df    = {-1.15, -0.99, -0.50, -0.38, -0.29, -0.19, -0.02, 0.09, 0.34, 0.50, 0.64, 0.82, 1.04, 1.30, 1.78 ,2.0 ,2.11, 2.20}; // N = 18

    // f functions
    f_scale       = {-0.782148766201303, -1.03058374167863 , -1.22368755900872 , -1.16847142154607 , -0.911231289024396, -0.601663029243979, -0.197880519756952,
                     -0.520051295462516, -0.719741468813803, -0.839379324439599, -0.942927436665641, -1.08442452979925};

    f_offset      = {0.439866538741061, 0.227645536860170, 0.111052936781279, 0.126181750141350, 0.150182742690696, 0.138771586278833, 0.0711398396794130,
                     0.256841159134046, 0.431658128795775, 0.569988622722396, 0.723948798521761, 0.989586451199345};

    // f inverse functions
    f_inv_scale   = {-0.917582779538605, -1.05391335884287, -1.19433447974907, -1.431898346737150, -1.964168793593670, -3.38610487911141, -10.9773764088800,
                     -8.898456050626390, -2.86762990339221, -1.47787267188521, -0.974112549912150, -0.813805489118257, -0.978063597388615, -1.23941256214315, -1.42940996863406};

    f_inv_offset  = {0.9188279760227930, 0.773465321854399,  0.675795923603619, 0.586185379576233,  0.496912516702108,  0.424335359750378,  0.351597222222222, 0.342953500623609,
                     0.283965570504740,  0.217012833101473,  0.141950288885497, 0.0903279294135654, 0.222664061454050,  0.525807921540722,  0.778435404235634};

   // df function
     df_scale     = {-0.502124686464031, -0.617569286637807, -0.786646228209404, -0.443887847250763, 0.0318882991362680, 0.642623008053360, 1.61741355519959,
                      2.37840936587392 ,  2.42982807851509 ,  1.82554337201208,   1.32452347512971 , 0.923449189994699 , 0.601344310816028, 0.389840076159333,
                      0.284011196830680,  0.385466139570889,  0.637843666986597,  1.01279322128042};

     df_offset    = {-1.29909527663917 , -1.432135538436430, -1.59994184747328 , -1.42755998530929, -1.24514798338551 ,-1.06634278699696, -0.879054630057223,
                     -0.859652872953407, -0.862880586642302, -0.659622532929257, -0.411137153002571,-0.155795702043499, 0.107118528620613, 0.326212634836997,
                     0.463277504816079,   0.282761648984580, -0.221130296730254, -1.01117012203451};

  }


  //////////////////////////////////////////////////////////////////////////////////////////////
  // Simple functions needed to evaluate trasmission ratio
  /////////////////////////////////////////////////////////////////////////////////////////////

  // mu = h_i(pos)
  double  MiaIndexTransmission::h_i(const double pos)
  {
    double Scale  = -8.673;
    double Offset = 580.8284;
    double mu = Scale * pos + Offset;
    return mu;
  }

  // pos = h_i_inv(mu)
  double  MiaIndexTransmission::h_i_inv(const double mu)
  {
    double Scale  = -0.1153;
    double Offset = 66.9697;
    double pos = (double) round( Scale * mu + Offset); // cmd
    return pos;
  }

  // omega_m = dh_i(spe)
  double  MiaIndexTransmission::dh_i(const double spe)
  {
    double Scale  = 65.4167;
    double Offset = 0;
    double omega_m = Scale * spe + Offset;
    return omega_m;
  }

  // spe = dh_i_inv(omega_m)
  double  MiaIndexTransmission::dh_i_inv(const double omega_m)
  {
    double Scale  = 0.015287;
    double Offset = 0;
    double spe = (double) round( Scale * omega_m + Offset); // cmd
    return spe;
  }


  // delta = f(alpha)
  double  MiaIndexTransmission::f(const double alpha )
  {

    double delta;

    // input larger than the last x interval limit
    if (alpha > x_intervals_f[x_intervals_f.size() - 1 ])
    {
      delta = f_scale[x_intervals_f.size() - 1] * alpha + f_offset[x_intervals_f.size() - 1];
    }
    else
    {

      for(unsigned int j=0; j < x_intervals_f.size() ; j++)
      {

        // First interval
        if (j == 0)
        {
          if (alpha <= x_intervals_f[j])
          {
            delta = f_scale[j] * alpha + f_offset[j];
            break;
          }
        }
        else // Other intervals
        {
          if (alpha > x_intervals_f[j-1] && alpha <= x_intervals_f[j])
          {
            delta = f_scale[j] * alpha + f_offset[j];
            break;
          }
        }
      }

    }
    return delta;


  }


  // alpha = f_inv(delta)
  double  MiaIndexTransmission::f_inv( const double delta)
  {

    double alpha;

    // input larger than the last x interval limit
    if (delta > x_intervals_f_inv[x_intervals_f_inv.size() - 1 ])
    {
      alpha = f_inv_scale[x_intervals_f_inv.size() - 1] * delta + f_inv_offset[x_intervals_f_inv.size() - 1];
    }
    else
    {

      for(unsigned int j=0; j < x_intervals_f_inv.size() ; j++)
      {

        // First interval
        if (j == 0)
        {
          if (delta <= x_intervals_f_inv[j])
          {
            alpha = f_inv_scale[j] * delta + f_inv_offset[j];
            break;
          }
        }
        else // Other intervals
        {
          if (delta > x_intervals_f_inv[j-1] && delta <= x_intervals_f_inv[j])
          {
            alpha = f_inv_scale[j] * delta + f_inv_offset[j];
            break;
          }
        }
      }

    }
    return alpha;

  }


  // Non linear TR-1
  double  MiaIndexTransmission::df( const double alpha )
  {

    double f_alpha;

    // input larger than the last x interval limit
    if (alpha > x_intervals_df[x_intervals_df.size() - 1 ])
    {
      f_alpha = df_scale[x_intervals_df.size() - 1] * alpha + df_offset[x_intervals_df.size() - 1];
    }
    else
    {

      for(unsigned int j=0; j < x_intervals_df.size() ; j++)
      {

        // First interval
        if (j == 0)
        {
          if (alpha <= x_intervals_df[j])
          {
            f_alpha = df_scale[j] * alpha + df_offset[j];
            break;
          }
        }
        else // Other intervals
        {
          if (alpha > x_intervals_df[j-1] && alpha <= x_intervals_df[j])
          {
            f_alpha = df_scale[j] * alpha + df_offset[j];
            break;
          }
        }
      }

    }

    if (0.0 == f_alpha)
    {
  	  f_alpha = 0.0051;
    }

    return abs(f_alpha);


  }



  //////////////////////////////////////////////////////////////////////////////////////////////
  // Macro functions of for the ros trasmission
  /////////////////////////////////////////////////////////////////////////////////////////////


  // Mapping from data pos returned by MIA to joint angle.
  void MiaIndexTransmission::actuatorToJointPosition(
    const ActuatorData &ind_act_state,
          JointData    &ind_jnt_data)
  {

    double pos = *ind_act_state.position[0]; // -255 ; 255 (discreto)
    double mu = h_i(pos);
    double alpha = mu / linear_reduction_;
    double delta = f(alpha);

    *ind_jnt_data.position[0] = delta;

    return;
  }

  // Mapping from data spe returned by MIA to joint velocity.
  void MiaIndexTransmission::actuatorToJointVelocity(
    const ActuatorData &ind_act_state,
          JointData    &ind_jnt_data)
  {

    double spe = *ind_act_state.velocity[0];  // discreto (-90 ; +90)
    double omega_m = dh_i(spe);
    double omega_a = omega_m / linear_reduction_;

     double pos = *ind_act_state.position[0]; // -255 ; 255 (discreto)
     double mu = h_i(pos);
     double alpha = mu / linear_reduction_;

     double df_alpha = df(alpha); // non linear reduction ratio inv
     double omega_d = omega_a * df_alpha;

    // ROS_INFO_STREAM("alpha: "<< alpha<< " | df_alpha " << df_alpha <<  " | omega_d " << omega_d);

    *ind_jnt_data.velocity[0] = omega_d;

    return;
  }

  // Mapping from joint position to Mia pos command.
  void MiaIndexTransmission::jointToActuatorPosition(
    const JointData    &ind_jnt_data,
          ActuatorData &ind_act_cmd)
  {

    double delta = *ind_jnt_data.position[0];
    double alpha = f_inv(delta);
    double mu = alpha * linear_reduction_;
    double pos = h_i_inv(mu);                // discreto (-255 ; 255)


    *ind_act_cmd.position[0] = pos;

    return;
  }

  // Mapping from joint velocity to Mia spe command.
  void MiaIndexTransmission::IndexjointToActuatorVelocity(
    const JointData    &ind_jnt_data,
    const ActuatorData &ind_act_state,
          ActuatorData &ind_act_cmd)
  {
    double omega_d = *ind_jnt_data.velocity[0] ;

    double pos = *ind_act_state.position[0]; // -255 ; 255 (discreto)
    double mu = h_i(pos);
    double alpha = mu / linear_reduction_;

    double df_alpha = df(alpha); // non linear reduction ratio inv
    double omega_a = omega_d/df_alpha;
    double omega_m = omega_a * linear_reduction_;
    double spe = dh_i_inv(omega_m);

    // ROS_INFO_STREAM("alpha: "<< alpha<< " | df_alpha " << df_alpha <<  " | omega_d " << omega_d << " | omega_m " << omega_m);

    *ind_act_cmd.velocity[0] = spe;

    return;
  }

  // NOT USED
  void MiaIndexTransmission::jointToActuatorVelocity(
        const transmission_interface::JointData&    ind_jnt_data,
              transmission_interface::ActuatorData& act_data)
  {
     *act_data.velocity[0] = *ind_jnt_data.velocity[0] * 1;
  }



  //////////////////////////////////////////////////////////////////////////////////////////////
  // NOT IMPLEMENTED FUNCTIONS :since these cannot be used with MIA
  /////////////////////////////////////////////////////////////////////////////////////////////

  void MiaIndexTransmission::actuatorToJointEffort(const ActuatorData &ind_act_state,
                                                    JointData    &ind_jnt_data)
  {
    /* TODO: replace this calculation with the mapping from motor to joint
     * effort.
     */
    *ind_jnt_data.effort[0] = *ind_act_state.effort[0] * 1;

    return;
  }

  // NOT IMPLEMENTED:
  void MiaIndexTransmission::jointToActuatorEffort(
    const JointData    &ind_jnt_data,
          ActuatorData &ind_act_cmd)
  {
    /* TODO: replace this calculation with the mapping from joint to motor
     * effort.
     */
    *ind_act_cmd.effort[0] = *ind_jnt_data.effort[0] / 1;

    return;
  }

/*-------------------------------------------------------------------------------------*/

}  // namespace
