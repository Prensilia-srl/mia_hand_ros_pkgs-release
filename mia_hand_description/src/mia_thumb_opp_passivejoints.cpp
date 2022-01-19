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

#include "mia_hand_description/mia_thumb_opp_passivejoints.h"


namespace mia_hand
{

   /** Initialize*/
    thumb_opp_passive_joint::thumb_opp_passive_joint()
    {

    }

    void thumb_opp_passive_joint::init(const bool LoadURDFInfo_ )
    {
      j_index_name ="j_index_fle";  // default
      j_thumb_name ="j_thumb_opp";  // default
      robot_description_ = "robot_description"; // default

      if(LoadURDFInfo_) { bool temp = LoadURDFInfo();}
      else
      {
        ThMinPos=0;
        ThMaxPos= 0;
      }

    }


    thumb_opp_passive_joint::~thumb_opp_passive_joint() {}

    /** Taking the position of the index_fle joint and returns the position of the thumb_opp joint.*/
     double thumb_opp_passive_joint::GetThumbOppPosition(double j_index_flex_pos)
     {
       // Implementation of g3(delta) according with Mia trasmission and empirical test
       double Empirical_Scale_j_thumb_opp_pos = 0.95238;
       double Empirical_Offset_j_thumb_opp_pos = -0.0052884;
       double jThOpp_Target_position_temp = Empirical_Scale_j_thumb_opp_pos * (j_index_flex_pos) + Empirical_Offset_j_thumb_opp_pos;
       double jThOpp_Target_position;

       if (jThOpp_Target_position_temp <= ThMinPos)
       	 jThOpp_Target_position = ThMinPos + 0.02;

       else if (jThOpp_Target_position_temp >= ThMaxPos)
       	jThOpp_Target_position = ThMaxPos - 0.02;

       else
       	jThOpp_Target_position = jThOpp_Target_position_temp;

       return jThOpp_Target_position;
     }


     /** Load joint limits from URD */
     bool thumb_opp_passive_joint::LoadURDFInfo()
     {
       bool result = false;

       const std::string urdf_string = getURDF(robot_description_);

        // get URDF model
        urdf::Model urdf_model;
        const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

       const urdf::JointConstSharedPtr urdf_joint = urdf_model_ptr->getJoint(j_thumb_name);

       joint_limits_interface::JointLimits limits;

        if (urdf_joint != NULL)
        {

             // Get limits from the URDF file.
             if (joint_limits_interface::getJointLimits(urdf_joint, limits))
             {

               ThMinPos = limits.min_position;
               ThMaxPos = limits.max_position;

               result = true;
             }
       }
       else
       {
         ThMinPos=0;
         ThMaxPos= 0;
         result = false;
       }

      return result;
     }

     /** Find the URDF parameter name */
     std::string thumb_opp_passive_joint::getURDF(std::string param_name) const
    {
        std::string urdf_string;

        // search and wait for robot_description on param server
        while (urdf_string.empty())
        {
           std::string search_param_name;

           if (n.searchParam(param_name, search_param_name))
           {
                ROS_INFO_ONCE_NAMED("remap_mia_joint_states", "node is waiting for model"
               " URDF in parameter  on the ROS param server.");

                n.getParam(search_param_name, urdf_string);
           }
           else
           {
                ROS_INFO_ONCE_NAMED("remap_mia_joint_states", "Node  is waiting for model"
           	" URDF in parameter on the ROS param server.");

                n.getParam(param_name, urdf_string);
           }

        usleep(100000);
        }
        ROS_DEBUG_STREAM_NAMED("remap_mia_joint_states", "Recieved urdf from param server, parsing...");
	return urdf_string;
    }





}  // namespace
