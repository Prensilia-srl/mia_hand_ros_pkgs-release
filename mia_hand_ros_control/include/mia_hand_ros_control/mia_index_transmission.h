/*********************************************************************************************************
 Author: Prensilia srl
 Desc:   Transmission of the MIA index finger

   version 1.0
**********************************************************************************************************/


#ifndef MIA_INDEX_TRANSMISSION_H
#define MIA_INDEX_TRANSMISSION_H

#include <iostream>
#include <vector>
#include <cassert>
#include <string>
#include <cmath>
#include <ros/ros.h>

#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_interface_exception.h>


namespace transmission_interface
{

/**
  * Implementation of the no-linear Mia Index Transmission.
  */
  class MiaIndexTransmission: public Transmission
  {

    public:

    /**
      * Class constructor.
      */
      MiaIndexTransmission ();

    /**
      * Do not use this methos since Mia hand has not effort control.
      */
      void actuatorToJointEffort(
        const transmission_interface::ActuatorData& ind_act_state,
              transmission_interface::JointData&    ind_jnt_data) override;

    /**
      * Transform position variables from actuator to joint space.
      * @param ind_act_state index actuator actual state.
      * @param ind_jnt_data returned index joint state.
      */
      void actuatorToJointPosition(
        const transmission_interface::ActuatorData& ind_act_state,
              transmission_interface::JointData&    ind_jnt_data) override;

    /**
      * Transform velocity variables from actuator to joint space.
      * @param ind_act_state index actuator actual state.
      * @param ind_jnt_data returned index joint joint.
      */
      void actuatorToJointVelocity(
        const transmission_interface::ActuatorData& ind_act_state,
              transmission_interface::JointData&    ind_jnt_data) override;

    /**
      * Do not use this methos since Mia hand has not effort control.
      */
      void jointToActuatorEffort(
        const transmission_interface::JointData&    ind_jnt_data,
            transmission_interface::ActuatorData& ind_act_cmd) override;

    /**
      * Transform position variables from joint to actuator space.
      * @param ind_jnt_data index joint target.
      * @param ind_act_cmd returned index actuator target.
      */
      void jointToActuatorPosition(
        const transmission_interface::JointData&    ind_jnt_data,
              transmission_interface::ActuatorData& ind_act_cmd) override;

    /**
      * unused
      */
      void jointToActuatorVelocity(
        const transmission_interface::JointData&    ind_jnt_data,
              transmission_interface::ActuatorData& act_data) override; // unused

    /**
      * Transform velocity variables from joint to actuator space.
      * @param ind_jnt_data index joint target.
      * @param ind_act_state index actuator
      * @param ind_act_cmd returned actuator joint target.
      */
      void IndexjointToActuatorVelocity(
        const transmission_interface::JointData&    ind_jnt_data,
  	  const transmission_interface::ActuatorData& ind_act_state,
              transmission_interface::ActuatorData& ind_act_cmd) ;

  /**
    * Index transmission first step function for pose: mu = h_i(pos).
    * @param pos pose in the ros actuator-space (i.e. as returned by Mia hand [-255; +255]).
    * @return mu: pose in the mia actuator space.
    */
  	double h_i(const double pos);            // mu = h_i(pos)

  /**
    * Index transmission first step inverse function for pose: pos = h_i_inv(mu)
    * @param mu pose in the mia actuator space
    * @return pos: pose in the ros actuator-space (i.e. as returned by Mia hand [-255; +255]).
    */
  	double h_i_inv(const double mu);         // pos = h_i_inv(mu)

  /**
    * Index transmission first step function for velocity: omega_m = dh_i(spe).
    * @param spe velocity in the ros actuator-space (i.e. as returned by Mia hand [-90; +90]).
    * @return omega_m: velocity in the mia actuator space.
    */
  	double dh_i(const double spe);           // omega_m = dh_i(spe)

  /**
    * Index transmission first step inverse function for velocity: spe = dh_i_inv(omega_m)
    * @param spe velocity in the ros actuator-space (i.e. as returned by Mia hand [-90; +90]).
    * @return omega_m: velocity in the mia actuator space.
    */
  	double dh_i_inv(const double omega_m);  // spe = dh_i_inv(omega_m)

  /**
    * Index transmission second step function for position: delta = f(alpha)
    * @param alpha position of the intermediate step of the transmission.
    * @return delta: position in the mia joint space.
    */
  	double f(const double alpha );         // delta = f(alpha)

  /**
    * Index transmission second step inverse function for position: alpha = f_inv(delta)
    * @param delta position in the mia joint space.
    * @return delta: alpha: position of the intermediate step of the transmission.
    */
  	double f_inv(const double delta );     // alpha = f_inv(delta)

  /**
    * Index transmission second step function for velocity: alpha = f_inv(delta)
    * @param omega_a velocity f the intermediate step of the transmission.
    * @return omega_d: velocity in the mia joint space.
    */
  	double df(const double omega_a );     // // Non linear TR-1

      // General trasmissions attributes
      std::size_t numActuators() const {return 1;} //!< Number of actuators of the tranmission.
      std::size_t numJoints() const {return 1;} //!< Number of joints of the tranmission.

    private:
      double linear_reduction_;

    /**
      * Upper limiits of the inputs intervals of the f funtion.
      * @see f().
      */
      std::vector<double> x_intervals_f;

    /**
      * Upper limiits of the inputs intervals of the f_inv funtion.
      * @see f_inv().
      */
      std::vector<double> x_intervals_f_inv;

    /**
      * Upper limiits of the inputs intervals of the df funtion.
      * @see df().
      */
      std::vector<double> x_intervals_df;

    /**
      * Scales factors of the f funtion.
      * @see f().
      */
      std::vector<double> f_scale;

    /**
      * Offset factors of the f funtion.
      * @see f().
      */
      std::vector<double> f_offset;

    /**
      * Scale factors of the f_inv funtion.
      * @see f_inv().
      */
      std::vector<double> f_inv_scale;

    /**
      * Offset factors of the f_inv funtion.
      * @see f_inv().
      */
      std::vector<double> f_inv_offset;

    /**
      * Scale factors of the df funtion.
      * @see df().
      */
      std::vector<double> df_scale;

    /**
      * Offset factors of the df funtion.
      * @see df().
      */
      std::vector<double> df_offset;


  };



}  // namespace

#endif  //MIA_INDEX_TRANSMISSION_H
