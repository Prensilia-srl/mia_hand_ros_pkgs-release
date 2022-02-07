# mia_hand_ros_control
Package for controlling the real Mia Hand within ROS Control.
The cpp class MiaHWInterface inherits from public hardware_interface::RobotHW and allows to control the Mia Hand coupled fingers using the ROS controllers.
In this package a specific transmission class has been declared for each of the three actuators of the Mia hand due to their non linear and peculiar behaviours.

To launch the real mia hand it is necessary to launch the Mia_hw_node that implements the actual control loop.
To do that one of the following pre-tested commands can be used:

  - to launch the control loop (frequency 100Hz) with one controller (position controller as default) for each DoF of the Mia Hand, attaching the Mia hand at COM 0 (default):

        roslaunch mia_hand_bringup mia_hand_hw.launch

      Other inputs arguments of the above launch file are:

          Mia_COM_:               default='0'            doc="Int number of the Mia hand COM port"
          Mia_fs_:                default='100'          doc="Hz, frequency of the control loop of the Mia Hw interface"
          controller_thumb_fle    default="position"     doc="type of feed forward controller for thumb flexion. Values: position, velocity"
          controller_index_fle    default="position"     doc="type of feed forward controller for indexe flexion. Values: position, velocity"
          controller_mrl_fle      default="position"     doc="type of feed forward controller for mrl flexion. Values: position, velocity"
          robotNamespace          default="mia_hand_hw"  doc="Namespace of the robot"

  - to launch the control loop (frequency 100Hz) with a traj controller (velocity traj controller as default) that control all the DoF of the Mia Hand, attaching the Mia hand at COM 0 (default):

        roslaunch mia_hand_bringup mia_hand_hw_traj.launch

      Other inputs arguments of the above launch file are:

          Mia_COM_:               default='0'            doc="Int number of the Mia hand COM port"
          Mia_fs_:                default='100'          doc="Hz, frequency of the control loop of the Mia Hw interface"
          tc_type:                default="vel"          doc="type of trajectory controller to launch. Values: vel, pos"
          robotNamespace          default="mia_hand_hw"  doc="Namespace of the robot"

      NB: to use this command it may be needed to manually install the ros pkg rqt_joint_trajectory_controller. The gains of the velocity trajectory controllers have been tuned with the Speed Scale = 100%. 

  - to launch the control loop (frequency 100Hz) with a traj controller (velocity traj controller as default) and the RVIZ Moveit GUI, attaching the Mia hand at COM 0 (default):

        roslaunch mia_hand_bringup mia_hand_hw_moveit.launch

      Other inputs arguments of the above launch file are:

          Mia_COM_:               default='0'            doc="Int number of the Mia hand COM port"
          Mia_fs_:                default='100'          doc="Hz, frequency of the control loop of the Mia Hw interface"
          tc_type:                default="vel"          doc="type of trajectory controller to launch. Values: vel, pos"
          robotNamespace          default="mia_hand_hw"  doc="Namespace of the robot"

      NB: to run this command the mia_hand_moveit_config package is needed.
      When the RVIz turns on it is possible to use the moveit plug-in to control the mia_hand subgroup chains (thumb_flexion, index_flexion and mrl_flexion).
      For the index two different links will be displayed (if the option to visualize the moveit goal position is selected): one index link displays the signed
      target position (for exaple index in position -1.2 rad), the other index link displays the actual configuration of the index ( so after having reached the target
      position of -1.2 rad this index link will be at pos 1.2 rad flexed and the thumb will be adducted).
      Please be sure to select the context OMPL to move the Mia hand fingers (the other contexts have not been tested).
