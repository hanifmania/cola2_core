^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cola2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Captain publishes mission_status message.
* Add enable_trajectory_non_block service.
  Necessary to enable a trajectory from the Vehicle-Interface.
* Modified captain trajectories to be compatible with the ones generaed by the GUI.
* Rename NewGoto.srv to Goto.srv and add keep_position field in srv Goto.srv.
  For the captain to understand that a goto request must never finish
  even if the accuracy has been accepted we use to use the tolerance.
* If tolerance is 0.0 the waypoint never finalizes (timeout >= 3600s).
  However, to avoid problems if someone forgets to add the tolerance
  (by default is 0.0). we have add an extra field to clearly indicate
  if the asked goto must keep the position or not.
* Improve captain console messages.
* Fixed BUG in captain.
  When initial waypoint was not forced to surface and in altitude mode
  the initial waypoint asked was at altitude 0.0.
* If tolerance in goto is 0.0, set timeout at 3600s. (For keep position behaviour)
* Even if the first waypoint is not on surface the vehicle does a goto to it.
  It uses first trajectory element for Z and altitude mode.
* Check goto max distance only if X or Y axis are enabled. Set minimum timeout to 30 seconds.
* Change keep position g500 or s2 by 4dof or 3dof.
* Removed old pilot/captain controllers.
* Remove old map_ack node.
* Use new PRIORITY definitions.
* New Path controller: the surge speed can be defined for each section of
  the path. In case that it is not specified, it would use the one from
  the pilot configuration file.
* Added path controller.
* Publish target goal from pilot and diagnostic from captain.
  Currently, the diagnostic only contains if the vehicle is performing
  a mission or not (to avoid WIFI timeout).
* Add mission types to be used in future mission_manager (GUI improvements).
* captain: added setTrajectory service.
* Modified captain and los_cte to pass velocity and tolerance.
  Velocity for each section and global tolerance.
  If no velocity or tolerance are defined in the mission file
  the ones in los_cte/captain config files are used.
* Add linear and angular velocity in goto service.
  WARNING: despite 6DoF can be configured only surge speed is used.
  The other DoFs are controlled in position and, therefore, is the
  pose controller that decides the maximum speed.
* LOS C++ version modified according to old version in Python. Delta is
  not given as a constant.
* Added heave in 3D, solved yaw problem in trajectory loading and minor improvements.
* Add anchor controller (previous keep_pose_sparus).
  This mode can be used to reach and keep a position
  for a non holonomic vehicle.
* Modified timeout computation.
* Modify pilot param and cfg names to be the same.
* Add dynamic_reconfigure for goto and dubins controllers
* Add dynamic reconfigure for pilot. Only LOS-CTE controller.
* If all joystick pose axes are disabled set priority to 0.
* Solved BUG with altitude control & visualization in both goto and trajectory Controllers.
* Add service "set_joystick_axes_to_velocity" and
  call it from recovery_actions.
* Add enable/disable keep holonomic services.
* Submerge service has more priority than joystick.
* Avoid control in position Z axis beyond 0.0.
* Add submerge service in captain.cpp.
* cola2_lib depends on cola2_msgs and auv_msgs.
* Move getParam method to ros::rosutil library.
* Add enable_trajectory service.
* Start de development of captain.cpp.
* Added the load_trajectory_str and load_trajectory_config. First method load from
  a string with the format of the file and load_trajectory_config load the mision from the rosserver param.
* Add holonomic_goto waypoint controller.
* Read goto controller params from ros param server. 
* Add new GOTO controller.
* Add paramters for LOS CTE in getConfig function.
* Rename Section action to WorldSectionReq and add tolerance to it.
* Add Line Of Sight with Cross Tracking Error Controller.
* Change disable_axis structure and add disable_z field in section.
* Remove marker publication from dubins controller.
* Use PI definition from math.h.
* Solved beta bug in Dubins controller.
* Added dirty section visualization. Bug detected but not solved in Dubins controller
* new captain and pilot node name changed to avoid conflict with old ones
* Improved Dubins controller with lookahead
* Solved important bug in Dubins controller.
* Added new Pilot and Dubins controller.
* Remove extra buttons in teleoperation.
  These functionalities now must be done in XXXX_to_teleoperation nodes.
* Add logitech buttons/axis definitions.
* Add XXXX_to_teleoperation nodes for keyboard and Joy drivers.
* Small modifications on keyboard driver.
* Add CHANGELOG files.
* Change submerge service tolerance from 0.2 to 1.0 in Z axis.
* Solved a bug when waiting in a waypoint in the middle of a mission.
* Added two load trajectories services.
  Added the load_trajectory_str and load_trajectory_config. First method load from
  a string with the format of the file and load_trajectory_config load the mision from the rosserver param.
* Add cola2_control package and libraries cola2_navigation & cola2_control.
* Contributors: CIRS user, Eduard Vidal Garcia, Guillem Vallicrosa, Narcis Palomeras, Narcís Palomeras, juandhv, s2_cmre, sparus.

* Solved a bug when waiting in a waypoint in the middle of a mission.
* Added two load trajectories services.
  Added the load_trajectory_str and load_trajectory_config. First method load from
  a string with the format of the file and load_trajectory_config load the mision from the rosserver param.
* Add cola2_control package and libraries cola2_navigation & cola2_control.
* Contributors: Narcis Palomeras.

