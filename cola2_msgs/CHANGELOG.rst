^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cola2_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add Float service to send a float value using service/client
* Captain publishes mission_status message
* Rename NewGoto.srv to Goto.srv and old Goto.srv to OldGoto.srv
* New Path controller: the surge speed can be defined for each section of
  the path. In case that it is not specified, it would use the one from
  the pilot configuration file.
* Added path controller
* Modified basic cfg value
* Add action service for generic action calls (GUI improvements)
* Corrected SetTrajectory service
* Added setTrajectory service
* Modified RangeDetection message
* Added service GetLandmark
* Add linear and angular velocity in goto service.
  WARNING: despite 6DoF can be configured only surge speed is used.
  The other DoFs are controlled in position and, therefore, is the
  pose controller that decides the maximum speed.
* Add range detection msg and AddLandmark srv
* Add priority in goto service
* altitude_mode must be a boolean
* Add enable_trajectory service and other minor things
* Start de development of captain.cpp
* Add holonomic_goto waypoint controller
* Add new GOTO controller.
* Rename Section action to WorldSectionReq and add tolerance to it.
* Add Line Of Sight with Cross Tracking Error Controller.
* Add action message and structure definition
* Added new Pilot and Dubins controller messages
* Add Detection message to be used by template_detection and others in the future.
* Add CHANGELOG files
* Add multibeam srv
* Eduard modifications after incident with S2 Adis IMU in Cartagena
* Messages are now defined in cola2_msgs
* Add msg and srv for the Soundmetrics Aris 3000 driver
* Add Set rotative payload angle service for s2_rotative_payload
* Add Pan Tilt Angle service
* Add SeaEyeData msg
* Includes G500 new messages
* Add Map and Landmark msgs
* Add messages for cola2_control, cola2_navigation & cola2_control
* Move cfg files to cola2_msgs
* Add messages for cola2_safety package
* Replace DigitalOutput msg by DigitalOutput srv. Add EmusBms msg.
* Add TelegyneExplorerDvl msg
* Add initial messages
* Contributors: CIRS user, Eduard Vidal Garcia, Guillem Vallicrosa, Narcis Palomeras, Narcís Palomeras, juandhv, sparus

* Add msg and srv for the Soundmetrics Aris 3000 driver
* Add Set rotative payload angle service for s2_rotative_payload
* Add Pan Tilt Angle service
* Add SeaEyeData msg
* Includes G500 new messages
* Add Map and Landmark msgs
* Add messages for cola2_control, cola2_navigation & cola2_control
* Move cfg files to cola2_msgs
* Add messages for cola2_safety package
* Replace DigitalOutput msg by DigitalOutput srv. Add EmusBms msg.
* Add TelegyneExplorerDvl msg
* Add initial messages
* Initial commit
* Contributors: Narcis Palomeras, sparus
