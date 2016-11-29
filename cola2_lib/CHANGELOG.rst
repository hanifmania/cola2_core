^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cola2_lib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add resetLandmarks method.
* Increase maximum delay per update from 0.1 to 0.15 seconds.
* The bag record is generic for both AUVs.
* Add node that allows start/stop a launch file for recording bags.
* Use new PRIORITY definitions.
* Added missing dependency for kinetic.
* In function loadParam clear STD vectors before add data.
* Solved problem in LOS_CTE when user surge was smaller than min_surve_velocity param.
* Change MAX-COV to accept a a new landmark
  From 0.0025 (~5cm) to 0.025 (~15cm).
* Add Functions to plot range updates
* Modified EkfBase and EkfSlamAUV to accept rangeUpdates.
* Add range detection msg and AddLandmark srv.
* ekfslamauv: added different returns for landmarks to do logging in ros.
* ekfrosbase: added usbl_ned publisher.
* cola2_lib depends on cola2_msgs and auv_msgs.
* Add load_vector and getParam template functions.
* cola2_lib: now is a compiled library.
* Remove refenreces to MORPH.
* getRPY: now really gets the RPY angle and not the YPR.
* Add declination computation in EKF SLAM AUV lib.
* Update buttons definition.
* Add JoystickBase class.
* Add CHANGELOG files.
* Contributors: Eduard Vidal Garcia, Guillem Vallicrosa, Narcis Palomeras, Narcís Palomeras, juandhv, sparus


* Add SGDerivation.hpp library
* Disable MORPH controller
* Forgotten var definition
* Add cola2 control and navigation libraries
* Replace cola2_safety/DigitalOutput msg by cola2_msgs/DigitalOutput one.
* Initial commit
* Contributors: Narcis Palomeras
