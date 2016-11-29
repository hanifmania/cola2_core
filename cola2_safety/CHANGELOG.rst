^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cola2_safety
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* If min_altitude is on the AUV can be still controlled in Surge, Sway and YAW
  WARNING: If with the joystick a Heave command is send then the whole
  request (Surge, Sway and Yaw) is ignored.
* Use new PRIORITY definitions.
* Add service "set_joystick_axes_to_velocity" and
  call it from recovery_actions.
* param_logger added to save all params in the bagfiles.
* Add CHANGELOG files.
* Solve error in loginfo message.
* Add rule to check DVL bad data (to be checked).
* Remove old MORPH rule.
* Add cola2_safety package.
* Contributors: Guillem Vallicrosa, Narcis Palomeras, Narcís Palomeras, sparus.

* Add rule to check DVL bad data (to be checked)
* Remove old MORPH rule
* Add cola2_safety package
* Contributors: Narcis Palomeras
