#!/usr/bin/env python

import rospy
from cola2_msgs.srv import Action, ActionResponse
from std_srvs.srv import Empty, EmptyRequest


class MissionWait():
    """Create a service to perform wait mission."""

    def __init__(self):
        """Constructor."""
        # Service clients
        rospy.wait_for_service('/cola2_control/load_trajectory')
        self.load_trajectory_srv = rospy.ServiceProxy(
            '/cola2_control/load_trajectory', Empty)

        rospy.wait_for_service('/cola2_control/enable_trajectory')
        self.enable_trajectory_srv = rospy.ServiceProxy(
            '/cola2_control/enable_trajectory', Empty)

        # Create service
        self.enable_mission_wait = rospy.Service('/enable_mission_wait',
                                                 Action,
                                                 self.enable_mission_wait)

        rospy.spin()

    def enable_mission_wait(self, req):
        """Create mission, load it and execute it."""
        if len(req.param) != 5:
            rospy.logwarn("WARNING: mission_wait needs 5 params (lat, lon, z, altitude_mode, time_s)")
            return ActionResponse()

        latitude = [float(req.param[0]),
                    float(req.param[0]),
                    float(req.param[0])]
        longitude = [float(req.param[1]),
                     float(req.param[1]),
                     float(req.param[1])]
        z = [0.0, float(req.param[2]), 0.0]
        wait = [0.0, float(req.param[4]), 0.0]
        altitude_mode = True
        if req.param[3].lower() == 'false':
            altitude_mode = False
        altitude_mode = [False, altitude_mode, False]
        mode = 'los_cte'
        timeout = float(req.param[4])/60.0 + 10
        tolerance = [2.0, 2.0, 1.0, 0.5, 0.5, 0.3]
        force_initial_final_waypoints_at_surface = False

        rospy.loginfo("Load trajectory wait params to param server.")
        rospy.set_param('/trajectory/latitude', latitude)
        rospy.set_param('/trajectory/longitude', longitude)
        rospy.set_param('/trajectory/z', z)
        rospy.set_param('/trajectory/wait', wait)
        rospy.set_param('/trajectory/altitude_mode', altitude_mode)
        rospy.set_param('/trajectory/mode', mode)
        rospy.set_param('/trajectory/timeout', timeout)
        rospy.set_param('/trajectory/tolerance', tolerance)
        rospy.set_param('/trajectory/force_initial_final_waypoints_at_surface',
                        force_initial_final_waypoints_at_surface)

        rospy.loginfo("Load trajectory to captain and execute it.")
        try:
            self.load_trajectory_srv(EmptyRequest())
            rospy.sleep(1.0)
            self.enable_trajectory_srv(EmptyRequest())
        except rospy.ServiceException as exc:
            rospy.logerror(
                "Service Load/Enable trajectory did not process request: "
                + str(exc))

        return ActionResponse()

if __name__ == '__main__':
    """ Main function. """
    rospy.init_node('mission_wait')
    mw = MissionWait()
