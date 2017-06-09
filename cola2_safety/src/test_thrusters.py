#!/usr/bin/env python
# ROS imports
import rospy
from std_srvs.srv import Empty, EmptyRequest
from cola2_msgs.msg import ThrustersData
from cola2_msgs.srv import Action

"""@@Test thrusters from an Action service.@@"""
"""
Created on Jan 2017
@author: narcis palomeras
"""


class TestThrusters:
    """Test thrusters from an Action service."""

    def __init__(self):
        """Constructor."""
        rospy.init_node('test_thrusters')

        self.name = rospy.get_name()

        # Create publisher
        self.pub_thrusters_data = rospy.Publisher(
            "/cola2_control/thrusters_data",
            ThrustersData, queue_size=1)

        # Create client to services:
        # ... enable thrusters
        rospy.wait_for_service(
            '/cola2_control/enable_thrusters', 10)
        try:
            self.enable_thrusters = rospy.ServiceProxy(
                '/cola2_control/enable_thrusters', Empty)
        except rospy.ServiceException, e:
            rospy.logwarn("%s: Service call failed: %s", self.name, e)

        # ... disable thrusters
        rospy.wait_for_service(
            '/cola2_control/disable_thrusters', 10)
        try:
            self.disable_thrusters = rospy.ServiceProxy(
                '/cola2_control/disable_thrusters', Empty)
        except rospy.ServiceException, e:
            rospy.logwarn("%s: Service call failed: %s", self.name, e)

        # Create test service
        self.test_srv = rospy.Service('/cola2_safety/test_thrusters',
                                      Action,
                                      self.test)
        rospy.spin()

    def test(self, req):
        """Test service."""
        self.enable_thrusters(EmptyRequest())

        data = ThrustersData()
        for p in req.param:
            data.setpoints.append(float(p))

        rospy.loginfo(self.name + ": test thrusters with " +
                      str(data.setpoints) + "\n")
        rate = rospy.Rate(10)
        for i in range(30):
            data.header.stamp = rospy.Time.now()
            self.pub_thrusters_data.publish(data)
            rate.sleep()

        self.disable_thrusters(EmptyRequest())

if __name__ == '__main__':
    try:
        TT = TestThrusters()
    except rospy.ROSInterruptException:
        pass
