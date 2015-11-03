#!/usr/bin/env python

"""
Created on Thu Mar 21 2013

@author: narcis palomeras
"""

import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from diagnostic_msgs.msg import DiagnosticArray

class DiagnosticHelper(object):
    def __init__(self, name, hw_id, topic='/diagnostics', desired_freq = 1000, max_freq_error = 10):
        self.pub_diagnostic = rospy.Publisher(topic,
                                              DiagnosticArray,
                                              queue_size = 2)
        self.diagnostic = DiagnosticStatus()
        self.diagnostic.name = name
        self.diagnostic.hardware_id = hw_id
        self.counter = -1
        self.desired_freq = desired_freq
        self.max_freq_error = max_freq_error


    def setLevel(self, level, message="none"):
        self.diagnostic.level = level
        if message == "none":
            if level == DiagnosticStatus.OK:
                self.diagnostic.message = "Ok"
            elif level == DiagnosticStatus.WARN:
                self.diagnostic.message = "Warning"
            else:
                self.diagnostic.message = "Error"
        else:
            self.diagnostic.message = message

        # Publish diagnostic message
        self.publish()


    def add(self, key, value):
        found = False
        i = 0
        while i < len(self.diagnostic.values) and not found:
            if self.diagnostic.values[i].key == key:
                self.diagnostic.values[i].value = value
                found = True
            else:
                i = i + 1

        if not found:
            self.diagnostic.values.append(KeyValue(key, value))


    def remove(self, key, value):
        found = False
        i = 0
        while i < len(self.diagnostic.values) and not found:
            if self.diagnostic.values[i].key == key:
                found = True
            else:
                i = i + 1

        if found:
            self.diagnostic.values.remove(self.diagnostic.values[i])


    def publish(self):
        """ Publish diagnostic. """
        diagnostic_array = DiagnosticArray()
        diagnostic_array.header.stamp = rospy.Time.now()
        diagnostic_array.status.append(self.diagnostic)
        self.pub_diagnostic.publish(diagnostic_array)


    def check_frequency(self):
        """ This method has to be called in each iteration. If the iteration
            frequency does not agree with the frequency set at the
            initialization a warning diagnostic message is published. """

        if self.counter == -1:
            self.init_period = rospy.Time.now()

        self.counter = self.counter + 1

        if self.counter == self.desired_freq:
            now = rospy.Time.now()
            period = (now - self.init_period).to_sec()

            self.add("frequency: ", str(self.desired_freq/period))
            if abs(period - 1.0) < (self.max_freq_error / 100.0):
                self.setLevel(DiagnosticStatus.OK)
            else:
                self.setLevel(DiagnosticStatus.WARN, "Invalid frequency!")
            self.publish()
            self.counter = 0
            self.init_period = now

