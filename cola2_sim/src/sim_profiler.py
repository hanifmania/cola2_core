#!/usr/bin/env python
"""
Created on Nov 4, 2015

@author: juandhv (Juan David Hernandez Vega, juandhv@eia.udg.edu)

Purpose: Create laser_scan messages from a simulated profiler.

Modified version of revolving_profiler_simulator.py (Enric Galceran)


EXAMPLE FOR GIRONA500 and SPARUS-II
On robot.urdf add:
==================
<link name="seaking_profiler">
</link>

<joint name="robot2profiler" type="continuous">
  <parent link="base"/> or <parent link="base_sparus"/> 
  <child  link="seaking_profiler"/>
  <axis xyz="0 0 1"/>
  <origin rpy="0 0 3.141592653589793" xyz="0.38 0.35 -0.55"/>
</joint>

On scene.xml add:
=================
<vehicle>
  ...
  <rangeSensor>
    <name>profiler</name>
    <relativeTo>seaking_profiler</relativeTo>
    <range>100</range>
    <visible>0</visible>
    <position>
      <x>0</x>
      <y>0</y>
      <z>0</z>
    </position>  
    <orientation>
      <r>0</r>
      <p>0</p>
      <y>0</y>
    </orientation>
  </rangeSensor>

  <multibeamSensor>
    ...
</vehicle>

<rosInterfaces>
  ...
  <RangeSensorToROSRange>
    <name>profiler</name>
    <topic>/seaking_range </topic>
    <rate>30</rate>
  </RangeSensorToROSRange>
  
  <multibeamSensorToLaserScan>
    ...
</rosInterfaces>

"""

#
# This ROS node publishes position commands to a revolving joint on AUV (G500 or SPARUS-II)
# and listens to its attached range sensor.
#

import rospy
from math import *
from sensor_msgs.msg import LaserScan, Range, JointState
from auv_msgs.msg import NavSts
import cola2_lib
from cola2_lib import cola2_ros_lib
from cola2_lib import cola2_lib
import random
import tf
from threading import Thread
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from cola2_msgs.cfg import tritech_msisConfig as MicronConfig
import numpy as np
from gi.overrides.Gdk import Event

class RevolvingProfilerSimulator :
    """
    Revolving profiler simulator.
    """

    def __init__(self):
        """
        Constructor
        """
        #=======================================================================
        # Config
        #=======================================================================
        self.config = cola2_ros_lib.Config()
        self.config.mode = 'cont'
        #=======================================================================
        # Parameters
        #=======================================================================
        self.dr_server = DynamicReconfigureServer(MicronConfig,self.reconfigure)
        self.get_params()

        # App data
        self.scan_right_ = True
        self.angle_ = self.config.llim
        
        # Received data 
        self.sonar_range_  = 0.0
        self.sonar_range_available_ = False
        self.joint_state_ = JointState()
        self.joint_state_available_ = False
        
        self.tf_broadcaster_ = tf.TransformBroadcaster()
        #=======================================================================
        # Subscribers 
        #=======================================================================
        #Navigation data (feedback)
        self.nav_sts_sub_ = rospy.Subscriber("/cola2_navigation/nav_sts", NavSts, self.navStsSubCallback, queue_size = 1)
        self.nav_sts_available_ = False
        #Simulated sonar range and joint state
        rospy.Subscriber('/seaking_range', Range, self.handleRange, queue_size = 1)
        rospy.Subscriber('/uwsim/joint_state', JointState, self.handleJointState, queue_size = 1)
        #=======================================================================
        # Publishers
        #=======================================================================
        self.joint_sonar_command_pub_ = rospy.Publisher('/uwsim/joint_state_command', JointState, queue_size=1)
        self.sonar_slice_pub_  = rospy.Publisher('/tritech_'+ self.config.sonarType + '_slice', LaserScan, queue_size=1)
        self.sonar_beam_pub_  = rospy.Publisher('/tritech_'+ self.config.sonarType + '_beam', LaserScan, queue_size=1)
        #=======================================================================
        # Waiting for nav sts
        #=======================================================================
        rospy.logwarn("%s: waiting for navigation data", rospy.get_name())
        r = rospy.Rate(5)
        while not rospy.is_shutdown() and not self.nav_sts_available_:
            r.sleep()
        rospy.logwarn("%s: navigation data received", rospy.get_name())
        #=======================================================================
        # Sonar to initial position
        #=======================================================================
        rospy.loginfo("%s: wait 5s to uwsim be completely loaded", rospy.get_name())
        rospy.sleep(5.0)        
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'seaking_profiler'
        msg.name = [self.jointname]
        msg.position = [cola2_lib.wrapAngle(self.angle_)]
        msg.velocity = []
        msg.effort = []
        self.joint_sonar_command_pub_.publish(msg)
        self.accum_ranges_ = []
        self.accum_angles_ = []
        self.last_angle_ = self.angle_
        #=======================================================================
        # Setup periodic task to revolve the profiler
        #=======================================================================
        rospy.Timer(rospy.Duration(1/self.config.angle_inc_rate), self.keepRevolving)
        
        return
    
    def navStsSubCallback(self,nav_sts_msg):
        """
        Callback to receive navSts message
        """
        self.nav_sts_data = nav_sts_msg
        if self.nav_sts_available_ == False:
            self.nav_sts_available_ = True
        return

    def handleRange(self, data):
        """
        Callback to receive range data
        """
        noise = random.gauss(0.0, self.config.noise_stdev)
        if data.range>self.config.max_range:
            self.sonar_range_ = self.config.max_range+2
        else:
            self.sonar_range_ = data.range + noise
        self.sonar_range_available_ = True
        if self.joint_state_available_:
            # Publish beam
            msg = LaserScan()
            msg.header = data.header
            msg.header.frame_id = self.config.frame_id
            msg.angle_increment = 0
            idx = self.joint_state_.name.index(self.jointname)
            msg.angle_min = self.joint_state_.position[idx]
            msg.angle_max = self.joint_state_.position[idx]
            msg.time_increment = 0
            msg.range_min = 0.5
            msg.range_max = self.config.max_range
            bindist = msg.range_max/self.config.nbins
            msg.ranges = np.arange(0+bindist,msg.range_max+0.000001,bindist).tolist()
            msg.intensities = [1]*self.config.nbins
            if self.sonar_range_ <= self.config.max_range:
                index = int(round(self.sonar_range_/bindist))
                msg.intensities[index-1] = 255
            self.tf_broadcaster_.sendTransform((self.translation[0], self.translation[1], self.translation[2]),                   
                     tf.transformations.quaternion_from_euler(self.rotation_rpy[0], self.rotation_rpy[1], self.rotation_rpy[2]),
                     msg.header.stamp,
                     self.config.frame_id,
                     self.config.robot_frame_name)
            self.sonar_beam_pub_.publish(msg)
            
            # adabt range to binning
            self.sonar_range_ = int(round(data.range/bindist))*bindist
    
    def handleJointState(self, data):
        """
        Callback to receive joint state data
        """
        self.joint_state_ = data
        self.joint_state_available_ = True              

    def reconfigure(self,config,level):
        """
        Callback to update dynamic parameters
        """
        
        rospy.logdebug("Dynamic reconfigure callback")
        self.config.max_range = float(config["rangeScale"])
        self.config.min_range = float(config["min_range"])
        self.config.llim = radians(config["llim"])
        self.config.rlim = radians(config["rlim"])
        self.config.cont = config["cont"]
        self.config.angle_inc = config["step"]*16*2*pi/6400
        self.config.nbins = config["nbins"]
        self.config.publish_every = config["publishEvery"]
        if self.config.cont:
            self.config.mode = 'cont'
        else:
            self.config.mode = 'sector'
        return config

    def get_params(self, event = None):
        """
        Reads the parameters needed.
        Parameter file is found at:
          cola2_perception_dev/config/tritech_seaking_profiler.yaml
          cola2_perception_dev/config/tritech_micron_sonar.yaml
        """
        self.config.angle_inc_rate = 30.0
        self.config.grabbing_rate = 5
        #self.config.max_range = 30.0
        self.config.noise_stdev = .1
        
        # joint in uwsim urdf
        self.jointname = rospy.get_param('~uwsimjointname', 'robot2profiler')
        
        # frames
        self.config.robot_frame_name = rospy.get_param('~frame_id','/girona500')
        self.config.frame_id = rospy.get_param('~child_id','/seaking_profiler')
        rospy.loginfo('frame_id: ' + self.config.robot_frame_name)
        rospy.loginfo('child_id: ' + self.config.frame_id)

        # Sensor transform
        tfx = rospy.get_param('~tf_x',0.7)
        tfy = rospy.get_param('~tf_y',0.0)
        tfz = rospy.get_param('~tf_z',-0.15)
        tfr = rospy.get_param('~tf_roll',0.0)
        tfp = rospy.get_param('~tf_pitch',0.0)
        tfw = rospy.get_param('~tf_yaw',0.0)
        self.translation = (tfx, tfy, tfz)
        self.rotation_rpy = (tfr, tfp, tfw)
        
        # Sensor type
        self.config.sonarType = rospy.get_param('~sonar_type','seaking_profiler')
        
        # Publishing
        self.config.publish_every = rospy.get_param('~publish_every',10)
        return

    def keepRevolving(self, event):
        """
        keepRevolving
        
        Revolves the joint on AUV (G500 and SPARUS-II) where the range sensor is attached to.
        """
        #r = rospy.Rate(self.config.angle_inc_rate)
        #while not rospy.is_shutdown():
        if self.joint_state_available_:
            #=======================================================================
            # Get last range
            #=======================================================================
            if len(self.accum_ranges_) == 0:
                self.first_beam_time_ = rospy.Time.now()

            if self.sonar_range_ < self.config.max_range:
                self.accum_ranges_.append(self.sonar_range_)
                rospy.logdebug("%s: Grabbed new beam", rospy.get_name())
            else:
                self.accum_ranges_.append(self.config.max_range)
                rospy.logdebug("%s: Discarding beam > max_range", rospy.get_name())
            self.accum_angles_.append(self.last_angle_)
            #=======================================================================
            # Revolve simulated sonar
            #=======================================================================
            rospy.logdebug("%s: moving joint", rospy.get_name())        
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'seaking_profiler'
            msg.name = [self.jointname]
            msg.position = [cola2_lib.wrapAngle(self.angle_)]
            msg.velocity = []
            msg.effort = []
            self.last_angle_ = self.angle_

            self.joint_sonar_command_pub_.publish(msg)
            
            rospy.logdebug("----------------------------------------------------------------------------------")
            if self.config.mode == 'sector':
                rospy.logdebug("Sector")
            else:
                rospy.logdebug("Cont")
            rospy.logdebug("Scan right = "+str(self.scan_right_))
            rospy.logdebug("Angle: "+str(degrees(self.angle_))+ "("+str(degrees(self.config.llim))+ " - "+str(degrees(self.config.rlim))+")")
            
            # Update angle
            #TODO what if sector goes through 0 and rlim>180 or llim<180?
            change_direction = False
            if self.config.mode == 'sector' and self.config.rlim>self.config.llim:
                if self.scan_right_ and self.angle_ >= self.config.rlim:
                    self.scan_right_ = False
                    
                    change_direction = True
                elif not self.scan_right_ and self.angle_ <= self.config.llim:
                    self.scan_right_ = True
                    
                    change_direction = True
            elif self.config.mode == 'sector' and  self.config.rlim<self.config.llim:
                angle = cola2_lib.wrapAngle(self.angle_)
                rlim = cola2_lib.wrapAngle(self.config.rlim)
                llim = cola2_lib.wrapAngle(self.config.llim)
                if self.scan_right_ and angle >= rlim:
                    self.scan_right_ = False
                    
                    change_direction = True
                elif not self.scan_right_ and angle <= llim:
                    self.scan_right_ = True
                    
                    change_direction = True
            elif self.config.mode != 'cont':
                self.angle_ = self.config.rlim
#                 r.sleep()
#                 continue

            if self.scan_right_:
                ainc = self.config.angle_inc
            else:
                ainc = -self.config.angle_inc
            self.angle_ = self.wrapAngle(self.angle_ + ainc)
            
            #=======================================================================
            # Checking if laser_scan is complete to be published
            #=======================================================================
            if len(self.accum_ranges_)+1 > self.config.publish_every or change_direction:
                rospy.logdebug("len(self.accum_ranges_): " + str(len(self.accum_ranges_)))
                rospy.logdebug("self.accum_ranges_: " + str(self.accum_ranges_))
                rospy.logdebug("self.accum_angles_: " + str(self.accum_angles_))
                msg = LaserScan()
                # timestamp in the header is the acquisition time of 
                # the first ray in the scan.
                #
                # in frame frame_id, angles are measured around 
                # the positive Z axis (counterclockwise, if Z is up)
                # with zero angle being forward along the x axis
                msg.header.stamp = self.first_beam_time_
                msg.header.frame_id = self.config.frame_id
                
                if self.scan_right_:
                    msg.angle_increment = self.config.angle_inc # angular distance between measurements [rad]
                    msg.angle_min = min(self.accum_angles_)        # start angle of the scan [rad]
                    msg.angle_max = max(self.accum_angles_)       # end angle of the scan [rad]
                else:
                    msg.angle_increment = -self.config.angle_inc
                    msg.angle_min = max(self.accum_angles_)        # start angle of the scan [rad]
                    msg.angle_max = min(self.accum_angles_)       # end angle of the scan [rad]

                msg.time_increment = 1.0/self.config.angle_inc_rate  # time between measurements [seconds] - if your scanner
                                         # is moving, this will be used in interpolating position
                                         # of 3d points
                msg.scan_time = msg.time_increment * (len(self.accum_ranges_)-1)        # time between scans [seconds]
                
                msg.range_min = self.config.min_range# minimum range value [m]
                msg.range_max = self.config.max_range#max(self.accum_ranges)     # maximum range value [m]                
                
                msg.ranges = self.accum_ranges_
                msg.intensities = []
                
                self.accum_ranges_ = []
                self.accum_angles_ = []               
                
                self.tf_broadcaster_.sendTransform((self.translation[0], self.translation[1], self.translation[2]),                   
                         tf.transformations.quaternion_from_euler(self.rotation_rpy[0], self.rotation_rpy[1], self.rotation_rpy[2]),
                         msg.header.stamp,
                         self.config.frame_id,
                         self.config.robot_frame_name) 

                self.sonar_slice_pub_.publish(msg)
        else:
            rospy.logwarn("%s: no joint state available", rospy.get_name())
        
        #r.sleep()
            
    def wrapAngle(self,angle):
        angle = cola2_lib.wrapAngle(angle)
        if angle<0:
            angle = angle + 2*pi
        return angle
                

#     def keepPublishingLaserScans(self):
#         """
#         keepPublishingLaserScans
#         
#         Publishes the range sensor data as laser scans.
#         """
#         r = rospy.Rate(self.config.angle_inc_rate)
#         lastScanRight = self.scan_right_
#         angles = [0.0,0.0,0.0]
#         while not rospy.is_shutdown():
#             rospy.logdebug("%s: Not enough beams yet, nb=%d, pubevery=%d", rospy.get_name(), len(self.accum_angles), self.config.publish_every)
#             if self.sonar_range_available_ and self.joint_state_available_ and angles[0] != self.joint_state_.position[self.joint_state_.name.index(self.jointname)]:
#                 
#                 # Grab new measurement
#                 if len(self.accum_angles) == 0:
#                     self.first_beam_time = rospy.Time.now()
#                 
#                 if self.sonar_range_ < self.config.max_range:
#                     rvalue = self.sonar_range_
#                     rospy.logdebug("%s: Grabbed new beam", rospy.get_name())
#                 else:
#                     rvalue = self.config.max_range
#                     rospy.logdebug("%s: Discarding beam > max_range", rospy.get_name())
#                 idx = self.joint_state_.name.index(self.jointname)
#                 self.accum_angles.append(self.joint_state_.position[idx])
#                 self.accum_ranges.append(rvalue)
#                 
#                 # Close and publish slice
#                 if len(self.accum_ranges)>1 and (len(self.accum_ranges) >= self.config.publish_every or self.complete_scan): #lastScanRight != self.scan_right_):
#                     if lastScanRight != self.scan_right_:
#                         # Drop last range
#                         self.accum_angles = self.accum_angles[0:len(self.accum_angles)-1]
#                         self.accum_ranges = self.accum_ranges[0:len(self.accum_ranges)-1]
#                         
#                     msg = LaserScan()
#                     # timestamp in the header is the acquisition time of 
#                     # the first ray in the scan.
#                     #
#                     # in frame frame_id, angles are measured around 
#                     # the positive Z axis (counterclockwise, if Z is up)
#                     # with zero angle being forward along the x axis
#                     msg.header.stamp = self.first_beam_time
#                     msg.header.frame_id = self.config.frame_id
#                     
#                     if self.scan_right_:
#                         msg.angle_increment = self.config.angle_inc # angular distance between measurements [rad]
#                         msg.angle_min = min(self.accum_angles)        # start angle of the scan [rad]
#                         msg.angle_max = max(self.accum_angles)       # end angle of the scan [rad]
#                     else:
#                         msg.angle_increment = -self.config.angle_inc
#                         msg.angle_min = max(self.accum_angles)        # start angle of the scan [rad]
#                         msg.angle_max = min(self.accum_angles)       # end angle of the scan [rad]
# 
#                     msg.time_increment = 1.0/self.config.angle_inc_rate  # time between measurements [seconds] - if your scanner
#                                              # is moving, this will be used in interpolating position
#                                              # of 3d points
#                     msg.scan_time = msg.time_increment * (len(self.accum_ranges)-1)        # time between scans [seconds]
#                     
#                     msg.range_min = self.config.min_range# minimum range value [m]
#                     msg.range_max = self.config.max_range#max(self.accum_ranges)     # maximum range value [m]                
#                     
#                     msg.ranges = self.accum_ranges
#                     msg.intensities = []
#                     
#                     self.accum_angles = []
#                     self.accum_ranges = []               
#                     
#                     self.tf_broadcaster_.sendTransform((self.translation[0], self.translation[1], self.translation[2]),                   
#                              tf.transformations.quaternion_from_euler(self.rotation_rpy[0], self.rotation_rpy[1], self.rotation_rpy[2]),
#                              msg.header.stamp,
#                              self.config.frame_id,
#                              self.config.robot_frame_name) 
# 
#                     self.sonar_slice_pub_.publish(msg)   
#                     
#                 lastScanRight = self.scan_right_             
#                
#             r.sleep()
    
if __name__ == '__main__':
    try:
        rospy.init_node('sim_profiler', anonymous=True, log_level=rospy.INFO) #log_level=rospy.DEBUG)
        profiler_simulator = RevolvingProfilerSimulator()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.logdebug("%s: inside revolving_profiler_simulator", rospy.get_name())
            r.sleep()
        
    except rospy.ROSInterruptException: pass
