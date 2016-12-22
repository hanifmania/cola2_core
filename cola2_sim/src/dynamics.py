#!/usr/bin/env python

"""@@This node uses simulated data of the actuators to compute the AUV dynamic
behavior. This node can be used to simulate real AUV behavior and its interaction
with the environtment. User can add currents and a preliminary version of collision
detection has been implemented.@@"""


"""
Modified 11/2015
@author: narcis palomeras
"""

# Basic ROS imports
import rospy
import tf

# Import msgs
from nav_msgs.msg import Odometry
from cola2_msgs.msg import Setpoints
from auv_msgs.msg import BodyForceReq
from gazebo_msgs.msg import ModelState

# Import srv
from cola2_msgs.srv import SimulatedCurrents

# More imports
import PyKDL
import math
import numpy as np

# Custom libs
from cola2_lib import cola2_lib, cola2_ros_lib


class Dynamics :
    """ Simulates the dynamics of an AUV from thrusters rpm and fins angles """

    def __init__(self, name):
        """ Simulates the dynamics of an AUV """
        self.name = name
        self.contact_sensor_available = False

        # Load dynamic parameters
        self.get_config()

        # Initialize vars and matrices. They are not init. in the constructor,
        # but readability is improved
        self.initialize()

        # Create publisher
        self.pub_odom = rospy.Publisher(self.odom_topic_name,
                                        Odometry,
                                        queue_size = 2)

        self.pub_odom_gazebo = rospy.Publisher('/gazebo/set_model_state',
                                               ModelState,
                                               queue_size=2)

        # Create subscribers
        rospy.Subscriber(self.thrusters_topic,
                         Setpoints,
                         self.update_thrusters,
                         queue_size = 1)

        rospy.Subscriber(self.force_topic,
                         BodyForceReq,
                         self.update_force,
                         queue_size = 1)

        self.current_srv = rospy.Service(
            'cola2_sim/current_simulation',
            SimulatedCurrents,
            self.current_simulation_srv)

        if self.fins > 0:
            rospy.Subscriber(self.fins_topic,
                             Setpoints,
                             self.update_fins,
                             queue_size = 1)

        # Collision parameters
        if self.contact_sensor_available:
            self.collisionForce = np.array([0,0,0,0,0,0])
            rospy.Subscriber(self.collisions_topic,
                             WrenchStamped,
                             self.updateCollision,
                             queue_size = 1)

        # Show message
        rospy.loginfo("%s: initialized", self.name)


    def initialize(self):
        """ Initialize vars and matrices """
        # Init pose, velocity and rate
        self.v = self.v_0
        self.p = self.p_0
        self.p_dot = np.zeros(len(self.p))
        self.v_dot = np.zeros(len(self.v))
        self.rate = 1.0 / self.period

        # Inertia Tensor. Principal moments of inertia,
        # and products of inertia [kg*m*m]
        Ixx = self.tensor[0]
        Ixy = self.tensor[1]
        Ixz = self.tensor[2]
        Iyx = self.tensor[3]
        Iyy = self.tensor[4]
        Iyz = self.tensor[5]
        Izx = self.tensor[6]
        Izy = self.tensor[7]
        Izz = self.tensor[8]
        m = self.mass
        xg = self.gravity_center[0]
        yg = self.gravity_center[1]
        zg = self.gravity_center[2]

        Mrb=[m,     0,      0,      0,      m*zg,       -m*yg,
             0,     m,      0,      -m*zg,  0,          m*xg,
             0,     0,      m,      m*yg,   -m*xg,      0,
             0,     -m*zg,  m*yg,   Ixx,    Ixy,        Ixz,
             m*zg,  0,      -m*xg,  Iyx,    Iyy,        Iyz,
             -m*yg, m*xg,   0,      Izx,    Izy,        Izz]
        Mrb = np.array(Mrb).reshape(6, 6)

        # Inertia matrix of the rigid body
        # Added Mass derivative TODO: This is not a valid added mass matrix!
        Ma=[m/2,    0,      0,      0,      0,      0,
            0,      m/2,    0,      0,      0,      0,
            0,      0,      m/2,    0,      0,      0,
            0,      0,      0,      0,      0,      0,
            0,      0,      0,      0,      0,      0,
            0,      0,      0,      0,      0,      0]
        Ma = np.array(Ma).reshape(6, 6)

        # Mass matrix: Mrb + Ma
        self.M = Mrb + Ma
        self.IM = np.matrix(self.M).I

        # Init currents
        np.random.seed()
        #self.e_vc = np.random.normal(self.current_mean, self.current_sigma)

        # Force message
        self.force = BodyForceReq()

        # Initial thrusters setpoint
        self.u = np.zeros(self.thrusters)
        self.old_u = self.u # Previous setpoints

        # Initial fins setpoint
        self.f = np.zeros(self.fins)
        self.old_f = self.f # Previous setpoints


    def update_thrusters(self, thrusters) :
        """ Thruster callback, input in rpm """
        self.old_u = self.u
        self.u = np.array( thrusters.setpoints ).clip(
           min=-abs(self.max_thrusters_rpm), max=abs(self.max_thrusters_rpm))


    def update_force(self, force) :
        """ Thruster callback, input in rpm """
        self.force = force


    def update_fins(self, fins) :
        """ Fins callback, input in rad """
        self.old_f = self.f
        self.f = np.array( fins.setpoints ).clip(
           min=-abs(self.max_fins_angle), max=abs(self.max_fins_angle))


    def compute_currents(self):
        """ Water currents, returns a velocity """
        # Compute random currents
        if self.current_enabled :
            e_vc = np.random.normal(self.current_mean,
                                    self.current_sigma)
            for i in range(3):
                if e_vc[i] > self.current_max[i]:
                    e_vc[i] = self.current_max[i]
                if e_vc[i] < self.current_min[i]:
                    e_vc[i] = self.current_min[i]
            t = PyKDL.Vector(e_vc[0], e_vc[1], e_vc[2])
            O = PyKDL.Rotation.RPY(self.p[3], self.p[4], self.p[5])
            currents = O.Inverse() * t
            return np.array([currents[0], currents[1], currents[2], 0, 0, 0])
        else:
            return np.array([0,0,0,0,0,0])


    def damping_matrix(self, vel):
        """ Damping matrix """
        # Linear hydrodynamic damping coeficients
        Xu = self.damping[0]
        Yv = self.damping[1]
        Zw = self.damping[2]
        Kp = self.damping[3]
        Mq = self.damping[4]
        Nr = self.damping[5]

        # Quadratic hydrodynamic damping coeficients
        Xuu = self.quadratic_damping[0]    #[Kg/m]
        Yvv = self.quadratic_damping[1]    #[Kg/m]
        Zww = self.quadratic_damping[2]    #[Kg/m]
        Kpp = self.quadratic_damping[3]    #[Kg*m*m]
        Mqq = self.quadratic_damping[4]    #[Kg*m*m]
        Nrr = self.quadratic_damping[5]    #[Kg*m*m]

        d = np.diag([Xu + Xuu*abs(vel[0]),
                     Yv + Yvv*abs(vel[1]),
                     Zw + Zww*abs(vel[2]),
                     Kp + Kpp*abs(vel[3]),
                     Mq + Mqq*abs(vel[4]),
                     Nr + Nrr*abs(vel[5])])
        return d


    def generalized_force(self, du):
        """ Compute the force of each thruster from rpm """
        # Build the signed (lineal/quadratic) thruster coeficient array
        # Signed square of each thruster setpoint
        du = du * abs(du)

        ct = np.zeros(len(du))
        i1 = np.nonzero(du >= 0.0)
        i2 = np.nonzero(du <= 0.0)
        ct[i1] = self.ctf
        ct[i2] = self.ctb
        b =  np.dot(self.thrusters_matrix, (np.identity(len(du)) * ct))

# Example of g500
#   b2 = [-ct[0],        -ct[1],         .0,             .0,            .0,
#        .0,             .0,             .0,             .0,            ct[4],
#        .0,             .0,             -ct[2],         -ct[3],        .0,
#        .0,             .0,             .0,             .0,            .0,
#        .0,             .0,             -ct[2]*self.dv, ct[3]*self.dv, .0,
#        -ct[0]*self.dh, ct[1]*self.dh,  .0,             .0,            .0]
#   b2 = np.array(b2).reshape(6,5)

        # The value of t is the generalized force
        t = np.dot(b, du)

        # Transforms a matrix into an array
        t = np.squeeze(np.asarray(t))
        return t


    #def compute_fins(self, vel, fins):
    #    """ Fins force """
    #    if self.fins > 0:
    #        f = np.array([-(2.0*self.a_fins)*0.5*self.density*vel[0]*abs(vel[0])*self.k_fins*abs(np.sin(fins[0])),
    #                      0.0,
    #                      -(2.0*self.a_fins)*0.5*self.density*vel[0]*abs(vel[0])*self.k_fins*np.sin(fins[0])-(2.0*self.a_fins)*0.5*vel[2]*abs(vel[2])*self.k_fins*np.cos(fins[0]),
    #                      0.0,
    #                      +(2.0*self.a_fins)*0.5*self.density*vel[0]*abs(vel[0])*self.k_fins*np.sin(fins[0])*self.dh+(2.0*self.a_fins)*0.5*self.density*vel[2]*abs(vel[2])*self.k_fins*np.cos(fins[0])*self.dh,
    #                      0.0])
    #    else:
    #        f = np.zeros(6)
    #    return f

    # New fins model
    # fins[0] -> left fin
    # fins[1] -> right fin
    # February of 2015
    def compute_fins(self, vel, fins):
        """ Fins force """

        # Water velocity on the fins
        water_vel = vel[0]
        if vel[0] > 0:
            water_vel = math.sqrt(vel[0]*vel[0]+(25*vel[0]/(self.density*3.141592*0.049*0.049)))

        if self.fins > 0:
            f = np.array([-(0.5*self.density*self.a_fins*water_vel*abs(water_vel)*self.k_cd_fins)*(abs(np.cos(1*fins[0]))+abs(np.cos(1*fins[1]))),
                          0.0,
                          +(0.5*self.density*self.a_fins*water_vel*abs(water_vel)*self.k_cl_fins)*(np.sin(4.5*fins[0])+np.sin(4.5*fins[1])),
                          +(0.5*self.density*self.a_fins*water_vel*abs(water_vel)*self.k_cl_fins)*(np.sin(4.5*fins[0])-np.sin(4.5*fins[1]))*0.14,
                          +(0.5*self.density*self.a_fins*water_vel*abs(water_vel)*self.k_cl_fins)*(np.sin(4.5*fins[0])+np.sin(4.5*fins[1]))*0.65,
                          0.0])
        else:
            f = np.zeros(6)
        return f


    def coriolis_matrix(self, vel):
        """ Coriolis matrix """
        s1 = __s__( np.dot(self.M[0:3,0:3], vel[0:3]) +
                    np.dot(self.M[0:3,3:6], vel[3:6]) )
        s2 = __s__( np.dot(self.M[3:6,0:3], vel[0:3]) +
                    np.dot(self.M[3:6,3:6], vel[3:6]) )
        c = np.zeros((6, 6))
        c[0:3,3:6] = -s1
        c[3:6,0:3] = -s1
        c[3:6,3:6] = -s2
        return c


    def gravity(self, pos):
        """ Gravity and weight matrix """
        # Weight and buoyancy from [Kg] to [N]
        W = self.mass * self.g
        B = self.buoyancy * self.g

        # If the vehicle moves out of the water the flotability decreases
        corr_pos = pos[2] + self.radius  # Corrected z position
        if corr_pos >= self.radius:
            F = B
        elif corr_pos <= -self.radius:
            F = 0.0
        else:
            r2 = pow(self.radius, 2.0)
            total_area = math.pi * r2
            c = np.sqrt(r2 - pow(corr_pos, 2.0))
            area_segment = math.atan2(c, corr_pos) * r2
            area_triangle = corr_pos * c
            area_outside = area_segment - area_triangle
            F = B * (1.0 - area_outside / total_area)

        # Gravity center position in the robot fixed frame (x',y',z') [m]
        zg = self.gravity_center[2]

        g = np.array([(W - F) * np.sin(pos[4]),
                   -(W - F) * np.cos(pos[4]) * np.sin(pos[3]),
                   -(W - F) * np.cos(pos[4]) * np.cos(pos[3]),
                   zg*W*np.cos(pos[4])*np.sin(pos[3]),
                   zg*W*np.sin(pos[4]),
                   0.0])
        return g


    def inverse_dynamic(self, pos, vel, u, f, current) :
        """ Given the setpoint for each thruster, the previous velocity
            and the previous position computes the v_dot """
        #rospy.loginfo('Current Value ' + str(current))
        if self.use_force_topic:
            a = np.array([self.force.wrench.force.x,
                          self.force.wrench.force.y,
                          self.force.wrench.force.z,
                          self.force.wrench.torque.x,
                          self.force.wrench.torque.y,
                          self.force.wrench.torque.z])
        else:
            t = self.generalized_force(u)
            f = self.compute_fins(vel, f)
            a = t+f
        d = self.damping_matrix(vel+current)
        c = self.coriolis_matrix(vel)
        g = self.gravity(pos)
        c_v = np.dot((c-d), vel+current)
        if self.contact_sensor_available:
            v_dot = np.dot(self.IM, (a-c_v-g-self.collisionForce))
        else:
            v_dot = np.dot(self.IM, (a-c_v-g))

        # Transforms a matrix into an array
        v_dot = np.squeeze(np.asarray(v_dot))

        if self.contact_sensor_available:
            for i in xrange(0,3):
                if (self.collisionForce[i]>0 and v_dot[i]>0) or (self.collisionForce[i]<0 and v_dot[i]<0):
                    v_dot[i]=0
                if (self.collisionForce[i]>0 and self.v[i]>0) or (self.collisionForce[i]<0 and self.v[i]<0):
                    self.v[i]=0
        return v_dot


    def kinematics(self, pos, vel) :
        """ Given the current velocity and the previous
            position computes the p_dot """
        roll = pos[3]
        pitch = pos[4]
        yaw = pos[5]
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)

        rec = [cy*cp, -sy*cr+cy*sp*sr, sy*sr+cy*cr*sp,
               sy*cp, cy*cr+sr*sp*sy, -cy*sr+sp*sy*cr,
               -sp, cp*sr, cp*cr]
        rec = np.array(rec).reshape(3,3)

        to = [1.0, sr*np.tan(pitch), cr*np.tan(pitch),
              0.0, cr, -sr,
              0.0, sr/cp, cr/cp]
        to = np.array(to).reshape(3,3)

        p_dot = np.zeros(6)
        p_dot[0:3] = np.dot(rec, vel[0:3])
        p_dot[3:6] = np.dot(to, vel[3:6])
        return p_dot


    def step(self, pos, vel, u, f, current):
        """ Compute kinematics and inverse dynamics """
        return self.kinematics(pos, vel), self.inverse_dynamic(pos, vel, u, f, current)


    def iterate(self):
        """ Main loop operations """
        # Compute current
        current = self.compute_currents()

        # Runge-Kutta, 4th order
        k1_pos, k1_vel = self.step(self.p, self.v, self.old_u, self.old_f, current)
        k2_pos, k2_vel = self.step(self.p + self.period * 0.5 * k1_pos, self.v + self.period * 0.5 * k1_vel, 0.5 * (self.old_u + self.u), 0.5 * (self.old_f + self.f), current)
        k3_pos, k3_vel = self.step(self.p + self.period * 0.5 * k2_pos, self.v + self.period * 0.5 * k2_vel, 0.5 * (self.old_u + self.u), 0.5 * (self.old_f + self.f), current)
        k4_pos, k4_vel = self.step(self.p + self.period * k3_pos, self.v + self.period * k3_vel, self.u, self.f, current)

        self.p = self.p + self.period / 6.0 * ( k1_pos +
                                                2.0 * k2_pos +
                                                2.0 * k3_pos +
                                                k4_pos )
        self.v = self.v + self.period / 6.0 * ( k1_vel +
                                                2.0 * k2_vel +
                                                2.0 * k3_vel +
                                                k4_vel )

        self.p[3] = cola2_lib.wrapAngle(self.p[3])
        self.p[4] = cola2_lib.wrapAngle(self.p[4])
        self.p[5] = cola2_lib.wrapAngle(self.p[5])

        # Publish odometry
        self.pub_odometry()


    def updateCollision(self, force) :
        self.collisionForce=np.array([-force.wrench.force.x / 10.0,
                                      -force.wrench.force.z / 10.0,
                                      force.wrench.force.y / 10.0,
                                      -force.wrench.torque.x / 10.,
                                      -force.wrench.torque.z / 10.,
                                      force.wrench.torque.y / 10.])

    def current_simulation_srv(self, request):
        self.current_enabled = request.enabled
        self.current_mean = np.array(request.current_mean)
        self.current_sigma = np.array(request.current_sigma)
        # check if any sigma is 0.0
        zero_elements = np.where(self.current_sigma==0.0)[0]
        if np.size(zero_elements) > 0:
            #change the zero of the vector to 0.1
            for element in zero_elements:
                self.current_sigma[element] = 0.1
        # Compute Max and min with Full with at half maximum FWHM
        # sigma is not sigma^2
        # FWHM = 2*np.sqrt(2*np.log(2))*sigma
        fwhm_value = 2*np.sqrt(2*np.log(2))*self.current_sigma
        self.current_max = (self.current_mean + fwhm_value).tolist()
        self.current_min = (self.current_mean - fwhm_value).tolist()
        return True

    def pub_odometry(self):
        """ Publish odometry message """
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.world_frame_id

        odom.pose.pose.position.x = self.p[0]
        odom.pose.pose.position.y = self.p[1]
        odom.pose.pose.position.z = self.p[2]

        orientation = tf.transformations.quaternion_from_euler(self.p[3],
                                                               self.p[4],
                                                               self.p[5],
                                                               'sxyz')
        odom.pose.pose.orientation.x = orientation[0]
        odom.pose.pose.orientation.y = orientation[1]
        odom.pose.pose.orientation.z = orientation[2]
        odom.pose.pose.orientation.w = orientation[3]

        odom.twist.twist.linear.x = self.v[0]
        odom.twist.twist.linear.y = self.v[1]
        odom.twist.twist.linear.z = self.v[2]
        odom.twist.twist.angular.x = self.v[3]
        odom.twist.twist.angular.y = self.v[4]
        odom.twist.twist.angular.z = self.v[5]

        self.pub_odom.publish(odom)

        # Publish position for gazebo
        gazebo_odom = ModelState()
        gazebo_odom.model_name = 'girona500'
        gazebo_odom.pose = odom.pose.pose
        gazebo_odom.twist = odom.twist.twist
        gazebo_odom.reference_frame = 'world'
        self.pub_odom_gazebo.publish(gazebo_odom)

        # Broadcast transform
        br = tf.TransformBroadcaster()
        br.sendTransform((self.p[0], self.p[1], self.p[2]), orientation,
        odom.header.stamp, odom.header.frame_id, self.world_frame_id)


    def get_config(self):
        """ Get config from config file """
        if rospy.has_param("vehicle_name") :
            self.vehicle_name = rospy.get_param('vehicle_name')
        else:
            rospy.logfatal("%s: vehicle_name parameter not found", self.name)
            exit(0)  # TODO: find a better way

        param_dict = {'thrusters': "dynamics/" + self.vehicle_name + "/number_of_thrusters",
                      'force_topic': "dynamics/" + self.vehicle_name + "/force_topic",
                      'use_force_topic': "dynamics/" + self.vehicle_name + "/use_force_topic",
                      'thrusters_topic': "dynamics/" + self.vehicle_name + "/thrusters_topic",
                      'thrusters_matrix': "dynamics/" + self.vehicle_name + "/thrusters_matrix",
                      'fins': "dynamics/" + self.vehicle_name + "/number_of_fins",
                      'fins_topic': "dynamics/" + self.vehicle_name + "/fins_topic",
                      'a_fins': "dynamics/" + self.vehicle_name + "/a_fins",
                      'k_cd_fins': "dynamics/" + self.vehicle_name + "/k_cd_fins",
                      'k_cl_fins': "dynamics/" + self.vehicle_name + "/k_cl_fins",
                      'period': "dynamics/" + self.vehicle_name + "/period",
                      'mass': "dynamics/" + self.vehicle_name + "/mass",
                      'buoyancy': "dynamics/" + self.vehicle_name + "/buoyancy",
                      'gravity_center': "dynamics/" + self.vehicle_name + "/gravity_center",
                      'g': "dynamics/" + self.vehicle_name + "/g",
                      'radius': "dynamics/" + self.vehicle_name + "/radius",
                      'max_thrusters_rpm': "dynamics/" + self.vehicle_name + "/max_thrusters_rpm",
                      'max_fins_angle': "dynamics/" + self.vehicle_name + "/max_fins_angle",
                      'ctf': "dynamics/" + self.vehicle_name + "/ctf",
                      'ctb': "dynamics/" + self.vehicle_name + "/ctb",
                      'dzv': "dynamics/" + self.vehicle_name + "/dzv",
                      'dv': "dynamics/" + self.vehicle_name + "/dv",
                      'dh': "dynamics/" + self.vehicle_name + "/dh",
                      'density': "dynamics/" + self.vehicle_name + "/density",
                      'tensor': "dynamics/" + self.vehicle_name + "/tensor",
                      'damping': "dynamics/" + self.vehicle_name + "/damping",
                      'quadratic_damping': "dynamics/" + self.vehicle_name + "/quadratic_damping",
                      'p_0': "dynamics/" + self.vehicle_name + "/initial_pose",
                      'v_0': "dynamics/" + self.vehicle_name + "/initial_velocity",
                      'odom_topic_name': "dynamics/" + self.vehicle_name + "/odom_topic_name",
                      'frame_id': "dynamics/" + self.vehicle_name + "/frame_id",
                      'world_frame_id': "dynamics/" + self.vehicle_name + "/world_frame_id",
                      'current_mean': "dynamics/current_mean",
                      'current_sigma': "dynamics/current_sigma",
                      'current_min': "dynamics/current_min",
                      'current_max': "dynamics/current_max",
                      'current_enabled': "dynamics/current_enabled",
                      'collisions_topic': "dynamics/" + self.vehicle_name + "/uwsim_contact_sensor"}

        if not cola2_ros_lib.getRosParams(self, param_dict, self.name):
            rospy.logfatal("%s: shutdown due to invalid config parameters!", self.name)
            exit(0)  # TODO: find a better way

        if len(self.collisions_topic) > 0:
            self.contact_sensor_available = True
        self.thrusters_matrix = np.array(self.thrusters_matrix).reshape(6, self.thrusters)
        #self.current_mean = np.array(self.current_mean) * self.period
        #self.current_sigma = np.array(self.current_sigma) * self.period


def __s__(x):
    """ Given a 3D vector computes a 3x3 matrix for .... ? """
    ret = np.array([0.0, -x[2], x[1], x[2], 0.0, -x[0], -x[1], x[0], 0.0 ])
    return ret.reshape(3,3)


if __name__ == '__main__':
    try:
        rospy.init_node('dynamics')
        dynamics = Dynamics(rospy.get_name())
        rate_it = rospy.Rate(dynamics.rate)
        while not rospy.is_shutdown():
            dynamics.iterate()
            rate_it.sleep()

    except rospy.ROSInterruptException:
        pass
