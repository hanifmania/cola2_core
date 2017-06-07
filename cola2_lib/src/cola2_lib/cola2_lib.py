#!/usr/bin/env python

#import pylab
from math import floor, pi, sqrt
#from numpy import zeros, ones, array
import numpy as np
from math import factorial
import NED
from auv_msgs.msg import GoalDescriptor


class PID:
    def __init__(self, p, i, d, fff, time_in_sec, i_limit = None):
        if i_limit == None:
            i_limit = np.ones(len(p))
        if len(p) == len(i) and len(p) == len(d) and len(p) == len(fff) and len(p) == len(i_limit):
            self.kp = p
            self.ti = i
            self.td = d
            self.fff = fff
            self.n = len(p)
            self.ek_1 = np.zeros(self.n)
            self.eik_1 = np.zeros(self.n)
            self.past_time = time_in_sec
            self.i_limit = i_limit
        else:
            print 'ERROR: Bad vectors size!!'


    def compute(self, desired, current, time_in_sec):
        # Note: All the operations are done element by element
        T = time_in_sec - self.past_time
        if T < 0.001:
            T = 0.001
        self.past_time = time_in_sec

        # Compute errors
        if  len(desired) == self.n and len(current) == self.n:
            ek = np.zeros(self.n)
            eik = np.zeros(self.n)
            edotk = np.zeros(self.n)

            ek = desired - current
            edotk = (ek - self.ek_1) / T
            eik = self.eik_1 + (ek * T)

            # Control law
            tau = np.zeros(self.n)
            for i in range(self.n):
                # Compute the integral part if ti > 0
                if self.ti[i] > 0.0:
                    # Integral part
                    integral_part = (self.kp[i]/self.ti[i])*eik[i]

                    # Saturate integral part (anti-windup condition, integral part not higher than a value)
                    integral_part = saturateValueFloat(integral_part, self.i_limit[i])

                    # Restore eik
                    if self.kp[i] > 0:
                        eik[i] = integral_part * self.ti[i] / self.kp[i]
                    else:
                        eik[i] = 0

                    # Compute tau
                    tau[i] = self.kp[i]*ek[i] + self.kp[i]*self.td[i]*edotk[i] + integral_part + self.fff[i]
                else:
                    # Compute tau without integral part
                    tau[i] = self.kp[i]*ek[i] + self.kp[i]*self.td[i]*edotk[i] + self.fff[i]

            # Saturate tau
            tau = saturateValue(tau, 1.0)

            self.ek_1 = ek
            self.eik_1 = eik
            return tau
        else:
            return None


    def computeWithoutSaturation(self, desired, current, time_in_sec):
        # Note: All the operations are done element by element
        T = time_in_sec - self.past_time
        if T == 0.0:
            T = 0.001
        self.past_time = time_in_sec

        # Compute errors
        if  len(desired) == self.n and len(current) == self.n:
            ek = np.zeros(self.n)
            eik = np.zeros(self.n)
            edotk = np.zeros(self.n)

            ek = desired - current
            edotk = (ek - self.ek_1) / T
            eik = self.eik_1 + (ek * T)

            # Control law
            tau = np.zeros(self.n)
            for i in range(self.n):
                # Compute the integral part if ti > 0
                if self.ti[i] > 0.0:
                    integral_part = (self.kp[i]/self.ti[i])*eik[i]

                    # Compute tau
                    tau[i] = self.kp[i]*ek[i] + self.kp[i]*self.td[i]*edotk[i] + integral_part + self.fff[i]

                    # Anti-windup condition: if tau is saturated and ek has the
                    # same sign than tau, eik does not increment
                    if abs(tau[i]) > 1.0 and ek[i]*tau[i] > 0:
                        eik[i] = self.eik_1[i]
                        # Because eik has been modified, recompute tau
                        integral_part = (self.kp[i]/self.ti[i])*eik[i]
                        tau[i] = self.kp[i]*ek[i] + self.kp[i]*self.td[i]*edotk[i] + integral_part + self.fff[i]
                else:
                    # Compute tau without integral part
                    tau[i] = self.kp[i]*ek[i] + self.kp[i]*self.td[i]*edotk[i] + self.fff[i]

            self.ek_1 = ek
            self.eik_1 = eik
            return tau

        else:
            return None


    def reset(self, now = None):
        self.ek_1 = np.zeros(self.n)
        self.eik_1 = np.zeros(self.n)
        if now != None:
            self.past_time = now


    def resetDof(self, dof, now = None):
        self.ek_1[dof] = 0
        self.eik_1[dof] = 0
        if now != None:
            self.past_time = now


class Trajectory:
    def __init__(self):
        self.loaded = False
        self.init_ned = False

    def load(self, trajectory_type, lat_or_north, lon_or_east, z,
             altitude_mode, mode,  wait, actions):

        assert(len(lat_or_north) == len(lon_or_east) ==
               len(z) == len(altitude_mode))

        if len(actions) != 0:
            assert(len(lat_or_north) == len(actions))
        else:
            actions = ['' for i in range(len(lat_or_north))]

        if len(wait) != 0:
            assert(len(lat_or_north) == len(wait))
        else:
            wait = [0 for i in range(len(lat_or_north))]

        self.trajectory_type = trajectory_type
        if (trajectory_type == 'absolute'):
            self.lat = lat_or_north
            self.lon = lon_or_east
        elif (trajectory_type == 'relative'):
            self.north = lat_or_north
            self.east = lon_or_east
        self.z = z
        self.altitude_mode = altitude_mode
        self.wait = wait
        self.actions = actions
        self.mode = mode
        self.tolerance = 2.0
        self.priority = GoalDescriptor.PRIORITY_NORMAL
        self.loaded = True


    def initNed(self, lat, lon):
        self.ned = NED.NED(lat, lon, 0.0)
        self.init_ned = True
        self.lat_origin = lat
        self.lon_origin = lon


    def getWaypointNed(self, i):
        assert self.init_ned, '[getWaypointNed] First initialize NED system'
        assert self.loaded, '[getWaypointNed] First load a trajectory'
        assert i < len(self.z), '[getWaypointNed] Waypoint out of range'
        if (self.trajectory_type == 'absolute'):
            aux = self.ned.geodetic2ned([self.lat[i], self.lon[i], 0.0])
            aux[NED.DEPTH] = self.z[i]
        elif (self.trajectory_type == 'relative'):
            aux = np.array([self.north[i], self.east[i], self.z[i]])
        return aux


    def computeWaypointTimeout(self, i, current_north, current_east,
                               current_depth, current_altitude, max_vel = 0.4):
        wp = self.getWaypointNed(i)
        if self.altitude_mode[i]:
            distance = sqrt((wp[NED.NORTH] - current_north)**2 +
                            (wp[NED.EAST] - current_east)**2 +
                            (current_altitude - wp[NED.DEPTH])**2)
        else:
            distance = sqrt((wp[NED.NORTH] - current_north)**2 +
                            (wp[NED.EAST] - current_east)**2 +
                            (wp[NED.DEPTH] - current_depth)**2)

        time = (distance / max_vel) * 1.5 + 30
        return time


class ErrorCode:
    INIT = 15
    BAT_WARNING = 14
    BAT_ERROR = 13
    NAV_STS_WARNING = 12
    NAV_STS_ERROR = 11
    INTERNAL_SENSORS_WARNING = 10
    INTERNAL_SENSORS_ERROR = 9
    DVL_BOTTOM_FAIL = 8
    CURRENT_WAYPOINT_BASE = 7 # to 0

    def __init__(self):
        pass

    def unpack(self, error_code):
        error_code_string = format(error_code, '016b')
        
        init = (error_code_string[ErrorCode.INIT] == '1')
        bat_warning = (error_code_string[ErrorCode.BAT_WARNING] == '1')
        bat_error = (error_code_string[ErrorCode.BAT_ERROR] == '1')
        nav_sts_warning = (error_code_string[ErrorCode.NAV_STS_WARNING] == '1')
        nav_sts_error = (error_code_string[ErrorCode.NAV_STS_ERROR] == '1')
        internal_sensors_warning = (error_code_string[ErrorCode.INTERNAL_SENSORS_WARNING] == '1')
        internal_sensors_error = (error_code_string[ErrorCode.INTERNAL_SENSORS_ERROR] == '1')
        dvl_bottom_fail = (error_code_string[ErrorCode.DVL_BOTTOM_FAIL] == '1')
        current_waypoint = int(error_code_string[0:ErrorCode.CURRENT_WAYPOINT_BASE + 1], 2)

        return {'init': init,
        		'bat_warning': bat_warning,
        		'bat_error': bat_error,
        		'nav_sts_warning': nav_sts_warning,
        		'nav_sts_error': nav_sts_error,
        		'internal_sensors_warning': internal_sensors_warning,
        		'internal_sensors_error': internal_sensors_error,
        		'dvl_bottom_fail': dvl_bottom_fail,
        		'current_waypoint': current_waypoint}




def saturateVector(v, min_max) :
    ret = np.zeros( len(v) )
    for i in range( len(v) ) :
        if v[i] < -min_max[i] : ret[i] = -min_max[i]
        elif v[i] > min_max[i] : ret[i] = min_max[i]
        else : ret[i] = v[i]
    return ret


def saturateValue(v, min_max) :
    ret = np.zeros( len(v) )
    for i in range( len(v) ) :
        if v[i] < -min_max : ret[i] = -min_max
        elif v[i] > min_max : ret[i] = min_max
        else : ret[i] = v[i]
    return ret


def saturateValueFloat(v, min_max):
    if v > min_max:
        v = min_max
    elif v < -min_max:
        v = -min_max

    return v

#def computePid6Dof(desired, current, kp, ki, kd, sat, ek_1, eik_1, T):
#    ek = np.zeros(6)
#    eik = np.zeros(6)
#    edotk = np.zeros(6)
#
#    # All the operations are done element by element
#    ek = desired - current
#    edotk = (ek - ek_1) / T
#    eik = eik_1 + (ek * T)
#    eik = saturateVector(eik, sat)
#    #print "ek: \n" + str(ek)
#    #print "eik: \n" + str(eik)
#    #print "edotk: \n" + str(edotk)
#    #print "ek_1: \n" + str(ek_1)
#    #print "eik_1: \n" + str(eik_1)
#
#    # Control law
#    tau = np.zeros(6)
#    tau = kp * ek + ki * eik + kd * edotk
#    tau = saturateValue(tau, 1.0)
#
#    return [tau, ek, eik]
#
#
#def computePid6Dofv2(desired, current, kp, ti, td, fff, ek_1, eik_1, T):
#    #Note: All the operations are done element by element
#    #Compute errors
#    ek = np.zeros(6)
#    eik = np.zeros(6)
#    edotk = np.zeros(6)
#
#    ek = desired - current
#    edotk = (ek - ek_1) / T
#    eik = eik_1 + (ek * T)
#
#    # Control law
#    tau = np.zeros(6)
#    for i in range(6):
#        # Compute the integral part if ti > 0
#        if ti[i] > 0.0:
#            integral_part = (kp[i]/ti[i])*eik[i]
#
#            #Compute tau
#            tau[i] = kp[i]*ek[i] + kp[i]*td[i]*edotk[i] + integral_part + fff[i]
#
#            #Anti-windup condition: if tau is saturated and ek has the same sign than tau, eik does not increment
#            if abs(tau[i]) > 1.0 and ek[i]*tau[i] > 0:
#                eik[i] = eik_1[i]
#
#                #because eik has been modified, recompute tau
#                integral_part = (kp[i]/ti[i])*eik[i]
#                tau[i] = kp[i]*ek[i] + kp[i]*td[i]*edotk[i] + integral_part + fff[i]
#        else:
#            #Compute tau without integral part
#            tau[i] = kp[i]*ek[i] + kp[i]*td[i]*edotk[i] + fff[i]
#
#    #Saturate tau
#    tau = saturateValue(tau, 1.0)
#
#    return [tau, ek, eik, kp[0]*ek[0], (kp[0]/ti[0])*eik[0]]


def normalizeAngle(angle):
    return wrapAngle(angle)

def wrapAngle(angle):  # Should be the default option, "wrap angle" is the proper term in English
    return (angle + ( 2.0 * pi * floor( ( pi - angle ) / ( 2.0 * pi ) ) ) )

def slopeFilter(max_slope, v, v1):
    dy = v - v1
    if dy > max_slope:
        return (v1 + max_slope)
    elif dy < -max_slope:
        return (v1 - max_slope)
    return v


def polyval(poly, f) :
    # TODO: To be matbal compatible. Make it more efficient!
    p = list(poly)
    p.reverse()

    value = 0.0
    change_sign = False
    ret = 0

    if f < 0.0:
        value = abs(f)
        change_sign = True
    else :
        value = f
        change_sign = False

    for i in range(len(p)) :
        ret = ret + pow(value, i) * p[i]

    if change_sign:
        ret = ret * -1.0

    return ret

def smooth_sg(y, window_size, order, deriv=0, rate=1):
    """
    Smooth function created following the Savitzky Golay method only using
    one dimension.
    @param y: Array vector with the data function
    @type y: np.array() Float64
    @param window_size: Size of the window to use to smooth the data
    @type window_size: integer
    @param order: Define the order of the polinomy where to fit the function
    @type order: integer
    @param deriv: optional
    @type deriv: integer
    @param rate: optional
    @type rate: integer
    @return smooth_y: return the smoother function
    @type smooth_y: np.array() Integer
    """
    try:
        window_size = np.abs(np.int(window_size))
        order = np.abs(np.int(order))
    except ValueError, msg:
        raise ValueError("window_size and order have to be of type int")
    if window_size % 2 != 1 or window_size < 1:
        raise TypeError("window_size size must be a positive odd number")
    if window_size < order + 2:
        raise TypeError("window_size is too small for the polynomials order")
    order_range = range(order+1)
    half_window = (window_size -1) // 2
    # precompute coefficients
    b = np.mat([[k**i for i in order_range] for k in range(-half_window, half_window+1)])
    m = np.linalg.pinv(b).A[deriv] * rate**deriv * factorial(deriv)
    # pad the signal at the extremes with
    # values taken from the signal itself
    firstvals = y[0] - np.abs( y[1:half_window+1][::-1] - y[0] )
    lastvals = y[-1] + np.abs(y[-half_window-1:-1][::-1] - y[-1])
    y = np.concatenate((firstvals, y, lastvals))
    return np.convolve( m[::-1], y, mode='valid')


def _datacheck_peakdetect(x_axis, y_axis):
    """
    Internal function of the peakdetect. it checks the integrity of the data
    @param x_axis: Values of the function usually the time
    @type x_axis: array float64
    @param y_axis: Values of the function usually the position or distance
    @type y_axis: array float64
    @param return: Return the x an y axis fi they are from correct lenght
    @type return: np.array, np.array
    """
    if x_axis is None:
        x_axis = range(len(y_axis))

    if len(y_axis) != len(x_axis):
        raise (ValueError,
                'Input vectors y_axis and x_axis must have same length')

    #needs to be a numpy array
    y_axis = np.array(y_axis)
    x_axis = np.array(x_axis)
    return x_axis, y_axis


def peakdetect(y_axis, x_axis = None, lookahead = 300, delta=0):
    """
    Converted from/based on a MATLAB script at:
    http://billauer.co.il/peakdet.html

    function for detecting local maximas and minmias in a signal.
    Discovers peaks by searching for values which are surrounded by lower
    or larger values for maximas and minimas respectively

    keyword arguments:
    y_axis -- A list containg the signal over which to find peaks
    x_axis -- (optional) A x-axis whose values correspond to the y_axis list
        and is used in the return to specify the postion of the peaks. If
        omitted an index of the y_axis is used. (default: None)
    lookahead -- (optional) distance to look ahead from a peak candidate to
        determine if it is the actual peak (default: 200)
        '(sample / period) / f' where '4 >= f >= 1.25' might be a good value
    delta -- (optional) this specifies a minimum difference between a peak and
        the following points, before a peak may be considered a peak. Useful
        to hinder the function from picking up false peaks towards to end of
        the signal. To work well delta should be set to delta >= RMSnoise * 5.
        (default: 0)
            delta function causes a 20% decrease in speed, when omitted
            Correctly used it can double the speed of the function

    return -- two lists [max_peaks, min_peaks] containing the positive and
        negative peaks respectively. Each cell of the lists contains a tupple
        of: (position, peak_value)
        to get the average peak value do: np.mean(max_peaks, 0)[1] on the
        results to unpack one of the lists into x, y coordinates do:
        x, y = zip(*tab)
    """
    max_peaks = []
    min_peaks = []
    dump = []   #Used to pop the first hit which almost always is false

    # check input data
    x_axis, y_axis = _datacheck_peakdetect(x_axis, y_axis)
    # store data length for later use
    length = len(y_axis)


    #perform some checks
    if lookahead < 1:
        raise ValueError, "Lookahead must be '1' or above in value"
    if not (np.isscalar(delta) and delta >= 0):
        raise ValueError, "delta must be a positive number"

    #maxima and minima candidates are temporarily stored in
    #mx and mn respectively
    mn, mx = np.Inf, -np.Inf

    #Only detect peak if there is 'lookahead' amount of points after it
    for index, (x, y) in enumerate(zip(x_axis[:-lookahead],
                                        y_axis[:-lookahead])):
        if y > mx:
            mx = y
            mxpos = x
        if y < mn:
            mn = y
            mnpos = x

        ####look for max####
        if y < mx-delta and mx != np.Inf:
            #Maxima peak candidate found
            #look ahead in signal to ensure that this is a peak and not jitter
            if y_axis[index:index+lookahead].max() < mx:
                max_peaks.append([mxpos, mx])
                dump.append(True)
                #set algorithm to only find minima now
                mx = np.Inf
                mn = np.Inf
                if index+lookahead >= length:
                    #end is within lookahead no more peaks can be found
                    break
                continue
            #else:  #slows shit down this does
            #    mx = ahead
            #    mxpos = x_axis[np.where(y_axis[index:index+lookahead]==mx)]

        ####look for min####
        if y > mn+delta and mn != -np.Inf:
            #Minima peak candidate found
            #look ahead in signal to ensure that this is a peak and not jitter
            if y_axis[index:index+lookahead].min() > mn:
                min_peaks.append([mnpos, mn])
                dump.append(False)
                #set algorithm to only find maxima now
                mn = -np.Inf
                mx = -np.Inf
                if index+lookahead >= length:
                    #end is within lookahead no more peaks can be found
                    break
            #else:  #slows shit down this does
            #    mn = ahead
            #    mnpos = x_axis[np.where(y_axis[index:index+lookahead]==mn)]


    #Remove the false hit on the first value of the y_axis
    try:
        if dump[0]:
            max_peaks.pop(0)
        else:
            min_peaks.pop(0)
        del dump
    except IndexError:
        #no peaks were found, should the function return empty lists?
        pass

    return [max_peaks, min_peaks]

def test() :
    print "SaturateVector:"
    print str(saturateVector([1.8,0.3,-3.2, -0.7], np.ones(4)))

    print "SaturateValue:"
    print str(saturateValue([1.8,0.3,-3.2, -0.7], 1.0))

    print "Normalize angle 7.14 = " + str(normalizeAngle(7.14))

if __name__ == '__main__':
    test()
