#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import sys
import rospy
from nav_msgs.msg import Odometry
import time


# AD stands for ArrayData
class AD(enumerate):
    TIME = 0  # time = '%time'
    HS = 1  # hs = 'field.header.seq'
    FHS = 2  # fhs = 'field.header.stamp'  # stamp for calculating differentiation
    VEL_X = 3  # velocity x-direction
    VEL_Y = 4  # velocity y-direction
    OME_Z = 5  # omega around z-axis
    POS_X = 6  # position x-axis
    POS_Y = 7  # position y-axis


# colours in terminal prints
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def smooth(x, window_len=11, window='hanning'):
    """smooth the data using a window with requested size.

    This method is based on the convolution of a scaled window with the signal.
    The signal is prepared by introducing reflected copies of the signal
    (with the window size) in both ends so that transient parts are minimized
    in the begining and end part of the output signal.

    input:
        x: the input signal
        window_len: the dimension of the smoothing window; should be an odd integer
        window: the type of window from 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'
            flat window will produce a moving average smoothing.

    output:
        the smoothed signal

    example:

    t=linspace(-2,2,0.1)
    x=sin(t)+randn(len(t))*0.1
    y=smooth(x)

    see also:

    numpy.hanning, numpy.hamming, numpy.bartlett, numpy.blackman, numpy.convolve
    scipy.signal.lfilter

    TODO: the window parameter could be the window itself if an array instead of a string
    NOTE: length(output) != length(input), to correct this: return y[(window_len/2-1):-(window_len/2)] instead of just y.
    """

    if x.ndim != 1:
        raise ValueError, "smooth only accepts 1 dimension arrays."

    if x.size < window_len:
        raise ValueError, "Input vector needs to be bigger than window size."

    if window_len < 3:
        return x

    if not window in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
        raise ValueError, "Window is on of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'"

    s = np.r_[x[window_len - 1:0:-1], x, x[-2:-window_len - 1:-1]]
    # print(len(s))
    if window == 'flat':  # moving average
        w = np.ones(window_len, 'd')
    else:
        w = eval('np.' + window + '(window_len)')

    y = np.convolve(w / w.sum(), s, mode='valid')

    return y[(window_len / 2 - 1):-(window_len / 2)]
    # return y


class CalculateJerkParamHandler:
    def __init__(self):
        """
        Class for returning the corresponding metric class with the given parameter.
        """
        pass

    def parse_parameter(self, testblock_name, params):
        """
        Method that returns the metric method with the given parameter.
        :param params: Parameter
        """
        metrics = []
        if type(params) is not list:
            rospy.logerr("metric config not a list")
            return False

        for metric in params:
            # check for optional parameters
            try:
                groundtruth = metric["groundtruth"]
                groundtruth_epsilon = metric["groundtruth_epsilon"]
            except (TypeError, KeyError):
                rospy.logwarn(
                    "No groundtruth parameters given, skipping groundtruth evaluation for metric 'jerk' in testblock '%s'",
                    testblock_name)
                groundtruth = None
                groundtruth_epsilon = None
            metrics.append(CalculateJerk(metric["topic"], groundtruth, groundtruth_epsilon))
            # metrics.append(CalculateJerk(groundtruth, groundtruth_epsilon))
        return metrics


class CalculateJerk:
    def __init__(self, topic, groundtruth, groundtruth_epsilon):
        self.active = False
        self.finished = False
        # self.topic = '/base/odometry_controller/odometry'
        self.topic = topic
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        self.start_time = None
        self.stop_time = None
        self.smo_para = 30
        # create array for further use
        self.A_listener = np.ones([0, 8], dtype=np.double)
        # queue_size=None --> queue size is infinite, needed to calculate the right jerk
        rospy.Subscriber(self.topic, Odometry, self.callback, queue_size=None)

        self.A_grad_smo_jerk = np.ones([0, 8], dtype=np.double)


    # def listener(self):
    #     # rospy.spin()
    #     while not rospy.is_shutdown():
    #         # check if idle time is too long and then shutdown node
    #         if time.time() - self.start_time >= 3:
    #             rospy.signal_shutdown('idle time too long')
    #         print 'Idle Time: %.2f' % (time.time() - self.start_time)
    #         rospy.sleep(0.25)

    def callback(self, msg):
        if self.active:
            data_list = [-1,
                         float(msg.header.seq),
                         (float(msg.header.stamp.secs) * 10 ** 9 + float(msg.header.stamp.nsecs)) * 10 ** -9,
                         float(msg.twist.twist.linear.x),
                         float(msg.twist.twist.linear.y),
                         float(msg.twist.twist.angular.z),
                         float(msg.pose.pose.position.x),
                         float(msg.pose.pose.position.y)]

            # append data to array
            self.A_listener = np.append(self.A_listener,
                                        [[data_list[0], data_list[1], data_list[2], data_list[3],
                                          data_list[4], data_list[5], data_list[6], data_list[7]]],
                                        axis=0)

    def start(self, timestamp):
        self.active = True
        self.start_time = timestamp
        # self.listener()
        rospy.loginfo(bcolors.FAIL+'----calc_jerk.py----'+bcolors.ENDC)

    def stop(self, timestamp):
        self.active = False
        self.stop_time = timestamp
        self.finished = True

#        rospy.loginfo('\033[94m' + '=' * 82 + '\033[0m')
#        result = self.get_result()
#        rospy.loginfo('\033[94m' + '=' * 82 + '\033[0m')
#        rospy.loginfo('\033[94m' + 'Result: ' + str(result) + '\033[0m')
#        rospy.loginfo('\033[94m' + '=' * 82 + '\033[0m')

    def pause(self, timestamp):
        # TODO: Implement pause time and counter calculation
        # FIXME: check rate calculation in case of pause (counter, start_time and stop_time)
        pass

    def purge(self, timestamp):
        pass

    # get differentiation from given data
    def differentiation(self):

        print bcolors.OKBLUE + 'Got this array: ', self.A_listener.shape, bcolors.ENDC

        # set time to start at 0s
        self.A_listener[:, AD.FHS] = self.A_listener[:, AD.FHS] - self.A_listener[0, AD.FHS]

        # differentiation
        # compute acceleration from velocity by differentiation
        A_grad_acc_x = np.gradient(self.A_listener[:, AD.VEL_X], self.A_listener[1, AD.FHS] - self.A_listener[0, AD.FHS])
        A_grad_acc_y = np.gradient(self.A_listener[:, AD.VEL_Y], self.A_listener[1, AD.FHS] - self.A_listener[0, AD.FHS])

        # differentiation
        # compute jerk from acceleration by differentiation using smoothed acc
        A_grad_smo_jerk_x = np.gradient(smooth(A_grad_acc_x[:, ], self.smo_para, window='hanning'),
                                        self.A_listener[1, AD.FHS] - self.A_listener[0, AD.FHS])
        A_grad_smo_jerk_y = np.gradient(smooth(A_grad_acc_y[:, ], self.smo_para, window='hanning'),
                                        self.A_listener[1, AD.FHS] - self.A_listener[0, AD.FHS])
        # (x^2+y^2)^0.5 to get absolute jerk
        A_grad_smo_jerk = np.sqrt(A_grad_smo_jerk_x[:, ] ** 2 + A_grad_smo_jerk_y[:, ] ** 2)

        self.A_grad_smo_jerk = A_grad_smo_jerk

    def get_result(self):
        groundtruth_result = None
        details = {"topic": self.topic}
        if self.finished:
            if self.groundtruth != None and self.groundtruth_epsilon != None:
                self.differentiation()
                for i in xrange(0, self.A_grad_smo_jerk.shape[0]):
                    if self.A_grad_smo_jerk[i,] >= self.groundtruth_epsilon:
                        output = bcolors.FAIL + 'Jerk: {:.3f} [m/s^3] at time: {:.6f} s is bigger than max ' \
                                                'allowed jerk: {:.3f} [m/s^3]' + bcolors.ENDC
                        print output.format(self.A_grad_smo_jerk[i,], self.A_listener[i, AD.FHS], self.groundtruth_epsilon)
                        print 'Max Jerk: {:.4f} [m/s^3]'.format(self.A_grad_smo_jerk.max())
                        data = float(self.A_grad_smo_jerk.max())
                        groundtruth_result = False
                        break
                    else:
                        data = float(self.A_grad_smo_jerk.max())
                        groundtruth_result = True
                if groundtruth_result:
                    print bcolors.OKGREEN + 'Jerk is in desired range!' + bcolors.ENDC
                    print 'Max Jerk: {:.4f} [m/s^3]'.format(self.A_grad_smo_jerk.max())
            return "jerk", data, groundtruth_result, self.groundtruth, self.groundtruth_epsilon, details
        else:
            return False
