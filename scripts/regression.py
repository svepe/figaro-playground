#!/usr/bin/python

import rospy
from std_msgs.msg import String
from ros_figaro.msg import *
from ros_figaro.srv import *

import numpy as np
import matplotlib.pyplot as plt

class RegressionPlot:

  def __init__(self):
    rospy.init_node('regression_plot')
    self.data_pub = rospy.Publisher('~data', String, queue_size=10)
    self.data = np.empty(shape=(0, 2))
    self.run_regression = rospy.ServiceProxy('/regression/run', RunRegression2)
  def onclick(self, event):
      # print 'button=%d, x=%d, y=%d, xdata=%f, ydata=%f'%(
      #   event.button, event.x, event.y, event.xdata, event.ydata)
      self.data = np.append(self.data, [[event.xdata, event.ydata]], axis = 0)

      req = RunRegression2Request()
      req.prior_w0.mean = 0;
      req.prior_w0.variance = 5;
      req.prior_w1.mean = 0;
      req.prior_w1.variance = 5;

      for i in xrange(self.data.shape[0]):
        dp = DataPoint2()
        dp.x = self.data[i, 0]
        dp.y = self.data[i, 1]
        req.observations.append(dp)

      resp = self.run_regression(req)

      print resp


  def loop(self):
      plt.ion()
      plt.figure().canvas.mpl_connect('button_press_event', self.onclick)
      while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        self.data_pub.publish(hello_str)

        plt.cla()
        plt.gca().set_xlim([-10, 10])
        plt.gca().set_ylim([-10, 10])
        plt.scatter(self.data[:, 0], self.data[:, 1])
        plt.show()
        plt.pause(0.01)

if __name__ == '__main__':
    try:
        p = RegressionPlot()
        p.loop()
    except rospy.ROSInterruptException:
        pass
