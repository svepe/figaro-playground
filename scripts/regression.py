#!/usr/bin/python

import rospy
from std_msgs.msg import String
from ros_figaro.msg import *
from ros_figaro.srv import *

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import math
import time


class RegressionPlot:

  def __init__(self):
    rospy.init_node('regression_plot')
    self.data_pub = rospy.Publisher('~data', String, queue_size=10)
    self.data = np.empty(shape=(0, 2))
    self.run_regression = rospy.ServiceProxy('/regression/run', RunRegression2)
    self.w0 = Normal2()
    self.w0.mean = 0
    self.w0.variance = 1000

    self.w1 = Normal2()
    self.w1.mean = 0
    self.w1.variance = 1000

    self.samples_w0 = []
    self.samples_w1 = []

  def onclick(self, event):
      if self.fig.axes[0] is not event.inaxes:
        return

      self.data = np.append(self.data, [[event.xdata, event.ydata]], axis=0)

      req = RunRegression2Request()
      req.prior_w0.mean = self.w0.mean
      req.prior_w0.variance = self.w0.variance
      req.prior_w1.mean = self.w1.mean
      req.prior_w1.variance = self.w1.variance

      req.observation.x = self.data[-1, 0]
      req.observation.y = self.data[-1, 1]

      time1 = time.time()
      resp = self.run_regression(req)
      time2 = time.time()
      print 'Regression took %0.3f ms' % ((time2-time1)*1000.0)

      self.w0.mean = resp.posterior_w0.mean
      self.w0.variance = resp.posterior_w0.variance
      self.w1.mean = resp.posterior_w1.mean
      self.w1.variance = resp.posterior_w1.variance

      self.samples_w0 = resp.samples_w0
      self.samples_w1 = resp.samples_w1


  def loop(self):
      plt.ion()
      self.fig = plt.figure()
      self.fig.canvas.mpl_connect('button_press_event', self.onclick)
      while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        self.data_pub.publish(hello_str)
        plt.subplot(2, 1, 1)
        plt.cla()
        plt.title('Data')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.gca().set_xlim([-10, 10])
        plt.gca().set_ylim([-10, 10])
        plt.scatter(self.data[:, 0], self.data[:, 1])
        x = np.arange(-10, 10, 0.1)
        y = self.w0.mean + self.w1.mean * x
        plt.plot(x, y, 'r')

        plt.subplot(2, 1, 2)
        plt.cla()
        plt.title('w0 + w1 * x')
        plt.xlabel('w0')
        plt.ylabel('w1')


        plt.scatter(
          np.asarray([w.value for w in self.samples_w0]),
          np.asarray([w.value for w in self.samples_w1]),
          c=np.asarray([-w.weight for w in self.samples_w0]),
          cmap=plt.cm.get_cmap('RdYlBu'))


        (xmin, xmax) = plt.gca().get_xlim()
        (ymin, ymax) = plt.gca().get_ylim()

        pdf_x, pdf_y = np.meshgrid(
          np.arange(xmin, xmax, 0.1), np.arange(ymin, ymax, 0.1))
        pdf = mlab.bivariate_normal(
          pdf_x, pdf_y,
          math.sqrt(self.w0.variance), math.sqrt(self.w1.variance),
          self.w0.mean, self.w1.mean, sigmaxy=0.0)
        plt.contour(pdf_x, pdf_y, pdf)
        # plt.colorbar(plt.gca())


        plt.show()
        plt.pause(0.01)

if __name__ == '__main__':
    try:
        p = RegressionPlot()
        p.loop()
    except rospy.ROSInterruptException:
        pass
