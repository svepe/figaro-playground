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
    self.run_regression = rospy.ServiceProxy(
      '/regression/run', RunRegression2, persistent=True)

    self.prior_w0 = Normal2()
    self.prior_w0.mean = 0
    self.prior_w0.variance = 1000

    self.posterior_w0 = Normal2()
    self.posterior_w0.mean = 0
    self.posterior_w0.variance = 1000

    self.prior_w1 = Normal2()
    self.prior_w1.mean = 0
    self.prior_w1.variance = 1000

    self.posterior_w1 = Normal2()
    self.posterior_w1.mean = 0
    self.posterior_w1.variance = 1000

    self.samples_w0 = []
    self.samples_w1 = []

    self.cbar = None

  def onclick(self, event):
      if self.fig.axes[0] is not event.inaxes:
        return

      self.data = np.append(self.data, [[event.xdata, event.ydata]], axis=0)

      req = RunRegression2Request()
      req.prior_w0.mean = self.posterior_w0.mean
      req.prior_w0.variance = self.posterior_w0.variance
      req.prior_w1.mean = self.posterior_w1.mean
      req.prior_w1.variance = self.posterior_w1.variance

      req.observation.x = self.data[-1, 0]
      req.observation.y = self.data[-1, 1]

      time1 = time.time()
      resp = self.run_regression(req)
      time2 = time.time()
      print 'Regression took %0.3f ms' % ((time2-time1)*1000.0)

      self.prior_w0.mean = self.posterior_w0.mean
      self.prior_w0.variance = self.posterior_w0.variance
      self.prior_w1.mean = self.posterior_w1.mean
      self.prior_w1.variance = self.posterior_w1.variance

      self.posterior_w0.mean = resp.posterior_w0.mean
      self.posterior_w0.variance = resp.posterior_w0.variance
      self.posterior_w1.mean = resp.posterior_w1.mean
      self.posterior_w1.variance = resp.posterior_w1.variance

      self.samples_w0 = resp.samples_w0
      self.samples_w1 = resp.samples_w1

  def plot_data(self):
    # Plot datapoints and fitted line
    plt.subplot(3, 1, 1)
    plt.cla()
    plt.grid(which='major', color='0.65', linestyle='-')
    plt.grid(which='minor', color='0.35', linestyle='--')
    plt.title('$w_0 + w_1 x$')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.gca().set_xlim([-10, 10])
    plt.gca().set_ylim([-10, 10])
    plt.scatter(self.data[:, 0], self.data[:, 1])
    x = np.arange(-10, 10, 0.1)
    y = self.posterior_w0.mean + self.posterior_w1.mean * x
    plt.plot(x, y, 'r')

  def plot_prior(self):
    # Plot prior and sampled weights
    plt.subplot(3, 1, 2)
    plt.cla()
    plt.grid(which='major', color='0.65', linestyle='-')
    plt.grid(which='minor', color='0.35', linestyle='--')
    plt.title('Prior')
    plt.xlabel('$w_0$')
    plt.ylabel('$w_1$')

    plt.scatter(
      np.asarray([w.value for w in self.samples_w0]),
      np.asarray([w.value for w in self.samples_w1]),
      c=np.asarray([w.weight for w in self.samples_w0]),
      cmap=plt.cm.get_cmap('RdYlBu_r'))

    l = plt.cm.ScalarMappable(cmap=plt.cm.get_cmap('RdYlBu_r'))
    l.set_array(
      [w.weight for w in self.samples_w0 if np.isfinite(w.weight)])
    cb = plt.colorbar(l)
    cb.set_label('Sample likelihood')

    (xmin, xmax) = plt.gca().get_xlim()
    (ymin, ymax) = plt.gca().get_ylim()
    pdf_x, pdf_y = np.meshgrid(
      np.arange(xmin, xmax, 0.1), np.arange(ymin, ymax, 0.1))
    pdf = mlab.bivariate_normal(
      pdf_x, pdf_y,
      math.sqrt(self.prior_w0.variance), math.sqrt(self.prior_w1.variance),
      self.prior_w0.mean, self.prior_w1.mean, sigmaxy=0.0)
    plt.contour(pdf_x, pdf_y, pdf)

  def plot_posterior(self):
    plt.subplot(3, 1, 3)
    plt.cla()
    plt.grid(which='major', color='0.65', linestyle='-')
    plt.grid(which='minor', color='0.35', linestyle='--')
    plt.title('Posterior')
    plt.xlabel('$w_0$')
    plt.ylabel('$w_1$')

    plt.gca().set_xlim(self.fig.axes[1].get_xlim())
    plt.gca().set_ylim(self.fig.axes[1].get_ylim())

    (xmin, xmax) = plt.gca().get_xlim()
    (ymin, ymax) = plt.gca().get_ylim()

    pdf_x, pdf_y = np.meshgrid(
      np.arange(xmin, xmax, 0.1), np.arange(ymin, ymax, 0.1))

    pdf = mlab.bivariate_normal(
      pdf_x, pdf_y,
      math.sqrt(self.posterior_w0.variance),
      math.sqrt(self.posterior_w1.variance),
      self.posterior_w0.mean,
      self.posterior_w1.mean,
      sigmaxy=0.0)

    plt.contour(pdf_x, pdf_y, pdf)

    l = plt.cm.ScalarMappable(cmap=plt.cm.get_cmap('RdYlBu_r'))
    l.set_array(pdf)
    cb = plt.colorbar(l)
    cb.set_label('Posterior PDF')

  def loop(self):
      plt.ion()

      self.fig = plt.figure()
      self.fig.canvas.mpl_connect('button_press_event', self.onclick)
      while not rospy.is_shutdown():
        self.plot_data()
        self.plot_prior()
        self.plot_posterior()
        plt.show()
        plt.pause(0.1)
        self.fig.clear()

if __name__ == '__main__':
    try:
        p = RegressionPlot()
        p.loop()
    except rospy.ROSInterruptException:
        pass
