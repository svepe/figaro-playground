#!/usr/bin/python

import rospy
from std_msgs.msg import String

import numpy as np
import matplotlib.pyplot as plt

class RegressionPlot:

  def __init__(self):
    rospy.init_node('regression')
    self.data_pub = rospy.Publisher('~data', String, queue_size=10)
    self.data = np.empty(shape=(0, 2))



  def onclick(self, event):
      # print 'button=%d, x=%d, y=%d, xdata=%f, ydata=%f'%(
      #   event.button, event.x, event.y, event.xdata, event.ydata)
      self.data = np.append(self.data, [[event.xdata, event.ydata]], axis = 0)

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
