import rospy
# import actionlib
import moveit_commander
import time
import threading

from tf.transformations import *

import tf

a = tf.transformations.euler_from_quaternion([0.7057174,0.1089564,0.0219912,0.6997198])
print(a)