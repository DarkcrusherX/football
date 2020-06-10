import rospy
from arm0 import armtakeoff0
from arm1 import armtakeoff1
rospy.init_node('main', anonymous=True)
arm0 = armtakeoff0()
arm0.arm()
arm1 = armtakeoff1()
arm1.arm()