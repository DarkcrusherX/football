import rospy
from controlarm import batlink

arm = batlink()
arm.bat('iris::bat')