import rospy
from gazebo_msgs.msg import LinkState
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist 

#rospy.init_node('test', anonymous=True)

class batlink():

  def __init__(self):
        
    self.pub_link = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=10)
    self.current_pose = Pose()
    self.current_twist = Twist()
    self.links = LinkStates()
    self.j=1  

  def bat(self,name):

    def callback(link_msg):
      self.links = link_msg
      self.j = 5
      i=0
      while self.links.name[i] != 'iris::bat':
        i=i+1

      self.current_pose = self.links.pose[i]
      self.current_twist = self.links.twist[i]

    for i in range(20000):
      sub = rospy.Subscriber("/gazebo/link_states",LinkStates,callback)
    batting = LinkState()
    batting.pose = self.current_pose
    batting.twist = self.current_twist
    batting.link_name = name
    batting.pose.orientation.y = batting.pose.orientation.y + 0.4
    print(self.current_pose)
    print(batting)
    self.pub_link.publish(batting)
