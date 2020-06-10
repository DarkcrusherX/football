import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
#rospy.init_node('arm_and_takeoff_node', anonymous=True)
class armtakeoff0():

    def __init__(self):

        def state_cb(state):
            self.current_state0 = state
        
        self.current_state0 = State()
        self.local_pos_pub0 = rospy.Publisher('uav0/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.state_sub0= rospy.Subscriber('/uav0/mavros/state', State, state_cb)  # $This topic was wrong
        self.arming_client0 = rospy.ServiceProxy('/uav0/mavros/cmd/arming', CommandBool)
        self.set_mode_client0 = rospy.ServiceProxy('uav0/mavros/set_mode', SetMode)
     
    
    def arm0(self):

        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 3

        prev_state = self.current_state0
        rate = rospy.Rate(20.0) # MUST be more then 2Hz

        # send a few setpoints before starting
        for i in range(100):
            self.local_pos_pub0.publish(pose)
        rate.sleep()
        print(self.current_state0)
        # wait for FCU connection
        while not self.current_state0.connected:
            print("Sleeping")
            rate.sleep()

        last_request = rospy.get_rostime()

        while (self.current_state0.armed != True) or (self.current_state0.mode !="OFFBOARD"): 
        #    print("Arm {}" .format(self.current_state0.armed))            
            now = rospy.get_rostime()
            if self.current_state0.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
                self.set_mode_client0(base_mode=0, custom_mode="OFFBOARD")
                last_request = now 
            else:
                if not self.current_state0.armed and (now - last_request > rospy.Duration(5.)):
                    self.arming_client0(True)
                    last_request = now 

            # older versions of PX4 always return success==True, so better to check Status instead
            if prev_state.armed != self.current_state0.armed:
                rospy.loginfo("Vehicle armed: %r" % self.current_state0.armed)
            if prev_state.mode != self.current_state0.mode: 
                rospy.loginfo("Current mode: %s" % self.current_state0.mode)
            prev_state = self.current_state0

            # Update timestamp and publish pose 
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = 0
            if prev_state.armed == self.current_state0.armed:
                for i in range(1000):
                    self.local_pos_pub0.publish(pose)     
            rate.sleep()

    def disarm(self):

        prev_state = self.current_state0
        rate = rospy.Rate(20.0) # MUST be more then 2Hz

        rate.sleep()
        print(self.current_state0)
        # wait for FCU connection
        while not self.current_state0.connected:
            print("Sleeping")
            rate.sleep()

        last_request = rospy.get_rostime()

        while self.current_state0.armed != False: 
            #print("Got in")
        #    print("Arm {}" .format(self.current_state0.armed))
            now = rospy.get_rostime()
            if self.current_state0.mode != "AUTO.LAND" and (now - last_request > rospy.Duration(5.)):
                self.set_mode_client0(base_mode=0, custom_mode="AUTO.LAND")
                last_request = now 
            else:
                if not self.current_state0.armed and (now - last_request > rospy.Duration(5.)):
                    self.arming_client0(False)
                    last_request = now 
        rate.sleep()            