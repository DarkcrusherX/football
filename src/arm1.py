import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
#rospy.init_node('arm_and_takeoff_node', anonymous=True)
class armtakeoff1():

    def __init__(self):

        def state_cb(state):
            self.current_state1 = state
        
        self.current_state1 = State()
        self.local_pos_pub1 = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.state_sub1 = rospy.Subscriber('/uav1/mavros/state', State, state_cb)  # $This topic was wrong
        self.arming_client1 = rospy.ServiceProxy('/uav1/mavros/cmd/arming', CommandBool)
        self.set_mode_client1 = rospy.ServiceProxy('uav1/mavros/set_mode', SetMode)
     
    
    def arm1(self):

        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 3

        prev_state = self.current_state1
        rate = rospy.Rate(20.0) # MUST be more then 2Hz

        # send a few setpoints before starting
        for i in range(100):
            self.local_pos_pub1.publish(pose)
        rate.sleep()
        print(self.current_state1)
        # wait for FCU connection
        while not self.current_state1.connected:
            print("Sleeping")
            rate.sleep()

        last_request = rospy.get_rostime()

        while (self.current_state1.armed != True) or (self.current_state1.mode !="OFFBOARD"): 
        #    print("Arm {}" .format(self.current_state1.armed))            
            now = rospy.get_rostime()
            if self.current_state1.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
                self.set_mode_client1(base_mode=0, custom_mode="OFFBOARD")
                last_request = now 
            else:
                if not self.current_state1.armed and (now - last_request > rospy.Duration(5.)):
                    self.arming_client1(True)
                    last_request = now 

            # older versions of PX4 always return success==True, so better to check Status instead
            if prev_state.armed != self.current_state1.armed:
                rospy.loginfo("Vehicle armed: %r" % self.current_state1.armed)
            if prev_state.mode != self.current_state1.mode: 
                rospy.loginfo("Current mode: %s" % self.current_state1.mode)
            prev_state = self.current_state1

            # Update timestamp and publish pose 
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = 0
            if prev_state.armed == self.current_state1.armed:
                for i in range(1000):
                    self.local_pos_pub1.publish(pose)     
            rate.sleep()

    def disarm(self):

        prev_state = self.current_state1
        rate = rospy.Rate(20.0) # MUST be more then 2Hz

        rate.sleep()
        print(self.current_state1)
        # wait for FCU connection
        while not self.current_state1.connected:
            print("Sleeping")
            rate.sleep()

        last_request = rospy.get_rostime()

        while self.current_state1.armed != False: 
            #print("Got in")
        #    print("Arm {}" .format(self.current_state1.armed))
            now = rospy.get_rostime()
            if self.current_state1.mode != "AUTO.LAND" and (now - last_request > rospy.Duration(5.)):
                self.set_mode_client1(base_mode=0, custom_mode="AUTO.LAND")
                last_request = now 
            else:
                if not self.current_state1.armed and (now - last_request > rospy.Duration(5.)):
                    self.arming_client1(False)
                    last_request = now 
        rate.sleep()            