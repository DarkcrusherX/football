import rospy
from arm1 import armtakeoff1
from controlarm import batlink
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

rospy.init_node('player1', anonymous=True)
mid = PoseStamped()
vel = TwistStamped()
bat = batlink()
current_pos = PoseStamped()

def callback(msg):
    global mid
    mid= msg

def current_pos_callback(position):

    global current_pos
    current_pos = position   
    


def main():
    rospy.Subscriber("/player1/midpoint",PoseStamped,callback)
    arm1 = armtakeoff1()
    arm1.arm1()
    pub_velocity1=rospy.Publisher('/uav1/mavros/setpoint_velocity/cmd_vel', TwistStamped,queue_size=5)
    #local_pos_pub1 = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=5)
    midx = mid.pose.position.x
    midy = mid.pose.position.y
    i=0
    while True:
        rospy.Subscriber('/uav1/mavros/local_position/pose',PoseStamped,current_pos_callback)
        rospy.Subscriber("/player1/midpoint",PoseStamped,callback)
        midx = mid.pose.position.x
        midy = mid.pose.position.y
        if midx==0 and midy ==0 and i==0:
            vel.twist.angular.z =2
            print("Searching")
        elif midx!=0 and midy !=0 :
            if midx<360 and midx >280:
                vel.twist.linear.x = 3
                print("Going to get")
                i=1                       
            else:
                vel.twist.linear.y = -1*(midx-320)/40
                print("Adjusting")         
        elif midx==0 and midy ==0 and i!=0:
                for j in range(3):
                    bat.bat('player1::bat')
                i=0
                print("batting")

        if current_pos.pose.position.z <1.8:
            vel.twist.linear.z = 2          

        pub_velocity1.publish(vel)
        vel.twist.angular.z =0
        vel.twist.linear.x = 0
        vel.twist.linear.y = 0



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass    
