import rospy
from arm0 import armtakeoff0
from controlarm import batlink
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

rospy.init_node('player0', anonymous=True)
mid = PoseStamped()
vel = TwistStamped()
bat = batlink()
pos = PoseStamped()

def callback(msg):
    global mid
    mid= msg
    


def main():
    rospy.Subscriber("/player0/midpoint",PoseStamped,callback)
    arm0 = armtakeoff0()
    arm0.arm0()
    pub_velocity0=rospy.Publisher('/uav0/mavros/setpoint_velocity/cmd_vel', TwistStamped,queue_size=20)
    local_pos_pub0 = rospy.Publisher('uav0/mavros/setpoint_position/local', PoseStamped, queue_size=20)
    pos.pose.position.z = 1.8
    midx = mid.pose.position.x
    midy = mid.pose.position.y
    i=0
    while True:
        if midx==0 and midy ==0 and i==0:
            vel.twist.angular.z =2
            pub_velocity0.publish(vel)
            print("Searching")
        elif midx!=0 and midy !=0 :
            if midx<340 and midx >300:
                vel.twist.linear.x = 3
                print("Going to get")
                i=1
            else:
                vel.twist.linear.y = (midx-320)/100
                print("Adjusting")
        elif midx==0 and midy ==0 and i!=0:
                for j in range(3):
                    bat.bat('player0::bat')
                i=0
                print("batting")
        pub_velocity0.publish(vel)
        local_pos_pub0.publish(pos)



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass    
