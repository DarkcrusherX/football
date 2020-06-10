import rospy
from arm1 import armtakeoff1
from controlarm import batlink
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

rospy.init_node('player1', anonymous=True)
mid = PoseStamped()
vel = TwistStamped()
bat = batlink()
pos = PoseStamped()

def callback(msg):
    global mid
    mid= msg
    


def main():
    rospy.Subscriber("/player1/midpoint",PoseStamped,callback)
    arm1 = armtakeoff1()
    arm1.arm1()
    pub_velocity1=rospy.Publisher('/uav1/mavros/setpoint_velocity/cmd_vel', TwistStamped,queue_size=20)
    local_pos_pub1 = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=20)
    pos.pose.position.z = 1.8
    midx = mid.pose.position.x
    midy = mid.pose.position.y
    i=0
    while True:
        if midx==0 and midy ==0 and i==0:
            vel.twist.angular.z =2
            pub_velocity1.publish(vel)
            print("Searching")
        elif midx!=0 and midy !=0 :
            if midx<340 and midx >300:
                vel.twist.linear.x = 3
                print("Going to get")
                i=1               
            else:
                vel.twist.linear.y = -1*(midx-320)/100
                print("Adjusting")
        elif midx==0 and midy ==0 and i!=0:
                for j in range(3):
                    bat.bat('player1::bat')
                i=0
                print("batting")
        pub_velocity1.publish(vel)
        local_pos_pub1.publish(pos)



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass    
