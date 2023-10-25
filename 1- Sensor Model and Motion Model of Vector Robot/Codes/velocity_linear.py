#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import pandas as pd


x = 0.0
y = 0.0 
theta = 0.0

PI = 3.1415926535897

def new_odometry(msg):
    global x
    global y
    global theta
 
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
 
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def move():
    # Starts a new node
    rospy.init_node('vector_controller', anonymous=True)
    velocity_publisher = rospy.Publisher('/vector/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber("/odom", Odometry, new_odometry)
    

    #Receiveing the user's input
    print("Let's move your robot")
    speed = float(input("Input your speed:"))
    time = float(input("Type your time:"))
    ideal_velocity = abs(speed)
    count = 0
    Error_data = []
    while not rospy.is_shutdown():
        
        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        init_position = x
        delta_t = 0
        #Loop to move the turtle in an specified distance
        while(delta_t <= time):
            #Publish the velocity
            vel_msg = Twist()
            vel_msg.linear.x = speed
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            delta_t = t1 - t0
            #Calculates distancePoseStamped
            
        #After the loop, stops the robot
        count += 1
        print("[round ",count,"]")
        final_position = x
        delta_x = final_position - init_position
        current_v = delta_x/time
        diff_v = ideal_velocity- current_v
        Error_data.append(diff_v)
        print("[Final Position : ", final_position,"]")
        print("[Traveled Distance : ", delta_x,"]")
        print("[Velocity Error : ", diff_v,"]")
        vel_msg.linear.x = 0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)

        if len(Error_data) == 50:
            df = pd.read_csv('linear_translation.csv')
            df['velocity_error_7_5sec'] = Error_data
            df.to_csv('linear_translation.csv', index=False)



if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass