#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import pandas as pd
import numpy as np

x = 0.0
y = 0.0 
yaw = 0.0
PI = 3.1415926535897

def new_odometry(msg):
    global x
    global y
    global yaw
 
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
 
    rot_q = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    
def rotate():
    #Starts a new node
    rospy.init_node('vector_controller', anonymous=True)
    velocity_publisher = rospy.Publisher('/vector/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber("/odom", Odometry, new_odometry)
    vel_msg = Twist()

    # Receiveing the user's input
    print("Let's rotate your robot")
    speed = int(input("Input your speed (degrees/sec):"))
    time = float(input("Type your time:"))
    angle = int(input("Type your distance (degrees):"))
    clockwise = int(input("Clockwise?: ")) #True or false

    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        velocity = -abs(angular_speed)
    else:
        velocity = abs(angular_speed)
    
    
    count = 0
    
    v_hat_data = []
    omega_hat_data = []
    gamma_hat_data = []

    while not rospy.is_shutdown():
        
        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        init_x = x
        init_y = y
        init_yaw = yaw
        
        delta_t = 0
        while(delta_t <= 2*time):
            vel_msg.angular.z = velocity
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            delta_t = t1 - t0
        


        #Forcing our robot to stop
        count += 1
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        
        final_x = x
        final_y = y
        final_yaw = yaw

        #Calculations
        mu = 0.5 * ((init_x - final_x)*np.cos(init_yaw) + (init_y - final_y)*np.sin(init_yaw))/((init_x - final_x)*np.sin(init_yaw) + (init_y - final_y)*np.cos(init_yaw))
        x_star = 0.5 * (init_x + final_x) + mu * (init_y - final_y)
        y_star = 0.5 * (init_y + final_y) + mu * (final_x - init_x)
        r_star = np.sqrt((init_x - x_star)**2 + (init_y - y_star)**2)
        delta_yaw = np.arctan2(final_y - y_star , final_x - x_star) - np.arctan2(init_y - y_star , init_x - x_star)
        v_hat = (delta_yaw/delta_t) * r_star
        omega_hat = delta_yaw / delta_t
        gamma_hat = (final_yaw - init_yaw)/delta_t - omega_hat
        print("[round ",count,"]")
        print("[v_hat : ", v_hat,"]")
        print("[omega_hat : ", omega_hat,"]")
        print("[gamma_hat : ", gamma_hat,"]")
        v_hat_data.append(v_hat)
        omega_hat_data.append(omega_hat)
        gamma_hat_data.append(gamma_hat)

        if count == 20:
            df = pd.read_csv('angular_90_CCL.csv')
            df['v_hat'] = v_hat_data
            df['omega_hat'] = omega_hat_data
            df['gamma_hat'] = gamma_hat_data
            df.to_csv('angular_90_CCL.csv', index=False)
    
if __name__ == '__main__':
    try:
        # Testing our function
        rotate()
    except rospy.ROSInterruptException:
        pass
