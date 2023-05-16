#!/usr/bin/env python3

# Imports
##########################
import os
from math import radians
import math
import time
import rospy  # ROS Python interface
from std_msgs.msg import Float32MultiArray, UInt16
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, Range
import numpy as np


import miro2 as miro  # Import MiRo Developer Kit library

try:  # For convenience, import this util separately
    from miro2.lib import wheel_speed2cmd_vel  # Python 3
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel  # Python 2
##########################


class MiRoClient:
    """
    Script settings below
    """
    TICK = 0.02  # This is the update interval for the main control loop in secs
    ##NOTE The following option is relevant in MiRoCODE
    NODE_EXISTS = False  # Disables (True) / Enables (False) rospy.init_node

    # Reset the position of the head
    def reset_head_pose(self):
        """
        Reset MiRo head to default position, to avoid having to deal with tilted frames
        """
        self.kin_joints = JointState()  # Prepare the empty message
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0.0,  self.head_lift,  0.0,  self.head_pitch]
        t = 0
        while not rospy.core.is_shutdown():  # Check ROS is running
            # Publish state to neck servos for 1 sec
            self.pub_kin.publish(self.kin_joints)
            rospy.sleep(self.TICK)
            t += self.TICK
            if t > 1:
                break
    
    # Move back a bit
    def move_back(self):
        print("Moving back")
        self.drive(-0.2 + self.tilt/150,-0.2 - self.tilt/150)
            
    # Detect cliffs and react accordingly
    def cliff_detect(self, lcliff, rcliff):
            
        if lcliff < 0.50:
            print("Left cliff detected")
            self.move_back()
            time.sleep(0.5)
            self.drive(0.2, -0.2)
            time.sleep(2)
        
        if rcliff < 0.50:
            print("Right cliff detected ")
            self.move_back()
            time.sleep(0.5)
            self.drive(-0.2, 0.2)
            time.sleep(2)
            
    # React to being touched on the head
    def touch_sensor_head(self, data):
        self.drive(0,0)
        self.kin_joints.position = [0.0,  0.6,  0.0,  self.head_pitch]
            
    # React to being touched on the body
    def touch_sensor_body(self, data):
        self.drive(0.2 - self.tilt/150,0.2 + self.tilt/150)
        time.sleep(0.5)
        
        # Find which sensor was activated
        for i in range(len(data)):
            if data[i] == 1:
                sensor = i
                break
        
        # Depending on the sensor activated, change the spinning direction 
        # and turning duration to face that location
        if sensor >= 0 and sensor <= 5:
            t_end = time.time() + (sensor+1)/10
            direction = -0.2
        elif sensor >= 6 and sensor <= 10:
            sensor = sensor - 6
            t_end = time.time() + (sensor+1)/10
            print(t_end)
            direction = 0.2
        else:
            t_end = time.time() + 1
            direction = 0.2
            
        while time.time() < t_end:
            self.drive(direction, -direction)
        time.sleep(2)
        

    # Drive in a certain direction
    def drive(self, speed_l=0.1, speed_r=0.1):  # (m/sec, m/sec)
        """
        Wrapper to simplify driving MiRo by converting wheel speeds to cmd_vel
        """
        # Prepare an empty velocity command message
        msg_cmd_vel = TwistStamped()

        # Desired wheel speed (m/sec)
        wheel_speed = [speed_l, speed_r]

        # Convert wheel speed to command velocity (m/sec, Rad/sec)
        (dr, dtheta) = wheel_speed2cmd_vel(wheel_speed)

        # Update the message with the desired speed
        msg_cmd_vel.twist.linear.x = dr
        msg_cmd_vel.twist.angular.z = dtheta

        # Publish message to control/cmd_vel topic
        self.vel_pub.publish(msg_cmd_vel)
        
    # Callback functions
    def callback_sonar(self, ros_range):
        self.range = ros_range.range
        self.min_range = ros_range.min_range
    
    def callback_joints(self,sensors):
        self.joints = sensors
        
    def cliff_callback(self, data):
        self.lcliff = data.data[0]
        self.rcliff = data.data[1]
        
    def callback_head(self, touch_data):
        self.callback(touch_data, 0)

    def callback_body(self, touch_data):
        self.callback(touch_data, 1)

    # Unified callback for both touch sensor arrays
    def callback(self, touch_data, index):
        # the touch data is encoded as a bit array
        # formatted strings can handle this natively
        bit_str = "{0:014b}".format(touch_data.data)
        self.touch_data[index] = list(map(int, [*(bit_str)]))

    def __init__(self):
        # Initialise a new ROS node to communicate with MiRo
        if not self.NODE_EXISTS:
            rospy.init_node("mainloop", anonymous=True)
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)
        self.touch_data = [None, None] # head and body touch data
        # Individual robot name acts as ROS topic prefix
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        
        # Create new subscriber to receive sonar data with attached callback
        self.sub_sonar = rospy.Subscriber(
            topic_base_name + "/sensors/sonar",
            Range,
            self.callback_sonar,
            queue_size=1,
            tcp_nodelay=True,
        )
        
        # Create new subscriber to receive kinematic data with attached callback
        self.sub_kinetic_head = rospy.Subscriber(
            topic_base_name + "/sensors/kinematic_joints",
            JointState,
            self.callback_joints,
            queue_size=1,
            tcp_nodelay=True,
        )
        # Create new subscriber to receive cliff data with attached callback
        self.sub_cliff_subscriber = rospy.Subscriber(
            topic_base_name + "/sensors/cliff", 
            Float32MultiArray, 
            self.cliff_callback
        )
        # Create new subscriber to receive touch data on head with attached callback
        self.sub_touch_head = rospy.Subscriber(
            topic_base_name + "/sensors/touch_head",
            UInt16,
            self.callback_head,
        )
        # Create new subscriber to receive touch data on body with attached callback
        self.sub_touch_body = rospy.Subscriber(
            topic_base_name + "/sensors/touch_body",
            UInt16,
            self.callback_body,
        )
        # Create a new publisher to send velocity commands to the robot
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )
        # Create a new publisher to move the robot head
        self.pub_kin = rospy.Publisher(
            topic_base_name + "/control/kinematic_joints", JointState, queue_size=0
        )
    
        # Move the head to default pose
        self.head_lift = 0.3
        self.head_pitch = miro.constants.PITCH_RAD_MIN + 0.3
        self.reset_head_pose()

        # Initialise parameters
        self.tilt = 0
        self.tilt_direc = 'L'
        self.close = 0.10
        self.medium = 0.25
        self.lcliff = 1.0
        self.rcliff = 1.0
        self.counter = 0
        self.touch = False
        
    # Calculates the gap that the robot can pass through
    def calc_gap(self, left_side, right_side, angle):        
        gap = math.sqrt(math.pow(left_side, 2) + math.pow(right_side, 2) - 2 * left_side * right_side * math.cos(angle))
        return gap
        
    # Use sonar to navigate obstacles
    def sonar_search(self):
        left_direction = False
        right_direction = False
        print("Radar searching")
        self.drive(0,0)

        left_distance = []
        right_distance = []
        current_left_distance = 0.0
        current_right_distance = 0.0
        last_left_distance = 0.0
        max_left_distance = 0.0
        max_left_degree = 0.0
        last_right_distance = 0.0
        max_right_distance = 0.0
        max_right_degree = 0.0
        turning_degree = 0.0
        
        # Block by an obstacle, looking for a way
        for i in range(3, 60, 3):
            self.kin_joints.position = [0.0,  self.head_lift,  radians(i),  self.head_pitch]
            self.pub_kin.publish(self.kin_joints)
            time.sleep(0.2)
            if self.range  == np.inf:
                self.range = 0
            left_distance.append(self.range)
            current_left_distance = self.range
            if last_left_distance == 0.0 :
                last_left_distance = current_left_distance
            elif  abs(current_left_distance-last_left_distance) <  0.2:
                last_left_distance = current_left_distance
            else:
                continue
            
        max_left_distance = np.max(left_distance)
        max_degree_index = np.argmax(left_distance)
        max_left_degree = 3 * (max_degree_index+1)

        for i in range(-3, -60, -3):
            self.kin_joints.position = [0.0,  self.head_lift, radians(i), self.head_pitch]
            self.pub_kin.publish(self.kin_joints)
            time.sleep(0.2)
            if self.range  == np.inf:
                self.range = 0
            right_distance.append(self.range)
            current_right_distance = self.range
            if last_right_distance == 0.0 :
                last_right_distance = current_right_distance
            elif  abs(current_right_distance-last_right_distance) < 0.2:
                last_right_distance = current_right_distance
            else:
                continue
            
        max_right_distance = np.max(right_distance)
        if max_right_distance == np.inf:
            max_right_distance = 1

        max_degree_index = np.argmax(right_distance)
        max_right_degree = 3 * (max_degree_index+1)
        
        angle_between = math.radians(abs(max_right_degree - max_left_degree))
        
        # Calculate the gap
        gap = self.calc_gap(max_left_distance, max_right_distance, angle_between)
        print("Gap callculated: ", gap)
            
        # Choose a direction to turn to
        if max_right_distance > max_left_distance:
            right_direction = True
        else:
            left_direction = True

        print("Max distance in right direction:", max_right_distance)
        print("Max distance in left direction:", max_left_distance)

        if left_direction:
                self.reset_head_pose()
                turning_degree = (angle_between * 4 * 0.18)
                self.drive(-turning_degree, turning_degree)
                for _ in range(10):
                    rospy.sleep(self.TICK)
                self.drive(0,0)     

        elif right_direction:
                self.reset_head_pose()
                turning_degree = (angle_between * 4 * 0.18)
                self.drive(turning_degree, -turning_degree)
                for _ in range(10):
                   rospy.sleep(self.TICK)
                self.drive(0,0)

        else:
            # If both sides are equal
            self.reset_head_pose()
            self.move_back()   

    # Move forward at a certain speed
    def move(self):
        self.drive(0.2,0.2)

    # Main control loop
    def loop(self):
        print("MiRo is walking forward, press CTRL+C to halt...")

        # This switch loops through MiRo behaviours:
        while not rospy.core.is_shutdown():
            
            # Detect cliff
            self.cliff_detect(self.lcliff, self.rcliff)
            
            # Detect head touch
            if 1 in self.touch_data[0]:
                self.touch_sensor_head(self.touch_data[0])
                self.touch_data[0] == 0
                self.reset_head_pose()
                
            # Detect body touch
            # Since the sensors sometimes get stuck, a boolean has been put in place to only allow one 
            # touch at a time
            if self.touch == False:
                if 1 in self.touch_data[1]:
                    self.touch_sensor_body(self.touch_data[1])
                    self.touch_data[1] == 0 
                    self.touch = True

            # Reset Miro's head in standard intervals to combat 
            # the head falling due to inertia
            # Also reset touch boolean
            self.counter += 1 
            if self.counter > 300:
                self.reset_head_pose()
                self.counter = 0
                self.touch = False

            # Move back if too close to an object
            if self.range < self.close:
                print('Too close, moving back')
                self.move_back()
                self.reset_head_pose()
                
                if self.tilt >= 40:
                    self.tilt_direc = 'R'
                elif self.tilt <= -40:
                    self.tilt_direc = 'L' 
                
                if self.tilt_direc =='R':
                    self.tilt -=20
                elif self.tilt_direc =='L':
                    self.tilt +=20
                rospy.sleep(self.TICK*3)
            
            # Scan for a way round an object
            elif self.range < self.medium:
                self.sonar_search()
            
            # Perform typical movement and basic area scanning
            elif self.range >= self.medium:
                self.move()
                
            # Incase not receiving valid range values
            rospy.sleep(self.TICK)

if __name__ == "__main__":
    main = MiRoClient()  # Instantiate class
    main.loop()  # Run the main control loop
