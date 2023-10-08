#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, UInt16
from collections import deque

'''
<node name="calculate_velocity_node" pkg="virtual_dc_motor" type="virtual_dc_motor" output="log" respawn="true"></node>
<node name="control_velocity_node" pkg="virtual_dc_motor" type="virtual_dc_motor" output="log" respawn="true"></node>
'''


previous_rpm = deque(maxlen=100) # Stores data about last 100 RPM values(~1 second)
previous_position = None
previous_timestamp = None


def position_callback(msg):
    global previous_position, previous_timestamp, previous_rpm

    # Sets up data for first iteration
    if previous_position == None and previous_timestamp == None:
        previous_position = msg.data
        previous_timestamp = rospy.Time.now()
        return
    
    # Sets data for current iteration
    current_position = msg.data
    current_timestamp = rospy.Time.now()
    
    # Calculates differences needed for RPM formula
    position_difference = current_position - previous_position
    time_difference = (current_timestamp - previous_timestamp).to_sec()

    # Calculates RPM as position and time difference
    rpm = (position_difference / 4096) / (time_difference / 60)
    previous_rpm.append(rpm)

    # Calculates average RPM from last 100 RPMs
    avg_rpm = sum(rpm for rpm in previous_rpm) / len(previous_rpm)

    # Publishes RPMs on topic
    rpm_message = Float32()
    rpm_message.data = avg_rpm
    rpm_publisher.publish(rpm_message)

    
    rpm_goal_message = Float32()
    rpm_goal_message.data = 5000
    rpm_goal_publisher.publish(rpm_goal_message)
    

    # Updates previous data with current, for the next iteration
    previous_timestamp = current_timestamp


if __name__ == "__main__":
    rospy.init_node("calculate_velocity_node")
    rpm_publisher = rospy.Publisher("/virtual_dc_motor_driver/get_velocity", Float32, queue_size=10)
    rpm_goal_publisher = rospy.Publisher("/virtual_dc_motor_controller/set_velocity_goal", Float32, queue_size=10)
    position_subscriber = rospy.Subscriber("/virtual_dc_motor/get_position", UInt16, callback=position_callback)
    rospy.spin()
