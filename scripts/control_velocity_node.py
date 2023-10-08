#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Int8


def velocity_goal_callback(msg):
    max_rpm = 6000 # Max possible RPM of engine
    target_rpm = msg.data

    # Adjusts recieved RPM to its possible range
    if target_rpm > max_rpm:
        target_rpm = max_rpm
    elif target_rpm < -max_rpm:
        target_rpm = -max_rpm

    # Calculates CS based on target RPM
    cs_strnegth = int((target_rpm / max_rpm) * 100)

    # Adjusts control signal for the engine's range <-100 - 100>
    if cs_strnegth > 100:
        cs_strnegth = 100
    elif cs_strnegth < -100:
        cs_strnegth = -100
    
    # Publishes CS
    cs_message = Int8()
    cs_message.data = cs_strnegth
    cs_publisher.publish(cs_message)


if __name__ == "__main__":
    rospy.init_node("control_velocity_node")
    cs_publisher = rospy.Publisher("/virtual_dc_motor/set_cs", Int8, queue_size=10)
    rpm_subscriber = rospy.Subscriber("/virtual_dc_motor_controller/set_velocity_goal", Float32, callback=velocity_goal_callback)
    rospy.spin()
    