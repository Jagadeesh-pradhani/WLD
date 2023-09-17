#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32  # Use appropriate message types for your control commands


try:
    global sensor_data
    sensor_data = 0

    def sensor_data_callback(data):
        # Callback function to process sensor data
        global sensor_data
        sensor_data = data.data
        return

    # Initialize the ROS node
    rospy.init_node('arm7_simulation', anonymous=True)

    # Create a publisher for control commands
    control_command_pub = rospy.Publisher('control_commands', Int32, queue_size=10)
    rate = rospy.Rate(1)

    # Create a subscriber for sensor data
    while not rospy.is_shutdown():
        rospy.Subscriber('sensor_data', Int32, sensor_data_callback)
        # Check conditions and send control commands (replace with your logic)
        if sensor_data > 50:
            control_command = 1  # Leak detected
        else:
            control_command = 0  # No leak

        # Publish control command
        control_command_pub.publish(control_command)
        rate.sleep()

    # Spin to keep the node alive
    #rospy.spin()

except rospy.ROSInterruptException:
    pass



