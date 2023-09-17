#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32  # Use an appropriate message type for your sensor data

import random  # Import the random module for generating mock sensor data

def sensor_data_simulation():
    # Initialize the ROS node
    rospy.init_node('sensor_data_simulation', anonymous=True)

    # Create a publisher for sensor data
    sensor_data_pub = rospy.Publisher('sensor_data', Int32, queue_size=10)

    # Set the simulation rate (adjust as needed)
    rate = rospy.Rate(1)  # 1 Hz

    # Simulation loop
    while not rospy.is_shutdown():
        # Simulate sensor data (replace with your simulation logic)
        sensor_value = random.randint(0, 100)  # Generate a random sensor value between 0 and 100

        # Publish the sensor data
        sensor_data_pub.publish(sensor_value)

        # Sleep to maintain the desired simulation rate
        rate.sleep()

if __name__ == '__main__':
    try:
        sensor_data_simulation()
    except rospy.ROSInterruptException:
        pass
