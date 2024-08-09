#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Int32
import numpy as np
from pymodbus.client import ModbusTcpClient as ModbusClient 

class VelocityCalculator:
    def __init__(self):
        self.prev_time = None
        self.prev_position = None
        self.velocity = None

        rospy.init_node('velocity_calculator', anonymous=True)

        # Subscribers
        rospy.Subscriber('/ndt_pose', PoseStamped, self.ndt_pose_callback)

        # Publishers
        self.velocity_pub = rospy.Publisher('/calculated_velocity', Vector3, queue_size=10)
        self.angle_pub = rospy.Publisher('/steering_angle', Int32, queue_size=10)

        self.client = None
        self.UNIT = None

    def ndt_pose_callback(self, data):
        # Extract the current time and position
        curr_time = data.header.stamp.to_sec()
        curr_position = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

        # Calculate velocity if we have previous position and time
        if self.prev_time is not None and self.prev_position is not None:
            dt = curr_time - self.prev_time
            if dt > 0:
                displacement = curr_position - self.prev_position
                self.velocity = displacement / dt
                X_velocity = self.velocity[0]
                Y_velocity = self.velocity[1]
                Z_velocity = self.velocity[2]
                
                # Calculate total velocity magnitude
                total_2D_velocity = np.sqrt(X_velocity**2 + Y_velocity**2)
                total_3D_velocity = np.sqrt(X_velocity**2 + Y_velocity**2 + Z_velocity**2)
                print("Current 2D Velocity: ", total_2D_velocity * 3.6)
                # print("Current 3D Velocity: ", total_3D_velocity * 3.6)

                # Publish the velocity
                velocity_msg = Vector3()
                velocity_msg.x = X_velocity
                velocity_msg.y = Y_velocity
                velocity_msg.z = Z_velocity
                self.velocity_pub.publish(velocity_msg)

        # Update previous time and position
        self.prev_time = curr_time
        self.prev_position = curr_position

    def read_angle(self):
        if self.client is not None:
            read1 = self.client.read_holding_registers(address=1, count=1, unit=self.UNIT)
            current_angle = read1.registers[0]
            normalised_angle = current_angle - 480
            self.angle_pub.publish(normalised_angle)
            print("Steering Feedback: ", normalised_angle)
            return current_angle
        return None

if __name__ == '__main__':
    try:
        host = '192.168.140.5'
        port = 502
        client = ModbusClient(host, port)
        client.connect()
        UNIT = 0x1
        print("Connected")

        velocity_calculator = VelocityCalculator()
        velocity_calculator.client = client
        velocity_calculator.UNIT = UNIT

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            velocity_calculator.read_angle()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
