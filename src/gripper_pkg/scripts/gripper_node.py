#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Int32

class ServoControllerNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('servo_controller', anonymous=True)
        
        # Get parameters
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        
        # Set up serial communication with Arduino
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            rospy.loginfo("Connected to Arduino on port %s at %d baud.", self.serial_port, self.baud_rate)
        except serial.SerialException as e:
            rospy.logerr("Failed to connect to Arduino: %s", e)
            rospy.signal_shutdown("Failed to connect to Arduino.")
            return
        
        # Subscribe to a topic to receive servo angle commands
        self.subscriber = rospy.Subscriber('/servo_angle', Int32, self.command_callback)
        rospy.loginfo("Subscribed to /servo_angle topic.")

    def command_callback(self, msg):
        angle = msg.data
        if 0 <= angle <= 180:
            try:
                # Send the angle command to the Arduino
                self.arduino.write(f"{angle}\n".encode())
                rospy.loginfo("Sent angle %d to Arduino.", angle)
            except serial.SerialException as e:
                rospy.logerr("Failed to send data to Arduino: %s", e)
        else:
            rospy.logwarn("Received invalid angle: %d. Angle must be between 0 and 180.", angle)

    def run(self):
        rospy.spin()
        self.arduino.close()
        rospy.loginfo("Shutting down ServoControllerNode.")

if __name__ == '__main__':
    try:
        node = ServoControllerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
