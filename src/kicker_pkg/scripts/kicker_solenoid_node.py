#!/usr/bin/env python
import time
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

class KickerNode():
    def __init__(self, topicToSubscribe='/kicking_decision'):
        # GPIO pin for the solenoid
        self.SOLENOID_PIN = 25  # Update with your GPIO pin number

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)  # Use BCM GPIO numbering
        GPIO.setup(self.SOLENOID_PIN, GPIO.OUT)  # Set the pin as output
        GPIO.output(self.SOLENOID_PIN, GPIO.LOW)  # Ensure solenoid is off initially

        # Initialize ROS node and topics
        rospy.init_node('kicker_node', anonymous=True)
        self.topicToSubscribe = topicToSubscribe
        self.subscriber = rospy.Subscriber(self.topicToSubscribe, Bool, self.callback)
        self.kickerPub = rospy.Publisher('/kicking_decision', Bool, queue_size=10)

        rospy.loginfo("KickerNode initialized and waiting for messages.")

    def callback(self, msg):
        """Callback function to handle incoming messages and control the solenoid."""
        rospy.loginfo(f"Just received the msg: {msg.data}")
        if msg.data:
            rospy.loginfo("KICKER ON")
            GPIO.output(self.SOLENOID_PIN, GPIO.HIGH)  # Turn solenoid on
            time.sleep(0.1)  # Simulate kick duration
            GPIO.output(self.SOLENOID_PIN, GPIO.LOW)  # Turn solenoid off

            # Publish False after the kick
            temp_Bool = Bool()
            temp_Bool.data = False
            self.kickerPub.publish(temp_Bool)
            rospy.loginfo("Trigger signal sent and solenoid turned off.")
        else:
            rospy.loginfo("WAITING TO KICK")

    def shutDown(self):
        """Clean up GPIO resources on shutdown."""
        GPIO.cleanup()
        rospy.loginfo("KickerNode shut down and GPIO cleaned up.")

if __name__ == '__main__':
    kickerNode = KickerNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        kickerNode.shutDown()
