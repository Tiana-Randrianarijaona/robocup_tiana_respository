#!/usr/bin/env python
import serial
import time
import rospy
from std_msgs.msg import Float32MultiArray, Bool

class KickerNode():
    def __init__(self,topicToSubscribe = '/kicking_decision'):
        self.ArduinoPort = '/dev/ttyACM0'
        self.string_to_send  = "Tigo"     
        # Set up serial connection
        self.ser = serial.Serial(self.ArduinoPort, 9600)  # Replace 'COM7' by '/dev/ttyUSB0' on Ubuntu   
        time.sleep(2)  # Allow some time for the serial connection to initialize
        rospy.init_node('image_subscriber')        
        self.topicToSubscribe = topicToSubscribe
        self.subscriber = rospy.Subscriber(self.topicToSubscribe, Bool, self.callback)
        self.kickerPub = rospy.Publisher('/kicking_decision', Bool, queue_size = 10)
    
    def callback(self,msg):    
        print(f"Just received the msg {msg.data}")   
        if(msg.data):
            print("KICKER ON")
            self.ser.write(self.string_to_send.encode())
            time.sleep(100)
            temp_Bool = Bool()
            temp_Bool.data = False
            self.kickerPub.publish(temp_Bool)
            print("Trigger signal sent.")            
        else:
            print("WAITING TO KICK")        
        # Send the string
               

    def shutDown(self):
        # Close serial connection
        self.ser.close()

if __name__ == '__main__':    
    kickerNode = KickerNode() 
    try:
        
        rospy.spin()
    except:
        pass
    finally:
        kickerNode.shutDown()