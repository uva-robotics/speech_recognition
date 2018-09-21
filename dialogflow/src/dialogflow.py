#!/usr/bin/python
import rospy
from std_msgs.msg import String

DIALOG_FLOW = '/dialogflow'


class DialogFlow():

    def __init__(self):
        rospy.init_node('dialogflow', anonymous=True)
        rospy.Subscriber(DIALOG_FLOW, String, self.callback)
        self.speech = rospy.Publisher('/speech', String, queue_size=10)
    
    def callback(self, data):
        print(data)
        self.speech.publish(data.data)  

if __name__ == '__main__':
    df = DialogFlow()
    rospy.spin()