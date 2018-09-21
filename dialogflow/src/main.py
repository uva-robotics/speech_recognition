#!/usr/bin/python
import rospy
import dialogflow_v2 as dialogflow
from std_msgs.msg import String

from std_msgs.msg import ColorRGBA
from naoqi_bridge_msgs.msg import FadeRGB

from gtts import gTTS
import os


RECOGNIZED_INTENT = '/recognized_intent'
SPEAK_TOPIC = '/speech'
LED_TOPIC = '/fade_rgb'


speech = rospy.Publisher(SPEAK_TOPIC, String, queue_size=10)

def detect_intent_texts(project_id, session_id, texts, language_code):
    """Returns the result of detect intent with texts as inputs.

    Using the same `session_id` between requests allows continuation
    of the conversaion."""

    print("TEXT: ", texts)
    
    session_client = dialogflow.SessionsClient()

    session = session_client.session_path(project_id, session_id)
    print('Session path: {}\n'.format(session))

    for text in texts:
        text_input = dialogflow.types.TextInput(
            text=text, language_code=language_code)

        query_input = dialogflow.types.QueryInput(text=text_input)

        response = session_client.detect_intent(
            session=session, query_input=query_input)

        print('Query text: {}'.format(response.query_result.query_text))
        print('Detected intent: {} (confidence: {})\n'.format(
            response.query_result.intent.display_name,
            response.query_result.intent_detection_confidence))
        print('Fulfillment text: {}\n'.format(
            response.query_result.fulfillment_text))

        speech.publish(response.query_result.fulfillment_text)
        # fp = "/tmp/tmp.mp3"
        # tts = gTTS(text=response.query_result.fulfillment_text, lang='en')
        # tts.save(fp)
        # os.system("play " + fp)


class Intent():

    def __init__(self):
        rospy.init_node('intent', anonymous=True)
        rospy.Subscriber(RECOGNIZED_INTENT, String, self.callback)        

    def callback(self, msg):

        if msg.data:
            texts = [msg.data]
            detect_intent_texts('pepper-39819', 'abc', texts, 'en')

        led_msg = FadeRGB()
        color = ColorRGBA()
        color.r = 0
        color.g = 0
        color.b = 0
        color.a = 0
        d = rospy.Duration.from_sec(0)
        led_msg = FadeRGB()
        led_msg.led_name = 'FaceLeds'
        led_msg.color = color
        led_msg.fade_duration = d
        led.publish(led_msg)

        led_msg = FadeRGB()
        led_msg.led_name = 'EarLeds'
        led_msg.color = color
        led_msg.fade_duration = d
        led.publish(led_msg)


if __name__ == '__main__':
    led = rospy.Publisher(LED_TOPIC, FadeRGB, queue_size=10)
    i = Intent()
    rospy.spin()