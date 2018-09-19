#!/usr/bin/python
import rospy
import dialogflow_v2 as dialogflow
from std_msgs.msg import String

from gtts import gTTS
import os


RECOGNIZED_INTENT = '/recognized_intent'
SPEAK_TOPIC = '/speech'

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

class DialogFlow():

    def __init__(self):
        rospy.init_node('dialogflow', anonymous=True)
        rospy.Subscriber(RECOGNIZED_INTENT, String, self.callback)        

    def callback(self, msg):
        print(msg)
        texts = [msg.data]
        detect_intent_texts('pepper-39819', 'abc', texts, 'en')


if __name__ == '__main__':
    d = DialogFlow()
    rospy.spin()