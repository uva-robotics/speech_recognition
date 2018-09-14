import rospy
from naoqi_bridge_msgs.msg import AudioBuffer
from std_msgs.msg import String
import speech_recognition as sr
import audioop
import numpy as np

recognizer = sr.Recognizer()

class AudioAnalyzer():

    def __init__(self):
        rospy.init_node('pepper_audio_listener', anonymous=True)
        rospy.Subscriber("/pepper_robot/audio", AudioBuffer, self.callback)
        
        self.big_buffer = ""
        self.audio_buffer = []
        self.speak = rospy.Publisher('/speech', String, queue_size=10)        

    def callback(self, msg):
        self.audio_buffer.extend(msg.data)

    def run(self):
        one_audio_data_size = 10920
        one_second = one_audio_data_size * 5
        ten_seconds = one_second * 15
        seconds_to_buffer = ten_seconds
        print("*** LISTENING ***")
        while not rospy.is_shutdown():
            #rospy.loginfo("len(self.audio_buffer):" + str(len(self.audio_buffer)))
            if len(self.audio_buffer) >= seconds_to_buffer: # if one second of audio is in the buffer
                rospy.loginfo("Buffer has " + str(  len(self.audio_buffer) / one_second  ) + "s of audio, playing.")
                tmp = list(self.audio_buffer)
                self.audio_buffer = self.audio_buffer[seconds_to_buffer:] # Leaving whatever extra data is there
                dataBuff = ""
                for i in range (0,len(tmp)) :
                    if tmp[i]<0 :
                        tmp[i]=tmp[i]+65536
                    dataBuff = dataBuff + chr(tmp[i]%256)
                    dataBuff = dataBuff + chr( (tmp[i] - (tmp[i]%256)) /256)

                mono = audioop.tomono(dataBuff, 2, 1, 1)
                audio_data = sr.AudioData(mono, 44100, 4)
                if audio_data:
                    try:
                        print("*** RECOGNIZING ***")
                        text = recognizer.recognize_google(audio_data, language="en-US")
                        print("You said: " + text)
                        self.speak.publish(String(text))
                        self.audio_buffer = []

                    except sr.UnknownValueError:
                        print("Google Speech Recognition could not understand audio")
                    except sr.RequestError as e:
                        print("Could not request results from Google Speech Recognition service; {0}".format(e))
                    except LookupError:
                        print("Oops! Didn't catch that")
    
if __name__ == '__main__':
    print("AUDIO")

    analyzer = AudioAnalyzer()
    analyzer.run()
    rospy.spin()
