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

        self.recognized_intent = rospy.Publisher('/recognized_intent', String, queue_size=10)
        
        self.big_buffer = ""
        self.audio_buffer = []
        self.speak = rospy.Publisher('/speech', String, queue_size=10) 
        self.mono_signal = rospy.Publisher('/mono_signal', String, queue_size=10)       

    def callback(self, msg):
        self.audio_buffer.extend(msg.data)
        # print(len(msg.data))
        # print(msg.frequency, msg.channelMap, msg.header)

    def run(self):
        # 16384 = sample size, 10 channels.
        one_audio_data_size = 16384
        one_second = one_audio_data_size * 10
        seconds_to_buffer = one_second * 5
        print("*** LISTENING ***")

        while not rospy.is_shutdown():
            if len(self.audio_buffer) >= seconds_to_buffer:
                rospy.loginfo("Buffer has " + str(  len(self.audio_buffer) / one_second  ) + "s of audio, playing.")
                tmp = list(self.audio_buffer)
                self.audio_buffer = self.audio_buffer[seconds_to_buffer:]
                dataBuff = ""
                for i in range (0,len(tmp)) :
                    if tmp[i]<0 :
                        tmp[i]=tmp[i]+65536
                    dataBuff = dataBuff + chr(tmp[i]%256)
                    dataBuff = dataBuff + chr((tmp[i] - (tmp[i]%256)) / 256)

                # to mono. Buffer, sample width and LR channel factorization..
                mono = audioop.tomono(dataBuff, 2, 1, 1)
                print(type(mono))
                self.mono_signal.publish(String(mono))



                rms = audioop.rms(mono, 2)

                rospy.loginfo("Loudness - " + str(rms))
                audio_data = sr.AudioData(mono, 48000, 4)
                if audio_data:
                    try:
                        print("*** RECOGNIZING ***")
                        text = recognizer.recognize_google(audio_data, language="en-US")
                        self.recognized_intent.publish(String(text))
                        self.audio_buffer = []
                    except sr.UnknownValueError:
                        print("Google Speech Recognition could not understand audio")
                    except sr.RequestError as e:
                        print("Could not request results from Google Speech Recognition service; {0}".format(e))
                    except LookupError:
                        print("Oops! Didn't catch that")
    
if __name__ == '__main__':
    analyzer = AudioAnalyzer()
    analyzer.run()
    rospy.spin()
