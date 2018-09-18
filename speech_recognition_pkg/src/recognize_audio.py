import rospy

import speech_recognition as sr
import numpy as np
import soundfile
import webrtcvad
import collections
import sys

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

CONVERTED_AUDIO_TOPIC = '/converted_audio'
RECOGNIZED_INTENT = '/recognized_intent'

recognizer = sr.Recognizer()


class AudioRecognizer():

    def __init__(self):
        rospy.init_node('audio_recognizer')
        self.sub = rospy.Subscriber(CONVERTED_AUDIO_TOPIC, Float32MultiArray, self.audio_cb)
        self.recognized_intent = rospy.Publisher(RECOGNIZED_INTENT, String, queue_size=10)

        self.vad = webrtcvad.Vad()

        self.buffer = []
        self.sample_rate = 16000
        self.channels = 1
    
    def calc_rms(self, audio):
        return np.sqrt(np.mean(audio**2))
    
    def audio_cb(self, data):
        audiodata = np.asarray(data.data)

        print("RMS", self.calc_rms(audiodata))
        frame_duration_ms = (1000. / self.sample_rate) * len(audiodata)

        self.buffer.append(audiodata)
        if len(self.buffer) > 75:
            print("WRITING")
            recorded_audio = np.concatenate(self.buffer, axis=0).astype(np.int16)
            # soundfile.write('./test.wav', recorded_audio, samplerate=self.sample_rate*self.channels, subtype='PCM_16')
    
            audio_data = sr.AudioData(recorded_audio, self.sample_rate, 2)
            # with open("speech-rec.wav", "wb") as f:
            #     f.write(audio_data.get_wav_data())
                
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
            self.buffer = []
            exit()


if __name__ == '__main__':
    ar = AudioRecognizer()
    rospy.spin()