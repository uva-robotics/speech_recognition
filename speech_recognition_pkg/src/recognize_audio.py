#!/usr/bin/python
import rospy

import speech_recognition as sr
import numpy as np
import soundfile

import collections
import sys

from std_msgs.msg import String, Bool
from std_msgs.msg import Float32MultiArray

WAKE_WORD_TOPIC = '/wake_word'
CONVERTED_AUDIO_TOPIC = '/converted_audio'
RECOGNIZED_INTENT = '/recognized_intent'

recognizer = sr.Recognizer()


class AudioRecognizer():

    def __init__(self):
        rospy.init_node('audio_recognizer')

        
        self.buffer = []
        self.sample_rate = 16000
        self.channels = 1
        self.wake_word_activated = False
        self.buffer_size = 512.25

        self.pause_ms = 4000

        self.recognized_intent = rospy.Publisher(RECOGNIZED_INTENT, String, queue_size=10)
        self.sub = rospy.Subscriber(CONVERTED_AUDIO_TOPIC, Float32MultiArray, self.audio_cb)
        self.wake_word = rospy.Subscriber(WAKE_WORD_TOPIC, Bool, self.on_wake_word)

    
    def calc_rms(self, audio):
        return np.sqrt(np.mean(audio**2))
    
    def on_wake_word(self, data):
        self.wake_word_activated = True
        print("*** HEARD WAKE WORD ***")

    def convert_to_audiodata(self, audio):
        tmp = list(audio)
        dataBuff = ""
        for i in range (0,len(tmp)) :
            if tmp[i]<0 :
                tmp[i]=tmp[i]+65536
            dataBuff = dataBuff + chr(tmp[i]%256)
            dataBuff = dataBuff + chr( (tmp[i] - (tmp[i]%256)) /256)
        return dataBuff

    def audio_cb(self, data):
        #TODO: end-of-utterance/end-of-query detection
        audiodata = np.asarray(data.data)
        if self.wake_word_activated:
            print("*** LISTENING AFTER WAKE WORD *** (RMS {})".format(self.calc_rms(audiodata)))
            self.buffer.append(audiodata)
            if len(self.buffer) > self.pause_ms / self.buffer_size:
                recorded_audio = np.concatenate(self.buffer, axis=0).astype(np.int16)

                audio = self.convert_to_audiodata(recorded_audio)
                
                audio_data = sr.AudioData(audio, self.sample_rate, 2)
                with open("speech-rec.wav", "wb") as f:
                    f.write(audio_data.get_wav_data())
                soundfile.write('./test.wav', recorded_audio, samplerate=self.sample_rate*self.channels, subtype='PCM_16')

                if audio_data:
                    self.wake_word_activated = False
                    try:
                        print("*** RECOGNIZING ***")
                        text = recognizer.recognize_google(audio_data, language="en-US")
                        print(text)
                        self.recognized_intent.publish(String(text))
                    except sr.UnknownValueError:
                        print("Google Speech Recognition could not understand audio")
                    except sr.RequestError as e:
                        print("Could not request results from Google Speech Recognition service; {0}".format(e))
                    except LookupError:
                        print("Oops! Didn't catch that")
                self.buffer = []
                    
                


if __name__ == '__main__':
    ar = AudioRecognizer()
    rospy.spin()