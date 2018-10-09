#!/usr/bin/python
import rospy

import speech_recognition as sr
import numpy as np
import soundfile

import collections
import sys

from std_msgs.msg import String, Bool
from std_msgs.msg import Float32MultiArray

from std_msgs.msg import ColorRGBA
from naoqi_bridge_msgs.msg import FadeRGB

import webrtcvad

WAKE_WORD_TOPIC = '/wake_word'
CONVERTED_AUDIO_TOPIC = '/converted_audio'
RECOGNIZED_INTENT = '/recognized_intent'

LED_TOPIC = '/fade_rgb'

recognizer = sr.Recognizer()


class AudioRecognizer():

    def __init__(self):
        rospy.init_node('audio_recognizer')

        
        self.buffer = []
        self.sample_rate = 16000
        self.channels = 1
        self.wake_word_activated = False
        self.is_listening = False
        self.buffer_size = 512

        self.CHUNK_SIZE = 30 # 30 ms for VAD
        self.vad_ring_buffer = collections.deque(maxlen=self.CHUNK_SIZE)
        self.vad_active = False
        self.speak_state = []
        
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

    def set_leds(self):
        led_msg = FadeRGB()
        color = ColorRGBA()
        color.r = 0
        color.g = 0
        color.b = 0
        color.a = 0
        d = rospy.Duration.from_sec(0)
        led_msg = FadeRGB()
        led_msg.led_name = 'EarLeds'
        led_msg.color = color
        led_msg.fade_duration = d
        led.publish(led_msg)

    def end_of_line_detection(self):
        return self.speak_state.count(False) >= 1
           
    def audio_cb(self, data):
        #TODO: end-of-utterance/end-of-query detection
        
        # RATE = 16000
        # CHUNK_DURATION_MS = 30
        # CHUNK_SIZE = int(RATE * CHUNK_DURATION_MS / 1000)
        # CHUNK_BYTES = CHUNK_SIZE * 2

        # active = vad.is_speech(chunk, RATE)

        audiodata = np.asarray(data.data)

        CHUNK_SIZE = int(self.sample_rate * 30 / 1000.)

        vad_buffer = []
        for i, a in enumerate(audiodata):
            if i == 0:
                vad_buffer.append(a)
            elif i % CHUNK_SIZE == 0:
                if len(vad_buffer) == CHUNK_SIZE-1:
                    vad_buffer.append(0)
                aud = np.asarray(vad_buffer, dtype=np.int16)
                pcm = self.convert_to_audiodata(aud)
                self.vad_active = vad.is_speech(pcm, self.sample_rate)
                vad_buffer = []
            else:
                vad_buffer.append(a)

        # 512 ms aan samples ===> 30 ms aan samples
        # self.vad_ring_buffer.extend(pcm)

        if self.wake_word_activated:
            self.is_listening = True
            self.wake_word_activated = False
            rospy.set_param('/speech_recognition/is_listening', True)
        elif self.is_listening is True:

            print("*** LISTENING AFTER WAKE WORD *** (RMS {})".format(self.calc_rms(audiodata)))
            if self.vad_active:
                print("*** VAD *** ", self.vad_active)
                self.speak_state.append(True)
                self.buffer.append(audiodata)
            elif self.speak_state.count(True) >= 2:
                self.speak_state.append(False)
            
            if self.end_of_line_detection(): # or (len(self.buffer) > self.pause_ms / self.buffer_size):

                self.set_leds()

                recorded_audio = np.concatenate(self.buffer, axis=0).astype(np.int16)

                audio = self.convert_to_audiodata(recorded_audio)
                
                audio_data = sr.AudioData(audio, self.sample_rate, 2)
                with open("/tmp/speech-rec.wav", "wb") as f:
                    f.write(audio_data.get_wav_data())
                # soundfile.write('/home/test.wav', recorded_audio, samplerate=self.sample_rate*self.channels, subtype='PCM_16')

                if audio_data:
                    self.is_listening = False
                    self.wake_word_activated = False
                    text = ''
                    try:
                        print("*** RECOGNIZING ***")
                        text = recognizer.recognize_google(audio_data, language="en-US")
                        print(text)
                    except sr.UnknownValueError:
                        print("Google Speech Recognition could not understand audio")
                    except sr.RequestError as e:
                        print("Could not request results from Google Speech Recognition service; {0}".format(e))
                    except LookupError:
                        print("Oops! Didn't catch that")
                self.recognized_intent.publish(String(text))
                self.buffer = []
                self.speak_state = []
                self.vad_active = False
                rospy.set_param('/speech_recognition/is_listening', False)
                    
                


if __name__ == '__main__':
    vad = webrtcvad.Vad()
    led = rospy.Publisher(LED_TOPIC, FadeRGB, queue_size=10)
    ar = AudioRecognizer()
    rospy.spin()