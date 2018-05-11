#!/usr/bin/python

import rospy
from std_msgs.msg import String

import speech_recognition as sr
from audio_common_msgs.msg import AudioData
from naoqi_bridge_msgs.msg import AudioBuffer

from pyaudio import PyAudio, paInt16, paInt8, paUInt8
import pyaudio
import audioop
import numpy as np
import librosa
import collections
import time

import alsaaudio

from threading import Thread
from Queue import Queue, Empty

class RingBuffer:
    def __init__(self, size):
        self.data = collections.deque(maxlen=size)

    def append(self, x):
        # self.data.pop(0)
        self.data.append(x)

    def get(self):
        return self.data


class Recognizer(Thread):

    def __init__(self, queue):
        super(Recognizer, self).__init__()
        self.daemon = True
        self.queue = queue
        self.recognizer = sr.Recognizer()
        self.publisher = rospy.Publisher('speech_synthesis', String, queue_size=10)

    def run(self):
        while(True):
            print("I AM RUNNING")
            self.read()

    def read(self):
        try:
            audio = self.queue.get(timeout=0.5)
        except Empty:
            return

        if audio is None:
            return
        else:
            self.transcribe(audio)

    def transcribe(self, audio_data):
        try:
            # for testing purposes, we're just using the default API key
            # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
            # instead of `r.recognize_google(audio)`
            text = self.recognizer.recognize_google(audio_data)
            self.publisher.publish(text)
        except sr.UnknownValueError:
            print("NO AUDIO")
        except sr.RequestError as e:
            print("ERROR {}".format(e))

class AudioDetector():

    def __init__(self, queue):
        # rate = rospy.Rate(10) # 10hz

        self.file_path = "/tmp/speech.wav"
        self.file_handle = open(self.file_path, 'w')
        self.pa = PyAudio()
        self.SAMPLE_RATE = 16000
        self.SAMPLE_WIDTH = pyaudio.get_sample_size(paInt16)

        self.stream = self.pa.open(format = paInt16,
                         channels = 1,
                         rate = self.SAMPLE_RATE,
                         output = True,
                         frames_per_buffer = 2048)

        self.frames = ""
        self.ringBuffer = RingBuffer(10 * self.SAMPLE_RATE)
        self.speechBuffer = ""
        self.counter = 0
        self.is_talking = False
        self.queue = queue


    def _create_audio_data(self, raw_data):
        """
        Constructs an AudioData instance with the same parameters
        as the source and the specified frame_data
        """
        return sr.AudioData(raw_data, self.SAMPLE_RATE, self.SAMPLE_WIDTH)

    def callback(self, data):
        if data._type == 'audio_common_msgs/AudioData':
            rms = audioop.rms(data.data, 2)

            self.speechBuffer += data.data
            if rms > 1500:
                self.is_talking = True
                self.counter = 0
            if rms < 500 and self.counter > 150:
                self.is_talking = False

            print(len(self.speechBuffer))
            self.counter += 1

            if self.counter > 250 and not self.is_talking:
                audio_data = self._create_audio_data(self.speechBuffer)
                self.queue.put(audio_data)
                # print("DONE TALKING")
                # self.file_handle.write(self.speechBuffer)
                self.speechBuffer = ""
                self.counter = 0
                self.is_talking = False

    def audio_listener(self):
        print("LISTENING")
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/audio", AudioData, self.callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        queue = Queue()
        recognizer = Recognizer(queue)
        recognizer.start()

        detector = AudioDetector(queue)
        detector.audio_listener()
    except rospy.ROSInterruptException:
        pass
