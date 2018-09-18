#!/usr/bin/env python3
# Copyright 2018 Mycroft AI Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

import sys
import io
import time

import numpy as np
from argparse import ArgumentParser
# from precise.util import activate_notify

import sounddevice as sd
print sd.query_devices() 

sys.path.append("./mycroft-precise/runner") 


import pyaudio

from precise_runner import PreciseRunner, PreciseEngine
from threading import Event

WAKE_WORD_TOPIC = '/wake_word'
CONVERTED_AUDIO_TOPIC = '/converted_audio'

class WakeWord():

    def __init__(self):
        rospy.init_node('wake_word')
        self.wake_word = rospy.Publisher(WAKE_WORD_TOPIC, Bool, queue_size=10)
        self.audio = rospy.Subscriber(CONVERTED_AUDIO_TOPIC, Float32MultiArray, self.audio_cb)

        # parser = ArgumentParser('Implementation demo of precise-engine')
        # parser.add_argument('engine', help='Location of binary engine file')
        # parser.add_argument('model')
        # args = parser.parse_args()

        # very low for now!
        probability = 0.05

        # time.sleep(1)
        # engine = PreciseEngine('mycroft-precise/.venv/bin/precise-engine', './pepper.pb', chunk_size=592)
        # # stream = binary 16 bit mono!
        # PreciseRunner(engine, stream=stream, on_prediction=self.on_prediction, on_activation=self.on_activation,
        #             trigger_level=1).start()
        # Event().wait()  # Wait forever

    def audio_cb(self, data):
        chunk_size = 592
        read_divisor = 1
        audiodata = np.asarray(data.data)
        tmp_stream = io.BytesIO(audiodata)

        chunk = tmp_stream.read(chunk_size // read_divisor)

        # print(audiodata.dtype)
        data = audiodata.astype(np.int16).tostring()
        stream.write(data)

        # self.stream.write(audiodata)

        # print(len(io.BytesIO(audiodata)))
        # self.stream = audiodata.tobytes()
        

    def on_prediction(self, prob):
        # print(prob)
        probability = 0.01
        if prob > probability:
            print'!',
            self.wake_word.publish(Bool(True))
        else:
            print'.',

    def on_activation(self):
        pass
        # activate_notify()
        # print("ACTIVATE")


if __name__ == '__main__':

    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16,
        channels=1,
        rate=16000,
        output=True,
        output_device_index=1,
        frames_per_buffer=592
    )

    wake_word = WakeWord()

    rospy.spin()
