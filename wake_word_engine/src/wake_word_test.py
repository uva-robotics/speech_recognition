#!/usr/bin/env python
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
from std_msgs.msg import Float32MultiArray, String
from std_msgs.msg import ColorRGBA

from naoqi_bridge_msgs.msg import FadeRGB

import os
import sys
import io
import time
import random

import numpy as np
from argparse import ArgumentParser
# from precise.util import activate_notify

import pyaudio
import sounddevice as sd


# PRECISE_DIR = rospy.get_param('~engine')
PRECISE_DIR = '/home/spijkervet/git/uva-robotics/main/src/vendor/src/mycroft-precise'
sys.path.append(os.path.join(PRECISE_DIR, "runner"))

from precise_runner import PreciseRunner, PreciseEngine
from threading import Event

import time

WAKE_WORD_TOPIC = '/wake_word'
CONVERTED_AUDIO_TOPIC = '/converted_audio'

LED_TOPIC = '/fade_rgb'

print(sd.query_devices())


p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16,
    channels=1,
    rate=16000,
    input=True,
    frames_per_buffer=1024,
    # input_device_index=7
)


class WakeWord():

    def __init__(self, model):
        self.wake_word = rospy.Publisher(WAKE_WORD_TOPIC, Bool, queue_size=10)
         
        # self.audio = rospy.Subscriber(CONVERTED_AUDIO_TOPIC, Float32MultiArray, self.audio_cb)

        engine = PreciseEngine(os.path.join(PRECISE_DIR, '.venv/bin/precise-engine'), 
            model,
            chunk_size=1024
        )

        # stream = binary 16 bit mono!
        PreciseRunner(engine, stream=stream, on_prediction=self.on_prediction, 
            on_activation=self.on_activation,
            trigger_level=3, sensitivity=0.9
        ).start()
        # Event().wait()

    def on_prediction(self, prob):
        pass
    

    def on_activation(self):
        self.wake_word.publish(Bool(True))
        color = ColorRGBA()
        color.r = 0
        color.g = 255
        color.b = 0
        color.a = 0.0

        d = rospy.Duration.from_sec(0.0)

        led_msg = FadeRGB()
        led_msg.led_name = 'FaceLeds'
        led_msg.color = color
        led_msg.fade_duration = d
        led.publish(led_msg)

        # SLEEP
        time.sleep(1)

        color = ColorRGBA()
        color.r = 0
        color.g = 0
        color.b = 255
        color.a = 0.0
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
    rospy.init_node('wake_word')
    led = rospy.Publisher(LED_TOPIC, FadeRGB, queue_size=10)
    model = rospy.get_param('~wake_word_model')
    wake_word = WakeWord(model)
    rospy.spin()
