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


PRECISE_DIR = './src/speech_recognition/wake_word_engine/mycroft-precise/'
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

    def __init__(self):
        self.wake_word = rospy.Publisher(WAKE_WORD_TOPIC, Bool, queue_size=10)
         
        # self.audio = rospy.Subscriber(CONVERTED_AUDIO_TOPIC, Float32MultiArray, self.audio_cb)

        # very low for now!
        probability = 0.05

        engine = PreciseEngine(os.path.join(PRECISE_DIR, '.venv/bin/precise-engine'), 
            os.path.join(PRECISE_DIR,'./pepper.pb'), 
            chunk_size=1024)
        # stream = binary 16 bit mono!
        PreciseRunner(engine, stream=stream, on_prediction=self.on_prediction, on_activation=self.on_activation,
                    trigger_level=3, sensitivity=0.999).start()
        # Event().wait()

    def on_prediction(self, prob):
        probability = 0.01
        if prob > probability:
            print'!',
        else:
            print'.',
    

    def on_activation(self):
        self.wake_word.publish(Bool(True))
        color = ColorRGBA()
        color.r = random.randint(0,255)
        color.g = random.randint(0,255)
        color.b = random.randint(0,255)
        color.a = 0.0

        for i in range(0, 8):
            d = rospy.Duration.from_sec(0.1)
            led_msg = FadeRGB()
            led_msg.led_name = 'FaceLedRight' + str(i)
            led_msg.color = color
            led_msg.fade_duration = d
            led.publish(led_msg)
            
        for i in range(0, 8):
            d = rospy.Duration.from_sec(0.1)
            led_msg = FadeRGB()
            led_msg.led_name = 'FaceLedLeft' + str(i)
            led_msg.color = color
            led_msg.fade_duration = d
            led.publish(led_msg)

        for i in range(1, 11):
            d = rospy.Duration.from_sec(0.1)
            led_msg = FadeRGB()
            led_msg.led_name = 'RightEarLed' + str(i)
            led_msg.color = color
            led_msg.fade_duration = d
            led.publish(led_msg)
            
        for i in range(1, 11):
            d = rospy.Duration.from_sec(0.1)
            led_msg = FadeRGB()
            led_msg.led_name = 'LeftEarLed' + str(i)
            led_msg.color = color
            led_msg.fade_duration = d
            led.publish(led_msg)
        
 
        

if __name__ == '__main__':
    rospy.init_node('wake_word')
    led = rospy.Publisher(LED_TOPIC, FadeRGB, queue_size=10)
    wake_word = WakeWord()
    rospy.spin()
