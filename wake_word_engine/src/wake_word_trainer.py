#!/usr/bin/python
import rospy
import sys
import os
import numpy as np
import uuid
import soundfile
from std_msgs.msg import Float32MultiArray, Bool

AUDIO_TOPIC = '/stereo_audio'



class SampleCollection():

    def __init__(self):
        rospy.init_node('wake_word_trainer')
        self.sample_rate = 48000
        self.channels = 2

        self.recording = False
        self.record_buffer = []

        self.file_name = 'pepper' # raw_input("What is the filename you want to write to? (e.g. pepper-##.wav)\n")

        rospy.Subscriber('/wake_word_recording', Bool, self.is_recording)
        rospy.Subscriber(AUDIO_TOPIC, Float32MultiArray, self.audio_cb)

    def is_recording(self, data):
        self.recording = data.data

    def wav_path(self):
        return os.path.join(os.path.expanduser("~"), self.file_name + '-' + str(uuid.uuid4()) + '.wav')

    def audio_cb(self, data):
        audiodata = np.asarray(data.data)
        if self.recording:
            print("*** RECORDING ***")
            self.record_buffer.append(audiodata)
        
        if not self.recording and len(self.record_buffer) > 0:
            print("*** WRITING TO FILE ***")
            recorded_audio = np.concatenate(self.record_buffer, axis=0).astype(np.int16)
            soundfile.write(self.wav_path(), recorded_audio, samplerate=self.sample_rate, subtype='PCM_16')        
            self.record_buffer = []

if __name__ == '__main__':
    print("*** WAKE WORD SAMPLE COLLECTION ***")
    print("*** PEPPER ***")

    sc = SampleCollection()
    rospy.spin()