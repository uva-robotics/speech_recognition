#!/usr/bin/python

# SPEECH RECOGNITION
#
# A package for, as the name says it, recognising speech. Made to run on pepper
# and sending the result over ros.

################### CHANGELOG ####################
## v0.1.0 (alpha):                              ##
##  + Begun development                         ##
##  + Created package                           ##
##  + Added initial test of ALSpeechRecognition ##
##################################################
## v0.2.0 (alpha):                              ##
##  + Temporarily creating program for laptop   ##
##    use, to refrain from installing possibly  ##
##    useless things on Pepper                  ##
##  + Added the recording of files using        ##
##    PyAudio                                   ##
##  - Removed test of ALSpeechRecognition       ##
##################################################
## v0.3.0 (alpha):                              ##
##  + Added recognising of speech using         ##
##    speech_recognition                        ##
##  - Removed recording of files (since we can  ##
##    directly access the mic)                  ##
##################################################
## v0.4.0 (alpha):                              ##
##  + Added keyword triggering to record a file ##
##    of 10 seconds upon saying 'Pepper'        ##
##################################################

import naoqi
import rospy
from std_msgs.msg import String
import argparse
import time
import speech_recognition as sr
from array import array
from struct import pack
import pyaudio
import wave

__VERSION = "0.3.0"

# Recogniser class
class SpeechRecogniser (naoqi.ALModule):
    def __init__(self, pepper_ip):
        self.setName("SpeechRecogniser")
        self.publisher = rospy.Publisher("/speech", String, queue_size=10)
        self.memory = naoqi.ALProxy("ALMemory", pepper_ip, 9559)
        self.memory.subscribeToEvent("WordRecognized",
            "SpeechRecogniser",
            "speechCallBack")
        self.proxy = naoqi.ALProxy("ALSpeechRecognition", pepper_ip, 9559)
        #self.proxy.setVocabulary(["Pepper"], False)
        self.recogniser = sr.Recognizer()
        self.log("Initialized")

    def start (self):
        self.proxy.subscribe("Speech_Recognition_Service")
        self.log("Started")

    def stop (self):
        self.proxy.unsubscribe("Speech_Recognition_Service")
        self.log("Stopped")


    def speechCallback(eventName, value, subscriberID):
        self.publisher.publish("Yes?")
        self.log("Triggered, listening for audio...")
        # Collect audio file for ten seconds
        time.sleep(5)
        # Done
        self.publisher.publish("Thinking")
        self.log("Got audio, processing...")
        result = "To Be done"#self.recogniser.recognize_sphinx(audio)
        self.publisher.publish("You said {}".format(result))
        self.log("Result: " + result)

    def log (self, text):
        print("{} > {}".format(self.name, text))

# Main
def main (mode, pepper_ip):
    # Print welcoming message
    print("\n########################")
    print("## SPEECH RECOGNITION ##")
    print("##       v"+__VERSION+"       ##")
    print("########################\n\n")

    print("USES:\n")
    print(" - Mode       : {}".format(mode))
    print(" - Pepper's IP: {}\n".format(pepper_ip))

    if mode == "TEST":
        PATH = "/home/lut_99/Desktop/test.wav"
        FORMAT = pyaudio.paInt16
        FRAMERATE = 44100
        CHUNK_SIZE = 1024

        # obtain audio from the microphone
        r = sr.Recognizer()
        with sr.Microphone() as source:
            print("Say something!")
            audio = r.listen(source)

        print("Done, recognising...")

        # recognize speech using Sphinx
        try:
            print("Sphinx thinks you said " + r.recognize_sphinx(audio))
        except sr.UnknownValueError:
            print("Sphinx could not understand audio")
        except sr.RequestError as e:
            print("Sphinx error; {0}".format(e))

        print("Done")

    # Init ros node
    rospy.init_node("SpeechRecogniser")

    # Load the class
    recogniser = SpeechRecogniser(pepper_ip)
    # Start
    recogniser.start()
    start = time.time()
    while True:
        try:
            if time.time() - start >= 30:
                break
        except KeyboardInterrupt:
            break
    recogniser.stop()
    print("Done")


# Entry point
if __name__ == '__main__':
    # Get arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mode", help="The mode for this program. Choose from: PEPPER and TEST")
    parser.add_argument("-ip", "--ipaddress", help="The IP address op pepper")
    args = parser.parse_args()

    # Parse arguments
    mode = "PEPPER"
    pepper_ip = "146.50.60.54"
    if args.mode:
        mode = args.mode
    if args.ipaddress:
        pepper_ip = args.ipaddress

    # Run main
    main(mode, pepper_ip)
