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

import ros
import argparse
import naoqi
import time
#import SpeechRecognition
from array import array
from struct import pack
import pyaudio
import wave

__VERSION = "0.2.0"

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
        # Get a sample of speech
        print("First, we need a sample of speech.\nPress Enter to begin recording..."),
        raw_input()
        # Start the recording
        p = pyaudio.PyAudio()
        stream = p.open(format=FORMAT,channels=1,rate=FRAMERATE,input=True,output=False,frames_per_buffer=CHUNK_SIZE)
        start = time.time()
        data = []
        # Record for 5 seconds
        while time.time() - start <= 5:
            data.extend(array('h', stream.read(CHUNK_SIZE)))
        sample_width = p.get_sample_size(FORMAT)
        stream.stop_stream()
        stream.close()
        p.terminate()
        print("Done\nWriting to '"+PATH+"'..."),

        # Now that we have the data, write it to a .WAV
        data = pack('<' + ('h'*len(data)), *data)
        wf = wave.open(PATH, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(sample_width)
        wf.setframerate(FRAMERATE)
        wf.writeframes(data)
        wf.close()

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
