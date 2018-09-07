#!/usr/bin/python

# SPEECH RECOGNITION
#
# A package for, as the name says it, recognising speech. Made to run on pepper
# and sending the result over ros.

################### CHANGELOG ####################
## v0.1.0 (alpha):                              ##
##  + Begun development                         ##
##  + Created package                           ##
##  + Added way to connect to Pepper audio      ##
##################################################

import ros
import argparse
import naoqi

__VERSION = "0.1.0"

# Class for recognising speech
class SpeechRecogniserALSpeech ():
    def __init__():
        pass


# Main
def main (pepper_ip):
    # Print welcoming message
    print("\n########################")
    print("## SPEECH RECOGNITION ##")
    print("##       v"+__VERSION+"       ##")
    print("########################\n\n")

    print("USES:\n")
    print(" - Pepper's IP: {}\n".format(pepper_ip))

    print("Setting up speech recogniser...")
    asr = naoqi.ALProxy("ALSpeechRecognition", pepper_ip, 9559)

# Entry point
if __name__ == '__main__':
    # Get arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-ip", "--ipaddress", help="The IP address op pepper")
    args = parser.parse_args()

    # Parse arguments
    pepper_ip = "146.50.60.54"
    if args.ipaddress:
        pepper_ip = args.ipaddress

    # Run main
    main(pepper_ip)
