import sys
import os
import argparse
import rospy
from std_msgs.msg import String
from naoqi_bridge_msgs.msg import AudioBuffer

sys.path.append(os.path.join(os
.path.dirname(__file__), '../Porcupine/binding/python'))
from porcupine import Porcupine
from threading import Thread
import platform
import audioop
import numpy as np
import struct
import soundfile

import collections
from Queue import Queue, Empty

class RingBuffer:
    def __init__(self, size):
        self.data = collections.deque(maxlen=size)

    def append(self, x):
        # self.data.pop(0)
        self.data.append(x)

    def get(self):
        return self.data


class PorcupineDemo(Thread):
    """
    Demo class for wake word detection (aka Porcupine) library. It creates an input audio stream from a microphone,
    monitors it, and upon detecting the specified wake word(s) prints the detection time and index of wake word on
    console. It optionally saves the recorded audio into a file for further review.
    """

    def __init__(
            self,
            library_path,
            model_file_path,
            keyword_file_paths,
            sensitivities,
            input_device_index=None,
            output_path=None):

        """
        Constructor.

        :param library_path: Absolute path to Porcupine's dynamic library.
        :param model_file_path: Absolute path to the model parameter file.
        :param keyword_file_paths: List of absolute paths to keyword files.
        :param sensitivities: Sensitivity parameter for each wake word. For more information refer to
        'include/pv_porcupine.h'. It uses the
        same sensitivity value for all keywords.
        :param input_device_index: Optional argument. If provided, audio is recorded from this input device. Otherwise,
        the default audio input device is used.
        :param output_path: If provided recorded audio will be stored in this location at the end of the run.
        """

        super(PorcupineDemo, self).__init__()

        self._library_path = library_path
        self._model_file_path = model_file_path
        self._keyword_file_paths = keyword_file_paths
        self._sensitivities = sensitivities
        self._input_device_index = input_device_index

        self._output_path = output_path
        if self._output_path is not None:
            self._recorded_frames = []

        rospy.init_node('wake_word', anonymous=True)
        # rospy.Subscriber("/mono_signal", String, self.callback)
        rospy.Subscriber("/pepper_robot/audio", AudioBuffer, self.callback)
        self.pcm = ""
        self.buffer = RingBuffer(512)

        self.num_keywords = len(self._keyword_file_paths)

        self.keyword_names =\
            [os.path.basename(x).replace('.ppn', '').replace('_tiny', '').split('_')[0] for x in self._keyword_file_paths]

        print('listening for:')
        for keyword_name, sensitivity in zip(self.keyword_names, sensitivities):
            print('- %s (sensitivity: %f)' % (keyword_name, sensitivity))

        self.porcupine = None
        self.pa = None
        self.audio_stream = None

    def callback(self, msg):
        print(len(msg.data))
        buffer_size = 1024
        # print("CHUNK", len(msg.data), len(msg.channelMap), len(msg.data)/4)
        # Signed 16 bit PCM, 16 bit = 2 bytes (sample width!)
        # mic = self.convertStr2SignedInt(msg.data)
        data = list(msg.data)
        mic = self.convertStr2Str(data)
        # mic = self.convertStr2SignedInt(mic)
        # mic = audioop.tomono(mic, 2, 1, 1)
        pcm = mic

        try:
            self.porcupine = Porcupine(
                library_path=self._library_path,
                model_file_path=self._model_file_path,
                keyword_file_paths=self._keyword_file_paths,
                sensitivities=self._sensitivities)

            if len(pcm) > 0:
                pcm = np.fromstring(pcm, np.int16)
                pcm = pcm[0::len(pcm)/1500]

                # pcm = struct.unpack_from("h" * self.porcupine.frame_length, pcm)                
                if self._output_path is not None:
                    self._recorded_frames.append(pcm)

                result = self.porcupine.process(pcm)
                if self.num_keywords == 1 and result:
                    print('[%s] detected keyword' % str(datetime.now()))
                elif self.num_keywords > 1 and result >= 0:
                    print('[%s] detected %s' % (str(datetime.now()), self.keyword_names[result]))
            
                if self.porcupine is not None:
                    self.porcupine.delete()

                if self.audio_stream is not None:
                    self.audio_stream.close()

                if self.pa is not None:
                    self.pa.terminate()

                if self._output_path is not None and len(self._recorded_frames) > 50:
                    print("SAMPLE RATE", self.porcupine.sample_rate)
                    recorded_audio = np.concatenate(self._recorded_frames, axis=0).astype(np.int16) # porcupine.sample_rate
                    soundfile.write(self._output_path, recorded_audio, samplerate=self.porcupine.sample_rate, subtype='PCM_16')
                _AUDIO_DEVICE_INFO_KEYS = ['index', 'name', 'defaultSampleRate', 'maxInputChannels']
                exit()
        except KeyboardInterrupt:
            pass


    def convertStr2Str(self, data):
        tmp = list(data)
        dataBuff = ""
        for i in range (0,len(tmp)) :
            if tmp[i]<0 :
                tmp[i]=tmp[i]+65536
            dataBuff = dataBuff + chr(tmp[i]%256)
            dataBuff = dataBuff + chr((tmp[i] - (tmp[i]%256)) / 256)
        return dataBuff
        

    def convertStr2SignedInt(self, data) :
        """
        This function takes a string containing 16 bits little endian sound
        samples as input and returns a vector containing the 16 bits sound
        samples values converted between -1 and 1.
        """
        signedData=[]
        ind=0
        for i in range (0,len(data)/2) :
            signedData.append(data[ind]+data[ind+1]*256)
            ind=ind+2

        for i in range (0,len(signedData)) :
            if signedData[i]>=32768 :
                signedData[i]=signedData[i]-65536

        for i in range (0,len(signedData)) :
            signedData[i]=signedData[i]/32768.0
        return signedData


    @classmethod
    def show_audio_devices_info(cls):
        """ Provides information regarding different audio devices available. """

        pa = pyaudio.PyAudio()

        for i in range(pa.get_device_count()):
            info = pa.get_device_info_by_index(i)
            print(', '.join("'%s': '%s'" % (k, str(info[k])) for k in cls._AUDIO_DEVICE_INFO_KEYS))

        pa.terminate()


def _default_library_path():
    system = platform.system()
    machine = platform.machine()

    if system == 'Darwin':
        return os.path.join(os.path.dirname(__file__), '../Porcupine/lib/mac/%s/libpv_porcupine.dylib' % machine)
    elif system == 'Linux':
        if machine == 'x86_64' or machine == 'i386':
            return os.path.join(os.path.dirname(__file__), '../Porcupine/lib/linux/%s/libpv_porcupine.so' % machine)
        else:
            raise Exception('cannot autodetect the binary type. Please enter the path to the shared object using --library_path command line argument.')
    elif system == 'Windows':
        if platform.architecture()[0] == '32bit':
            return os.path.join(os.path.dirname(__file__), '..\\Porcupine\\lib\\windows\\i686\\libpv_porcupine.dll')
        else:

            return os.path.join(os.path.dirname(__file__), '..\\Porcupine\\lib\\windows\\amd64\\libpv_porcupine.dll')
    raise NotImplementedError('Porcupine is not supported on %s/%s yet!' % (system, machine))



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--keyword_file_paths', help='comma-separated absolute paths to keyword files', type=str)

    parser.add_argument(
        '--library_path',
        help="absolute path to Porcupine's dynamic library",
        type=str)

    parser.add_argument(
        '--model_file_path',
        help='absolute path to model parameter file',
        type=str,
        default=os.path.join(os.path.dirname(__file__), '../Porcupine/lib/common/porcupine_params.pv'))

    parser.add_argument('--sensitivities', help='detection sensitivity [0, 1]', default=0.5)
    parser.add_argument('--input_audio_device_index', help='index of input audio device', type=int, default=None)

    parser.add_argument(
        '--output_path',
        help='absolute path to where recorded audio will be stored. If not set, it will be bypassed.',
        type=str,
        default=None)


    args = parser.parse_args()

    if not args.keyword_file_paths:
        raise ValueError('keyword file paths are missing')
    
    keyword_file_paths = [x.strip() for x in args.keyword_file_paths.split(',')]

    if isinstance(args.sensitivities, float):
            sensitivities = [args.sensitivities] * len(keyword_file_paths)
    else:
        sensitivities = [float(x) for x in args.sensitivities.split(',')]

    try:
        p = PorcupineDemo(
                library_path=args.library_path if args.library_path is not None else _default_library_path(),
                model_file_path=args.model_file_path,
                keyword_file_paths=keyword_file_paths,
                sensitivities=sensitivities,
                output_path=args.output_path,
                input_device_index=args.input_audio_device_index)
        p.show_audio_devices_info()
        # p.run()
        rospy.spin()
    except KeyboardInterrupt:
        pass


