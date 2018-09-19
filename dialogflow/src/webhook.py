#!/usr/bin/python
from __future__ import unicode_literals

from flask import Flask, request, jsonify, render_template
import os
import dialogflow
import requests
import json
import subprocess
import random
import signal

from six.moves import urllib
import re

import youtube_dl

app = Flask(__name__)

# to kill the currently playing song.
player_pid = None

def dl_hook(d):
    if d['status'] == 'finished':
        print('Done downloading, now converting ...')

def play_song(path):
    global player_pid
    if player_pid is not None:
        os.kill(player_pid, signal.SIGINT)
    player = subprocess.Popen(['play', path])
    player_pid = player.pid


def music_play_song(data):
    DL_PATH = './'

    genre = data['queryResult']['parameters']['music-genre']
    artist = data['queryResult']['parameters']['music-artist']
    song_name = data['queryResult']['parameters']['given-name']
    
    rand = False
    print('DIALOGFLOW_SONG: ' + genre + ' - ' + artist + ' - ' + song_name)
    if genre:
        print("DIALOGFLOW_SONG: Downloading genre.")
        q = str(genre)
    elif artist and song_name:
        print('DIALOGFLOW_SONG: Downloading ' + song_name + ' by ' + artist)
        q = str(song_name + ' ' + artist)
    else:
        print('DIALOGFLOW_SONG: Downloading random song by ' + artist)
        q = str(artist)
        rand = True

    query_string = urllib.parse.urlencode({"search_query" : q})
    html_content = urllib.request.urlopen("http://www.youtube.com/results?" + query_string)
    search_results = re.findall(r'href=\"\/watch\?v=(.{11})', html_content.read().decode('utf-8'))
   
    if not rand:
        yt_url = "http://www.youtube.com/watch?v=" + search_results[0]
    else:
        yt_url = "http://www.youtube.com/watch?v=" + search_results[random.randint(0, len(search_results)-1)]

    ydl_opts = {
        'outtmpl': DL_PATH + '%(title)s.%(ext)s',
        'format': 'bestaudio/best',
        'postprocessors': [{
            'key': 'FFmpegExtractAudio',
            'preferredcodec': 'mp3',
            'preferredquality': '192',
        }],
        'progress_hooks': [dl_hook]
    }
    
    with youtube_dl.YoutubeDL(ydl_opts) as ydl:
        ydl.download([yt_url])
        info = ydl.extract_info(yt_url, download=True)

        song_path = os.path.join(DL_PATH, info.get('title', None) + '.mp3')
        if os.path.isfile(song_path):
            play_song(song_path)
    

@app.route('/test', methods=['GET', 'POST'])
def test():
    data = request.get_json(silent=True)
    try:
        intent = data['queryResult']['intent']['displayName']
        if intent == 'music.play.song':
            music_play_song(data)
    except Exception as e:
        print('*** ERROR *** - ' + e)
        pass

    reply = {
        "fulfillmentText": '',
    }
    return jsonify(reply)

# run Flask app
if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=True)