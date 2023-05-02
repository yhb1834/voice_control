#!/usr/bin/env python3
import requests
import json

import pyaudio
import wave
from gtts import gTTS
import playsound

import rospy
import time
from geometry_msgs.msg import Twist 

import sys
import requests

'''
client_id = ""
client_secret = ""
lang = "Kor" # 언어 코드 ( Kor )
url = "https://naveropenapi.apigw-pub.fin-ntruss.com/recog/v1/stt?lang=" + lang
data = open('/home/r1d2/Desktop/example.wav', 'rb')
headers = {
    "X-NCP-APIGW-API-KEY-ID": client_id,
    "X-NCP-APIGW-API-KEY": client_secret,
    "Content-Type": "application/octet-stream"
}
response = requests.post(url,  data=data, headers=headers)
rescode = response.status_code
if(rescode == 200):
    print (response.text)
else:
    print("Error : " + response.text)
'''

class ClovaSpeechClient:
    # Clova Speech invoke URL
    invoke_url = ''
    # Clova Speech secret key
    secret = ''

    def req_url(self, url, completion, callback=None, userdata=None, forbiddens=None, boostings=None, wordAlignment=True, fullText=True, diarization=None):
        request_body = {
            'url': url,
            'language': 'ko-KR',
            'completion': completion,
            'callback': callback,
            'userdata': userdata,
            'wordAlignment': wordAlignment,
            'fullText': fullText,
            'forbiddens': forbiddens,
            'boostings': boostings,
            'diarization': diarization,
        }
        headers = {
            'Accept': 'application/json;UTF-8',
            'Content-Type': 'application/json;UTF-8',
            'X-CLOVASPEECH-API-KEY': self.secret
        }
        return requests.post(headers=headers,
                             url=self.invoke_url + '/recognizer/url',
                             data=json.dumps(request_body).encode('UTF-8'))

    def req_object_storage(self, data_key, completion, callback=None, userdata=None, forbiddens=None, boostings=None,
                           wordAlignment=True, fullText=True, diarization=None):
        request_body = {
            'dataKey': data_key,
            'language': 'ko-KR',
            'completion': completion,
            'callback': callback,
            'userdata': userdata,
            'wordAlignment': wordAlignment,
            'fullText': fullText,
            'forbiddens': forbiddens,
            'boostings': boostings,
            'diarization': diarization,
        }
        headers = {
            'Accept': 'application/json;UTF-8',
            'Content-Type': 'application/json;UTF-8',
            'X-CLOVASPEECH-API-KEY': self.secret
        }
        return requests.post(headers=headers,
                             url=self.invoke_url + '/recognizer/object-storage',
                             data=json.dumps(request_body).encode('UTF-8'))

    def req_upload(self, file, completion, callback=None, userdata=None, forbiddens=None, boostings=None,
                   wordAlignment=True, fullText=True, diarization=None):
        request_body = {
            'language': 'ko-KR',
            'completion': completion,
            'callback': callback,
            'userdata': userdata,
            'wordAlignment': wordAlignment,
            'fullText': fullText,
            'forbiddens': forbiddens,
            'boostings': boostings,
            'diarization': diarization,
        }
        headers = {
            'Accept': 'application/json;UTF-8',
            'X-CLOVASPEECH-API-KEY': self.secret
        }
        print(json.dumps(request_body, ensure_ascii=False).encode('UTF-8'))
        files = {
            'media': open(file, 'rb'),
            'params': (None, json.dumps(request_body, ensure_ascii=False).encode('UTF-8'), 'application/json')
        }
        response = requests.post(headers=headers, url=self.invoke_url + '/recognizer/upload', files=files)
        return response

def speak(text):
     tts = gTTS(text=text, lang='ko')
     filename='voice.mp3'
     tts.save(filename)
     playsound.playsound(filename)


class CmdVelPublisher:
    def __init__(self):
        rospy.init_node('cmd_vel_publisher')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def publish_cmd_vel(self, linear_x, angular_z):
        vel_msg = Twist()
        vel_msg.linear.x = linear_x
        vel_msg.angular.z = angular_z
        self.pub.publish(vel_msg)
        self.rate.sleep()



if __name__ == '__main__':
    # res = ClovaSpeechClient().req_url(url='http://example.com/media.mp3', completion='sync')
    # res = ClovaSpeechClient().req_object_storage(data_key='data/media.mp3', completion='sync')
    # res = ClovaSpeechClient().req_upload(file='/data/media.mp3', completion='sync')
    
    cmd_vel_publisher = CmdVelPublisher()

    # 녹음할 설정
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 44100
    CHUNK = 1024
    RECORD_SECONDS = 3

    # PyAudio 객체 생성
    audio = pyaudio.PyAudio()
    
    try:
        while not rospy.is_shutdown():
                # 마이크에서 오디오 녹음 시작
                stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK, input_device_index=2)
                

                frames = []
                for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
                    try:
                        data = stream.read(CHUNK)
                        frames.append(data)
                        # 처리할 코드
                    except OSError as ex:
                        if ex.errno != pyaudio.paInputOverflowed:
                            raise ex
                        continue
                    # data = stream.read(CHUNK)
                    

                # 마이크에서 오디오 녹음 종료
                stream.stop_stream()
                stream.close()
                audio.terminate()


                # 녹음한 데이터를 WAV 파일로 저장
                wf = wave.open("/home/r1d2/catkin_ws/src/waypoint_process/src/example.wav", "wb")
                wf.setnchannels(CHANNELS)
                wf.setsampwidth(audio.get_sample_size(FORMAT))
                wf.setframerate(RATE)
                wf.writeframes(b"".join(frames))
                wf.close()


                res = ClovaSpeechClient().req_upload(file='/home/r1d2/catkin_ws/src/waypoint_process/src/example.wav', completion='sync')
                rescode = res.status_code

                stt = res.text
                json_data = json.loads(stt)
                text = json_data["text"]
                print(stt)

                if(rescode == 200):
                    print(text)
                    if ('앞으로' or '앞으로가' or "전진") in stt:
                        cmd_vel_publisher.publish_cmd_vel(0.5, 0.0)
                        speak(stt)
                    elif ('뒤로' or '뒤로가' or '후진') in stt:
                        cmd_vel_publisher.publish_cmd_vel(-0.5, 0.0)
                        speak(stt)
                    elif '왼쪽으로가' in stt:
                        cmd_vel_publisher.publish_cmd_vel(0.2, 0.2)
                        speak(stt)
                    elif '오른쪽으로가' in stt:
                        cmd_vel_publisher.publish_cmd_vel(0.2, -0.2)
                        speak(stt)
                    elif '멈춰' in stt:
                        cmd_vel_publisher.publish_cmd_vel(0.0, 0.0)
                        speak("stop")
                else:
                    pass


    except KeyboardInterrupt:
        pass

    ####################################### Return ###################################################
    #planner.add_waypoint(0.0, 0.0, 0.0)      
    #rospy.loginfo('Completed!')

    rospy.is_shutdown()
    

