#!/usr/bin/env python3
# coding: utf-8
import os
import azure.cognitiveservices.speech as speechsdk
#import speech_recognition as sr
from gtts import gTTS
import os
import time
import playsound

import rospy
import time
from geometry_msgs.msg import Twist 


speech_key = "key"
service_region = "region"


def recognize_from_microphone():
    # This example requires environment variables named "SPEECH_KEY" and "SPEECH_REGION"
    #speech_config = speechsdk.SpeechConfig(subscription=os.environ.get('SPEECH_KEY'), region=os.environ.get('SPEECH_REGION'))
    speech_config = speechsdk.SpeechConfig(subscription=speech_key, region=service_region)
    speech_config.speech_recognition_language="ko-KR"

    audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)
    speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_config)

    print("Speak into your microphone.")
    speech_recognition_result = speech_recognizer.recognize_once_async().get()

    if speech_recognition_result.reason == speechsdk.ResultReason.RecognizedSpeech:
        print("Recognized: {}".format(speech_recognition_result.text))
        return speech_recognition_result.text
    elif speech_recognition_result.reason == speechsdk.ResultReason.NoMatch:
        print("No speech could be recognized: {}".format(speech_recognition_result.no_match_details))
    elif speech_recognition_result.reason == speechsdk.ResultReason.Canceled:
        cancellation_details = speech_recognition_result.cancellation_details
        print("Speech Recognition canceled: {}".format(cancellation_details.reason))
        if cancellation_details.reason == speechsdk.CancellationReason.Error:
            print("Error details: {}".format(cancellation_details.error_details))
            print("Did you set the speech resource key and region values?")
    


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
    cmd_vel_publisher = CmdVelPublisher()

    ################ IMPORTANT!!!!!!!!!!!!!!!
    rospy.sleep(3)
    ################

    #cmd_vel_publisher.publish_cmd_vel(0.5, 0.5)
    #cmd_vel_publisher.publish_cmd_vel(0.0, 0.0)

    try:
        while not rospy.is_shutdown():
            stt = recognize_from_microphone()
            print('음성변환 : ' + stt)
                    
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
                
                    
    except KeyboardInterrupt:
        pass
