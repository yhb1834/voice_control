#!/usr/bin/env python3
# coding: utf-8

import speech_recognition as sr
from gtts import gTTS
import os
import time
import playsound

import rospy
import time
from geometry_msgs.msg import PoseStamped

def speak(text):
     tts = gTTS(text=text, lang='ko')
     filename='voice.mp3'
     tts.save(filename)
     playsound.playsound(filename)

speak("안녕하세요.")

class WaypointPlanner:
    def __init__(self):
        self.waypoints = []
        self.publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        #self.current_head = pi / 2 #-90.0 * 3.14 / 180.0

    def add_waypoint(self, x, y, theta):
        pose = PoseStamped()
        pose.header.frame_id = 'map'

        ### PoseStamped
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = theta
        pose.pose.orientation.w = 1.0
        self.waypoints.append(pose)
       

    def execute_waypoints(self):
        # run all waypoints
        for waypoint in self.waypoints:
            rospy.sleep(3) # Wait for 5 seconds for robot to reach the goal
            rospy.loginfo('Moving to waypoint: {}'.format(waypoint))
            self.publisher.publish(waypoint)
            rospy.loginfo("Wait for 5s for robot to reach the goal")
            rospy.sleep(5) # Wait for 5 seconds for robot to reach the goal
        
        

if __name__ == '__main__':
    rospy.init_node('VoiceControl', anonymous=True)
    planner = WaypointPlanner()

    try:
        while True:
            r = sr.Recognizer()
            
            with sr.Microphone() as source:
                print('음성을 입력하세요.')
                audio = r.listen(source)
                try:
                    stt = r.recognize_google(audio, language='ko-KR')
                    print('음성변환 : ' + stt)

                    if ('앞으로 가' or "전진") in stt:
                        planner.add_waypoint(1.0, 0.0, 0.0)
                        planner.execute_waypoints()
                        speak("전진")
                    elif ('뒤로 가' or '후진') in stt:
                        planner.add_waypoint(1.0, 0.0, 0.0)
                        planner.execute_waypoints()
                        speak("후진")
                    elif '왼쪽으로 가' in stt:
                        planner.add_waypoint(1.0, 0.0, 0.0)
                        planner.execute_waypoints()
                        speak("안녕하세요. 길동씨")
                    elif '오른쪽으로 가' in stt:
                        planner.add_waypoint(1.0, 0.0, 0.0)
                        planner.execute_waypoints()
                        speak("안녕하세요. 길동씨")
                    elif '멈춰' in stt:
                        planner.add_waypoint(0.0, 0.0, 0.0)
                        planner.execute_waypoints()
                        speak("안녕하세요. 길동씨")
                    
                    
                except sr.UnknownValueError:
                    print('오디오를 이해할 수 없습니다.')
                except sr.RequestError as e:
                    print(f'에러가 발생하였습니다. 에러원인 : {e}')
                    
    except KeyboardInterrupt:
        pass

    ####################################### Return ###################################################
    #planner.add_waypoint(0.0, 0.0, 0.0)      
    rospy.loginfo('Completed!')
    

