#!/usr/bin/env python

import sounddevice
import rospy
from std_msgs.msg import String
import speech_recognition as sr

def googlesr():
    rospy.init_node('googlesrmalay', anonymous=True)
    # Initialize google sr result publisher
    pub = rospy.Publisher('malay_sr', String, queue_size=10)

    while not rospy.is_shutdown():
        # obtain audio from the microphone
        r = sr.Recognizer()
        
        with sr.Microphone() as source:
            print(">>> Say something!")
            #audio = r.listen(source)
            audio = r.record(source, duration=10)
            
        # recognize speech using Google Speech Recognition
        try:
            result = r.recognize_google(audio, language="ms-MY")
            print("SR result: " + result)
        except sr.UnknownValueError:
            print("SR could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
        
        pub.publish(result)

if __name__ == '__main__':
    try:
        googlesr()
    except rospy.ROSInterruptException:
        pass
