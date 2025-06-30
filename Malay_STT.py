#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import speech_recognition as sr

class MalaySTTNode:
    def __init__(self):
        rospy.init_node('malay_stt_node')
        self.pub = rospy.Publisher('malay_gtts_input', String, queue_size=10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        rospy.loginfo("Malay STT Node Ready. Listening...")

    def listen_and_publish(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
            try:
                rospy.loginfo("Speak now (Malay)...")
                audio = self.recognizer.listen(source, timeout=3)
                text = self.recognizer.recognize_google(audio, language="ms-MY")
                rospy.loginfo(f"Recognized: {text}")
                self.pub.publish(text)
            except sr.UnknownValueError:
                rospy.logwarn("Could not understand audio")
            except sr.RequestError as e:
                rospy.logerr(f"API error: {e}")
            except sr.WaitTimeoutError:
                rospy.logwarn("Listening timeout")

    def run(self):
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            self.listen_and_publish()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = MalaySTTNode()
        node.run()
    except rospy.ROSInterruptException:
        pass