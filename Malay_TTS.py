#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gtts import gTTS
import os
import tempfile
import pygame
from pygame import mixer

class MalayTTSNode:
    def __init__(self):
        rospy.init_node('malay_tts_node', anonymous=True)

        # Initialize pygame mixer for audio playback
        mixer.init()

        # Subscriber for text input
        self.subscriber = rospy.Subscriber('malay_gtts_input', String, self.tts_callback)

        rospy.loginfo("Malay TTS Node initialized. Waiting for text input...")

    def tts_callback(self, msg):
        text = msg.data
        if not text:
            rospy.logwarn("Received empty text message")
            return

        rospy.loginfo(f"Received text for TTS conversion: {text}")

        try:
            # Create temporary file for the audio
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as temp_audio_file:
                temp_filename = temp_audio_file.name

            # Convert text to speech in Malay
            tts = gTTS(text=text, lang='ms')  # 'ms' is the language code for Malay
            tts.save(temp_filename)

            # Play the audio
            self.play_audio(temp_filename)

            # Clean up
            os.unlink(temp_filename)

            rospy.loginfo("TTS conversion and playback completed successfully")

        except Exception as e:
            rospy.logerr(f"Error in TTS conversion: {str(e)}")

    def play_audio(self, filename):
        """Play the audio file using pygame mixer"""
        mixer.music.load(filename)
        mixer.music.play()

        # Wait for the audio to finish playing
        while mixer.music.get_busy():
            pygame.time.Clock().tick(10)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = MalayTTSNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        pygame.quit()