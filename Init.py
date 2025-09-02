#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Competition Task Initialization for Yahboom X3 Plus Robot
This script implements the first part of the competition where the robot:
1. Navigate from starting area to working cabin
2. Ask human what help they need
3. Process voice response and repeat back the request

Author: WRC EGY Team
Date: 2025
Robot: Yahboom X3 Plus with ROS1 Noetic
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import PoseStamped, Twist, Pose, Point, Quaternion
from std_msgs.msg import String, Bool, Int32
from yahboomcar_msgs.msg import *
from actionlib_msgs.msg import GoalID
from whisper_speech_lib import WhisperSpeechRecognizer
import pyttsx3
import threading
import time

class YahboomCompetitionRobot:
    def __init__(self):
        rospy.init_node('yahboom_competition_robot', anonymous=False)
        rospy.on_shutdown(self.cancel)
        
        # Initialize Yahboom robot parameters
        self.InitialParam()
        
        # Initialize move_base action client for navigation
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server(rospy.Duration(30))
        rospy.loginfo("Move_base action server connected!")
        
        # Yahboom-specific publishers
        self.pub_CmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_cancel = rospy.Publisher("move_base/cancel", GoalID, queue_size=10)
        self.pub_buzzer = rospy.Publisher('/Buzzer', Bool, queue_size=1)
        self.status_pub = rospy.Publisher('/robot_status', String, queue_size=1)
        self.task_complete_pub = rospy.Publisher('/task_complete', Bool, queue_size=1)
        
        # Yahboom-specific subscribers
        self.sub_goal_result = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.goal_result_callback)
        
        # Whisper speech recognition setup (replacing speech_recognition)
        rospy.loginfo("Initializing Whisper Speech Recognition...")
        self.speech_recognizer = WhisperSpeechRecognizer(model_size="base", sample_rate=44100, channels=1)
        rospy.loginfo("Whisper Speech Recognition initialized successfully!")
        
        # Text-to-speech setup (keep as requested)
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)  # Slower speech for clarity
        self.tts_engine.setProperty('volume', 0.9)
        
        rospy.loginfo("Yahboom Competition Robot with Whisper initialized and ready!")
    
    def InitialParam(self):
        """Initialize Yahboom robot parameters (following Yahboom pattern)"""
        # Robot state variables
        self.current_state = "STARTING"
        self.human_request = ""
        self.navigation_success = False
        self.goal_result = 0
        
        # Predefined locations for competition (adjust coordinates based on actual venue)
        self.locations = {
            'starting_area': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'working_cabin_entrance': {'x': 2.5, 'y': 1.0, 'yaw': 1.57},  # Face into cabin
            'working_cabin_center': {'x': 3.0, 'y': 1.6, 'yaw': 0.0},     # Inside cabin
            'tool_cabin': {'x': 1.0, 'y': 4.0, 'yaw': 0.0}
        }
    
    def goal_result_callback(self, msg):
        """Callback for move_base result (Yahboom pattern)"""
        if not isinstance(msg, MoveBaseActionResult): 
            return
        self.goal_result = msg.status.status
    
    def create_navigation_goal(self, location_name):
        """Create a navigation goal for move_base (Yahboom style)"""
        if location_name not in self.locations:
            rospy.logerr(f"Unknown location: {location_name}")
            return None
            
        loc = self.locations[location_name]
        
        # Create PoseStamped goal instead of MoveBaseGoal
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = rospy.Time.now()
        
        # Set position
        pose_stamped.pose.position.x = loc['x']
        pose_stamped.pose.position.y = loc['y']
        pose_stamped.pose.position.z = 0.0
        
        # Set orientation manually (simple yaw to quaternion conversion)
        yaw = loc['yaw']
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = yaw / 2.0  # Simple approximation
        pose_stamped.pose.orientation.w = 1.0
        
        return pose_stamped
    
    def navigate_to_location(self, location_name, timeout=30):
        """Navigate to specified location using Yahboom navigation style"""
        rospy.loginfo(f"Navigating to: {location_name}")
        self.publish_status(f"Navigating to {location_name}")
        
        goal_pose = self.create_navigation_goal(location_name)
        if goal_pose is None:
            return False
            
        # Publish goal using Yahboom style
        self.pub_goal.publish(goal_pose)
        
        # Wait for navigation to complete (check result)
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if self.goal_result == 3:  # SUCCEEDED
                rospy.loginfo(f"Successfully reached {location_name}")
                return True
            elif self.goal_result in [4, 5, 6, 7, 8]:  # Various failure states
                rospy.logwarn(f"Navigation to {location_name} failed with state: {self.goal_result}")
                return False
            time.sleep(0.1)
        
        rospy.logwarn(f"Navigation to {location_name} timed out")
        # Cancel goal using Yahboom style
        cancel_msg = GoalID()
        self.pub_cancel.publish(cancel_msg)
        return False
    
    def speak_text(self, text, wait_for_completion=True):
        """Make robot speak using text-to-speech (using requested pyttsx3 library)"""
        rospy.loginfo(f"Robot speaking: {text}")
        
        # Use buzzer to get attention (Yahboom style)
        buzzer_msg = Bool()
        buzzer_msg.data = True
        self.pub_buzzer.publish(buzzer_msg)
        time.sleep(0.3)  # Brief beep
        buzzer_msg.data = False
        self.pub_buzzer.publish(buzzer_msg)
        
        # Use pyttsx3 TTS as requested
        if wait_for_completion:
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()
        else:
            # Non-blocking speech using threading as requested
            threading.Thread(target=lambda: (self.tts_engine.say(text), self.tts_engine.runAndWait())).start()
    
    def listen_for_speech(self, max_silence_duration=3.0):
        """Listen for human's speech and convert to text using Whisper library"""
        rospy.loginfo("Starting speech recognition using Whisper...")
        
        try:
            # Use the Whisper speech recognizer library
            text = self.speech_recognizer.listen_and_transcribe(
                max_silence_duration=max_silence_duration,
                max_recording_time=30
            )
            
            if text and len(text.strip()) > 0:
                rospy.loginfo(f"Whisper recognized: {text}")
                return text.lower()
            else:
                rospy.logwarn("No speech recognized by Whisper")
                return None
                
        except Exception as e:
            rospy.logerr(f"Whisper speech recognition error: {e}")
            return None
    
    def cancel(self):
        """Shutdown callback (Yahboom pattern)"""
        rospy.loginfo("Shutting down Yahboom Competition Robot...")
        
    def publish_status(self, status):
        """Publish current robot status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def execute_human_interaction_task(self):
        """
        Execute the first part of the competition task:
        1. Leave starting area
        2. Navigate to working cabin
        3. Ask human for their needs
        4. Listen and repeat back the request
        """
        rospy.loginfo("Starting human interaction task...")
        self.current_state = "LEAVING_START"
        
        # Step 1: Leave the starting area (100 points)
        self.publish_status("Leaving starting area")
        time.sleep(2)  # Brief pause to ensure clean departure using time library as requested
        
        # Step 2: Navigate to working cabin entrance first
        self.current_state = "NAVIGATING_TO_CABIN"
        if not self.navigate_to_location('working_cabin_entrance'):
            rospy.logerr("Failed to reach working cabin entrance")
            return False
        
        # Step 3: Enter working cabin (100 points)
        self.current_state = "ENTERING_CABIN"
        rospy.loginfo("Entering working cabin...")
        if not self.navigate_to_location('working_cabin_center'):
            rospy.logerr("Failed to enter working cabin")
            return False
        
        rospy.loginfo("Successfully entered working cabin")
        time.sleep(1)  # Brief pause after entering using time library as requested
        
        # Step 4: Greet and ask human for needs (50 points)
        self.current_state = "ASKING_NEEDS"
        greeting = "Hello! I am the Yahboom X3 Plus robot here to assist you. What do you need help with today?"
        self.speak_text(greeting, wait_for_completion=True)
        
        # Step 5: Listen for human's response
        self.current_state = "LISTENING"
        max_attempts = 3
        
        for attempt in range(max_attempts):
            rospy.loginfo(f"Listening attempt {attempt + 1}/{max_attempts}")
            
            # Listen for human's request using Whisper speech recognition library
            request = self.listen_for_speech(max_silence_duration=3.0)
            
            if request:
                self.human_request = request
                rospy.loginfo(f"Received request: {request}")
                break
            else:
                if attempt < max_attempts - 1:
                    self.speak_text("I'm sorry, I didn't catch that. Could you please repeat your request?")
                    time.sleep(1)  # Using time library as requested
                else:
                    rospy.logwarn("Failed to understand human's request after multiple attempts")
                    self.speak_text("I'm having trouble understanding. Let me try to help you anyway.")
                    self.human_request = "general assistance"
        
        # Step 6: Repeat back the human's needs (50 points)
        if self.human_request:
            self.current_state = "CONFIRMING_REQUEST"
            confirmation = f"I understand you need: {self.human_request}. I will get that for you right away."
            self.speak_text(confirmation, wait_for_completion=True)
            
            rospy.loginfo("Human interaction completed successfully")
            self.publish_status("Human interaction completed")
            return True
        else:
            rospy.logwarn("Human interaction completed with issues")
            return False
    
    def run_competition_mission(self):
        """Run the complete competition mission"""
        rospy.loginfo("=== Starting Yahboom X3 Plus Competition Mission ===")
        
        try:
            # Execute human interaction (first part of competition task)
            success = self.execute_human_interaction_task()
            
            if success:
                rospy.loginfo("Human interaction phase completed successfully!")
                
                # Publish task completion status
                self.task_complete_pub.publish(Bool(True))
                
                # Here you would continue with navigation to tool cabin
                # and item retrieval based on human's request
                rospy.loginfo("Ready to proceed to tool cabin for item retrieval...")
                
                return True
            else:
                rospy.logerr("Human interaction phase failed")
                self.task_complete_pub.publish(Bool(False))
                return False
                
        except rospy.ROSInterruptException:
            rospy.loginfo("Mission interrupted")
            return False
        except Exception as e:
            rospy.logerr(f"Mission failed with error: {e}")
            return False

def main():
    try:
        # Create Yahboom robot instance
        robot = YahboomCompetitionRobot()
        
        # Wait for all systems to be ready
        rospy.loginfo("Waiting for Yahboom robot systems to initialize...")
        time.sleep(3)  # Using time library as requested
        
        # Start the competition mission
        robot.run_competition_mission()
        
        # Keep the node running
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Yahboom Competition Robot node shutting down")
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")

if __name__ == '__main__':
    main()