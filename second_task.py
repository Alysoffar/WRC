#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Autonomous and Collaboration Tasks for Yahboom X3 Plus Robot
This script implements:
1. Navigate to tool cabin and retrieve requested items (Autonomous Task)
2. Friendly greetings with other robot (Collaboration Task)

Author: WRC EGY Team
Date: 2025
Robot: Yahboom X3 Plus with ROS1 Noetic
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import PoseStamped, Twist, Pose, Point, Quaternion
from std_msgs.msg import String, Bool, Int32,Float64
from sensor_msgs.msg import Image, CompressedImage
from yahboomcar_msgs.msg import *
from actionlib_msgs.msg import GoalID
from whisper_speech_lib import WhisperSpeechRecognizer
import pyttsx3
import threading
import time
import cv2
from cv_bridge import CvBridge
import numpy as np

class YahboomAutonomousCollaborationRobot:
    def __init__(self):
        rospy.init_node('yahboom_autonomous_collaboration', anonymous=False)
        rospy.on_shutdown(self.cancel)
        
        # Initialize robot parameters
        self.InitialParam()
        
        # Initialize move_base action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server(rospy.Duration(30))
        rospy.loginfo("Move_base action server connected!")
        
        # Publishers
        self.pub_CmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_cancel = rospy.Publisher("move_base/cancel", GoalID, queue_size=10)
        self.pub_buzzer = rospy.Publisher('/Buzzer', Bool, queue_size=1)
        self.pub_arm_control = rospy.Publisher('/TargetAngle', ArmJoint, queue_size=1)
        self.pub_gripper = rospy.Publisher('/Gripper', Float64, queue_size=1)
        self.status_pub = rospy.Publisher('/robot_status', String, queue_size=1)
        self.task_complete_pub = rospy.Publisher('/task_complete', Bool, queue_size=1)
        self.collaboration_complete_pub = rospy.Publisher('/collaboration_ready', Bool, queue_size=1)
        
        # Subscribers
        self.sub_goal_result = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.goal_result_callback)
        self.sub_image = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        
        # Computer vision setup
        self.bridge = CvBridge()
        self.current_image = None
        self.detected_objects = []
        
        # Speech recognition and TTS
        rospy.loginfo("Initializing Whisper Speech Recognition...")
        self.speech_recognizer = WhisperSpeechRecognizer(model_size="base", sample_rate=44100, channels=1)
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)
        self.tts_engine.setProperty('volume', 0.9)
        
        # Item database for recognition
        self.initialize_item_database()
        
        rospy.loginfo("Yahboom Autonomous and Collaboration Robot initialized!")
    
    def InitialParam(self):
        """Initialize robot parameters"""
        # Competition state
        self.current_task = "AUTONOMOUS"  # AUTONOMOUS, COLLABORATION
        self.human_request = ""
        self.target_items = []
        self.collected_items = []
        self.collaboration_target = ""
        
        # Navigation results
        self.goal_result = 0
        self.navigation_success = False
        
        # Predefined locations (adjust based on actual competition venue)
        self.locations = {
            'starting_area': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'working_cabin_A': {'x': 3.0, 'y': 1.6, 'yaw': 0.0},
            'working_cabin_B': {'x': 3.0, 'y': 3.4, 'yaw': 0.0},
            'tool_cabin': {'x': 1.0, 'y': 4.0, 'yaw': 0.0},
            'collection_area_A': {'x': 4.5, 'y': 1.0, 'yaw': 0.0},
            'collection_area_B': {'x': 4.5, 'y': 4.0, 'yaw': 0.0},
            'standby_point_A': {'x': 2.0, 'y': 2.0, 'yaw': 0.0},
            'standby_point_B': {'x': 2.0, 'y': 3.0, 'yaw': 0.0},
            'standby_point_C': {'x': 3.5, 'y': 2.5, 'yaw': 0.0},
        }
        
        # Arm control positions
        self.arm_positions = {
            'home': [90, 135, 20, 20, 90, 30],
            'pick_up': [90, 100, 50, 50, 90, 30],
            'carry': [90, 135, 45, 45, 90, 30],
            'place': [90, 120, 40, 40, 90, 30]
        }
    
    def initialize_item_database(self):
        """Initialize database of competition items for recognition"""
        self.item_database = {
            # Autonomous task items (experimental vessels)
            'red_flask': {'color': [0, 0, 255], 'shape': 'flask', 'size': 'medium'},
            'blue_flask': {'color': [255, 0, 0], 'shape': 'flask', 'size': 'medium'},
            'green_flask': {'color': [0, 255, 0], 'shape': 'flask', 'size': 'medium'},
            
            # Manual task items (for recognition purposes)
            'pipeline_parts_box': {'color': [128, 128, 128], 'shape': 'box', 'size': 'large'},
            'aviation_oil_bottle': {'color': [255, 255, 0], 'shape': 'bottle', 'size': 'medium'},
            'gloves': {'color': [255, 255, 255], 'shape': 'irregular', 'size': 'small'}
        }
    
    def goal_result_callback(self, msg):
        """Callback for move_base navigation results"""
        if not isinstance(msg, MoveBaseActionResult):
            return
        self.goal_result = msg.status.status
        if self.goal_result == 3:  # SUCCEEDED
            self.navigation_success = True
    
    def image_callback(self, msg):
        """Process incoming camera images for object detection"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.detect_objects()
        except Exception as e:
            rospy.logerr(f"Image processing error: {e}")
    
    def create_navigation_goal(self, location_name):
        """Create navigation goal for move_base"""
        if location_name not in self.locations:
            rospy.logerr(f"Unknown location: {location_name}")
            return None
            
        loc = self.locations[location_name]
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = rospy.Time.now()
        
        pose_stamped.pose.position.x = loc['x']
        pose_stamped.pose.position.y = loc['y']
        pose_stamped.pose.position.z = 0.0
        
        yaw = loc['yaw']
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = np.sin(yaw / 2.0)
        pose_stamped.pose.orientation.w = np.cos(yaw / 2.0)
        
        return pose_stamped
    
    def navigate_to_location(self, location_name, timeout=30):
        """Navigate to specified location"""
        rospy.loginfo(f"Navigating to: {location_name}")
        
        goal_pose = self.create_navigation_goal(location_name)
        if goal_pose is None:
            return False
            
        self.pub_goal.publish(goal_pose)
        self.navigation_success = False
        
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if self.navigation_success:
                rospy.loginfo(f"Successfully reached {location_name}")
                return True
            elif self.goal_result in [4, 5, 6, 7, 8]:  # Failure states
                rospy.logwarn(f"Navigation failed to {location_name}")
                return False
            time.sleep(0.1)
        
        # Timeout - cancel goal
        cancel_msg = GoalID()
        self.pub_cancel.publish(cancel_msg)
        return False
    
    def speak_text(self, text, wait_for_completion=True):
        """Make robot speak using TTS"""
        rospy.loginfo(f"Robot speaking: {text}")
        
        # Attention beep
        buzzer_msg = Bool()
        buzzer_msg.data = True
        self.pub_buzzer.publish(buzzer_msg)
        time.sleep(0.3)
        buzzer_msg.data = False
        self.pub_buzzer.publish(buzzer_msg)
        
        if wait_for_completion:
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()
        else:
            threading.Thread(target=lambda: (self.tts_engine.say(text), self.tts_engine.runAndWait())).start()
    
    def listen_for_speech(self, max_silence_duration=3.0):
        """Listen for speech using Whisper"""
        try:
            text = self.speech_recognizer.listen_and_transcribe(
                max_silence_duration=max_silence_duration,
                max_recording_time=20
            )
            
            if text and len(text.strip()) > 0:
                rospy.loginfo(f"Speech recognized: {text}")
                return text.lower()
            return None
                
        except Exception as e:
            rospy.logerr(f"Speech recognition error: {e}")
            return None
    
    def control_arm(self, position_name, speed=20):
        """Control robot arm to predefined positions"""
        if position_name not in self.arm_positions:
            rospy.logerr(f"Unknown arm position: {position_name}")
            return False
        
        angles = self.arm_positions[position_name]
        arm_msg = ArmJoint()
        arm_msg.id = [1, 2, 3, 4, 5, 6]
        arm_msg.angle = angles
        arm_msg.run_time = speed
        
        self.pub_arm_control.publish(arm_msg)
        time.sleep(2)  # Wait for movement completion
        return True
    
    def control_gripper(self, position):
        """Control gripper (0=closed, 90=open)"""
        gripper_msg = Float64()
        gripper_msg.data = position
        self.pub_gripper.publish(gripper_msg)
        time.sleep(1)
        return True
    
    def detect_objects(self):
        """Detect and identify objects in current camera image"""
        if self.current_image is None:
            return
        
        self.detected_objects = []
        
        # Simple color-based object detection
        hsv = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2HSV)
        
        for item_name, item_props in self.item_database.items():
            # Create color mask based on item properties
            color_bgr = np.array(item_props['color'])
            color_hsv = cv2.cvtColor(np.uint8([[color_bgr]]), cv2.COLOR_BGR2HSV)[0][0]
            
            # Define HSV range for detection
            lower_hsv = np.array([max(0, color_hsv[0]-10), 50, 50])
            upper_hsv = np.array([min(179, color_hsv[0]+10), 255, 255])
            
            mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum area threshold
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    self.detected_objects.append({
                        'name': item_name,
                        'center': (center_x, center_y),
                        'bbox': (x, y, w, h),
                        'area': area
                    })
    
    def find_target_item(self, item_name):
        """Find specific target item in detected objects"""
        for obj in self.detected_objects:
            if item_name.lower() in obj['name'].lower():
                return obj
        return None
    
    def pick_up_item(self, item_name):
        """Pick up specified item using arm and gripper"""
        rospy.loginfo(f"Attempting to pick up: {item_name}")
        
        # Find item in camera view
        target_obj = self.find_target_item(item_name)
        if not target_obj:
            rospy.logwarn(f"Item {item_name} not found in view")
            return False
        
        # Move arm to pickup position
        self.control_arm('pick_up')
        
        # Open gripper
        self.control_gripper(90)
        
        # Move robot to align with item (simple approach)
        center_x = target_obj['center'][0]
        image_center = self.current_image.shape[1] // 2
        
        if abs(center_x - image_center) > 50:  # Need to align
            twist = Twist()
            if center_x < image_center:
                twist.angular.z = 0.2
            else:
                twist.angular.z = -0.2
            
            self.pub_CmdVel.publish(twist)
            time.sleep(1)
            
            # Stop movement
            twist.angular.z = 0.0
            self.pub_CmdVel.publish(twist)
        
        # Close gripper to grab item
        self.control_gripper(0)
        
        # Move to carry position
        self.control_arm('carry')
        
        rospy.loginfo(f"Successfully picked up {item_name}")
        return True
    
    def place_item_in_area(self, area_name):
        """Place carried item in specified area"""
        rospy.loginfo(f"Placing item in {area_name}")
        
        # Navigate to target area
        if not self.navigate_to_location(area_name):
            return False
        
        # Move arm to place position
        self.control_arm('place')
        
        # Open gripper to release item
        self.control_gripper(90)
        
        # Return arm to home position
        self.control_arm('home')
        
        rospy.loginfo(f"Item placed in {area_name}")
        return True
    
    def execute_autonomous_task_continuation(self, human_request):
        """Continue autonomous task - navigate to tool cabin and retrieve items"""
        rospy.loginfo("=== Continuing Autonomous Task: Tool Retrieval ===")
        
        # Parse human request to determine target items
        self.parse_human_request(human_request)
        
        # Navigate to tool cabin (50 points)
        rospy.loginfo("Navigating to tool cabin...")
        if not self.navigate_to_location('tool_cabin'):
            rospy.logerr("Failed to reach tool cabin")
            return False
        
        rospy.loginfo("Successfully entered tool cabin")
        
        # Search for and pick up designated items (100 points)
        for item_name in self.target_items:
            rospy.loginfo(f"Searching for: {item_name}")
            
            # Give time for object detection
            time.sleep(2)
            
            if self.pick_up_item(item_name):
                # Navigate back to working cabin or collection area (100 points)
                target_area = 'working_cabin_A'  # Default, adjust based on competition rules
                if self.place_item_in_area(target_area):
                    rospy.loginfo(f"Successfully delivered {item_name}")
                    self.collected_items.append(item_name)
                else:
                    rospy.logwarn(f"Failed to deliver {item_name}")
                    
                # Return to tool cabin for next item if needed
                if len(self.target_items) > 1 and item_name != self.target_items[-1]:
                    self.navigate_to_location('tool_cabin')
            else:
                rospy.logwarn(f"Failed to pick up {item_name}")
        
        # Return to starting area for next task
        rospy.loginfo("Returning to starting area after autonomous task...")
        self.navigate_to_location('starting_area')
        
        return len(self.collected_items) > 0
    
    def execute_collaboration_task(self):
        """Execute collaboration task - greet other robot and navigate to standby point"""
        rospy.loginfo("=== Starting Collaboration Task ===")
        
        # Navigate to other team's working cabin
        target_cabin = 'working_cabin_B'  # Adjust based on team assignment
        
        if not self.navigate_to_location(target_cabin):
            rospy.logerr("Failed to reach collaboration target")
            return False
        
        # Greet the other team's astronaut
        greeting = "Hello! I am here to collaborate. Please give me instructions on where to stand by."
        self.speak_text(greeting, wait_for_completion=True)
        
        # Listen for collaboration instructions
        rospy.loginfo("Listening for collaboration instructions...")
        instructions = self.listen_for_speech(max_silence_duration=5.0)
        
        if instructions:
            # Parse instructions to determine target standby point
            target_point = self.parse_collaboration_instructions(instructions)
            
            if target_point:
                # Confirm understanding
                confirmation = f"I understand. I will go to {target_point}."
                self.speak_text(confirmation)
                
                # Navigate to designated standby point (200 points for successful completion)
                if self.navigate_to_location(target_point):
                    self.speak_text("I have reached the designated standby point.")
                    rospy.loginfo("Collaboration task completed successfully")
                    
                    # Signal that collaboration is complete and robot is ready for remote control
                    self.collaboration_complete_pub.publish(Bool(True))
                    return True
        
        # Fallback - move to default standby point
        rospy.logwarn("Using default standby point")
        success = self.navigate_to_location('standby_point_A')
        if success:
            self.collaboration_complete_pub.publish(Bool(True))
        return success
    
    def parse_human_request(self, request):
        """Parse human's request to determine target items"""
        request_lower = request.lower()
        self.target_items = []
        
        # Map request to target items
        if 'red' in request_lower and 'flask' in request_lower:
            self.target_items.append('red_flask')
        if 'blue' in request_lower and 'flask' in request_lower:
            self.target_items.append('blue_flask')
        if 'green' in request_lower and 'flask' in request_lower:
            self.target_items.append('green_flask')
        if 'pipeline' in request_lower or 'parts' in request_lower:
            self.target_items.append('pipeline_parts_box')
        if 'oil' in request_lower or 'lubricant' in request_lower:
            self.target_items.append('aviation_oil_bottle')
        if 'gloves' in request_lower:
            self.target_items.append('gloves')
        
        # Default fallback
        if not self.target_items:
            self.target_items = ['red_flask']  # Default item
        
        rospy.loginfo(f"Target items identified: {self.target_items}")
    
    def parse_collaboration_instructions(self, instructions):
        """Parse collaboration instructions to determine standby point"""
        instructions_lower = instructions.lower()
        
        if 'point a' in instructions_lower or 'area a' in instructions_lower:
            return 'standby_point_A'
        elif 'point b' in instructions_lower or 'area b' in instructions_lower:
            return 'standby_point_B'
        elif 'point c' in instructions_lower or 'area c' in instructions_lower:
            return 'standby_point_C'
        
        # Try to extract location from instructions
        if 'center' in instructions_lower:
            return 'standby_point_C'
        elif 'left' in instructions_lower:
            return 'standby_point_A'
        elif 'right' in instructions_lower:
            return 'standby_point_B'
        
        return 'standby_point_A'  # Default
    
    def publish_status(self, status):
        """Publish current robot status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def cancel(self):
        """Shutdown callback"""
        rospy.loginfo("Shutting down Yahboom Autonomous Collaboration Robot...")
        
        # Stop all movement
        twist = Twist()
        self.pub_CmdVel.publish(twist)
        
        # Return arm to home position
        self.control_arm('home')
    
    def run_autonomous_and_collaboration_mission(self, human_request=""):
        """Run autonomous and collaboration tasks in sequence"""
        rospy.loginfo("=== Starting Autonomous and Collaboration Mission ===")
        
        try:
            # Task 1: Complete autonomous task (continue from human interaction)
            if human_request:
                success_auto = self.execute_autonomous_task_continuation(human_request)
                if success_auto:
                    rospy.loginfo("Autonomous task completed successfully!")
                    self.publish_status("Autonomous task completed")
                else:
                    rospy.logwarn("Autonomous task had issues")
                    self.publish_status("Autonomous task completed with issues")
            
            # Brief pause between tasks (competition rules)
            time.sleep(3)
            
            # Task 2: Collaboration task
            success_collab = self.execute_collaboration_task()
            if success_collab:
                rospy.loginfo("Collaboration task completed successfully!")
                self.publish_status("Collaboration task completed - ready for remote control")
            else:
                rospy.logwarn("Collaboration task had issues")
                self.publish_status("Collaboration task completed with issues")
            
            # Mission summary for autonomous and collaboration phases
            if success_auto and success_collab:
                self.speak_text("Autonomous and collaboration tasks completed successfully. Ready for remote control phase.")
                rospy.loginfo("Autonomous and Collaboration phases completed successfully!")
            else:
                self.speak_text("Autonomous and collaboration tasks completed with some issues.")
                rospy.loginfo("Autonomous and Collaboration phases completed with issues")
            
            self.task_complete_pub.publish(Bool(success_auto and success_collab))
            return success_auto and success_collab
            
        except rospy.ROSInterruptException:
            rospy.loginfo("Mission interrupted")
            return False
        except Exception as e:
            rospy.logerr(f"Mission failed with error: {e}")
            return False

def main():
    try:
        # Create autonomous and collaboration robot instance
        robot = YahboomAutonomousCollaborationRobot()
        
        # Wait for systems to initialize
        rospy.loginfo("Initializing autonomous and collaboration robot systems...")
        time.sleep(3)
        
        # Example usage - in real competition, human_request would come from first script
        example_request = "I need the red flask for my experiment"
        
        # Run autonomous and collaboration mission
        robot.run_autonomous_and_collaboration_mission(human_request=example_request)
        
        # Keep node running
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Autonomous and collaboration robot shutting down")
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")

if __name__ == '__main__':
    main()