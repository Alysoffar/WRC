# WRC 2025 - Yahboom X3 Plus Competition Robot

This folder contains the competition code for the World Robot Challenge 2025 using the Yahboom X3 Plus robot with ROS1 Noetic.

## 🚀 Project Overview

The WRC competition robot performs autonomous tasks including:
1. **Navigation**: Move from starting area to working cabin
2. **Human Interaction**: Ask humans what help they need
3. **Speech Recognition**: Process voice responses using Whisper AI
4. **Task Execution**: Complete requested tasks autonomously

## 📁 File Structure

```
WRC/
├── Init.py                    # Main competition initialization and human interaction
├── whisper_speech_lib.py      # Whisper-based speech recognition library
├── Speech-to-text.py          # Original speech-to-text implementation
├── second_task.py             # Second phase competition tasks
├── requirements.txt           # Python dependencies
└── README.md                  # This file
```

## 🔧 Files Description

### `Init.py`
- **Purpose**: Main competition robot controller
- **Features**: 
  - Yahboom X3 Plus robot initialization
  - Navigation using move_base
  - Human interaction and dialogue
  - Whisper-based speech recognition integration
  - Text-to-speech using pyttsx3
- **Class**: `YahboomCompetitionRobot`

### `whisper_speech_lib.py`
- **Purpose**: Modular speech recognition library
- **Features**:
  - OpenAI Whisper integration for offline speech-to-text
  - Real-time audio recording with silence detection
  - Configurable model sizes (tiny, base, small, medium, large)
  - Audio preprocessing and cleanup
- **Class**: `WhisperSpeechRecognizer`

### `Speech-to-text.py`
- **Purpose**: Standalone speech recognition implementation
- **Features**: Direct Whisper usage for testing and development

### `second_task.py`
- **Purpose**: Autonomous tool retrieval and collaboration phase
- **Features**:
  - Navigation to tool cabin and collection areas
  - Arm and gripper control for item pickup and placement
  - Computer vision (OpenCV, cv_bridge) for object detection
  - Collaboration with other robots (greeting, standby point selection)
  - Advanced speech recognition and TTS
  - Flexible task parsing and execution
- **Class**: `YahboomAutonomousCollaborationRobot`

## 🛠️ Setup and Installation

### Prerequisites
- Ubuntu 20.04 LTS
- ROS1 Noetic
- Python 3.8+
- Yahboom X3 Plus robot hardware

### Dependencies Installation

1. **Install Python dependencies:**
```bash
cd /home/yahboom/yahboomcar_ws/src/yahboomcar_nav/scripts/WRC
pip install -r requirements.txt
```

2. **Required Python packages:**
- `numpy>=1.21.0`: Numerical computations
- `scipy>=1.7.0`: Audio file I/O
- `sounddevice>=0.4.4`: Real-time audio recording
- `openai-whisper>=20230314`: AI speech recognition
- `pyttsx3>=2.90`: Text-to-speech synthesis

### Additional Dependencies for second_task.py
- `opencv-python`: Computer vision for object detection
- `cv_bridge`: ROS-Python bridge for image messages
- `sensor_msgs`: ROS image message types
- `numpy`, `scipy`, `sounddevice`, `openai-whisper`, `pyttsx3` (already listed)

Install OpenCV and cv_bridge:
```bash
pip install opencv-python
sudo apt install ros-noetic-cv-bridge ros-noetic-sensor-msgs
```

### ROS Dependencies
Ensure these ROS packages are installed:
```bash
sudo apt install ros-noetic-move-base
sudo apt install ros-noetic-navigation
sudo apt install ros-noetic-yahboomcar-msgs  # If available
```

## 🚀 Usage

### Running the Competition Robot

1. **Start ROS core:**
```bash
roscore
```

2. **Launch navigation stack:**
```bash
roslaunch yahboomcar_nav yahboomcar_navigation.launch  # Adjust as needed
```

3. **Run the competition robot:**
```bash
cd /home/yahboom/yahboomcar_ws/src/yahboomcar_nav/scripts/WRC
python3 Init.py
```

### Testing Speech Recognition

**Test Whisper library independently:**
```bash
cd /home/yahboom/yahboomcar_ws/src/yahboomcar_nav/scripts/WRC
python3 whisper_speech_lib.py
```

**Test original speech implementation:**
```bash
python3 Speech-to-text.py
```

## 🎯 Competition Tasks

### Phase 1: Human Interaction (Init.py)
1. **Navigation**: Robot moves from starting area to working cabin
2. **Approach**: Enter the working cabin safely
3. **Interaction**: Greet human and ask for assistance needs
4. **Listening**: Record and process speech using Whisper
5. **Confirmation**: Repeat back the understood request
6. **Scoring**: Points awarded for successful completion

### Phase 2: Task Execution & Collaboration (second_task.py)
- **Autonomous Tool Retrieval**: Navigate to tool cabin, detect and pick up items, deliver to target area
- **Collaboration**: Greet other robot, receive standby instructions, navigate to standby point
- **Computer Vision**: Detect items by color/shape using OpenCV
- **Arm/Gripper Control**: Pick/place items using Yahboom arm
- **Speech Recognition**: Use Whisper for instructions

## 🔧 Configuration

### Speech Recognition Settings
```python
# In whisper_speech_lib.py
model_size = "base"          # Whisper model: tiny, base, small, medium, large
sample_rate = 44100          # Audio sample rate (Hz)
channels = 1                 # Mono audio
silence_threshold = 0.01     # Silence detection sensitivity
silence_duration = 2.0       # Silence duration before stopping (seconds)
```

### Navigation Locations
```python
# In Init.py - Adjust coordinates based on venue
locations = {
    'starting_area': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    'working_cabin_entrance': {'x': 2.5, 'y': 1.0, 'yaw': 1.57},
    'working_cabin_center': {'x': 3.0, 'y': 1.6, 'yaw': 0.0},
    'tool_cabin': {'x': 1.0, 'y': 4.0, 'yaw': 0.0}
}
```

### Additional Configuration for second_task.py
```python
# In second_task.py
self.arm_positions = {
    'home': [90, 135, 20, 20, 90, 30],
    'pick_up': [90, 100, 50, 50, 90, 30],
    'carry': [90, 135, 45, 45, 90, 30],
    'place': [90, 120, 40, 40, 90, 30]
}

# Item database for object recognition
self.item_database = {
    'red_flask': {'color': [0, 0, 255], 'shape': 'flask', 'size': 'medium'},
    'blue_flask': {'color': [255, 0, 0], 'shape': 'flask', 'size': 'medium'},
    'green_flask': {'color': [0, 255, 0], 'shape': 'flask', 'size': 'medium'},
    'pipeline_parts_box': {'color': [128, 128, 128], 'shape': 'box', 'size': 'large'},
    'aviation_oil_bottle': {'color': [255, 255, 0], 'shape': 'bottle', 'size': 'medium'},
    'gloves': {'color': [255, 255, 255], 'shape': 'irregular', 'size': 'small'}
}
```

## 📊 ROS Topics

### Published Topics
- `/cmd_vel`: Robot velocity commands
- `/move_base_simple/goal`: Navigation goals
- `/Buzzer`: Audio feedback
- `/robot_status`: Current robot status
- `/task_complete`: Task completion signals

### Subscribed Topics
- `/move_base/result`: Navigation results
- `/scan`: Laser sensor data (if used)
- `/camera/image_raw`: Camera feed (if used)

### Additional Topics for second_task.py
- `/usb_cam/image_raw`: Raw camera images for vision
- `/TargetAngle`: Arm joint control
- `/Gripper`: Gripper control
- `/collaboration_ready`: Collaboration phase status

## 🐛 Troubleshooting

### Common Issues

1. **Whisper model loading fails:**
   - Check internet connection for first-time model download
   - Ensure sufficient disk space (~1GB for base model)

2. **Audio recording issues:**
   - Verify microphone permissions: `sudo usermod -a -G audio $USER`
   - Check audio devices: `python3 -c "import sounddevice as sd; print(sd.query_devices())"`

3. **Navigation failures:**
   - Verify move_base is running: `rostopic list | grep move_base`
   - Check map and localization
   - Adjust navigation coordinates

4. **ROS connection errors:**
   - Ensure roscore is running
   - Check network configuration
   - Verify ROS environment variables

### Debug Commands
```bash
# Check ROS topics
rostopic list

# Monitor navigation status
rostopic echo /move_base/result

# Test TTS
python3 -c "import pyttsx3; engine = pyttsx3.init(); engine.say('Test'); engine.runAndWait()"

# Test Whisper
python3 -c "import whisper; print('Whisper available')"
```

## 🏆 Competition Features

- **Offline Operation**: No internet required during competition
- **Robust Speech Recognition**: Whisper AI for accurate transcription
- **Modular Design**: Separate libraries for easy maintenance
- **Safety Features**: Emergency stop and collision avoidance
- **Real-time Feedback**: Visual and audio status indicators
- **Configurable Parameters**: Easy venue-specific adjustments
- **Computer Vision**: Detect and manipulate objects using OpenCV and ROS image topics
- **Multi-Phase Tasks**: Seamless transition from autonomous to collaboration phase
- **Flexible Item Parsing**: Understand and act on a variety of human requests

## 👥 Development Team

**WRC EGY Team**
- Author: Competition Development Team
- Date: 2025
- Robot: Yahboom X3 Plus with ROS1 Noetic

## 📝 License

This project is developed for the World Robot Challenge 2025 competition.

## 📞 Support

For technical support or questions:
1. Check this README first
2. Review ROS logs: `rosrun rqt_console rqt_console`
3. Test individual components separately
4. Check hardware connections and power

---

**Good luck in the competition! 🤖🏆**
