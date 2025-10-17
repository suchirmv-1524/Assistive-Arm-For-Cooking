<h1 align="center">🤖 ServoSaute : Assistive Arm for Cooking 🍳</h1>

ServoSaute is an assistive robotic arm capable of performing **stirring**, **flipping**, and **pouring** actions based on **hand gestures** recognized via webcam. Using **MediaPipe** for gesture detection and **PyBullet** for real-time physics simulation, the robotic arm mimics actions useful in a kitchen setting.

## 📁 Directory Structure
ServoSaute/ <br/>
├── gesture_arm_control.py<br/>
├── instructions.txt <br/>
├── model.urdf <br/>
├── README.md <br/>
└── requirements.txt <br/>


## 🔍 Features

- **Hand Gesture Recognition** using [MediaPipe Hands](https://google.github.io/mediapipe/solutions/hands)
- **Simulated Robot Arm Control** using [PyBullet](https://pybullet.org/)
- **Three Cooking Motions**:
  - 🌀 **Stir**
  - 🔄 **Flip**
  - 💧 **Pour**
- Real-time gesture tracking via webcam
- Visual feedback in the PyBullet simulation window

## � Installation

### ✅ Prerequisites

- Python 3.9+ (We have used Python 3.10.17)
- pip

### 📦 Setup

1. **Clone the repository**:
   ```bash
   git clone https://github.com/suchirmv-1524/ServoSaute.git
   cd Assistive-Arm-For-Cooking
   ```

2. Create a virtual environment (optional but recommended):
   ```bash
   python -m venv cooking_arm
   source cooking_arm/bin/activate 
   ```
3. Install dependencies:
   ```bash
   pip install -r requirements.txt 
   ```

## 📷 Gesture Instructions <br/>
To control the robotic arm, use the following finger gestures in front of your webcam. The system tracks your index and middle fingers to detect actions.

🌀 Stir
- Hold your index and middle fingers close together

- Keep them at the same height

- The green line on screen should be short

- Hold for 1 second until progress fills

🔄 Flip
- Spread your index and middle fingers horizontally

- Keep them at the same height

- The green line should be long and horizontal

- Hold for 1 second until progress fills

💧 Pour
- Spread your index and middle fingers vertically

- One finger should be higher

- The green line should be long and vertical

- Hold for 1 second until progress fills

## 🧠 How It Works
Gesture Recognition
- Uses MediaPipe to track hand landmarks in real-time

- Calculates distance and orientation between index and middle fingers

- Detects gesture based on direction and length of the vector between them

Motion Mapping
Each gesture is mapped to a specific motion:

- stir_motion() – circular trajectory

- flip_motion() – lift + tilt

- pour_motion() – forward tilt with hold

Simulation
- The robot arm is rendered in a PyBullet GUI

- A compound object (pan + handle) is attached to the robot's end effector

- All motions are animated using inverse kinematics and smooth transitions

## � Running the Project
   ```bash
   python gesture_arm_control.py
   ```
- A PyBullet window should open

- Allow camera access

- Show the appropriate gesture in front of your webcam

- The arm should respond after detecting and validating your gesture

## 📄 Model Notes
- model.urdf is the URDF description of the robot (KUKA iiwa used)

- Additional models like plane.urdf are loaded from PyBullet's pybullet_data

## 📚 References
- MediaPipe Hands

- PyBullet Physics Engine

- OpenCV

## 💡 Future Improvements
- Add support for more gestures (e.g., scoop, mix)

- Integrate speech input for voice commands

- Deploy on a physical robotic arm using ROS
<br/>
<h3 align="center">Feel free to fork, clone, and build upon it! Happy cooking 🤖🍳</h3>


