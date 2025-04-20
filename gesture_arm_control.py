import cv2
import mediapipe as mp
import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.3,
    min_tracking_confidence=0.3,
    model_complexity=0
)
mp_drawing = mp.solutions.drawing_utils

# Initialize PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# GUI Cleanup
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

# Set default camera view
p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=50,
    cameraPitch=-35,
    cameraTargetPosition=[0.5, 0, 1]
)

# Load plane and robot
plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

# Pan + handle parameters
pan_radius = 0.15
pan_height = 0.05
handle_radius = 0.02
handle_length = 0.30

# Visual & collision shapes for pan
pan_visual = p.createVisualShape(
    shapeType=p.GEOM_CYLINDER,
    radius=pan_radius,
    length=pan_height,
    rgbaColor=[0.1, 0.1, 0.1, 1]
)
pan_collision = p.createCollisionShape(
    shapeType=p.GEOM_CYLINDER,
    radius=pan_radius,
    height=pan_height
)

# Visual & collision shapes for handle
handle_visual = p.createVisualShape(
    shapeType=p.GEOM_CYLINDER,
    radius=handle_radius,
    length=handle_length,
    rgbaColor=[0.2, 0.2, 0.2, 1]
)
handle_collision = p.createCollisionShape(
    shapeType=p.GEOM_CYLINDER,
    radius=handle_radius,
    height=handle_length
)

# Create compound multibody: pan (base) + handle (link)
pan_with_handle = p.createMultiBody(
    baseMass=0.1,
    baseCollisionShapeIndex=pan_collision,
    baseVisualShapeIndex=pan_visual,
    basePosition=[0, 0, 1],
    linkMasses=[0.01],
    linkCollisionShapeIndices=[handle_collision],
    linkVisualShapeIndices=[handle_visual],
    linkPositions=[[pan_radius, 0, 0]],
    linkOrientations=[p.getQuaternionFromEuler([0, math.pi / 2, 0])],
    linkInertialFramePositions=[[0, 0, 0]],
    linkInertialFrameOrientations=[[0, 0, 0, 1]],
    linkParentIndices=[0],
    linkJointTypes=[p.JOINT_FIXED],
    linkJointAxis=[[0, 0, 0]]
)

# Attach pan to robot end-effector
attach_offset = pan_radius + handle_length / 2
p.createConstraint(
    parentBodyUniqueId=robot,
    parentLinkIndex=6,
    childBodyUniqueId=pan_with_handle,
    childLinkIndex=-1,
    jointType=p.JOINT_FIXED,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[attach_offset, 0, 0]
)

# Move robot to starting joint positions
init_joints = [0, 0, 0, 0, 0, 0, 0]
for idx, pos in enumerate(init_joints):
    p.resetJointState(robot, idx, pos)

# Get initial end-effector pose
link_state = p.getLinkState(robot, 6, computeForwardKinematics=True)
home_pos, home_orn = link_state[0], link_state[1]
stir_center_pos, stir_center_orn = home_pos, home_orn

# --- Motion Functions ---
def stir_motion(radius=0.15, revolutions=2, duration=4.0):
    steps = int(duration * 240)
    for i in range(steps):
        theta = 2 * math.pi * revolutions * (i / steps)
        target = [
            stir_center_pos[0] + radius * math.cos(theta),
            stir_center_pos[1] + radius * math.sin(theta),
            stir_center_pos[2]
        ]
        jp = p.calculateInverseKinematics(
            robot, 6, target,
            targetOrientation=stir_center_orn,
            maxNumIterations=50,
            residualThreshold=1e-4
        )
        for j in range(7):
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL,
                                   targetPosition=jp[j], force=200)
        p.stepSimulation()
        time.sleep(1 / 240)

    # Return to home
    home_jp = p.calculateInverseKinematics(robot, 6, home_pos, home_orn)
    for j in range(7):
        p.setJointMotorControl2(robot, j, p.POSITION_CONTROL,
                               targetPosition=home_jp[j], force=200)
    for _ in range(20):
        p.stepSimulation()
        time.sleep(1 / 240)

def flip_motion(angle_deg=30, lift=0.15, speed=100):
    e = p.getEulerFromQuaternion(home_orn)
    delta = math.radians(angle_deg)
    flip_orn = p.getQuaternionFromEuler([e[0], e[1] + delta, e[2]])

    for i in range(50):
        t = (i + 1) / 50
        pos = [home_pos[0], home_pos[1], home_pos[2] + lift * t]
        jp = p.calculateInverseKinematics(robot, 6, pos, home_orn)
        for j in range(7):
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL,
                                   targetPosition=jp[j], force=speed)
        p.stepSimulation()
        time.sleep(1 / 240)

    for i in range(50):
        frac = (i + 1) / 50
        orn = p.getQuaternionSlerp(home_orn, flip_orn, frac)
        jp = p.calculateInverseKinematics(robot, 6, pos, orn)
        for j in range(7):
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL,
                                   targetPosition=jp[j], force=speed)
        p.stepSimulation()
        time.sleep(1 / 240)

    for i in range(50):
        t = (i + 1) / 50
        pos = [home_pos[0], home_pos[1], home_pos[2] + lift * (1 - t)]
        jp = p.calculateInverseKinematics(robot, 6, pos, flip_orn)
        for j in range(7):
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL,
                                   targetPosition=jp[j], force=speed)
        p.stepSimulation()
        time.sleep(1 / 240)

    home_joints = p.calculateInverseKinematics(robot, 6, home_pos, home_orn)
    for j in range(7):
        p.setJointMotorControl2(robot, j, p.POSITION_CONTROL,
                               targetPosition=home_joints[j], force=speed)
    for _ in range(20):
        p.stepSimulation()
        time.sleep(1 / 240)

def pour_motion(angle_deg=40, speed=150, tilt_steps=50, hold_duration=4.0):
    ex, ey, ez = p.getEulerFromQuaternion(home_orn)
    delta = math.radians(angle_deg)
    pour_orn = p.getQuaternionFromEuler([ex + delta, ey, ez])

    for i in range(tilt_steps):
        frac = (i + 1) / tilt_steps
        orn = p.getQuaternionSlerp(home_orn, pour_orn, frac)
        jp = p.calculateInverseKinematics(robot, 6, home_pos, orn)
        for j in range(7):
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL,
                                   targetPosition=jp[j], force=speed)
        p.stepSimulation()
        time.sleep(1 / 240)

    for _ in range(int(hold_duration * 240)):
        jp = p.calculateInverseKinematics(robot, 6, home_pos, pour_orn)
        for j in range(7):
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL,
                                   targetPosition=jp[j], force=speed)
        p.stepSimulation()
        time.sleep(1 / 240)

    for i in range(tilt_steps):
        frac = (i + 1) / tilt_steps
        orn = p.getQuaternionSlerp(pour_orn, home_orn, frac)
        jp = p.calculateInverseKinematics(robot, 6, home_pos, orn)
        for j in range(7):
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL,
                                   targetPosition=jp[j], force=speed)
        p.stepSimulation()
        time.sleep(1 / 240)

# --- Gesture Detection ---
def detect_gesture(landmarks):
    if not landmarks:
        return None
    
    # Get index and middle finger tips
    index_tip = landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    middle_tip = landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    
    # Calculate differences
    x_diff = abs(index_tip.x - middle_tip.x)
    y_diff = abs(index_tip.y - middle_tip.y)
    
    # Gesture thresholds (made more lenient)
    if x_diff < 0.08 and y_diff < 0.08:  # Increased from 0.05
        return "stir"  # Fingers close together
    elif x_diff > 0.08 and y_diff < 0.08:  # Decreased from 0.1
        return "flip"  # Fingers spread horizontally
    elif y_diff > 0.08 and x_diff < 0.08:  # Decreased from 0.1
        return "pour"  # Fingers spread vertically
    
    return None

# --- Main Loop ---
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

last_gesture = None
gesture_start_time = 0
gesture_hold_time = 1.0  # Hold gesture for 1 second to trigger

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # Flip the frame horizontally for a later selfie-view display
        frame = cv2.flip(frame, 1)
        
        # Convert the BGR image to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process the frame and detect hands
        results = hands.process(rgb_frame)
        
        # Draw hand landmarks and gesture feedback
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                # Detect gesture
                current_gesture = detect_gesture(hand_landmarks)
                
                # Add visual feedback
                if current_gesture:
                    # Get index and middle finger tips for drawing
                    index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                    middle_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
                    
                    # Convert normalized coordinates to pixel coordinates
                    h, w, _ = frame.shape
                    index_x, index_y = int(index_tip.x * w), int(index_tip.y * h)
                    middle_x, middle_y = int(middle_tip.x * w), int(middle_tip.y * h)
                    
                    # Draw line between fingers
                    cv2.line(frame, (index_x, index_y), (middle_x, middle_y), (0, 255, 0), 2)
                    
                    # Add gesture text
                    cv2.putText(frame, f"Gesture: {current_gesture}", (10, 30),
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    # Add hold time feedback
                    if current_gesture == last_gesture:
                        hold_progress = min(1.0, (time.time() - gesture_start_time) / gesture_hold_time)
                        cv2.rectangle(frame, (10, 50), (int(10 + 200 * hold_progress), 70), (0, 255, 0), -1)
                        cv2.rectangle(frame, (10, 50), (210, 70), (255, 255, 255), 2)
                
                if current_gesture:
                    current_time = time.time()
                    
                    if current_gesture == last_gesture:
                        if current_time - gesture_start_time >= gesture_hold_time:
                            print(f"Executing {current_gesture} motion...")
                            if current_gesture == "stir":
                                stir_motion()
                            elif current_gesture == "flip":
                                flip_motion()
                            elif current_gesture == "pour":
                                pour_motion()
                            gesture_start_time = current_time
                    else:
                        last_gesture = current_gesture
                        gesture_start_time = current_time
        
        # Display the frame
        cv2.imshow('Gesture Control', frame)
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    p.disconnect() 