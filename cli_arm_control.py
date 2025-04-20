import pybullet as p
import pybullet_data
import time
import math

# --- PyBullet setup ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# --- GUI Cleanup ---  # <-- NEW
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

# --- Set default camera view ---  # <-- NEW
p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=50,
    cameraPitch=-35,
    cameraTargetPosition=[0.5, 0, 1]
)

# --- Load plane and robot ---
plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

# --- Pan + handle parameters ---
pan_radius    = 0.15
pan_height    = 0.05
handle_radius = 0.02
handle_length = 0.30

# --- Visual & collision shapes for pan ---
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

# --- Visual & collision shapes for handle ---
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

# --- Create compound multibody: pan (base) + handle (link) ---
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

# --- Attach pan to robot end-effector ---
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

# --- Move robot to starting joint positions ---
init_joints = [0, 0, 0, 0, 0, 0, 0]
for idx, pos in enumerate(init_joints):
    p.resetJointState(robot, idx, pos)

# --- Get initial end-effector pose ---
link_state = p.getLinkState(robot, 6, computeForwardKinematics=True)
home_pos, home_orn = link_state[0], link_state[1]
stir_center_pos, stir_center_orn = home_pos, home_orn

# --- Stir Motion ---
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

# --- Flip Motion ---
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

# --- Pour Motion ---
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

# --- Interactive Command Loop ---
print("Type 'stir', 'flip', 'pour' or 'quit' to exit.")
while True:
    cmd = input("> ").strip().lower()
    if cmd == "stir":
        stir_motion()
    elif cmd == "flip":
        flip_motion()
    elif cmd == "pour":
        pour_motion()
    elif cmd == "quit":
        break
    else:
        print("Unknown — try stir, flip, pour, quit.")

p.disconnect()