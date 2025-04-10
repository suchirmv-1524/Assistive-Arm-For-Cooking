import pybullet as p
import pybullet_data
import time
import os

class RobotArmSim:
    def __init__(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.loadURDF("plane.urdf")
        self.robot = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)
        self.joint_indices = list(range(p.getNumJoints(self.robot)))

    def move_arm(self, command):
        if command == "stir":
            for joint in self.joint_indices:
                p.setJointMotorControl2(self.robot, joint, p.POSITION_CONTROL, targetPosition=0.5)
        elif command == "flip":
            for joint in self.joint_indices:
                p.setJointMotorControl2(self.robot, joint, p.POSITION_CONTROL, targetPosition=-0.5)
        elif command == "pour":
            for joint in self.joint_indices:
                p.setJointMotorControl2(self.robot, joint, p.POSITION_CONTROL, targetPosition=1.0)

    def step(self):
        p.stepSimulation()
        time.sleep(0.05)
