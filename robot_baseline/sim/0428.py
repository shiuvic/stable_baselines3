import time
from threading import Thread
import math
import pybullet_data
import pybullet as p

op3_joints = ['l_hip_yaw',
              'l_hip_roll',
              'l_hip_pitch',
              'l_knee',
              'l_ank_pitch',
              'l_ank_roll',
              'r_hip_yaw',
              'r_hip_roll',
              'r_hip_pitch',
              'r_knee',
              'r_ank_pitch',
              'r_ank_roll',
              'l_sho_pitch',
              'l_sho_roll',
              'l_el',
              'r_sho_pitch',
              'r_sho_roll',
              'r_el',
              'head_pan',
              'head_tilt']

class OP3:
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setGravity(0, 0, -9.8)
        op3StartPos = [0, 0, 0.28]
        self.ballpos = [0.2,0.2,0.3]
        self.x = 0
        op3StartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.planeId = p.loadURDF("plane.urdf")
        self.robot = p.loadURDF("../models/robotis_op3.urdf", op3StartPos, op3StartOrientation)
        self.numJoints = p.getNumJoints(self.robot)
        self.targetVel = 0
        self.maxForce = 100
        # self.update_angle_th()
        # We go real-time simulation rather than call stepSimulation
        p.setRealTimeSimulation(1)
        self.ball = p.loadURDF("F:/py/bullet3-master/data/sphere_small.urdf", self.ballpos)
        self._set_joint()
        self.joints = op3_joints
        self.angles = {}

    def get_angles(self):
        if self.joints is None: return None
        if self.angles is None: return None
        return dict(zip(self.joints, self.angles))

    def set_angles(self, angles):
        for j, v in angles.items():
            if j not in self.joints:
                AssertionError("Invalid joint name " + j)
                continue
            p.setJointMotorControl(self.robot, op3_joints.index(j), p.POSITION_CONTROL, v, self.maxForce)

    def set_angles_slow(self, stop_angles, delay=2):
        start_angles = {'l_hip_yaw': -0.0006003781295184577, 'l_hip_roll': 0.00027541140297364884, 'l_hip_pitch': -0.0027132214875610623, 'l_knee': -0.0003529878949643889, 'l_ank_pitch': 0.00234694503638634, 'l_ank_roll': 0.00221300029210034, 'r_hip_yaw': -0.0003914124825885725, 'r_hip_roll': 0.00041712889275416005, 'r_hip_pitch': -0.0011724624759162654, 'r_knee': 0.0047281000738706065, 'r_ank_pitch': -0.0036229814257040358, 'r_ank_roll': 0.0038011178078730906, 'l_sho_pitch': -5.9948193227529784e-05, 'l_sho_roll': 0.0003006644446475642, 'l_el': 0.00027580599176571416, 'r_sho_pitch': -3.449540793066871e-05, 'r_sho_roll': 0.0001028452376745412, 'r_el': 9.637206668694198e-05, 'head_pan': 0.00048666303033982546, 'head_tilt': -0.00015481243935058867}
        start = time.time()
        stop = start + delay
        while True:
            t = time.time()
            if t > stop: break
            ratio = (t - start) / delay
            angles = interpolate(stop_angles, start_angles, ratio)
            print(p.getJointState(self.robot,14))
            print(p.getBasePositionAndOrientation(self.ball))
            self.set_angles(angles)
            time.sleep(0.1)

    def _set_joint(self):
        for joint in range(self.numJoints):
            print(p.getJointInfo(self.robot, joint))
            p.setJointMotorControl(self.robot, joint, p.POSITION_CONTROL, self.targetVel, self.maxForce)

    def run(self):
        try:
            while True:
                # p.stepSimulation()
                time.sleep(1./240.)
        finally:
            OP3Pos, OP3Orn = p.getBasePositionAndOrientation(self.robot)
            print(OP3Pos, OP3Orn)
            p.disconnect()


def interpolate(anglesa, anglesb, coefa):
    z = {}
    joints = anglesa.keys()
    for j in joints:
        z[j] = anglesa[j] * coefa + anglesb[j] * (1 - coefa)
    return z

class Move:

    def __init__(self,op3, **kwargs):
        self.op3 = op3
        self.move = False
        print(self.get_ball_pos())
        self.arm = dict()
        self.arm['l_sho_pitch'] = -1.5
        self.arm['l_sho_roll'] = 0.8
        self.arm['l_el'] = -0.8
        while self.move is False:
            time.sleep(0.5)
            self.op3.set_angles_slow(self.arm)


    def get_ball_pos(self):
        return self.op3.ballpos
    def set_arm(self,a,b,c):
        self.arm['l_sho_pitch'] = 1.0
        self.arm['l_sho_roll'] = 0.0
        self.arm['l_el'] = 0.0


if __name__ == '__main__':
    op3 = OP3()
    move = Move(op3)
    pass
