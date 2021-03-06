import time
from threading import Thread
import numpy as np
np.set_printoptions(precision=2)
import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import sys
import math
import cv2

if sys.platform == "win32":
    from ctypes import windll

    timeBeginPeriod = windll.winmm.timeBeginPeriod
    timeBeginPeriod(1)

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
    def __init__(self,client,fallen_reset=True):
        self.fallen_reset = fallen_reset
        self.physicsClient = client  # or p.DIRECT for non-graphical version
        p.configureDebugVisualizer(p.COV_ENABLE_GUI)
        # self.sld_sim_speed = p.addUserDebugParameter("sim_speed", 1.0, 1000.0, sim_speed)
        self.sld_sim_speed = 1.0
        self.bt_rst = p.addUserDebugParameter("reset OP3", 1, 0, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 1)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setGravity(0, 0, -9.8)
        self.op3StartPos = [-0.0, 0, 3.53]
        self.op3StartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robot = p.loadURDF(".//robotis_op3.urdf", self.op3StartPos, self.op3StartOrientation)
        self.numJoints = p.getNumJoints(self.robot)
        self.targetVel = 0
        self.maxForce = 100
        self.img = None
        self.r = [0, 0, 0]
        self.b = [0, 0, 0]
        self.black = [0, 0, 0]
        self.w = [0, 0, 0]
        # self.camera_follow()
        self.angles = None
        self.update_angle_th()
        self.check_reset_th()
        self.update_camera_th()
        # You can use real-time simulation rather than call stepSimulation
        # p.setRealTimeSimulation(1)
        self.run_sim_th()
        self._set_joint()

        self.joints = op3_joints

    @property
    def sim_speed(self):
        return 1.0

    def get_orientation(self):
        _, orientation = p.getBasePositionAndOrientation(self.robot)
        return np.array(orientation)

    def get_position(self):
        position, _ = p.getBasePositionAndOrientation(self.robot)
        return np.array(position)

    def camera_follow(self, distance=1.0, pitch=-50.0, yaw=-90.0):
        lookat = self.get_position() - [0, 0, 0.1]
        p.resetDebugVisualizerCamera(distance, yaw, pitch, lookat)

    def is_fallen(self):
        """Decide whether the rex has fallen.
        If the up directions between the base and the world is large (the dot
        product is smaller than 0.85), the rex is considered fallen.
        Returns:
          Boolean value that indicates whether the rex has fallen.
        """
        rot_mat = p.getMatrixFromQuaternion(self.get_orientation())
        local_up = rot_mat[6:]
        return np.dot(np.asarray([0, 0, 1]), np.asarray(local_up)) < 0.85

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
        start_angles = self.get_angles()
        start = time.time()
        stop = start + delay
        while True:
            t = time.time()
            if t > stop: break
            ratio = (t - start) / (delay / self.sim_speed)
            angles = interpolate(stop_angles, start_angles, ratio)
            self.set_angles(angles)
            time.sleep(0.1 / self.sim_speed)

    def run_sim_th(self):
        def _cb_sim():
            while True:
                p.stepSimulation()
                time.sleep(1.0 / 240.0)
                # self.camera_follow(distance=1.5)
        Thread(target=_cb_sim).start()

    def check_reset_th(self):
        def _cb_reset():
            self.prev_state = 1.0
            while True:
                curr_state = p.readUserDebugParameter(self.bt_rst)
                if curr_state != self.prev_state or (self.fallen_reset and self.is_fallen()):
                    self.reset_and_start()
                    self.prev_state = curr_state
                time.sleep(0.001)

        Thread(target=_cb_reset).start()

    def update_angle_th(self):
        def _cb_angles():
            while True:
                angles = []
                for joint in range(self.numJoints):
                    angles.append(p.getJointState(self.robot, joint)[0])
                self.angles = angles
                time.sleep(0.001)

        Thread(target=_cb_angles).start()

    def _set_joint(self):
        for joint in range(self.numJoints):
            # print(p.getJointStates(self.robot, joint))
            p.setJointMotorControl(self.robot, joint, p.POSITION_CONTROL, self.targetVel, self.maxForce)

    def run(self):
        try:
            while True:
                p.stepSimulation()
                time.sleep(1. / 240.)
        finally:
            OP3Pos, OP3Orn = p.getBasePositionAndOrientation(self.robot)
            print(OP3Pos, OP3Orn)
            p.disconnect()

    def reset_and_start(self):
        p.resetBasePositionAndOrientation(self.robot, self.op3StartPos, self.op3StartOrientation)

    def update_camera_th(self):
        def setCameraPicAndGetPic():
            while True:
                width = 50
                height = 50
                BASE_RADIUS = 0.1
                BASE_THICKNESS = 0.1
                # basePos = p.getLinkState(self.robot,19)[0]
                basePos, baseOrientation = p.getBasePositionAndOrientation(self.robot)

                matrix = p.getMatrixFromQuaternion(baseOrientation)
                tx_vec = np.array([matrix[0], matrix[3], matrix[6]])  # ????????????x???
                tz_vec = np.array([matrix[2], matrix[5], matrix[8]])  # ????????????z???

                basePos = np.array(basePos)
                # ??????????????????
                cameraPos = basePos + BASE_RADIUS * tx_vec + 0.5 * BASE_THICKNESS * tz_vec
                targetPos = cameraPos + 0.3 * tx_vec -[0,0,0.6]

                viewMatrix = p.computeViewMatrix(
                    cameraEyePosition=cameraPos,
                    cameraTargetPosition=targetPos,
                    cameraUpVector=tz_vec,
                    # physicsClientId=self.physicsClient
                )
                projectionMatrix = p.computeProjectionMatrixFOV(
                    fov=60.0,  # ????????????????????????
                    aspect=1.0,
                    nearVal=0.01,  # ?????????????????????
                    farVal=20,  # ?????????????????????
                    # physicsClientId=self.physicsClient
                )

                width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                    width=width, height=height,
                    viewMatrix=viewMatrix,
                    projectionMatrix=projectionMatrix,
                    renderer=p.ER_BULLET_HARDWARE_OPENGL,
                    # physicsClientId=self.physicsClient
                )
                time.sleep(0.15)
                self.img = rgbImg
        Thread(target=setCameraPicAndGetPic).start()


    def angle(self,goal):
        X = np.array(goal) - np.array(self.get_position())
        U = math.atan2(X[1], X[0])
        # U = U*(180/math.pi)
        OR = p.getEulerFromQuaternion(self.get_orientation())[2]

        y = U - OR
        return y


    def get_observation(self):
        img_arr = self.img
        np_img_arr = np.reshape(img_arr, (50, 50, 4))
        x = p.getBasePositionAndOrientation(self.robot)[0][0]
        y = p.getBasePositionAndOrientation(self.robot)[0][1]
        ob = np.append(np_img_arr[:, :, :4].flatten(), np.full((50, 2), [x, y]).flatten())
        return ob

    def img_pro(self, arr):
        r_x = []
        r_y = []
        b_x = []
        b_y = []
        w_x = []
        w_y = []
        black_x = []
        black_y = []
        self.r = [0, 0, 0]
        self.b = [0, 0, 0]
        self.black = [0, 0, 0]
        self.w = [0, 0, 0]
        for i in range(50):
            for j in range(50):
                if arr.item(i, j, 0) > 100 and arr.item(i, j, 1) < 50 and arr.item(i, j, 2) < 50:
                    r_x.append(i)
                    r_y.append(j)
                if arr.item(i, j, 0) < 50 and arr.item(i, j, 1) < 50 and arr.item(i, j, 2) > 100:
                    b_x.append(i)
                    b_y.append(j)
                if arr.item(i, j, 0) > 250 and arr.item(i, j, 1) > 250 and arr.item(i, j, 2) > 250:
                    w_x.append(i)
                    w_y.append(j)
                if arr.item(i, j, 0) < 10 and arr.item(i, j, 1) < 10 and arr.item(i, j, 2) < 10:
                    black_x.append(i)
                    black_y.append(j)


        if r_x != [] and r_y != []:
            x = round(np.mean(r_x))
            y = round(np.mean(r_y))
            self.r = [1, x, y]

        if b_x != [] and b_y != []:
            x = round(np.mean(b_x))
            y = round(np.mean(b_y))
            self.b = [1, x, y]

        if black_x != [] and black_y != []:
            x = round(np.mean(black_x))
            y = round(np.mean(black_y))
            self.black = [1, x, y]
        if w_x != [] and w_y != []:
            x = round(np.mean(w_x))
            y = round(np.mean(w_y))
            self.w = [1, x, y]




def interpolate(anglesa, anglesb, coefa):
    z = {}
    joints = anglesa.keys()
    for j in joints:
        z[j] = anglesa[j] * coefa + anglesb[j] * (1 - coefa)
    return z


if __name__ == '__main__':
    op3.run()
    pass
