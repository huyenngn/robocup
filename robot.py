# -*- coding: utf-8 -*-
from __future__ import print_function
import traceback
import cv2
import kick
import sys
import sympy

import requests

from naoqi import ALProxy
from PIL import Image
import numpy as np
import os
import math
import almath
import pickle
import time
import base64
from io import BytesIO


CAMERA_FOV_H = np.deg2rad(60.97)
CAMERA_FOV_V = np.deg2rad(47.64)
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
# IMAGE_WIDTH = 640
# IMAGE_HEIGHT = 480
BALL_SIZE = 0.1  # in meters

MAX_HEAD_PITCH = 0.5149
MIN_HEAD_PITCH = -0.6720
detection_img_path = "./detection_img"
if not os.path.exists(detection_img_path):
    os.makedirs(detection_img_path)

offline_pic = "./output_raw/170.png"


def store_detection_img(in_img, res, num):
    x_ext = int(res["size"] / 2)
    y_ext = int(res["size"] / 2)
    start_point = (res["x"] - x_ext, res["y"] - y_ext)
    end_point = (res["x"] + x_ext, res["y"] + y_ext)
    res_img = np.asarray(in_img)
    res_img = cv2.rectangle(res_img, start_point, end_point, (255, 0, 0), 2)
    cv2.imwrite(os.pardir.join(detection_img_path, "detection_" +
                str(num)+".png"), res_img[:, :, ::-1])
    # cv2.imshow("Sheep", res_img[:, :, ::-1])


def pil_to_base64(pil_img):
    im_file = BytesIO()
    pil_img.save(im_file, format="PNG")
    im_bytes = im_file.getvalue()  # im_bytes: image in binary format.
    im_b64 = base64.b64encode(im_bytes)

    return im_b64


class DetectionResult:
    def __init__(self, data):
        self.found = data['found']
        self.x = (IMAGE_WIDTH // 2 - data['x'])*-1
        self.y = (IMAGE_HEIGHT // 2 - data['y'])
        # output coordinate system:
        # --------------
        # |            |
        # |     ↑      |
        # |      →     |
        # |            |
        # --------------
        self.size = data['size']

    def get_angles(self):
        size_angle = self.size * CAMERA_FOV_H / IMAGE_WIDTH
        dist = BALL_SIZE / (2 * math.tan(size_angle / 2))
        # Convert to radians
        x_angle = self.x * CAMERA_FOV_H / IMAGE_WIDTH
        y_angle = self.y * CAMERA_FOV_V / IMAGE_HEIGHT

        return x_angle, y_angle, dist

    def __str__(self):
        return "Found: {}, x: {}, y: {}, size: {}".format(self.found, self.x, self.y, self.size)


class Camera:

    def __init__(self, ip, port, online):
        self.online = online
        self.detection_img_count = 0
        if online:
            self.camProxy = ALProxy("ALVideoDevice", ip, port)
            resolution = 1  # 320x240
            colorSpace = 11   # RGB

            self.unsub()
            self.videoClient = self.camProxy.subscribe(
                "python_client", resolution, colorSpace, 5)
        self.set_camera("CameraTop")

    def set_camera(self, id):
        self.active_camera = id
        if self.online:
            if id == "CameraTop":
                self.camProxy.setActiveCamera(0)
            else:
                self.camProxy.setActiveCamera(1)

    def unsub(self):
        if self.online:
            subs = self.camProxy.getSubscribers()
            for sub in subs:
                self.camProxy.unsubscribe(sub)

    def get_image(self):
        if not self.online:
            return Image.open(offline_pic)

        naoImage = self.camProxy.getImageRemote(self.videoClient)
        imageWidth = naoImage[0]
        imageHeight = naoImage[1]
        array = naoImage[6]

        # Create a PIL Image from our pixel array.
        im = Image.frombytes("RGB", (imageWidth, imageHeight), array)
        return im

    def detect(self):
        im = self.get_image()
        bim = pil_to_base64(im)

        res = requests.post("http://localhost:5000/analyse", data=bim)
        data = res.json()

        store_detection_img(im, data, self.detection_img_count)
        self.detection_img_count += 1

        if data["found"] == False:
            return None

        return DetectionResult(data)


class Robot:

    def __init__(self, ip, port, online=True):
        self.online = online
        self.cam = Camera(ip, port, online)

        if online:
            self.motionProxy = ALProxy("ALMotion", ip, port)
            self.postureProxy = ALProxy("ALRobotPosture", ip, port)
            self.tts = ALProxy("ALTextToSpeech", ip, port)
            self.postureProxy.goToPosture("StandInit", 0.5)
            self.tts.say("Initialized")

    def find_ball_body(self):
        print("Starting find ball")
        self.tts.say("I start to find the ball")
        steps_deg = 15
        step_size = np.deg2rad(steps_deg)
        for cur_angle in range(0, 360, steps_deg):
            print("current angle:", cur_angle)
            dec = self.cam.detect()
            if dec.found:
                print("ball detected")
                self.tts.say("I found the ball!")
                return dec

            # time.sleep(1)
            self.motionProxy.moveTo(0, 0, step_size)
            self.motionProxy.waitUntilMoveIsFinished()

        return None

    def get_coords_nao_space_manual(self, dec):
        currentCamera = self.cam.active_camera

        # get angles of ball postition
        x_img_angle, y_img_angle, _ = dec.get_angles()

        z_angle, y_angle = x_img_angle*-1, y_img_angle*-1

        # Get current robot to cam transformation
        if self.online:
            transform = self.motionProxy.getTransform(currentCamera, 2, True)
            with open('motionProxyTransform_' + currentCamera + '.pickle', 'wb') as f:
                pickle.dump(transform, f, protocol=pickle.HIGHEST_PROTOCOL)
        else:
            with open('motionProxyTransform_' + currentCamera + '.pickle', "rb") as f:
                transform = pickle.load(f)

        transformList = almath.vectorFloat(transform)
        robotToCamera = almath.Transform(transformList)
        cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(
            0, y_angle, z_angle)
        robotToLandmarkRotation = robotToCamera*cameraToLandmarkRotationTransform
        # get (trans_end_point-trans_start_point) as ball direction from cam
        start_point = almath.Position3D(0., 0., 0.)
        end_point = almath.Position3D(1., 0., 0.)
        trans_start_point = robotToLandmarkRotation*start_point
        trans_end_point = robotToLandmarkRotation*end_point

        # plane on z position of ball center
        ball_z_pos = BALL_SIZE/2
        plane = sympy.Plane(sympy.Point3D(0, 0, ball_z_pos), sympy.Point3D(
            1, 0, ball_z_pos), sympy.Point3D(0, 1, ball_z_pos))
        # get line from cam position in direction of ball position
        trans_start_point = sympy.Point3D(trans_start_point.toVector())
        trans_end_point = sympy.Point3D(trans_end_point.toVector())
        line = sympy.Line3D(trans_start_point,
                            direction_ratio=trans_end_point-trans_start_point)

        # intersection of plane and line -> ball position
        intr = plane.intersection(line)

        intersection = np.array(intr[0], dtype=float)

        print("Ball position relative to robot feet (forward: {0}m, left: {1}m, top: {2}m, left_rotation: {3}°, bottom_rotation: {4}°)".format(
            np.round(intersection[0], 2), np.round(intersection[1], 2), np.round(intersection[2], 2), np.round(np.rad2deg(z_angle), 2), np.round(np.rad2deg(y_angle), 2)))

        return (intersection[0], intersection[1], intersection[2]), z_angle, y_angle

    def reset(self):
        if self.online:
            self.motionProxy.rest()

    def kick(self):
        kick.kick()

    def move_head(self, y_angle):
        #move head pitch relative to old position 
        self.motionProxy.changeAngles("HeadPitch", y_angle, 0.05)

    def start_head_tracking(self):
        while True:
            dec_result = self.cam.detect()
            if dec_result.found:
                self.move_head(dec_result)
            time.sleep(1)

    def rotate_to_ball(self):
        dec = self.cam.detect()

        if dec is None:
            return False
        _, theta, y_angle = self.get_coords_nao_space_manual(
            dec)
        self.move_head(y_angle)
        self.motionProxy.moveTo(0, 0, theta)
        self.motionProxy.waitUntilMoveIsFinished()
        return True

    def move_head_until_dection(self):
        while self.motionProxy.getAngles("HeadPitch", False) < MAX_HEAD_PITCH+np.rad2deg(5):
            self.move_head(np.rad2deg(10))
            dec = self.cam.detect()
            if dec is not None:
                _, _, y_angle = self.get_coords_nao_space_manual(dec)
                self.move_head(y_angle)
                return True
        return False

    def move_to_ball(self, dist_fac=1, x_offset=0):
        dec = self.cam.detect()
        if dec is None:
            return (False, None)

        (x, y, _), _, y_angle = self.get_coords_nao_space_manual(
            dec)

        self.move_head(y_angle)
        dec = self.cam.detect()
        self.motionProxy.moveTo((x-x_offset)*dist_fac, y*dist_fac, 0)
        self.motionProxy.waitUntilMoveIsFinished()
        return True, x-x_offset

    def correct_move_to_ball(self, min_x_dist=0):
        while True:
            # rotate to Ball Position
            r_res = self.rotate_to_ball()
            # move half distance to the ball
            m_res, x_dist = self.move_to_ball(
                dist_fac=0.5, x_offset=min_x_dist) if r_res else (False, None)
            if not m_res:
                # no ball detected -> move head down
                print("no ball detected move head down")
                if not self.move_head_until_dection():
                    # no ball detected
                    return False
            if x_dist <= min_x_dist+0.02:
                # reach position
                return True

    def start(self):
        self.cam.set_camera("CameraTop")
        # rotate until find ball
        # dec = self.find_ball_body()
        # if dec is None:
        #     self.tts.say("I couldn't find the ball!")
        #     return False

        # move to the ball until no one is recognized anymore or reach postion
        self.tts.say("I start to move to the ball")

        print("start to move to the ball (top camera)")
        top_reach_min_dist = self.correct_move_to_ball(min_x_dist=0.15)
        self.tts.say("no ball in the top camera")
        self.cam.set_camera("CameraBottom")

        # move head to absolut start position (20 grad)
        rob.motionProxy.angleInterpolationWithSpeed(
            "HeadPitch", np.deg2rad(20), 0.5)

        # move to the ball until no one is recognized anymore or reach postion
        print("start to move to the ball (bottom camera)")
        bottom_reach_min_dist = self.correct_move_to_ball(min_x_dist=0.15)

        if bottom_reach_min_dist:
            # reached the ball
            self.tts.say("I reached the ball")

            print("Standing infront of the ball")
            kick.kick()
            return True
        else:
            # lost the ball on the way
            self.tts.say("I lost the ball on the way")
            return False


if __name__ == "__main__":
    robotIp = "10.0.7.101"  # Replace here with your NAOqi's IP address.
    robotPort = 9559
    rob = Robot(robotIp, robotPort, online=True)
    try:
        rob.motionProxy.angleInterpolationWithSpeed(
            "HeadPitch", np.deg2rad(20), 0.5)
        rob.start()
    except:
        print("reset...")
        rob.cam.unsub()
        rob.reset()
        traceback.print_exc()
