# -*- coding: utf-8 -*-
from __future__ import print_function
import traceback
import cv2
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
IMAGE_WIDTH = 160
IMAGE_HEIGHT = 120
# IMAGE_WIDTH = 640
# IMAGE_HEIGHT = 480
BALL_SIZE = 0.1  # in meters
meter_per_sec = np.sqrt(2 * (0.25 ** 2)) / 6.14
sec_per_meter = 1 / meter_per_sec


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
    cv2.imwrite(detection_img_path + "/detection_" +
                               str(num) + ".png", res_img[:, :, ::-1])
    cv2.imwrite(detection_img_path + "/detection_cur.png", res_img[:, :, ::-1])
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
        self.x = (IMAGE_WIDTH // 2 - data['x']) * -1
        self.y = (IMAGE_HEIGHT // 2 - data['y'])

        if abs(self.x) > 120 and abs(self.y) > 80:
            self.found = False

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
            resolution = 0  # 320x240
            colorSpace = 11  # RGB

            self.camProxy.setParameter(0, 22, 3)
            self.camProxy.setParameter(1, 22, 3)

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

        dec_result = DetectionResult(data)

        if not dec_result.found:
            return None

        return dec_result


class Robot:

    def __init__(self, ip, port, online=True):
        self.online = online
        self.cam = Camera(ip, port, online)

        if online:
            self.motionProxy = ALProxy("ALMotion", ip, port)
            self.postureProxy = ALProxy("ALRobotPosture", ip, port)
            self.tts = ALProxy("ALTextToSpeech", ip, port)
            self.postureProxy.goToPosture("StandInit", 0.5)

    def find_ball_body(self, steps_deg=30):
        print("Starting find ball")
        step_size = np.deg2rad(steps_deg)
        for cur_angle in range(0, 360, steps_deg):
            print("current angle:", cur_angle)

            for cam in ["CameraTop", "CameraBottom"]:
                self.cam.set_camera(cam)
                dec = self.cam.detect()
                if dec is not None:
                    print("Ball detected")
                    self.tts.say("I found the ball!")
                    return dec, cam == "CameraTop"

            self.motionProxy.moveTo(0, 0, step_size)
            self.motionProxy.waitUntilMoveIsFinished()

        self.tts.say("I couldn't find the ball!")
        return None, None

    def get_coords_nao_space_manual(self, dec):
        currentCamera = self.cam.active_camera

        # get angles of ball postition
        x_img_angle, y_img_angle, _ = dec.get_angles()

        z_angle, y_angle = x_img_angle * -1, y_img_angle * -1

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
        robotToLandmarkRotation = robotToCamera * cameraToLandmarkRotationTransform
        # get (trans_end_point-trans_start_point) as ball direction from cam
        start_point = almath.Position3D(0., 0., 0.)
        end_point = almath.Position3D(1., 0., 0.)
        trans_start_point = robotToLandmarkRotation * start_point
        trans_end_point = robotToLandmarkRotation * end_point

        # plane on z position of ball center
        ball_z_pos = BALL_SIZE / 2
        plane = sympy.Plane(sympy.Point3D(0, 0, ball_z_pos), sympy.Point3D(
            1, 0, ball_z_pos), sympy.Point3D(0, 1, ball_z_pos))
        # get line from cam position in direction of ball position
        trans_start_point = sympy.Point3D(trans_start_point.toVector())
        trans_end_point = sympy.Point3D(trans_end_point.toVector())
        line = sympy.Line3D(trans_start_point,
                            direction_ratio=trans_end_point - trans_start_point)

        # intersection of plane and line -> ball position
        intr = plane.intersection(line)

        intersection = np.array(intr[0], dtype=float)

        print(
            "Ball position relative to robot feet (forward: {0}m, left: {1}m, top: {2}m, left_rotation: {3}°, bottom_rotation: {4}°)".format(
                np.round(intersection[0], 2), np.round(intersection[1], 2), np.round(intersection[2], 2),
                np.round(np.rad2deg(z_angle), 2), np.round(np.rad2deg(y_angle), 2)))

        return (intersection[0], intersection[1], intersection[2]), z_angle, y_angle

    def reset(self):
        if self.online:
            self.motionProxy.rest()
        self.cam.unsub()

    def kick(self):
        legName = ["LLeg"]
        X       = 1
        Y       = 0.15
        Theta   = np.deg2rad(20)
        footSteps = [[X, Y, Theta]]
        fractionMaxSpeed = [1.0]
        clearExisting = False
        rob.motionProxy.setFootStepsWithSpeed(legName, footSteps, fractionMaxSpeed, clearExisting)
        rob.motionProxy.waitUntilMoveIsFinished()

        self.tts.say("Kick!")

        legName = ["RLeg"]
        X       = 1
        Y       = 0
        Theta   = 0
        footSteps = [[X, Y, Theta]]
        fractionMaxSpeed = [1.0]
        clearExisting = False
        rob.motionProxy.setFootStepsWithSpeed(legName, footSteps, fractionMaxSpeed, clearExisting)
        rob.motionProxy.waitUntilMoveIsFinished()


        rob.motionProxy.moveTo(0.01, 0, 0)
        rob.motionProxy.waitUntilMoveIsFinished()

    def move_head(self, y_angle):
        # move head pitch relative to old position
        curAngle = self.motionProxy.getAngles("HeadPitch", False)[0]
        self.motionProxy.angleInterpolationWithSpeed(
            "HeadPitch", curAngle + y_angle, 1)

    def rotate_to_ball(self):
        dec = self.cam.detect()
        if dec is None:
            return False

        _, theta, y_angle = self.get_coords_nao_space_manual(dec)
        self.move_head(y_angle)
        self.motionProxy.moveTo(0, 0, theta)
        self.motionProxy.waitUntilMoveIsFinished()
        return True

    def move_head_until_detection(self, steps_deg=5):
        print("Moving Head")
        curAngle = self.motionProxy.getAngles("HeadPitch", False)[0]
        while np.abs(curAngle - MAX_HEAD_PITCH) > np.deg2rad(1):
            print("CurAngle: ", curAngle, MAX_HEAD_PITCH)
            self.move_head(np.deg2rad(steps_deg))
            dec = self.cam.detect()
            if dec is not None:
                _, _, y_angle = self.get_coords_nao_space_manual(dec)
                self.move_head(y_angle)
                return True
            curAngle = self.motionProxy.getAngles("HeadPitch", False)[0]
        print("CurAngle: ", curAngle, MAX_HEAD_PITCH)
        print("Not detected...")
        return False

    def move_to_ball(self, dist_fac=1., x_offset=0., old_move_vel = np.zeros(2)):
        start = time.time()
        dec = self.cam.detect()
        if dec is None:
            return False, None, None, None

        (x, y, _), theta, y_angle = self.get_coords_nao_space_manual(dec)

        self.move_head(y_angle)
        # TODO: Schauen ob hier dec nötig ist
        # dec = self.cam.detect()
        # (x, y, _), _, y_angle = self.get_coords_nao_space_manual(dec)
        theta *= 0.8
        if theta > np.deg2rad(5):
            self.motionProxy.stopMove()
            self.motionProxy.moveTo(0, 0, theta)
            self.motionProxy.waitUntilMoveIsFinished()

        end = time.time()

        dist_travelled = old_move_vel * (end - start)

        move_x = (x - x_offset) * dist_fac
        move_y = y * dist_fac
        move_pos = np.array([move_x, move_y])
        move_pos -= dist_travelled

        dist = np.sqrt(move_pos[0] ** 2 + move_pos[1] ** 2)
        dist_time = sec_per_meter * dist
        move_vel = move_pos / dist_time

        if move_x >= 0:
            self.motionProxy.post.moveTo(move_pos[0], move_pos[1], theta)
            return True, x - move_x, move_vel, dist
        else:
            return True, x, move_vel, dist

    def correct_move_to_ball(self, min_x_dist=0., detect_on_the_move = False):
        move_vel = np.zeros(2)
        close = False
        not_detected_count = 0
        while True:
            # rotate to Ball Position
            #r_res = self.rotate_to_ball()
            # move half distance to the ball
            ball_found, x_dist, move_vel, dist_estimate = self.move_to_ball(
                dist_fac=0.6, x_offset=min_x_dist, old_move_vel=move_vel)
                # if r_res else (False, None)
            
            if not ball_found:
                # no ball detected -> move head down
                self.tts.say("Lost ball")
                print("no ball detected move head down")
                not_detected_count += 1

                if not_detected_count >= 3 or not self.motionProxy.moveIsActive():
                    self.motionProxy.waitUntilMoveIsFinished()
                    if not self.move_head_until_detection():
                        # no ball detected
                        return False
                    else:
                        not_detected_count = 0
            else:
                not_detected_count = 0

            print("Dist Estimate", dist_estimate)
            
            if close:
                print("Waiting")
                self.motionProxy.waitUntilMoveIsFinished()
                move_vel = np.zeros(2)

            elif not detect_on_the_move and dist_estimate < 0.15:
                self.motionProxy.stopMove()
                self.tts.say("I am close to the ball")
                move_vel = np.zeros(2)
                close = True
                continue
            
            time.sleep(0.25)
            print("foo")

            print("x_dist", x_dist, "Dist Thresh", min_x_dist + 0.05)
            if x_dist is not None and x_dist <= min_x_dist + 0.05:
                self.motionProxy.waitUntilMoveIsFinished()
                # reach position
                return True

    def start(self):
        self.cam.set_camera("CameraTop")

        # rotate until find ball

        dec, detectedTopCam = self.find_ball_body()
        if dec is None:
            return False

        min_x_dist = 0.15
        if detectedTopCam:
        # move to the ball until no one is recognized anymore or reach position
            print("start to move to the ball (top camera)")
            self.cam.set_camera("CameraTop")

            reached_position_top_cam = self.correct_move_to_ball(min_x_dist=0.3, detect_on_the_move = True)

            if not reached_position_top_cam:
                print("No ball in top camera")
                #self.tts.say("no ball in the top camera")
            else:
                #self.tts.say("I reached the ball")
                print("reached the ball (top camera)")

            # move head to absolut start position (20 grad)
            rob.motionProxy.angleInterpolationWithSpeed(
                "HeadPitch", np.deg2rad(20), 0.5)

        self.cam.set_camera("CameraBottom")

        # move to the ball until no one is recognized anymore or reach position
        print("start to move to the ball (bottom camera)")
        reached_position_bottom_cam = self.correct_move_to_ball(min_x_dist=min_x_dist)

        if reached_position_bottom_cam:
            # reached the ball
            self.tts.say("I reached the ball")

            print("Standing in front of the ball")

            self.motionProxy.waitUntilMoveIsFinished()
            self.tts.say("Adjusting for kick")
            dec = self.cam.detect()
            if dec is None:
                return False, None

            (x, y, _), theta, y_angle = self.get_coords_nao_space_manual(dec)
            self.motionProxy.moveTo((x - min_x_dist) * 0.5, y + 0.03, 0)
            self.kick()
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
        # rob.motionProxy.angleInterpolationWithSpeed(
        #     "HeadPitch", np.deg2rad(20), 0.5)
        rob.start()

        # rob.kick()
        # rob.motionProxy.post.moveTo(0.2, 0.1, 0)
        # rob.motionProxy.waitUntilMoveIsFinished()
        # print("FOOOOOOo")

        rob.reset()
    except:
        print("reset...")
        rob.reset()
        traceback.print_exc()
