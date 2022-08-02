# -*- coding: utf-8 -*-
from __future__ import print_function
import cv2
import kick
import sys
import sympy

import requests

from naoqi import ALProxy
from PIL import Image
import numpy as np
import math
import almath
import pickle
import time
import base64
from io import BytesIO


def display(in_img, res):
    x_ext = int(res["size"] / 2)
    y_ext = int(res["size"] / 2)
    start_point = (res["x"] - x_ext, res["y"] - y_ext)
    end_point = (res["x"] + x_ext, res["y"] + y_ext)
    res_img = np.asarray(in_img)
    res_img = cv2.rectangle(res_img, start_point, end_point, (255, 0, 0), 2)
    cv2.imwrite("detection.png", res_img[:, :, ::-1])
    # cv2.imshow("Sheep", res_img[:, :, ::-1])


def pil_to_base64(pil_img):
    im_file = BytesIO()
    pil_img.save(im_file, format="PNG")
    im_bytes = im_file.getvalue()  # im_bytes: image in binary format.
    im_b64 = base64.b64encode(im_bytes)

    return im_b64


DISPLAY_IMG = True
CAMERA_FOV_H = np.deg2rad(60.97)
CAMERA_FOV_V = np.deg2rad(47.64)
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
# IMAGE_WIDTH = 640
# IMAGE_HEIGHT = 480
BALL_SIZE = 0.1  # in meters

FX = (320/2)/np.tan(CAMERA_FOV_H/2)
FY = (240/2)/np.tan(CAMERA_FOV_V/2)

offline_pic = "./output_raw/170.png"
#offline_pic = "./output_raw/127.png"
#offline_pic = "./output_raw/102.png"


def angle(a, b):
    return np.arctan2(b[1], b[0]) - np.arctan2(a[1], a[0])


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

    def get_angles_by_position(self):
        z = (FX * 0.1) / (self.size)
        x = ((self.x)*z)/FX
        y = ((self.y)*z)/FY
        v_t = np.asarray([z, x, y])
        dist = np.linalg.norm(v_t)
        x_ang = [angle([v_t[0], 0], [v_t[0], v_t[1]]),
                 angle([v_t[0], 0], [v_t[0], v_t[2]])]
        return x_ang[0], x_ang[1], dist

    def __str__(self):
        return "Found: {}, x: {}, y: {}, size: {}".format(self.found, self.x, self.y, self.size)


class Camera:

    def __init__(self, ip, port, online):
        self.online = online
        self.img_count = 0
        if online:
            self.camProxy = ALProxy("ALVideoDevice", ip, port)
            resolution = 1  # 320x240
            colorSpace = 11   # RGB

            self.unsub()
            self.videoClient = self.camProxy.subscribe(
                "python_client", resolution, colorSpace, 5)
        self.set_camera("CameraTop")

    def set_camera(self, id):
        self.active_camera=id
        if self.online:
            if id == "CameraTop":
                self.camProxy.setActiveCamera(0)
            else:
                self.camProxy.setActiveCamera(1)

    def unsub(self):
        if self.online:
            subs = self.camProxy.getSubscribers()
            print(subs)
            for sub in subs:
                self.camProxy.unsubscribe(sub)

    def get_image(self):
        if not self.online:
            return Image.open(offline_pic)
            # return Image.open("./cur_2.png")

        naoImage = self.camProxy.getImageRemote(self.videoClient)
        imageWidth = naoImage[0]
        imageHeight = naoImage[1]
        array = naoImage[6]

        # Create a PIL Image from our pixel array.
        im = Image.frombytes("RGB", (imageWidth, imageHeight), array)
        im.save("./cur_" + str(self.img_count) + ".png")
        self.img_count += 1
        print("got image")
        return im

    def detect(self):
        im = self.get_image()
        bim = pil_to_base64(im)
        print("Sending image")

        res = requests.post("http://localhost:5000/analyse", data=bim)
        data = res.json()

        if DISPLAY_IMG:
            display(im, data)

        if data["found"] == False:
            return None

        print("Recv response", data)

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
        steps_deg = 15
        step_size = np.deg2rad(steps_deg)
        for cur_angle in range(0, 360, steps_deg):
            print("Starting find ball", cur_angle)
            dec = self.cam.detect()
            print(dec)
            if dec.found:
                self.tts.say("I found the ball!")
                return dec

            # time.sleep(1)
            self.motionProxy.moveTo(0, 0, step_size)
            self.motionProxy.waitUntilMoveIsFinished()

        return None

    def get_coords_nao_space_manual(self, dec):
        currentCamera=self.cam.active_camera
        print(self.cam.active_camera)
        x_img_angle, y_img_angle, _ = dec.get_angles()

        z_angle, y_angle = x_img_angle*-1, y_img_angle*-1
        if self.online:
            # Get current camera position in NAO space.
            print("getTrans")
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
        robotToLandmarkRotation=robotToCamera*cameraToLandmarkRotationTransform
        start_point=almath.Position3D(0.,0.,0.)
        end_point=almath.Position3D(1.,0.,0.)
        trans_start_point=robotToLandmarkRotation*start_point
        trans_end_point=robotToLandmarkRotation*end_point

        ball_z_pos=0.05
        plane = sympy.Plane(sympy.Point3D(0,0,ball_z_pos),sympy.Point3D(1,0,ball_z_pos),sympy.Point3D(0,1,ball_z_pos))
        trans_start_point=sympy.Point3D(trans_start_point.toVector())
        trans_end_point=sympy.Point3D(trans_end_point.toVector())
        line = sympy.Line3D(trans_start_point,direction_ratio=trans_end_point-trans_start_point)
        intr = plane.intersection(line)

        intersection =np.array(intr[0],dtype=float)
        return (intersection[0], intersection[1], intersection[2]), z_angle,y_angle

    def get_coords_nao_space(self, dec, get_angles_by_position=False, dist_offset=0, dist_fac=1):
        # Compute distance to landmark.
        # higher image angle on x/y axis -> lower z/y angle robot rotation
        if not get_angles_by_position:
            x_img_angle, y_img_angle, dist = dec.get_angles()
        else:
            x_img_angle, y_img_angle, dist = dec.get_angles_by_position()

        dist -= dist_offset
        dist *= dist_fac
        print(np.rad2deg(x_img_angle), np.rad2deg(y_img_angle), dist)

        z_angle, y_angle = x_img_angle*-1, y_img_angle*-1
        currentCamera=self.cam.active_camera

        if self.online:
            # Get current camera position in NAO space.
            transform = self.motionProxy.getTransform(currentCamera, 2, True)
            with open('motionProxyTransform_' + currentCamera + '.pickle', 'wb') as f:
                pickle.dump(transform, f, protocol=pickle.HIGHEST_PROTOCOL)
        else:
            with open('motionProxyTransform_' + currentCamera + '.pickle', "rb") as f:
                transform = pickle.load(f)

        transformList = almath.vectorFloat(transform)
        robotToCamera = almath.Transform(transformList)

        # Compute the rotation to point towards the landmark.
        cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(
            0, y_angle, z_angle)

        # Compute the translation to reach the landmark.
        cameraToLandmarkTranslationTransform = almath.Transform(dist, 0, 0)

        # Combine all transformations to get the landmark position in NAO space.
        robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform * \
            cameraToLandmarkTranslationTransform
        x = robotToLandmark.r1_c4
        y = robotToLandmark.r2_c4
        z = robotToLandmark.r3_c4
        return (x, y, z), z_angle

    def reset(self):
        # self.postureProxy.goToPosture("Crouch", 0.8)
        if self.online:
            self.motionProxy.rest()

    def kick(self):
        kick.kick()

    def move_head(self, y_angle,z_angle):
        # self.motionProxy.sti
        #self.motionProxy.angleInterpolationWithSpeed("HeadYaw", y_angle, 0.5)
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
        (x, y, z), theta,y_angle = self.get_coords_nao_space_manual(
            dec)
        self.move_head(y_angle,theta)
        print("Ball position relative to robot feet (forward: {0}m, left: {1}m, top: {2}m, left_rotation: {3}°)".format(
            np.round(x, 2), np.round(y, 2), np.round(z, 2), np.round(np.rad2deg(theta), 2)))
        self.motionProxy.moveTo(0, 0, theta)
        self.motionProxy.waitUntilMoveIsFinished()
        return True


    def move_to_ball(self,dist_fac=1,x_offset=0):
        dec = self.cam.detect()
        if dec is None:
            return (False,None)
        
        (x, y, z), theta,y_angle = self.get_coords_nao_space_manual(
            dec)
        print(np.rad2deg(theta),np.rad2deg(y_angle))
        print("Ball position relative to robot feet (forward: {0}m, left: {1}m, top: {2}m, left_rotation: {3}°)".format(
            np.round(x, 2), np.round(y, 2), np.round(z, 2), np.round(np.rad2deg(theta), 2)))
        print("angle",np.rad2deg(CAMERA_FOV_V/2+y_angle),np.rad2deg(y_angle))
        
        self.move_head(CAMERA_FOV_V/1.5-y_angle,theta)
        dec = self.cam.detect()
        self.motionProxy.moveTo((x-x_offset)*dist_fac, y*dist_fac, 0)
        self.motionProxy.waitUntilMoveIsFinished()
        return True,x-x_offset
    
    def start(self):
        # dec = self.find_ball_body()
        # if dec is None:
        #     self.tts.say("I couldn't find the ball!")
        #     return


        while True:
            r_res = self.rotate_to_ball()
            m_res,_ = self.move_to_ball(dist_fac=0.5)
            if not r_res or not m_res:
                break
        # print("Standing infront of the ball")
        self.cam.set_camera("CameraBottom")
        rob.motionProxy.angleInterpolationWithSpeed(
        "HeadPitch", np.deg2rad(20), 0.5)



        # Get final ball position
        # TODO: Adjust offset parameter for kick
        offset = 0
        while True:
            dec = self.cam.detect()
            if dec is None:
                self.tts.say("Lost the ball")
                break
            else:
                # Move to Ball Position
                r_res=self.rotate_to_ball()

                m_res,x_dist=self.move_to_ball(dist_fac=0.5,x_offset=0.15)
                if not m_res or not r_res:
                    break
                if x_dist<=0.16:
                    self.kick()
                    break 
                #self.move_to_ball_manual(dist_offset=0.5)
                #if x < 0.25:
                #    break
                #break
                #self.motionProxy.moveTo(x, y, 0)
                #self.motionProxy.waitUntilMoveIsFinished()

        

        


if __name__ == "__main__":
    robotIp = "10.0.7.101"  # Replace here with your NAOqi's IP address.
    robotPort = 9559

    rob = Robot(robotIp, robotPort, online=True)

    rob.motionProxy.angleInterpolationWithSpeed(
        "HeadPitch", np.deg2rad(20), 0.5)
    rob.start()
    rob.cam.unsub()
    rob.reset()

    # dec = rob.find_ball_body()
    # print(dec)
    # rob.reset()
    # rob.motionProxy.moveInit()
    # rob.start_head_tracking()

    # print(dec)
    #print(rob.get_coords_nao_space(dec, "CameraTop"))

    # while True:
    #     try:
    #         dec = rob.cam.detect()

    #         (x, y, z), theta = rob.get_coords_nao_space(dec, "CameraTop")
    #         print("Ball position relative to robot feet (forward: {0}m, left: {1}m, top: {2}m, left_rotation: {3}°)".format(
    #             np.round(x, 2), np.round(y, 2), np.round(z, 2), np.round(np.rad2deg(theta), 2)))

    #         print("get_angles_by_position")
    #         (x, y, z), theta = rob.get_coords_nao_space(dec, "CameraTop", True)
    #         print("Ball position relative to robot feet (forward: {0}m, left: {1}m, top: {2}m, left_rotation: {3}°)".format(
    #             np.round(x, 2), np.round(y, 2), np.round(z, 2), np.round(np.rad2deg(theta), 2)))
    #     except Exception as e:
    #         print(e)

    #     cv2.waitKey(0)

    # rob.start()
    # rob.reset()
