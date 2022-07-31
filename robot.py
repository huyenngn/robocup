# -*- coding: utf-8 -*-
from __future__ import print_function
import sys

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
import kick

def pil_to_base64(pil_img):
    im_file = BytesIO()
    pil_img.save(im_file, format="PNG")
    im_bytes = im_file.getvalue()  # im_bytes: image in binary format.
    im_b64 = base64.b64encode(im_bytes)

    return im_b64


CAMERA_FOV_H = np.deg2rad(60.97)
CAMERA_FOV_V = np.deg2rad(47.64)
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
# IMAGE_WIDTH = 640
# IMAGE_HEIGHT = 480
BALL_SIZE = 0.1 # in meters
offline_pic = "./output_raw/170.png"
offline_pic = "./output_raw/127.png"
offline_pic = "./output_raw/102.png"



class DetectionResult:
    def __init__(self, data):
        self.found = data['found']
        self.x = (IMAGE_WIDTH // 2 - data['x'] )*-1
        self.y = (IMAGE_HEIGHT // 2 - data['y'])
        #normal coordinate system:
        #--------------
        #|            |
        #|     ↑      |
        #|      →     |
        #|            |
        #--------------
        self.size = data['size']

    def get_angles(self):
        size_angle = self.size * CAMERA_FOV_H / IMAGE_WIDTH
        dist = BALL_SIZE / ( 2 * math.tan( size_angle / 2))
        # Convert to radians
        x_angle = self.x * CAMERA_FOV_H / IMAGE_WIDTH
        y_angle = self.y * CAMERA_FOV_V / IMAGE_HEIGHT

        return x_angle, y_angle, dist
    
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
            self.camProxy.setActiveCamera(0)
            self.videoClient = self.camProxy.subscribe("python_client", resolution, colorSpace, 5)
        
    def set_camera(self, id):
        if id == "CameraTop":
            self.camProxy.setActiveCamera(0)
        else:
            self.camProxy.setActiveCamera(1)
    
    def get_image(self):
        if not self.online:
            return Image.open(offline_pic)
            #return Image.open("./cur_2.png")

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

        print("Recv response", data)

        return DetectionResult(data)
    
class Robot:

    def __init__(self, ip, port, online=True):
        self.online = online
        self.cam = Camera(ip, port, online)

        if online:
            self.motionProxy  = ALProxy("ALMotion", ip, port)
            self.postureProxy = ALProxy("ALRobotPosture", ip, port)
            self.tts = ALProxy("ALTextToSpeech", ip, port)
            # self.postureProxy.goToPosture("StandInit", 0.5)
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


    def get_coords_nao_space(self, dec, currentCamera="CameraTop"):
        # Compute distance to landmark.
        #higher image angle on x/y axis -> lower z/y angle robot rotation
        x_img_angle, y_img_angle, dist = dec.get_angles()
        z_angle, y_angle=x_img_angle*-1,y_img_angle*-1


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
        cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, y_angle, z_angle)

        # Compute the translation to reach the landmark.
        cameraToLandmarkTranslationTransform = almath.Transform(dist, 0, 0)


        # Combine all transformations to get the landmark position in NAO space.
        robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform * cameraToLandmarkTranslationTransform
        x = robotToLandmark.r1_c4
        y = robotToLandmark.r2_c4
        z = robotToLandmark.r3_c4
        return (x, y, z), z_angle

    def reset(self):
        # self.postureProxy.goToPosture("Crouch", 0.8)
        self.motionProxy.rest()
    
    def kick(self):
        kick.kick()

    def move_head(self, dec):
        #self.motionProxy.sti
        self.motionProxy.angleInterpolationWithSpeed("HeadYaw", dec.x, 0.5)
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", dec.y, 0.5)
    
    def start_head_tracking(self):
        while True:
            dec_result = self.cam.detect()
            if dec_result.found:
                self.move_head(dec_result)
            time.sleep(1)
    
    def start(self):
        dec = self.find_ball_body()
        if dec is None:
            self.tts.say("I couldn't find the ball!")
            return
        
        # Move to Ball Position
        (x, y, z), theta = self.get_coords_nao_space(dec, "CameraTop")
        print(x, y, z, theta)
        #TODO: Adjust head
        # self.move_head(dec)
        self.motionProxy.moveTo(0, 0, -theta)
        self.motionProxy.moveTo(x, 0, 0)
        self.motionProxy.waitUntilMoveIsFinished()
        return

        # TODO Schauen ob hier fine adjustments noetig sind
        # time.sleep(1)
        # while self.motionProxy.moveIsActive():
        #     #TODO: Adjust head
        #     dec = self.cam.detect()
        #     if dec is None:
        #         self.tts.say("Lost the ball")
        #     else:
        #         self.move_head(dec)
        #         self.motionProxy.stopMove()
        #         (x, y, z), theta = self.get_coords_nao_space(dec, "CameraTop")
        #         self.motionProxy.moveTo(x, y, theta, [["Frequency", 1]])
        #         time.sleep(1)

        self.motionProxy.waitUntilMoveIsFinished()
        print("Standing infront of the ball")

        # Fine adjustment for kick
        self.cam.set_camera("CameraBottom")
        dec = self.cam.detect()
        if dec is None:
            self.tts.say("Can't see the ball bot!")
            return

        # Get final ball position
        # TODO: Adjust offset parameter for kick
        offset = 0.1
        (x, y, z), theta = self.get_coords_nao_space(dec, "CameraBottom")
        self.motionProxy.moveTo(x, y + offset, theta, [["Frequency", 1]])
        self.motionProxy.waitUntilMoveIsFinished()

        self.kick()
    
if __name__ == "__main__":
    robotIp = "10.0.7.100"  # Replace here with your NAOqi's IP address.
    robotPort = 9559

    rob = Robot(robotIp, robotPort, online=False)

    # rob.motionProxy.angleInterpolationWithSpeed("HeadPitch", np.deg2rad(30), 0.5)

    # rob.reset()
    # rob.start()
    # dec = rob.find_ball_body()
    # print(dec)
    # rob.reset()
    # rob.motionProxy.moveInit()
    # rob.start_head_tracking()

    dec = rob.cam.detect()
    print(dec)
    #print(rob.get_coords_nao_space(dec, "CameraTop"))
    print(rob.get_coords_nao_space(dec, "CameraBottom"))


    # rob.start()
    # rob.reset()
