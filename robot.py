import sys

import requests 

from naoqi import ALProxy
from PIL import Image
import numpy as np
import math
import almath

import base64
from io import BytesIO

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
BALL_SIZE = 0.15 # in meters

class Camera:

    def __init__(self, ip, port):
        self.camProxy = ALProxy("ALVideoDevice", ip, port)
        resolution = 2  # 320x240
        colorSpace = 11   # RGB
        self.camProxy.setActiveCamera(0)
        self.videoClient = self.camProxy.subscribe("python_client", resolution, colorSpace, 5)
    
    def get_image(self):
        naoImage = self.camProxy.getImageRemote(self.videoClient)
        imageWidth = naoImage[0]
        imageHeight = naoImage[1]
        array = naoImage[6]

        # Create a PIL Image from our pixel array.
        im = Image.frombytes("RGB", (imageWidth, imageHeight), array)
        return im

    def get_coords(self):
        # im = self.get_image()

        # bim = pil_to_base64(im)

        # res = requests.post("http://localhost:5000/analyse", json={"b64_img": bim})
        # data = res.json()

        return True, 0, 50, np.deg2rad(20)

        return data["found"], data["x"], data["y"], data["size"]

    def has_ball(self):
        return False

    def get_coords_angle(self):
        found, x, y, size = self.get_coords()

        dist = BALL_SIZE / ( 2 * math.tan( size / 2))
        # Convert to radians
        x_angle = np.deg2rad(x * CAMERA_FOV_H / IMAGE_WIDTH)
        y_angle = np.deg2rad(y * CAMERA_FOV_V / IMAGE_HEIGHT)

        return found, x_angle, y_angle, dist
    
class Robot:

    def __init__(self, ip, port):
        self.motionProxy  = ALProxy("ALMotion", ip, port)
        self.postureProxy = ALProxy("ALRobotPosture", ip, port)
        self.tts = ALProxy("ALTextToSpeech", ip, port)
        self.cam = Camera(ip, port)

        self.head_yaw_min_max = [np.deg2rad(-119.5), np.deg2rad(119.5)]

        self.postureProxy.goToPosture("StandInit", 0.5)

        self.tts.say("Initialized")
    
    def find_ball_body(self):
        # self.tts.say("Looking for ball")

        steps_deg = 60
        step_size = np.deg2rad(steps_deg)
        for _ in range(0, 360, steps_deg):
            if self.cam.has_ball():
                # self.tts.say("I found the ball!")
                return True
            
            self.motionProxy.moveTo(0, 0, step_size)
            self.motionProxy.waitUntilMoveIsFinished()
        
        return False

    def find_ball_head(self, head_angle_step = np.deg2rad(15)):
        # self.tts.say("Looking for ball")

        self.motionProxy.setStiffnesses("Head", 1.0)

        self.motionProxy.angleInterpolationWithSpeed("HeadYaw", self.head_yaw.min_max[0], 0.5)

        self.turned = False
        while True:
            curAngle = self.motionProxy.getAngles("HeadYaw", True)[0]

            if self.cam.has_ball():
                # self.tts.say("I found the ball!")
                return curAngle

            # Ball is not in current view
            # Turn around 180 degrees and look again using head
            if curAngle + head_angle_step > self.head_yaw_min_max[1]:

                # If we have already turned around and the ball is not in the view, there is no ball
                if self.turned:
                    return None


                self.motionProxy.moveToward(0, 0, 1.0, [["Frequency", 1]])
                self.motionProxy.waitUntilMoveIsFinished()
                self.turned = True
                self.motionProxy.angleInterpolationWithSpeed("HeadYaw", self.head_yaw.min_max[0], 0.5)

            # Move Head slightly
            self.motionProxy.angleInterpolationWithSpeed("HeadYaw", curAngle + head_angle_step, 0.5)

    def get_coords_nao_space(self):
        # Compute distance to landmark.
        found, x_angle, y_angle, dist = self.cam.get_coords_angle()

        currentCamera = "CameraTop"

        # Get current camera position in NAO space.
        transform = self.motionProxy.getTransform(currentCamera, 2, True)
        transformList = almath.vectorFloat(transform)
        robotToCamera = almath.Transform(transformList)

        # Compute the rotation to point towards the landmark.
        cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, x_angle, y_angle)

        # Compute the translation to reach the landmark.
        cameraToLandmarkTranslationTransform = almath.Transform(dist, 0, 0)

        # Combine all transformations to get the landmark position in NAO space.
        robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform * cameraToLandmarkTranslationTransform
        return robotToLandmark, x_angle
        return robotToLandmark, robotToLandmark.getTheta()


    def move_to_ball(self):
        # self.tts.say("Moving to ball")

        # Get the position of the landmark in NAO space.
        robotToLandmark, theta = self.get_coords_nao_space()

        self.motionProxy.moveTo(robotToLandmark[0], robotToLandmark[1], theta, [["Frequency", 1]])
        self.motionProxy.waitUntilMoveIsFinished()
    
if __name__ == "__main__":
    robotIp = "10.0.7.101"  # Replace here with your NAOqi's IP address.
    robotPort = 9559

    rob = Robot(robotIp, robotPort)

    # 1. Test find ball body
    # rob.find_ball_body()

    # rob.motionProxy.moveTo(0.2, 0.2, np.deg2rad(45))
    # rob.motionProxy.waitUntilMoveIsFinished()


    # 2.1 Test coords
    # found, x, y, dist = rob.cam.get_coords_angle()
    # print (found, np.rad2deg(x), np.rad2deg(y), dist)


    # 2.2 Test coords nao space
    robotToLandmark, x_angle = rob.get_coords_nao_space()
    print "x " + str(robotToLandmark.r1_c4) + " (in meters)"
    print "y " + str(robotToLandmark.r2_c4) + " (in meters)"
    print "z " + str(robotToLandmark.r3_c4) + " (in meters)"
    rob.postureProxy.goToPosture("Crouch", 0.8)

    # 3. Test move to ball
    # rob.move_to_ball()



    # Final
    # rob.find_ball_body()
    # rob.move_to_ball()