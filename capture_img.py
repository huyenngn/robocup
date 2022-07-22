# -*- encoding: UTF-8 -*-
# Get an image from NAO. Display it and save it using PIL.

import sys
import time

# Python Image Library
from PIL import Image

from naoqi import ALProxy

def setupCam(IP, PORT):
  camProxy = ALProxy("ALVideoDevice", IP, PORT)
  resolution = 1  # VGA
  colorSpace = 11   # RGB
  videoClient = camProxy.subscribe("python_client", resolution, colorSpace, 5)
  camProxy.setActiveCamera(0)

  def save_img(idx, cam):
    naoImage = camProxy.getImageRemote(videoClient)

    if naoImage is None:
      print "Recv None"
      return 
    # Get the image size and pixel array.
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]

    # Create a PIL Image from our pixel array.
    im = Image.frombytes("RGB", (imageWidth, imageHeight), array)
    
    save_path = "./save_top/cam_" + str(cam) + "_" + str(idx) + ".png"
    im.save(save_path, "PNG")
    print save_path
  
  return save_img

if __name__ == '__main__':
  IP = "10.0.7.100"  # Replace here with your NAOqi's IP address.
  PORT = 9559

  # posture = ALProxy("ALRobotPosture", IP, PORT)
  # tts = ALProxy("ALTextToSpeech", IP, PORT)
  # tts.say("Hello guys")

  save_cam = setupCam(IP, PORT)

  # posture.goToPosture("StandInit", fractionMaxSpeed)

  i = 0
  while True:
    i += 1

    save_cam(i, 0)

  
  