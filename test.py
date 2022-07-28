# -*- encoding: UTF-8 -*-
# Get an image from NAO. Display it and save it using PIL.

import sys
import time

# Python Image Library
from PIL import Image

from naoqi import ALProxy


def showNaoImage(IP, PORT):
  """
  First get an image from Nao, then show it on the screen with PIL.
  """

  camProxy = ALProxy("ALVideoDevice", IP, PORT)
  posture = ALProxy("ALRobotPosture", IP, PORT)
  resolution = 2  # VGA
  colorSpace = 11   # RGB
  camProxy.setActiveCamera(0)

  videoClient = camProxy.subscribe("python_client", resolution, colorSpace, 5)


  posture.goToPosture("StandInit", fractionMaxSpeed)

  i = 0
  while True:
    i += 1
    naoImage = camProxy.getImageRemote(videoClient)

    # Now we work with the image returned and save it as a PNG  using ImageDraw
    # package.

    # Get the image size and pixel array.
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]

    # Create a PIL Image from our pixel array.
    im = Image.frombytes("RGB", (imageWidth, imageHeight), array)

    # Save the image.
    im.save("./save_bot/top_camImage" + str(i) + ".png", "PNG")
    print "./save_bot/top_camImage" + str(i) + ".png"



if __name__ == '__main__':
  IP = "10.0.7.100"  # Replace here with your NAOqi's IP address.
  PORT = 9559

  # Read IP address from first argument if any.
  if len(sys.argv) > 1:
    IP = sys.argv[1]

  naoImage = showNaoImage(IP, PORT)