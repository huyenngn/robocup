# RoboCup Ball Tracking

In this project we worked with the nao programmable robot. The goal is to locate a black and white ball on the pitch, move towards it and then kick it.

## Ball Detection

One of the main challenges was to locate the black and white ball. In contrast to the red ball, there are a lot of black and white elements in the background,
which makes it hard to locate the ball. We first developed a own neural network and trained it on both data from the internet and own data which we labeled ourself.
The network was based on the U-Net architecture and was supposed to segment the ball from the background. The network worked quite good, but we also tested the pretrained yoloV5 neural network,
which has a "sports ball" categorie and performs better.

In order to not have any python version problems we decided to run the neural network as an python3 server which our robot can communicate with. 
We send a picture and the server returns the position and the size of the ball.

Example Response:
```json
{ "found": true, "x": 64, "y": 40, "size": 20 }
```

## Procedure

The whole procedure is divided into 4 stages:

### Stage 1: Locating the ball

First the robot tries to locate the ball on the field. For that it rotates counterclockwise and clockwise and looks for the ball.
In each step the robot takes a picture using the top and the bottom camera and sends it to our local analyse server.
The server returns the position and the size of the ball.

### Stage 2: Moving towards the ball

After the ball was located the robot moves towards the ball. To do that we need the position of the ball in nao space.
We first calculate radian positions of the ball from the pixel position in the image. We then calculate the position of the ball,
by calculating the intersection between the floor plane at the height of the center of the ball (5cm) and the vector starting at the camera and pointing to the ball.
To calculate the vector we use the camera transformation matrix and the position of the ball in the image in radians.

We then first adjust the head pitch so the ball is centered vertically and rotate the robot so the ball is centered horizontally.
We then use the `moveTo` function to move the robot to the ball using our calculated positions.

#### Recalculating the position continuously

As the initial position of the robot is not that accurate we decided to take pictures using the top camera of the robot and recalculate the position of the ball, while the robot is moving towards it.
This is challenging as the taking an image using the robot takes some time (around 0.5 to 1s). Thus, after getting the newly calculated position of the ball,
the robot has moved quite a bit from the position it was when the picture was taken. To account for this we calculate the difference between the current position and the position of the ball
when the robot was at the position when the picture was taken, by measuring the time it took and the velocity of the robot.
By doing this we achieve a smooth movement with updates while the robot is moving.

#### Stopping the robot

We either stop the robot when the robot is 30cm away from the ball or when the ball is no longer visible in the top camera.
This could happen because the fov simply does not cover the area the ball is in or because the ball is not detected due to e.g. blur which can happend during movement.
To prevent the robot from stopping to early, we only abort the movement if the ball is not detected for 3 times in a row and the robot is moving. If it is not moving and the ball is not detected,
then we know the ball is no longer visible from the top camera.

### Stage 3: Find adjusting position

After the ball is no longer visible in the top camera, the robot tries to find the ball in the bottom camera.
We then fine adjust the robots position to be in range of 20cm away from the ball using the same approach as described in the previous stage.
We then to one final detection and move the robot 3cm to the left of the ball.

### Stage 4: Kicking the ball

We tried many ways to kick the ball we found on the internet. First we tried a approach which used the balanching capabilities of the robot, to first
balance on one leg and then kicking the ball with the free leg. Unfortunately this approach did not work as in one case the animation was too slow to effectively kick the ball or 
the robot would lose balance. One keyframe approach was able to kick the ball really well, but was inconsistent. Most of the time the robot would fall over during the kick.

Therefore, we decided to use a different approach. We first do a lunge forward with the left leg and then to a step with the right leg which results in a powerful kick. This approach
worked really well and most importantly very consistent.

## Installation

Install the requirements for the server:
```
pip install numpy Flask Pillow
```

and pytorch using the instructions from https://pytorch.org/get-started/locally/


Install the requirements for the robot:

First install the naopy library and the other required libraries for python 2.7
```
pip install numpy opencv-python requests Pillow sympy almath
```

## Usage

First start the image detection server, this requires internet access to download the model.

```bash
cd server && python3 analyse_server.py
```

Then adjust the ip in robot.py and start the robot.

```bash
python2.7 robot.py
```
