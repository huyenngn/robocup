from pickle import TRUE
import naoqi
from naoqi import ALProxy
import time
import motion
import time
import almath
import numpy as np

IP = "10.0.7.101"
PORT = 9559


def find_ball(tts, motion, posture, tracker):

    # Say Hi
    tts.setLanguage("English")
    tts.say("woof")

    # First, wake up.
    motion.wakeUp()

    fractionMaxSpeed = 0.8
    # Go to posture stand
    posture.goToPosture("StandInit", fractionMaxSpeed)

    # Add target to track.
    targetName = "RedBall"
    diameterOfBall = 0.06
    tracker.registerTarget(targetName, diameterOfBall)

    # set mode
    mode = "Move"
    tracker.setMode(mode)

    # Then, start tracker.
    tracker.track(targetName)

    print("ALTracker successfully started, now show a red ball to robot!")
    print("Use Ctrl+c to stop this script.")

    try:
        while c:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Interrupted by user")
        print("Stopping...")

    # Stop tracker, go to posture Sit.
    tracker.stopTracker()
    tracker.unregisterAllTargets()
    # posture.goToPosture("Sit", fractionMaxSpeed)
    # motion.rest()

    print("ALTracker stopped.")


def computePath(proxy, effector, frame):
    dx = 0.05                 # translation axis X (meters)
    dz = 0.05                 # translation axis Z (meters)
    dwy = 5.0*almath.TO_RAD    # rotation axis Y (radian)

    useSensorValues = False

    path = []
    currentTf = []
    try:
        currentTf = proxy.getTransform(effector, frame, useSensorValues)
    except Exception as e:
        print ("This example is not allowed on this robot.", e)
        exit()

    # 1
    targetTf = almath.Transform(currentTf)
    targetTf *= almath.Transform(-dx, 0.0, dz)
    targetTf *= almath.Transform().fromRotY(dwy)
    path.append(list(targetTf.toVector()))

    # 2
    targetTf = almath.Transform(currentTf)
    targetTf *= almath.Transform(dx, 0.0, dz)
    path.append(list(targetTf.toVector()))

    # 3
    path.append(currentTf)

    return path


def kick(motionProxy, postureProxy):
    ''' Example of a whole body kick
    Warning: Needs a PoseInit before executing
             Whole body balancer must be inactivated at the end of the script
    '''

    # Wake up robot
    motionProxy.wakeUp()

    # Send robot to Stand Init
    postureProxy.goToPosture("StandInit", 0.5)

    # Activate Whole Body Balancer
    isEnabled = True
    motionProxy.wbEnable(isEnabled)

    # Legs are constrained fixed
    stateName = "Fixed"
    supportLeg = "Legs"
    motionProxy.wbFootState(stateName, supportLeg)

    # Constraint Balance Motion
    isEnable = True
    supportLeg = "Legs"
    motionProxy.wbEnableBalanceConstraint(isEnable, supportLeg)

    # Com go to LLeg
    supportLeg = "LLeg"
    duration = 2.0
    motionProxy.wbGoToBalance(supportLeg, duration)

    # RLeg is free
    stateName = "Free"
    supportLeg = "RLeg"
    motionProxy.wbFootState(stateName, supportLeg)

    # RLeg is optimized
    effector = "RLeg"
    axisMask = 63
    frame = motion.FRAME_WORLD

    # Motion of the RLeg
    times = [2.0, 2.7, 4.5]

    path = computePath(motionProxy, effector, frame)

    motionProxy.transformInterpolations(effector, frame, path, axisMask, times)

    # Example showing how to Enable Effector Control as an Optimization
    isActive = False
    motionProxy.wbEnableEffectorOptimization(effector, isActive)

    # Com go to LLeg
    # supportLeg = "RLeg"
    # duration = 2.0
    # motionProxy.wbGoToBalance(supportLeg, duration)

    # RLeg is free
    # stateName = "Free"
    # supportLeg = "LLeg"
    # motionProxy.wbFootState(stateName, supportLeg)

    # effector = "LLeg"
    # path = computePath(motionProxy, effector, frame)
    # motionProxy.transformInterpolations(effector, frame, path, axisMask, times)

    time.sleep(1.0)

    # Deactivate Head tracking
    isEnabled = False
    motionProxy.wbEnable(isEnabled)

    # send robot to Pose Init
    postureProxy.goToPosture("StandInit", 0.3)

    # Go to rest position
    motionProxy.rest()

if __name__ == '__main__':

    print("Connecting to", IP, "with port", PORT)
    tts = ALProxy("ALTextToSpeech", IP, PORT)
    mtn = ALProxy("ALMotion", IP, PORT)
    posture = ALProxy("ALRobotPosture", IP, PORT)
    tracker = ALProxy("ALTracker", IP, PORT)
    
    find_ball(tts, mtn, posture, tracker)
    mo
    kick(mtn, posture)
