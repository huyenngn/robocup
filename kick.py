import argparse
import motion
import time
import almath
from naoqi import ALProxy

IP = "10.0.7.101" 
PORT = 9559

def computePath(proxy, effector, frame):
    dx      = 0.05                 # translation axis X (meters)
    dz      = 0.05                 # translation axis Z (meters)
    dwy     = 5.0*almath.TO_RAD    # rotation axis Y (radian)

    useSensorValues = False

    path = []
    currentTf = []
    try:
        currentTf = proxy.getTransform(effector, frame, useSensorValues)
    except Exception as e:
        print ("This example is not allowed on this robot.", e)
        exit()

    # 1
    targetTf  = almath.Transform(currentTf)
    targetTf *= almath.Transform(-dx, 0.0, dz)
    targetTf *= almath.Transform().fromRotY(dwy)
    path.append(list(targetTf.toVector()))

    # 2
    targetTf  = almath.Transform(currentTf)
    targetTf *= almath.Transform(dx, 0.0, dz)
    path.append(list(targetTf.toVector()))

    # 3
    path.append(currentTf)

    return path


def kick():
    ''' Example of a whole body kick
    Warning: Needs a PoseInit before executing
             Whole body balancer must be inactivated at the end of the script
    '''

    motionProxy  = ALProxy("ALMotion", IP, PORT)
    postureProxy = ALProxy("ALRobotPosture", IP, PORT)

    # Wake up robot
    motionProxy.wakeUp()

    # Send robot to Stand Init
    postureProxy.goToPosture("StandInit", 0.5)

    # Activate Whole Body Balancer
    isEnabled  = True
    motionProxy.wbEnable(isEnabled)

    # Legs are constrained fixed
    stateName  = "Fixed"
    supportLeg = "Legs"
    motionProxy.wbFootState(stateName, supportLeg)

    # Constraint Balance Motion
    isEnable   = True
    supportLeg = "Legs"
    motionProxy.wbEnableBalanceConstraint(isEnable, supportLeg)

    # Com go to LLeg
    supportLeg = "LLeg"
    duration   = 2.0
    motionProxy.wbGoToBalance(supportLeg, duration)

    # RLeg is free
    stateName  = "Free"
    supportLeg = "RLeg"
    motionProxy.wbFootState(stateName, supportLeg)

    # RLeg is optimized
    effector = "RLeg"
    axisMask = 63
    frame    = motion.FRAME_WORLD

    # Motion of the RLeg
    times   = [2.0, 2.7, 4.5]

    path = computePath(motionProxy, effector, frame)

    motionProxy.transformInterpolations(effector, frame, path, axisMask, times)

    # send robot to Pose Init
    postureProxy.goToPosture("StandInit", 0.3)

if __name__ == "__main__":

    kick()