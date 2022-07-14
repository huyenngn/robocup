import naoqi
from naoqi import ALProxy
import time
import motion
import time
import almath

IP = "10.0.7.101" 
PORT = 9559

def find_ball():
    print("Connecting to", IP, "with port", PORT)
    tts = ALProxy("ALTextToSpeech", IP, PORT)
    motion = ALProxy("ALMotion", IP, PORT)
    posture = ALProxy("ALRobotPosture", IP, PORT)
    tracker = ALProxy("ALTracker", IP, PORT)
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
    old_target = tracker.getTargetPosition(2)
    counter = 0
    try:
        while counter < 10:
            if abs(old_target - tracker.getTargetPosition(2)) < 0.1:
                counter = counter + 1
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

    # Example showing how to Enable Effector Control as an Optimization
    isActive     = False
    motionProxy.wbEnableEffectorOptimization(effector, isActive)

    # Com go to LLeg
    supportLeg = "RLeg"
    duration   = 2.0
    motionProxy.wbGoToBalance(supportLeg, duration)

    # RLeg is free
    stateName  = "Free"
    supportLeg = "LLeg"
    motionProxy.wbFootState(stateName, supportLeg)

    effector = "LLeg"
    path = computePath(motionProxy, effector, frame)
    motionProxy.transformInterpolations(effector, frame, path, axisMask, times)

    time.sleep(1.0)

    # Deactivate Head tracking
    isEnabled = False
    motionProxy.wbEnable(isEnabled)

    # send robot to Pose Init
    postureProxy.goToPosture("StandInit", 0.3)

    # Go to rest position
    motionProxy.rest()

def kick_right():

    names = list()
    times = list()
    keys = list()
    
    names.append("LHipYawPitch")
    times.append([ 2.60000, 5.20000])
    keys.append([ [ -0.00456, [ 3, -0.86667, 0.00000], [ 3, 0.86667, 0.00000]], [ 0.01538, [ 3, -0.86667, 0.00000], [ 3, 0.00000, 0.00000]]])
    
    names.append("LHipRoll")
    times.append([ 2.60000, 5.20000])
    keys.append([ [ -0.11202, [ 3, -0.86667, 0.00000], [ 3, 0.86667, 0.00000]], [ -0.11202, [ 3, -0.86667, 0.00000], [ 3, 0.00000, 0.00000]]])
    
    names.append("LHipPitch")
    times.append([ 2.60000, 5.20000])
    keys.append([ [ -0.20253, [ 3, -0.86667, 0.00000], [ 3, 0.86667, 0.00000]], [ -0.20406, [ 3, -0.86667, 0.00000], [ 3, 0.00000, 0.00000]]])
    
    names.append("LKneePitch")
    times.append([ 2.60000, 5.20000])
    keys.append([ [ 0.83761, [ 3, -0.86667, 0.00000], [ 3, 0.86667, 0.00000]], [ 0.84528, [ 3, -0.86667, 0.00000], [ 3, 0.00000, 0.00000]]])
    
    names.append("LAnklePitch")
    times.append([ 2.60000, 5.20000])
    keys.append([ [ -0.45862, [ 3, -0.86667, 0.00000], [ 3, 0.86667, 0.00000]], [ -0.46016, [ 3, -0.86667, 0.00000], [ 3, 0.00000, 0.00000]]])
    
    names.append("LAnkleRoll")
    times.append([ 2.60000, 5.20000])
    keys.append([ [ 0.23466, [ 3, -0.86667, 0.00000], [ 3, 0.86667, 0.00000]], [ 0.23466, [ 3, -0.86667, 0.00000], [ 3, 0.00000, 0.00000]]])
    
    names.append("RHipRoll")
    times.append([ 2.60000, 5.20000])
    keys.append([ [ -0.13810, [ 3, -0.86667, 0.00000], [ 3, 0.86667, 0.00000]], [ -0.13964, [ 3, -0.86667, 0.00000], [ 3, 0.00000, 0.00000]]])
    
    names.append("RHipPitch")
    times.append([ 2.60000, 5.00000, 5.20000])
    keys.append([ [ -0.28528, [ 3, -0.86667, 0.00000], [ 3, 0.80000, 0.00000]], [ -0.56549, [ 3, -0.80000, 0.10950], [ 3, 0.06667, -0.00913]], [ -0.64117, [ 3, -0.06667, 0.00000], [ 3, 0.00000, 0.00000]]])
    
    names.append("RKneePitch")
    times.append([ 2.60000, 5.00000, 5.20000])
    keys.append([ [ 1.02007, [ 3, -0.86667, 0.00000], [ 3, 0.80000, 0.00000]], [ 1.91812, [ 3, -0.80000, 0.00000], [ 3, 0.06667, 0.00000]], [ 0.97558, [ 3, -0.06667, 0.00000], [ 3, 0.00000, 0.00000]]])
    
    names.append("RAnklePitch")
    times.append([ 2.60000, 5.00000, 5.20000])
    keys.append([ [ -0.69955, [ 3, -0.86667, 0.00000], [ 3, 0.80000, 0.00000]], [ -0.46251, [ 3, -0.80000, -0.17606], [ 3, 0.06667, 0.01467]], [ -0.12736, [ 3, -0.06667, 0.00000], [ 3, 0.00000, 0.00000]]])
    
    names.append("RAnkleRoll")
    times.append([ 2.60000, 5.20000])
    keys.append([ [ -0.00311, [ 3, -0.86667, 0.00000], [ 3, 0.86667, 0.00000]], [ -0.00311, [ 3, -0.86667, 0.00000], [ 3, 0.00000, 0.00000]]])

    try:
        motion = ALProxy("ALMotion", IP, PORT)
        motion.angleInterpolationBezier(names, times, keys)
    except BaseException as err:
        print(err)

if __name__ == '__main__':


    find_ball()
    kick_right()

