import numpy as np
import mujoco_py
import rospy
import os
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from roboy_middleware_msgs.msg import MotorCommand
import subprocess
import psutil
import sys
import time
import threading
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import matplotlib.cm as cm

import argparse
from importlib.resources import path



parser = argparse.ArgumentParser(description='Simulate Roboy in MuJoCo.')

parser.add_argument('P', type=int, help='Proportional Gain')
parser.add_argument('I', type=int, help='Derivational Gain')
parser.add_argument('D', type=int, help='Integral Gain')
parser.add_argument('simRate', type=int, help='Simulation Rate')

parser.add_argument('--render',  metavar='RATE', type=int, help='Render frequency. Default: Off')
parser.add_argument('--log', metavar='RATE', type=int, help='Logging frequency. Default: Off')

group = parser.add_mutually_exclusive_group(required=True)
group.add_argument('--bag', metavar=('BAGFILE','START[s]', 'DURATION[s]'), type=str, nargs = 3, help='Specify a bagfile inside test_data')
group.add_argument('--dur', metavar=('DURATION[s]'), type=float, help='If --bag not used: How long to run the simulation')
parser.add_argument('--ctrlOnly', metavar='JOINTNAME', type=str, help='Fix all But the specified Joint. Default: Off')
parser.add_argument('--plot', action='store_true', help='Plotting after Finish. Default: Off.')
parser.add_argument('--addName', metavar='TEXT', type=str, help='Additional Information added to Logfolder Name')
parser.add_argument('--model', metavar='MODELFILE', default = "model.xml", type=str, help='Select another model than the standard model.xml inside mujoco_models')


args = parser.parse_args()
P, I, D, simRate = args.P, args.I, args.D, args.simRate
if args.render:
    render = True
    renderRate = args.render
else:
    render = False
    renderRate = 1

if args.log:
    logging = True
    loggingRate = args.log
else:
    logging = False

if args.bag:
    playBag = True
    bagFileStr = args.bag[0]
    startBagAt = float(args.bag[1])
    playDuration = float(args.bag[2])
    # Compensate for .bag ending provided or not
    if bagFileStr.find('.bag')==-1:
        bagFile = bagFileStr + '.bag'
    else:
        bagFile = bagFileStr
    bagName = bagFile[:bagFile.find('.bag')]
else:
    playBag = False
    bagName = ""
    bagFile = ""
    playDuration = float(args.dur)

if args.ctrlOnly:
    controlOnlyJoint = args.ctrlOnly
else:
    controlOnlyJoint = False

if args.addName:
    addName = args.addName
else:
    addName = ""

if args.plot:
    plot = True
else:
    plot = False




# Information about code structure
# Doing Simulation, log, sontrol, etc. AND rendering in one main loop does not work. 
# Assume a rate of 100, or 10ms per iterion.
# Most render iterations are fast enough (say 5ms) and ROS rate.sleep() tries to wait for the remaining time
# however, some render iterations take longer than allowed. Like every 20th takes 30ms. But this is enough to always slow down the loop to a bit below real time, even for very low rates.
# Running the rendering from a subthread does not work
# So the solution was to move everything else to a subthread and only render in the main loop.
# As long as the render rate is low enough (and there is enough power remaining for the subthread) the simulation can run at a very high rate.


modelPath = "/code/mujoco_models/"
modelXML = modelPath + args.model

bagLocation = '/code/test_data'



nMotors = 38
simStep = 0
dt = 1/simRate



stopThreads = False
startAll = False






logDir = os.getcwd()+'/logfiles/%s_%s-s%i-u%i_F%i' % (addName, bagName , startBagAt, playDuration, simRate)
currLog = os.path.join(logDir, 'P%iI%iD%i' % (P, I, D))
if not os.path.exists(currLog):
    os.makedirs(currLog)

if logging:
    global logvariable
    logvariable = ''
    logEveryN = int(simRate/loggingRate)
# Start logging after 1s
logStartStep = simRate


if controlOnlyJoint:
    tree = ET.parse(modelXML)
    root = tree.getroot()
    root.find("option").set("timestep", str(1/simRate))
    joint_found = [element for element in root.iter() if element.tag == "joint"]
    for joint in joint_found:
        if "range" in joint.attrib and controlOnlyJoint not in joint.attrib["name"]:
            joint.set("range", "-0.001 0.0")

    xmlstr = ET.tostring(root, encoding='unicode', method='xml')

    with open(modelPath + "tmp.xml", "w") as f:
        f.write(xmlstr)
    model = mujoco_py.load_model_from_path(modelPath + "tmp.xml")
else:
    tree = ET.parse(modelXML)
    root = tree.getroot()
    root.find("option").set("timestep", str(1/simRate))
    model = mujoco_py.load_model_from_path(modelXML)


sim = mujoco_py.MjSim(model)
if render:
    viewer = mujoco_py.MjViewer(sim)



jointNames = [
    ['shoulder_right_axis0', 'shoulder_right_axis1', 'shoulder_right_axis2', 
    'elbow_right_axis0', 'elbow_right_axis1', 
    'wrist_right_axis0', 'wrist_right_axis1', 'wrist_right_axis2'],
    ['head_axis0', 'head_axis1', 'head_axis2'], 
    ['shoulder_left_axis0','shoulder_left_axis1','shoulder_left_axis2',
    'elbow_left_axis0', 'elbow_left_axis1',
    'wrist_left_axis0', 'wrist_left_axis1', 'wrist_left_axis2']
]

tendonNamesInner = []
for i in range(37):
    tendonNamesInner.append('motor%i' % i)
tendonNames = [tendonNamesInner]



Kp = float(P)* np.ones(nMotors)
Kd = float(D) * np.ones(nMotors)
Ki = float(I) * np.ones(nMotors)
setpoint = np.zeros(nMotors)

# Head
Kp[np.arange(20, 25)] = 500.0
Kd[np.arange(20, 25)] = 10.0

# In the current verion this is not used. Logging to the harddrive direclly can slow down the loop by a bit. If Very long trajectories are recoreded, the log system can be switched.
def log(logfile):
    print(rospy.Time.now(), file=logfile)
    print(*setpoint, file=logfile)
    print(*sim.data.ten_length, file=logfile)
    print(*sim.data.actuator_force, file=logfile)
    print('', file=logfile)


def logToRam(step):
    if step > logStartStep:
        global logvariable
        logvariable += (str(rospy.Time.now()) + '\n' + ' '.join(map(str,setpoint)) + '\n' + ' '.join(map(str,sim.data.ten_length)) + '\n' + ' '.join(map(str,sim.data.actuator_force)) + '\n\n' )


def tendonTargetCb(data):

    global setpoint

    control_id = list(data.global_id)

    setpoint[control_id] = -np.array(data.setpoint)


def publishJoitStates(publisher, joint_states):

    startIdx = 0

    for body in jointNames:
            
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        msg.name = body
        msg.position = joint_states[startIdx:startIdx+len(body)]
        msg.velocity = [0] * len(body)
        msg.effort = [0] * len(body)
        
        publisher.publish(msg)
        startIdx += len(body)


def publishTendonForces(publisher, tendon_control, tendon_forces):

    startIdx = 0

    for tendon in tendonNames:
            
        msg = JointState()  #JointState is the wrong message type for this. But there is no direct Forece measurement message type.
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        msg.name = tendon
        msg.position = tendon_control[startIdx:startIdx+len(tendon)]
        msg.velocity = tendon_forces[startIdx:startIdx+len(tendon)]
        msg.effort = [0] * len(tendon)
        
        publisher.publish(msg)

        rospy.loginfo(msg)


        startIdx += len(tendon)


def allButRender():
    simStep = 0
    stopStep = playDuration*simRate

    errorPrev = np.zeros(nMotors)
    errorInt = np.zeros(nMotors)

    # global variables that are evaluated in the end
    global maxTimeLogDur, maxControlDur, maxLogDur, maxSimDur, maxTotalDur, maxTotalDurStep, AllButRenderEndTime
    maxTimeLogDur, maxControlDur, maxLogDur, maxSimDur, maxTotalDur, maxTotalDurStep, AllButRenderEndTime = 0, 0, 0, 0, 0, 0, 0
    # local variables. Xtime is time AFTER operation X
    startTime, durLogTime, controlTime, logTime, simTime = 0, 0, 0, 0, 0



    while not startAll:
        pass

    while simStep<=stopStep:

        oldstarttime = startTime
        startTime = time.time_ns()
            
        # DURATIONS from prev. iteration
        CurrTimeLogDur = durLogTime - oldstarttime
        currcontroldur = controlTime - durLogTime 
        currlogdur = logTime - controlTime
        currsimulationdur = simTime - logTime
        currtotaldur = simTime - oldstarttime

        # set the maximum
        if (CurrTimeLogDur > maxTimeLogDur):
            maxTimeLogDur = CurrTimeLogDur
        if (currcontroldur > maxControlDur):
            maxControlDur = currcontroldur
        if (currlogdur > maxLogDur):
            maxLogDur = currlogdur
        if (currsimulationdur > maxSimDur):
            maxSimDur = currsimulationdur
        if (currtotaldur > maxTotalDur):
            maxTotalDur = currtotaldur
            maxTotalDurStep = simStep

        durLogTime = time.time_ns()
        

        #CONTROL calculation
        error = setpoint - sim.data.ten_length
        errorInt += Ki * error * dt
        force = Kp * error + Kd * (error - errorPrev) / dt + errorInt

        CtrlIdx = np.where(setpoint != 0)[0]
        sim.data.ctrl[CtrlIdx] = force[CtrlIdx]

        errorPrev = error

        controlTime = time.time_ns()


        #LOGGING
        if logging:
            if simStep%logEveryN == 0:
                logToRam(simStep)

        logTime = time.time_ns()


        #SIMULATION
        sim.step()
        simStep += 1

        simTime = time.time_ns()


        RosSimRate.sleep()

    AllButRenderEndTime = time.time_ns()


def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()


def printHeader():
    print('Bagfile: %s' %bagName)
    print('P=%i I=%i D=%i'%(P,I,D))
    if playBag:
        print('Bagfile played from %is for %is.'%(startBagAt,playDuration))
    if controlOnlyJoint:
        print('Only joint %s was enabled.' %controlOnlyJoint)
    print("")
    print("Real Time Duraion:      %.3fs" %playDuration)
    print("Duarion for Simulation: %.3fs\n" %((AllButRenderEndTime-simStartTime)*1e-9) )
    print("Recording started after 1s at step %i\n" %(logStartStep) )
    if (AllButRenderEndTime-simStartTime)*9.9e-10 > playDuration:
        print("\nERROR. \nThe Simulation ran more than one Percent below real time. \nTry to decrease the Render Frequency in order to free up resources for the simulation subthread.\n")

    print('Simulation Frequency:   %i, equivalent to a Timestep of %.2fms,' % (simRate, dt*1000))
    if logging:
        print('Logging Frequency:      %i' % (loggingRate))
    else:
        print('Logging Frequency:      OFF')
    if render:
        print('Render Frequency:       %i\n' % (renderRate))
    else:
        print('Render Frequency:       OFF\n')

    print("Maximum Time Durations")
    print("Total:                  %.2fms at step %i (must not be larger than %.2fms)" % (maxTotalDur*1e-6, maxTotalDurStep, 1000/simRate))
    print("Control:                %.2fms" % (maxControlDur*1e-6))
    print("Log:                    %.2fms" % (maxLogDur*1e-6))
    print("Simulation:             %.2fms\n" % (maxSimDur*1e-6))



if __name__ == '__main__':

    topicRoot = "/roboy/pinky"
    try:
        rospy.get_master()
    except:
        print('ros not running')


    tendonTargetSub = rospy.Subscriber(f"{topicRoot}/middleware/MotorCommand", MotorCommand, tendonTargetCb)

    # Subthread that does Everzthing but Rendering
    allButRender_thread = threading.Thread(target=allButRender)

    # ROS Publishers are not used in this version
    #pub = rospy.Publisher(f"{topic_root}/external_joint_states", JointState, queue_size=1)
    #pub2 = rospy.Publisher(f"{topic_root}/tendon_forces", JointState, queue_size=1)

    rospy.init_node("mujoco_roboy")
    print("Simulation started!")

    # ROS Rates can only be created after rospy node init
    RosSimRate = rospy.Rate(simRate)
    RosRenderRate = rospy.Rate(renderRate)


    # calculate first simulation step
    sim.step()
    # and load the GUI
    if render:
        viewer.render()
    # to not slow down first few steps

    allButRender_thread.start()
    time.sleep(1)

    # Start simulation
    startAll = True
    simStartTime = time.time_ns()
    

    # Define loop End Time, Start the bagfile
    loopEndTime = time.time_ns() +  int(1e9*playDuration)
    if playBag:
        playerProc = subprocess.Popen(['rosbag', 'play', bagFile, '-s %f' %startBagAt, '-u %f' %playDuration], cwd=bagLocation)

    # Render for the expected time
    while time.time_ns() < loopEndTime:
        if render:
            viewer.render()
        RosRenderRate.sleep()
    
    # ensure everything finished
    time.sleep(1)
    if playBag:
        kill(playerProc.pid)
    stopThreads = True

    # Create the logfile
    if logging:
        # Redirect print output to logfile
        logfile = open('%s/logfile' % currLog, 'w')
        originalStdout = sys.stdout
        sys.stdout = logfile
        # Create the Header
        printHeader()
        # Write the logvariable
        print('\nRECORDING START')
        print(logvariable)
        sys.stdout = originalStdout


    print("\n\nDONE!\n")
    printHeader()


    print("Potential Maximal Simulation Frequency:            %i"%(1e9/maxTotalDur))
    print("Recommended Frequency with with 10%% safety margin: %i\n"%(9e8/maxTotalDur))

    if plot:
        print('Plotting...')


    os._exit(1)