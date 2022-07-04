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
import io
import yaml


parser = argparse.ArgumentParser(
    description='Simulate Roboy in MuJoCo. Use --help argument to see options')

parser.add_argument('P', type=int,
                    help='Proportional Gain')
parser.add_argument('I', type=int,
                    help='Integral Gain')
parser.add_argument('D', type=int,
                    help='Derivational Gain')
parser.add_argument('simRate', type=int, help='Simulation Rate')

parser.add_argument('--render',  metavar='RATE', type=int,
                    help='Render rate. Default: Off')
parser.add_argument('--log', metavar='RATE', type=int,
                    help='Logging rate. Default: Off')
parser.add_argument('--plot', action='store_true',
                    help='Requires --log: Plotting after Finish. Default: Off')

group = parser.add_mutually_exclusive_group(required=True)
group.add_argument('--bag', metavar=('BAGFILE', 'START[s]', 'DURATION[s]'), type=str, nargs=3,
                   help='Specify a bagfile inside test_data. Use DURATION[s] = 0 for playing until the end')
group.add_argument('--dur', metavar=('DURATION[s]'), type=float,
                   help='If --bag not used: How long to run the simulation.')
# ['shoulder_right_axis0', 'shoulder_right_axis1', 'shoulder_right_axis2', 'elbow_right_axis0', 'elbow_right_axis1', 'wrist_right_axis0', 'wrist_right_axis1', 'wrist_right_axis2', 'head_axis0', 'head_axis1', 'head_axis2', 'shoulder_left_axis0', 'shoulder_left_axis1', 'shoulder_left_axis2', 'elbow_left_axis0', 'elbow_left_axis1', 'wrist_left_axis0', 'wrist_left_axis1', 'wrist_left_axis2']
parser.add_argument('--ctrlOnly', metavar='JOINTNAME', type=str, choices=['left', 'right', 'elbow_left', 'elbow_right', 'shoulder_left', 'shoulder_left_axis0', 'shoulder_left_axis1', 'shoulder_left_axis2', 'shoulder_right', 'shoulder_right_axis0', 'shoulder_right_axis1',
                    'shoulder_right_axis2', 'wrist_left', 'wrist_left_axis0', 'wrist_left_axis1', 'wrist_left_axis2', 'wrist_right', 'wrist_right_axis0', 'wrist_right_axis1', 'wrist_right_axis2', 'head', 'head_axis0', 'head_axis1', 'head_axis2'], help='Fix all But the specified Joint. Default: Off')
parser.add_argument('--folder', metavar='NAME', type=str,
                    help='Specify additional Parent folder where Log is placed inside.')
parser.add_argument('--model', metavar='MODELFILE', default="model.xml", type=str,
                    help='Select another model than the standard model.xml inside mujoco_models.')
parser.add_argument('--speed', metavar='FACTOR', default="1.0", type=float,
                    help='Run simulation and bagfile at more than real time.')

args = parser.parse_args()

if args.plot and (args.log is None):
    parser.error("--plot requires generation of logfile by --log")


P, I, D, simRate = args.P, args.I, args.D, args.simRate

if args.render:
    render = True
    renderRate = args.render
else:
    render = False
    renderRate = 1

if args.log:
    log = True
    logRate = args.log
else:
    log = False
    logRate = 0

if args.bag:
    playBag = True
    bagFileStr = args.bag[0]
    startRecAt = float(args.bag[1])
    playDuration = float(args.bag[2])
    # Compensate for .bag ending provided or not
    if bagFileStr.find('.bag') == -1:
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
    addCtrlOnlyName = '_ctrlOnly_'+controlOnlyJoint
else:
    controlOnlyJoint = False
    addCtrlOnlyName = ''

if args.folder:
    parentFolder = args.folder
else:
    parentFolder = ""

if args.plot:
    plot = True
else:
    plot = False
    
if args.model:
    if args.model.find('.xml') == -1:
        modelname = args.model + '.xml'
    else:
        modelname = args.model

if args.speed != 1:
    speedup = True
    speed = args.speed
    simRate = simRate * speed
    renderRate = renderRate * speed
    logRate = logRate * speed
else:
    speedup = False
    speed = args.speed


modelPath = "/code/mujoco_models/"
modelXML = modelPath + modelname


bagLocation = '/code/test_data/'


nMotors = 38
simStep = 0
dt = 1/simRate


allButRenderFinished = False
startAll = False




# Logging is started after Second 1
logStartStep = simRate
if startRecAt >= 1:
    startBagAt = startRecAt-1
else:
    startBagAt = 0
    startRecAt = 1

if log:
    global logvariable
    logvariable = ''
    logEveryN = int(simRate/logRate)

    if parentFolder:
        logDir = os.path.join(os.getcwd(), 'logfiles', parentFolder, '%s-s%i-u%i_F%i%s' %
                            (bagName, startRecAt, playDuration, simRate, addCtrlOnlyName))
    else:
        logDir = os.path.join(os.getcwd(), 'logfiles', '%s-s%i-d%i-r%i-ctrl_%s' %
                            (bagName, startRecAt, playDuration, simRate, addCtrlOnlyName))
                            
    currLog = os.path.join(logDir, 'P%iI%iD%i' % (P, I, D))
    if not os.path.exists(currLog):
        os.makedirs(currLog)


if controlOnlyJoint:
    tree = ET.parse(modelXML)
    root = tree.getroot()
    root.find("option").set("timestep", str(1/simRate))
    joint_found = [element for element in root.iter()
                   if element.tag == "joint"]
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


if playBag:
    # Read out the duration of the bagfile
    info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', '%s%s' % (
        bagLocation, bagFile)], stdout=subprocess.PIPE).communicate()[0], Loader=yaml.BaseLoader)
    bagDuration = float(info_dict['duration'])
    # if no Duration was specified OR the specified duration is longer than the bagfile
    if playDuration == 0 or playDuration > bagDuration - startBagAt:
        # Play until the end
        playDuration = bagDuration - startBagAt
    endRecAt = startRecAt+playDuration
    print('\nPlaying %s of length %.2fs from Second %.2f for %.2fs until Second %.2f\n' % (
        bagFile, bagDuration, startBagAt, playDuration, endRecAt))


sim = mujoco_py.MjSim(model)
if render:
    viewer = mujoco_py.MjViewer(sim)


jointNames = [
    ['shoulder_right_axis0', 'shoulder_right_axis1', 'shoulder_right_axis2',
     'elbow_right_axis0', 'elbow_right_axis1',
     'wrist_right_axis0', 'wrist_right_axis1', 'wrist_right_axis2'],
    ['head_axis0', 'head_axis1', 'head_axis2'],
    ['shoulder_left_axis0', 'shoulder_left_axis1', 'shoulder_left_axis2',
     'elbow_left_axis0', 'elbow_left_axis1',
     'wrist_left_axis0', 'wrist_left_axis1', 'wrist_left_axis2']
]

jname2id = dict()

startIdx = 0
for body in jointNames:
    jname2id.update({name:startIdx+id for id, name in enumerate(body)})
    startIdx += len(body)

jointTargets = np.array([0. for _ in range(len(jname2id))])

tendonNamesInner = []
for i in range(37):
    tendonNamesInner.append('motor%i' % i)
tendonNames = [tendonNamesInner]


Kp = float(P) * np.ones(nMotors)
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
    global logvariable
    if step > logStartStep:
        logvariable += (str(rospy.Time.now()) + '\n' + ' '.join(map(str, setpoint)) + '\n' + 
                        ' '.join(map(str, sim.data.ten_length)) + '\n' + ' '.join(map(str, sim.data.actuator_force)) + '\n' + 
                        ' '.join(map(str, jointTargets)) + '\n' + ' '.join(map(str, sim.data.qpos)) + '\n\n')


def tendonTargetCb(data):

    global setpoint

    control_id = list(data.global_id)

    setpoint[control_id] = -np.array(data.setpoint)


def jointTargetCb(data):

    global jointTargets

    for name, pos in zip(list(data.name), list(data.position)):
        jointTargets[jname2id[name]] = pos


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

        # JointState is the wrong message type for this. But there is no direct Forece measurement message type.
        msg = JointState()
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
    stopStep = simRate*(playDuration+1)

    errorPrev = np.zeros(nMotors)
    errorInt = np.zeros(nMotors)

    # global variables that are evaluated in the end
    global maxTimeLogDur, maxControlDur, maxLogDur, maxSimDur, maxTotalDur, maxTotalDurStep, AllButRenderEndTime, avgTimeLogDur, avgControlDur, avgLogDur, avgSimDur, avgTotalDur, allButRenderRunning
    maxTimeLogDur, maxControlDur, maxLogDur, maxSimDur, maxTotalDur, maxTotalDurStep, AllButRenderEndTime, totalTimeLogDur, totalControlDur, totalLogDur, totalSimDur, totalTotalDur = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    # local variables. Xtime is time AFTER operation X
    startTime, durLogTime, controlTime, logTime, simTime = 0, 0, 0, 0, 0

    while not startAll:
        pass

    allButRenderRunning = True

    while simStep <= stopStep:

        oldStartTime = startTime
        startTime = time.time_ns()//1000

        # DURATIONS from prev. iteration
        curTimeLogDur = durLogTime - oldStartTime
        totalTimeLogDur += curTimeLogDur
        curControlDur = controlTime - durLogTime
        totalControlDur += curControlDur
        curLogDur = logTime - controlTime
        totalLogDur += curLogDur
        curSimDur = simTime - logTime
        totalSimDur += curSimDur
        curTotalDur = simTime - oldStartTime
        totalTotalDur += curTotalDur

        # set the maximum
        if (curTimeLogDur > maxTimeLogDur):
            maxTimeLogDur = curTimeLogDur
        if (curControlDur > maxControlDur):
            maxControlDur = curControlDur
        if (curLogDur > maxLogDur):
            maxLogDur = curLogDur
        if (curSimDur > maxSimDur):
            maxSimDur = curSimDur
        if (curTotalDur > maxTotalDur):
            maxTotalDur = curTotalDur
            maxTotalDurStep = simStep

        durLogTime = time.time_ns()//1000

        # CONTROL calculation
        error = setpoint - sim.data.ten_length
        errorInt += Ki * error * dt
        force = Kp * error + Kd * (error - errorPrev) / dt + errorInt

        CtrlIdx = np.where(setpoint != 0)[0]
        sim.data.ctrl[CtrlIdx] = force[CtrlIdx]

        errorPrev = error

        controlTime = time.time_ns()//1000

        # LOGGING
        if log:
            if simStep % logEveryN == 0:
                logToRam(simStep)

        logTime = time.time_ns()//1000

        # SIMULATION
        sim.step()
        simStep += 1

        simTime = time.time_ns()//1000

        RosSimRate.sleep()

    AllButRenderEndTime = time.time_ns()//1000

    [avgTimeLogDur, avgControlDur, avgLogDur, avgSimDur, avgTotalDur] = [
        x/(stopStep - 1) for x in [totalTimeLogDur, totalControlDur, totalLogDur, totalSimDur, totalTotalDur]]

    allButRenderRunning = False


def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()


def readin(logvariable):
    data = []
    element = [0, [], [], [], [], []]
    f = io.StringIO(logvariable)
    while True:
        timestampline = f.readline().strip()
        if not timestampline:
            break
        else:
            # first line of recording: Timestamp
            element[0] = int(timestampline)
            element[1] = [float(k)
                          for k in f.readline().strip().split()]  # setpoints
            element[2] = [float(k)
                          for k in f.readline().strip().split()]  # lengths
            element[3] = [float(k)
                          for k in f.readline().strip().split()]  # forces
            element[4] = [float(k)
                          for k in f.readline().strip().split()]  # joint targets
            element[5] = [float(k)
                          for k in f.readline().strip().split()]  # joint states
            data.append(element.copy())
            f.readline()  # read in empty line
    return data


def generatePlot(plotData, currLog, startSec, endSec=0):

    PIDvalues = 'P%iI%iD%i' % (P, I, D)

    xmin = min([m[0] for m in plotData])
    t = [(x[0]-xmin)*1e-9 for x in plotData]
    setpoints = [m[1] for m in plotData]
    lengths = [m[2] for m in plotData]
    forces = [m[3] for m in plotData]
    j_targets = [m[4] for m in plotData]
    j_states = [m[5] for m in plotData]

    startindex = next(t[0] for t in enumerate(t) if t[1] >= startSec)
    if endSec:
        endindex = next(t[0] for t in enumerate(t) if t[1] >= endSec)
    else:
        endindex = len(plotData)-1


    # Initialize Data for Plot
    t = t[startindex:endindex]
    setpoints = setpoints[startindex:endindex]
    lengths = lengths[startindex:endindex]
    forces = forces[startindex:endindex]
    j_targets = j_targets[startindex:endindex]
    j_states = j_states[startindex:endindex]

    # Plot the Shoulder
    shoulder_indexgroup = [
        [0, 1, 2, 3, 4, 5, 6, 7], [0, 2, 4, 6], [1, 3, 5, 7]]

    shoulder_colors = cm.rainbow(
        np.linspace(0, 1, len(shoulder_indexgroup[0])))

    fig_shoulder_spl, axs_shoulder_spl = plt.subplots(
        len(shoulder_indexgroup), constrained_layout=True)
    fig_shoulder_spl.set_size_inches(40, 15*len(shoulder_indexgroup))

    fig_shoulder_f, axs_shoulder_f = plt.subplots(
        len(shoulder_indexgroup), constrained_layout=True)
    fig_shoulder_f.set_size_inches(40, 15*len(shoulder_indexgroup))

    for s in range(len(shoulder_indexgroup)):
        for u in shoulder_indexgroup[s]:
            axs_shoulder_spl[s].plot(t, [s[u] for s in setpoints], '-.',
                                     color=shoulder_colors[shoulder_indexgroup[0].index(u)])
            axs_shoulder_spl[s].plot(t, [l[u] for l in lengths], linewidth=3,
                                     color=shoulder_colors[shoulder_indexgroup[0].index(u)])
            axs_shoulder_f[s].plot(t, [f[u] for f in forces], linewidth=3,
                                   color=shoulder_colors[shoulder_indexgroup[0].index(u)])

    axs_shoulder_spl[0].set_title('All shoulder Tendons', fontsize=34)
    axs_shoulder_spl[1].set_title(
        'Shoulder Tendons %s' % shoulder_indexgroup[1], fontsize=34)
    axs_shoulder_spl[2].set_title(
        'Shoulder Tendons %s' % shoulder_indexgroup[2], fontsize=34)

    axs_shoulder_f[0].set_title('All shoulder Tendons', fontsize=34)
    axs_shoulder_f[1].set_title('Shoulder Tendons %s' %
                                shoulder_indexgroup[1], fontsize=34)
    axs_shoulder_f[2].set_title('Shoulder Tendons %s' %
                                shoulder_indexgroup[2], fontsize=34)

    for a in axs_shoulder_spl:
        a.set_xlabel('Time [s]', fontsize=26)
        a.set_ylabel('Length [m]', fontsize=26)
        a.tick_params(labelsize=20)

    for a in axs_shoulder_f:
        a.set_xlabel('Time [s]', fontsize=26)
        a.set_ylabel('Force [N]', fontsize=26)
        a.tick_params(labelsize=20)

    fig_shoulder_spl.suptitle('Setpoints (--) and Lenghts (―) - P%i I%i D%i\nBagfile %s from %is for %is - Logging Rate %i - Simulation Rate %i - %.2f%% real time speed\n' %
                              (P, I, D, bagName, startRecAt, endRecAt, logRate, simRate, realTimeSpeed), fontsize=34)
    fig_shoulder_spl.savefig(currLog+'/SPL_shoulder_%s_s%ie%i_%r.svg' %
                             (PIDvalues, startSec, endSec, simRate), dpi=100)

    fig_shoulder_f.suptitle('Forces - P%i I%i D%i\nBagfile %s from %is for %is - Logging Rate %i - Simulation Rate %i - %.2f%% real time speed\n' %
                            (P, I, D, bagName, startRecAt, endRecAt, logRate, simRate, realTimeSpeed), fontsize=34)
    fig_shoulder_f.savefig(currLog+'/F_shoulder_%s_s%ie%i_%r.svg' %
                           (PIDvalues, startSec, endSec, simRate), dpi=100)
    plt.close('all')

    # Plot the Elbow
    fig_elbow_spl, axs_elbow_spl = plt.subplots(1, constrained_layout=True)
    fig_elbow_spl.set_size_inches(40, 15)

    axs_elbow_spl.plot(t, [s[16] for s in setpoints], '-.', color='green')
    axs_elbow_spl.plot(t, [l[16] for l in lengths],
                        linewidth=3, color='green')

    axs_elbow_spl.plot(t, [s[17] for s in setpoints], '-.', color='red')
    axs_elbow_spl.plot(t, [l[17] for l in lengths], linewidth=3, color='red')

    axs_elbow_spl.set_title('Biceps (red) and Triceps (green)', fontsize=34)
    axs_elbow_spl.set_xlabel('Time [s]', fontsize=26)
    axs_elbow_spl.set_ylabel('Length [m]', fontsize=26)
    axs_elbow_spl.tick_params(labelsize=20)

    fig_elbow_f, axs_elbow_f = plt.subplots(1, constrained_layout=True)
    fig_elbow_f.set_size_inches(40, 15)

    axs_elbow_f.plot(t, [f[16] for f in forces], linewidth=3, color='green')

    axs_elbow_f.plot(t, [f[17] for f in forces], linewidth=3, color='red')

    axs_elbow_f.set_title('Biceps (red) and Triceps (green)', fontsize=34)
    axs_elbow_f.set_xlabel('Time [s]', fontsize=26)
    axs_elbow_f.set_ylabel('Length [m]', fontsize=26)
    axs_elbow_f.tick_params(labelsize=20)

    fig_elbow_spl.suptitle('Setpoints (--) and Lenghts (―) - P%i I%i D%i\nBagfile %s from %is for %is - Logging Rate %i - Simulation Rate %i - %.2f%% real time speed\n' %
                            (P, I, D, bagName, startRecAt, endRecAt, logRate, simRate, realTimeSpeed), fontsize=34)
    fig_elbow_spl.savefig(currLog+'/SPL_elbow_%s_s%ie%i_%r.svg' %
                           (PIDvalues, startSec, endSec, simRate), dpi=100)

    fig_elbow_f.suptitle('Forces - P%i I%i D%i\nBagfile %s from %is for %is - Logging Rate %i - Simulation Rate %i - %.2f%% real time speed\n' %
                          (P, I, D, bagName, startRecAt, endRecAt, logRate, simRate, realTimeSpeed), fontsize=34)
    fig_elbow_f.savefig(currLog+'/F_elbow_%s_s%ie%i_%r.svg' %
                         (PIDvalues, startSec, endSec, simRate), dpi=100)
    
    plt.close('all')

    # Plot for joints
    fig_joint, axs_joint = plt.subplots(2, constrained_layout=True)
    fig_joint.set_size_inches(40, 15*2)

    joint_colors = cm.hsv(np.linspace(0, 1, len(jointTargets)))

    # Shoulder_left
    for id in range(11,14):
        axs_joint[0].plot(t, [t[id]*57.2958 for t in j_targets], '-.', color=joint_colors[id])
        axs_joint[0].plot(t, [s[id]*57.2958 for s in j_states], color=joint_colors[id])

    axs_joint[0].set_title('Shoulder left', fontsize=34)

    # Elbow_left 
    for id in range(14,16):
        axs_joint[1].plot(t, [t[id]*57.2958 for t in j_targets], '-.', color=joint_colors[id])
        axs_joint[1].plot(t, [s[id]*57.2958 for s in j_states], color=joint_colors[id])

    axs_joint[1].set_title('Elbow left', fontsize=34)

    for i in range(2):
        axs_joint[i].set_xlabel('Time [s]', fontsize=26)
        axs_joint[i].set_ylabel('Angle [deg]', fontsize=26)
        axs_joint[i].tick_params(labelsize=20)

    fig_joint.suptitle('Joints - P%i I%i D%i\nBagfile %s from %is for %is - Logging Rate %i - Simulation Rate %i - %.2f%% real time speed\n' %
                          (P, I, D, bagName, startRecAt, endRecAt, logRate, simRate, realTimeSpeed), fontsize=34)
    fig_joint.savefig(currLog+'/J_%s_s%ie%i_%r.svg' %
                         (PIDvalues, startSec, endSec, simRate), dpi=100)

    plt.close('all')


def printHeader():

    if playBag:
        print('Bagfile: %s, Length: %.1fs.' % (bagName, bagDuration))
        print('Recorded from %.1fs for %.1fs until %.1fs.' %
              (startRecAt, playDuration, (startRecAt+playDuration)))
    if controlOnlyJoint:
        print('Only joint %s was enabled.' % controlOnlyJoint)
    print('\nP=%i I=%i D=%i\n' % (P, I, D))
    print("Real Time Duraion:      %.3fs" % playDuration)
    # -1 because first second omitted
    print("Duarion for Simulation: %.3fs" %
          ((AllButRenderEndTime-simStartTime)*1e-6-1))
    print('Simulation ran at %.3f%% real time speed\n' % realTimeSpeed)
    if realTimeSpeed < 99:
        print("ERROR. \nThe simulation ran more than one percent below real time. \nTry to decrease the render rate in order to free up resources for the simulation subthread.\n")

    print('Simulation rate:        %i (equivalent to a Timestep of %.2fms)' %
          (simRate, dt*1000))
    if log:
        print('Logging rate:           %i' % (logRate))
    else:
        print('Logging rate:           OFF')
    if render:
        print('Render rate:            %i\n' % (renderRate))
    else:
        print('Render rate:            OFF\n')

    print("Maximum Time Durations")
    print("Total:                  %.2fms at step %i (must not be larger than %.2fms)" % (
        maxTotalDur*1e-3, maxTotalDurStep, 1000/simRate))
    print("Control:                %.2fms" % (maxControlDur*1e-3))
    print("Log:                    %.2fms" % (maxLogDur*1e-3))
    if (maxLogDur*1e-3) > (1000/simRate):
        print("ERROR. \nLogging took longer than expected. This is probably because the logvariable became too big. Reduce logging rate or decrease bag play duration.\n")

    print("Simulation:             %.2fms\n" % (maxSimDur*1e-3))

    print("Average Time Durations")
    print("Total:                  %.2fms" % (avgTotalDur*1e-3))
    print("Control:                %.2fms" % (avgControlDur*1e-3))
    print("Log:                    %.2fms" % (avgLogDur*1e-3))
    print("Simulation:             %.2fms\n" % (avgSimDur*1e-3))


if __name__ == '__main__':

    topicRoot = "/roboy/pinky"
    try:
        rospy.get_master()
    except:
        print('ros not running')

    tendonTargetSub = rospy.Subscriber(
        f"{topicRoot}/middleware/MotorCommand", MotorCommand, tendonTargetCb)

    jointTargetSub = rospy.Subscriber(
        f"{topicRoot}/simulation/joint_targets", JointState, jointTargetCb)

    # Subthread that does Everything but Rendering
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
    simStartTime = time.time_ns()//1000

    # Define loop End Time, Start the bagfile
    # StartBagAt is one second before StartRecAt. PlayDuration is one Second more.
    loopEndTime = time.time_ns()//1000 + int(1e6*(playDuration+1))
    if playBag:
        playerProc = subprocess.Popen(
            ['rosbag', 'play', bagFile, '-s %f' % startBagAt, '-u %f' % (playDuration+1)], cwd=bagLocation)

    # Render for the expected time
    while time.time_ns()//1000 < loopEndTime:
        if render:
            viewer.render()
        RosRenderRate.sleep()

    # ensure everything finished
    while allButRenderRunning:
        print(allButRenderRunning)
        RosRenderRate.sleep()

    if playBag:
        kill(playerProc.pid)
    

    realTimeSpeed = 100*playDuration / \
        ((AllButRenderEndTime-simStartTime)*1e-6-1)

    # Create the logfile
    if log:
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

    print()
    printHeader()

    if plot:
        print('Plotting...')
        plotData = readin(logvariable)
        generatePlot(plotData, currLog, startRecAt)

    os._exit(1)
