
from tracemalloc import start
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.cm as cm

import argparse



parser = argparse.ArgumentParser(
    description='Plot only one logfile')
parser.add_argument('logfile', metavar='LOGFILE', type=str,
                    help='Exact Path to logfile')
parser.add_argument('startsec', metavar='START[s]', type=int,
                    help='Start of plot')
parser.add_argument('endsec',metavar='END[s]', type=int,
                    help='End of plot')
parser.add_argument('--type',metavar='IMAGETYPE', type=str, default='png',
                    help='png, svg')

args = parser.parse_args()


if args.logfile.rfind('/logfile') == -1:
    parser.error("no valid logfile. e.g. /home/shoulder/P1I1D1/logfile")
else:
    logfile = args.logfile
    currLog = args.logfile[:args.logfile.rfind('logfile')]

startSec = args.startsec
endSec = args.endsec
imagetype = args.type



jointTargets = [0]*19

def info(line, before, after):
    return line[line.rindex(before)+len(before):line.rfind(after)]

def readin(logfile):
    global P, I, D, bagName, startRecAt, endRecAt, logRate, simRate, realTimeSpeed 
    data = []
    element = [0, [], [], [], [], []]
    with open(logfile) as f:
        #read in first line
        line = f.readline()
        bagName = info(line,'Bagfile: ',', Length:')
        line = f.readline()
        startRecAt = float(info(line,'Recorded from ','s for '))
        playDuration = float(info(line,'s for ','s until '))
        endRecAt = startRecAt + playDuration
        while line.find('P=') == -1:
            line = f.readline()
        P=int(info(line,'P=',' I=')) 
        I=int(info(line,' I=',' D=')) 
        D=int(info(line,' D=','\n')) 
        while line.find('Simulation ran at ') == -1:
            line = f.readline()
        realTimeSpeed=float(info(line, 'Simulation ran at ', '% real time speed'))
        while line.find('Simulation rate:        ') == -1: 
            line = f.readline()
        simRate = int(info(line,'Simulation rate:        ', ' (equivalent'))
        while line.find('Logging rate:           ') == -1:
            line = f.readline()
        logRate = int(info(line,'Logging rate:           ','\n')) 

        while line.find('RECORDING START') == -1:
            line = f.readline()
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
    fig_shoulder_spl.savefig(currLog+'/SPL_shoulder_%s_s%ie%i_%r.%s' %
                             (PIDvalues, startSec, endSec, simRate, imagetype), dpi=100)

    fig_shoulder_f.suptitle('Forces - P%i I%i D%i\nBagfile %s from %is for %is - Logging Rate %i - Simulation Rate %i - %.2f%% real time speed\n' %
                            (P, I, D, bagName, startRecAt, endRecAt, logRate, simRate, realTimeSpeed), fontsize=34)
    fig_shoulder_f.savefig(currLog+'/F_shoulder_%s_s%ie%i_%r.%s' %
                           (PIDvalues, startSec, endSec, simRate, imagetype), dpi=100)
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
    fig_elbow_spl.savefig(currLog+'/SPL_elbow_%s_s%ie%i_%r.%s' %
                           (PIDvalues, startSec, endSec, simRate, imagetype), dpi=100)

    fig_elbow_f.suptitle('Forces - P%i I%i D%i\nBagfile %s from %is for %is - Logging Rate %i - Simulation Rate %i - %.2f%% real time speed\n' %
                          (P, I, D, bagName, startRecAt, endRecAt, logRate, simRate, realTimeSpeed), fontsize=34)
    fig_elbow_f.savefig(currLog+'/F_elbow_%s_s%ie%i_%r.%s' %
                         (PIDvalues, startSec, endSec, simRate, imagetype), dpi=100)
    
    plt.close('all')

    # Plot for joints
    fig_joint_elbow, axs_joint_elbow = plt.subplots(1, constrained_layout=True)
    fig_joint_elbow.set_size_inches(40, 15)
    fig_joint_shoulder, axs_joint_shoulder = plt.subplots(1, constrained_layout=True)
    fig_joint_shoulder.set_size_inches(40, 15)

    joint_colors = cm.hsv(np.linspace(0, 1, len(jointTargets)))

    # Shoulder_left
    shoulder_colors = ['red', 'blue', 'yellow']
    for i in range(2,-1,-1):
        id = range(11,14)[i]
        axs_joint_shoulder.plot(t, [t[id]*57.2958 for t in j_targets], '-.', color=shoulder_colors[i])
        axs_joint_shoulder.plot(t, [s[id]*57.2958 for s in j_states], color=shoulder_colors[i])


    # Elbow_left 
    for id in range(14,15):
        axs_joint_elbow.plot(t, [t[id]*57.2958 for t in j_targets], '-.', color = 'purple')
        axs_joint_elbow.plot(t, [s[id]*57.2958 for s in j_states], color = 'purple')

    axs_joint_shoulder.set_title('Axis 0 (red), Axis 1 (blue), Axis 2 (yellow)', fontsize=34)
    axs_joint_shoulder.set_xlabel('Time [s]', fontsize=26)
    axs_joint_shoulder.set_ylabel('Angle [deg]', fontsize=26)
    axs_joint_shoulder.tick_params(labelsize=20)

    axs_joint_elbow.set_title('Elbow axis', fontsize=34)
    axs_joint_elbow.set_xlabel('Time [s]', fontsize=26)
    axs_joint_elbow.set_ylabel('Angle [deg]', fontsize=26)
    axs_joint_elbow.tick_params(labelsize=20)

    fig_joint_elbow.suptitle('Setpoint Angles (--) and Angles (―) - P%i I%i D%i\nBagfile %s from %is for %is - Logging Rate %i - Simulation Rate %i - %.2f%% real time speed\n' %
                          (P, I, D, bagName, startRecAt, endRecAt, logRate, simRate, realTimeSpeed), fontsize=34)
    fig_joint_elbow.savefig(currLog+'/EJ_%s_s%ie%i_%r.%s' %
                         (PIDvalues, startSec, endSec, simRate, imagetype), dpi=100)
    fig_joint_shoulder.suptitle('Setpoint Angles (--) and Angles (―) - P%i I%i D%i\nBagfile %s from %is for %is - Logging Rate %i - Simulation Rate %i - %.2f%% real time speed\n' %
                          (P, I, D, bagName, startRecAt, endRecAt, logRate, simRate, realTimeSpeed), fontsize=34)
    fig_joint_shoulder.savefig(currLog+'/SJ_%s_s%ie%i_%r.%s' %
                         (PIDvalues, startSec, endSec, simRate, imagetype), dpi=100)

    plt.close('all')


plotData = readin(logfile)

if startSec == 0:
    startSec = startRecAt


if startSec<startRecAt or endSec > endRecAt:
    print('ERROR. Recording starts at %.2fs and end at %.2fs. Choose values in between or 0, 0 for the full recording.' %(startRecAt, endRecAt) )
else:
    generatePlot(plotData, currLog, startSec, endSec)