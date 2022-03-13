#!/usr/bin/env python
# coding: utf-8

import matplotlib.pyplot as plt
import numpy as np
import matplotlib.cm as cm
import os
import sys
from pathlib import Path


bagname = str(input('Bagfile name (no .bag): '))
closeup_start=input('Start closeup Plots at Second (or pass for no closeup): ')
if closeup_start:
    startsec_pruned = int(closeup_start)
    closeup_end=input('end closeup Plots at Second: ')
    endsec_pruned = int(closeup_end)
else:
    startsec_pruned = 0
    endsec_pruned = 0


logdir = 'logfiles'
bagdir = os.path.join(os.getcwd(),logdir,bagname)

plot_closeup = False
if (startsec_pruned < endsec_pruned):
    plot_closeup = True



def readin(logfile):
    data = []
    element =[0,[],[],[]]
    with open(logfile) as f:
        line = f.readline() #read in first line
        while line != 'RECORDING START\n': #if not start of recording
            line = f.readline()
            
        if line == 'RECORDING START\n':#reording starts in next line
            while line:
                line = f.readline().strip()
                if line == "":
                    break
                else:
                    element[0] = int(line) #first line of recording: Timestamp
                    element[1] = [float(k) for k in f.readline().strip().split()] #setpoints
                    element[2] = [float(k) for k in f.readline().strip().split()] #lengths
                    element[3] = [float(k) for k in f.readline().strip().split()] #forces
                    data.append(element.copy())
                    line = f.readline() #read in empty line
    return data


def PLOT(bagdir, PIDvalues, startsec=None, endsec=None):

    currentdir = os.path.join(bagdir,PIDvalues)
    Path(currentdir).mkdir(parents=True, exist_ok=True)


    # In[3]:
    data=readin(currentdir+'/logfile')
    


    tendonnr = len(data[1][1])    
    xmin = min([m[0] for m in data])
    t = [(x[0]-xmin)*1e-9 for x in data]
    setpoints = [m[1] for m in data]
    lengths = [m[2] for m in data]
    forces = [m[3] for m in data]

    
    if startsec and endsec:
        startindex = next(t[0] for t in enumerate(t) if t[1]>= startsec)
        endindex = next(t[0] for t in enumerate(t) if t[1]>= endsec)
    else:
        startindex = 0
        endindex = len(data)-1 #OMIT last data entry becasuse it might be incomplete

    t=t[startindex:endindex]
    setpoints=setpoints[startindex:endindex]
    lengths=lengths[startindex:endindex]
    forces=forces[startindex:endindex]


# PLOT SHOULDER
    shoulder_indexgroup=[[0,1,2,3,4,5,6,7],[0,2,4,6],[1,3,5,7]]


    shoulder_colors = cm.rainbow(np.linspace(0, 1, len(shoulder_indexgroup[0])))


    fig_shoulder_spl, axs_shoulder_spl = plt.subplots(len(shoulder_indexgroup), constrained_layout=True)
    fig_shoulder_spl.set_size_inches(40, 15*len(shoulder_indexgroup))

    fig_shoulder_f, axs_shoulder_f = plt.subplots(len(shoulder_indexgroup), constrained_layout=True)
    fig_shoulder_f.set_size_inches(40, 15*len(shoulder_indexgroup))

    for s in range(len(shoulder_indexgroup)):
        for u in shoulder_indexgroup[s]:
            axs_shoulder_spl[s].plot(t, [s[u] for s in setpoints], '-.',color=shoulder_colors[shoulder_indexgroup[0].index(u)])
            axs_shoulder_spl[s].plot(t, [l[u] for l in lengths], linewidth=3, color=shoulder_colors[shoulder_indexgroup[0].index(u)])
            axs_shoulder_f[s].plot(t, [f[u] for f in forces], linewidth=3, color=shoulder_colors[shoulder_indexgroup[0].index(u)])

    axs_shoulder_spl[0].set_title('All shoulder Tendons', fontsize = 30)
    axs_shoulder_spl[1].set_title('Shoulder Tendons %s' %shoulder_indexgroup[1], fontsize = 30)
    axs_shoulder_spl[2].set_title('Shoulder Tendons  %s' %shoulder_indexgroup[2], fontsize = 30)

    axs_shoulder_f[0].set_title('All shoulder Tendons', fontsize = 30)
    axs_shoulder_f[1].set_title('Shoulder Tendons %s' %shoulder_indexgroup[1], fontsize = 30)
    axs_shoulder_f[2].set_title('Shoulder Tendons  %s' %shoulder_indexgroup[2], fontsize = 30)

    for a in axs_shoulder_spl:
        a.set_xlabel('Time [s]', fontsize=20)
        a.set_ylabel('Length [m]', fontsize=20)

    for a in axs_shoulder_f:
        a.set_xlabel('Time [s]', fontsize=20)
        a.set_ylabel('Force [N]', fontsize=20)

    fig_shoulder_spl.suptitle('Setpoints (dashed) and Lenghts - %s - Time from %is to %is' % (PIDvalues, t[0], t[-1]), fontsize = 30)
    fig_shoulder_spl.savefig(currentdir+'/shoulder_%s_s%ie%i_SPL.png' %(PIDvalues, t[0], t[-1]) , dpi = 100)

    fig_shoulder_f.suptitle('Forces - %s - Time from %is to %is' % (PIDvalues, t[0], t[-1]), fontsize = 30)
    fig_shoulder_f.savefig(currentdir+'/shoulder_%s_s%ie%i_F.png' %(PIDvalues, t[0], t[-1]) , dpi = 100)
    plt.close('all')


#PLOT BI AND TRICEPS
    fig_cepses_spl, axs_cepses_spl = plt.subplots(1, constrained_layout=True)
    fig_cepses_spl.set_size_inches(40, 15)

    axs_cepses_spl.plot(t,[s[16] for s in setpoints], '-.', color='green')
    axs_cepses_spl.plot(t,[l[16] for l in lengths], linewidth=3, color='green')

    axs_cepses_spl.plot(t,[s[17] for s in setpoints], '-.', color='red')
    axs_cepses_spl.plot(t,[l[17] for l in lengths], linewidth=3, color='red')

    axs_cepses_spl.set_title('Biceps (red) and Triceps (green)', fontsize = 30)
    axs_cepses_spl.set_xlabel('Time [s]', fontsize=20)
    axs_cepses_spl.set_ylabel('Length [m]', fontsize=20)

    fig_cepses_spl.suptitle('Setpoints (dashed) and Lenghts - %s - Time from %is to %is' % (PIDvalues, t[0], t[-1]), fontsize = 30)
    fig_cepses_spl.savefig(currentdir+'/cepses%s_s%ie%i_SPL.png' %(PIDvalues, t[0], t[-1]) , dpi = 100)



    fig_cepses_f, axs_cepses_f = plt.subplots(1, constrained_layout=True)
    fig_cepses_f.set_size_inches(40, 15)

    axs_cepses_f.plot(t,[f[16] for f in forces], linewidth=3, color='green')

    axs_cepses_f.plot(t,[f[17] for f in forces], linewidth=3, color='red')

    axs_cepses_f.set_title('Biceps (red) and Triceps (green)', fontsize = 30)
    axs_cepses_f.set_xlabel('Time [s]', fontsize=20)
    axs_cepses_f.set_ylabel('Length [m]', fontsize=20)

    fig_cepses_f.suptitle('Forces - %s - Time from %is to %is' % (PIDvalues, t[0], t[-1]), fontsize = 30)
    fig_cepses_f.savefig(currentdir+'/cepses%s_s%ie%i_F.png' %(PIDvalues, t[0], t[-1]) , dpi = 100)

    plt.close('all')


for folders in os.listdir(bagdir):
    PIDvalues = str(folders)
    
    PLOT(bagdir, PIDvalues)
    if plot_closeup:
        PLOT(bagdir, PIDvalues, startsec_pruned, endsec_pruned)