#!/usr/bin/env python
# coding: utf-8

import matplotlib.pyplot as plt
import numpy as np
import matplotlib.cm as cm
import os
import sys
from pathlib import Path


bagname = str(input('Bagfile name (no .bag): '))
closeup_start=input('Start closeup Plots at Second: ')
if closeup_start:
    startsec_pruned = int(closeup_start)
else:
    startsec_pruned = 0

closeup_end=input('end closeup Plots at Second: ')
if closeup_end:
    endsec_pruned = int(closeup_end)
else:
    endsec_pruned = 0


logdir = 'logfiles'
bagdir = os.path.join(os.getcwd(),logdir,bagname)

plot_closeup = False
if (startsec_pruned < endsec_pruned):
    plot_closeup = True


def PLOT_SP_LEN(bagdir, PIDvalues, startsec_pruned, endsec_pruned, plot_closeup):

    currentdir = os.path.join(bagdir,PIDvalues)
    sp_len_dir = os.path.join(currentdir, 'SP_Len')
    Path(sp_len_dir).mkdir(parents=True, exist_ok=True)

    setpoints=[]
    lengths=[]
    element =[0,[0]]

    filename_sp = currentdir + '/setpoints'
    filename_l = currentdir + '/lengths'
    


    with open(filename_sp) as f_sp:
        line = f_sp.readline()
        while line:
            if (line.find("1") == 0):
                element[0]=int(line.strip())
                line = f_sp.readline()
                if (line.find("[") == 0):
                    stripped_line = line[1:].strip()
                    line = f_sp.readline()
                    while (line.find("]") == -1):
                        stripped_line += " "+line.strip()
                        line = f_sp.readline()
                    if (line.find("]") > 0):
                        stripped_line += " "+line[:-2].strip()
                        element[1] = [float(k) for k in stripped_line.split()]
                        setpoints.append(element.copy())
                        line = f_sp.readline()
                        
    with open(filename_l) as f_l:
        line = f_l.readline()
        while line:
            if (line.find("1") == 0):
                element[0]=int(line.strip())
                line = f_l.readline()
                if (line.find("[") == 0):
                    stripped_line = line[1:].strip()
                    line = f_l.readline()
                    while (line.find("]") == -1):
                        stripped_line += " "+line.strip()
                        line = f_l.readline()
                    if (line.find("]") > 0):
                        stripped_line += " "+line[:-2].strip()
                        element[1] = [float(k) for k in stripped_line.split()]
                        lengths.append(element.copy())
                        line = f_l.readline()        
                    


    # In[3]:


    datanr = len(setpoints)
    tendonnr = len(setpoints[1][1])


    # In[4]:


    datanr


    # In[5]:


    #first nonzero element
    startindices = []
    for t in range(tendonnr):
        startindices.append(next((i for i, x in enumerate([n[1][t] for n in setpoints]) if x), None))

    Not_none_values = filter(None.__ne__, startindices)
    startindices = list(Not_none_values)


    # In[6]:


    #start at specific index?
    startindex = min(startindices)
    #start some indices before?
    endindex = datanr

    plot_setpoints = setpoints[startindex:endindex]
    plot_lengths = lengths[startindex:endindex]

    plotdatanr = len(plot_setpoints)

    print("Data Points: %i \nTendons: %i" % (plotdatanr,tendonnr))
    print("Plot Data starts at %i and ends at %i" % (startindex,endindex))


    # In[7]:


    x = [m[0] for m in plot_setpoints]



    #start x axis with 0
    tmin = min([m[0] for m in plot_setpoints])

    x = [t - x[0] for t in x]
    x = [e*1e-9 for e in x]

    startsec = int(x[0])
    endsec = int(x[-1])


    #x is in seconds now


    # In[8]:


    min([n[1][17] for n in plot_setpoints])


    # In[9]:


    max([n[1][17] for n in plot_setpoints])


    # In[10]:


    shoulder_idx=[0,1,2,3,4,5,6,7]
    shoulder_idx1=[0,2,4,6]
    shoulder_idx2=[1,3,5,7]
    #biceps 17 triceps 16


    # In[11]:


    shoulder_colors = cm.rainbow(np.linspace(0, 1, len(shoulder_idx)))

    fig_shoulder, axs_shoulder = plt.subplots(3, constrained_layout=True)
    fig_shoulder.set_size_inches(40, 15*3)
    for u in shoulder_idx:
        axs_shoulder[0].plot(x,[n[1][u] for n in plot_setpoints], '-.',color=shoulder_colors[shoulder_idx.index(u)])
        axs_shoulder[0].plot(x,[n[1][u] for n in plot_lengths], linewidth=3, color=shoulder_colors[shoulder_idx.index(u)])
        
    for u in shoulder_idx1:
        axs_shoulder[1].plot(x,[n[1][u] for n in plot_setpoints], '-.',color=shoulder_colors[shoulder_idx.index(u)])
        axs_shoulder[1].plot(x,[n[1][u] for n in plot_lengths], linewidth=3, color=shoulder_colors[shoulder_idx.index(u)])
        
    for u in shoulder_idx2:
        axs_shoulder[2].plot(x,[n[1][u] for n in plot_setpoints], '-.',color=shoulder_colors[shoulder_idx.index(u)])
        axs_shoulder[2].plot(x,[n[1][u] for n in plot_lengths], linewidth=3, color=shoulder_colors[shoulder_idx.index(u)])

    axs_shoulder[0].set_title('All shoulder Tendons', fontsize = 30)
    axs_shoulder[1].set_title('Shoulder Tendons %s' %shoulder_idx1, fontsize = 30)
    axs_shoulder[2].set_title('Shoulder Tendons  %s' %shoulder_idx2, fontsize = 30)

    for a in axs_shoulder:
        a.set_xlabel('Time [s]', fontsize=20)
        a.set_ylabel('Length [m]', fontsize=20)

    fig_shoulder.suptitle('%s - Time from %ss to %ss' % (PIDvalues, startsec, endsec), fontsize = 30)
        
    fig_shoulder.savefig(sp_len_dir+'/shoulder_%s_s%se%s.png' %(PIDvalues, startsec, endsec) , dpi = 100)


    # In[12]:


    fig_cepses, axs_cepses = plt.subplots(1, constrained_layout=True)

    fig_cepses.set_size_inches(40, 15)

    axs_cepses.plot(x,[n[1][16] for n in plot_setpoints], '-.', color='green')
    axs_cepses.plot(x,[n[1][16] for n in plot_lengths], linewidth=3, color='green')

    axs_cepses.plot(x,[n[1][17] for n in plot_setpoints], '-.', color='red')
    axs_cepses.plot(x,[n[1][17] for n in plot_lengths], linewidth=3, color='red')

    axs_cepses.set_title('Biceps (red) and Triceps (green)', fontsize = 30)
    axs_cepses.set_xlabel('Time [s]', fontsize=20)
    axs_cepses.set_ylabel('Length [m]', fontsize=20)

    fig_cepses.suptitle('%s - Time from %ss to %ss' % (PIDvalues, startsec, endsec), fontsize = 30)

    fig_cepses.savefig(sp_len_dir+'/cepses_%s_s%se%s.png' %(PIDvalues, startsec, endsec) , dpi = 100)


    # In[13]:


    # was to plot whole data. Now plots of relevant data follow

    if plot_closeup:
    # In[14]:

        startindex = next(z[0] for z in enumerate(x) if z[1] > startsec_pruned)
        endindex = next(z[0] for z in enumerate(x) if z[1] > endsec_pruned)


        # In[15]:


        plot_setpoints_pruned = plot_setpoints[startindex:endindex]
        plot_lengths_pruned = plot_lengths[startindex:endindex]
        x_pruned = x[startindex:endindex]

        plot_pruned_datanr = len(plot_setpoints_pruned)

        print("Data Points: %i \nTendons: %i" % (plot_pruned_datanr,tendonnr))
        print("Plot Data starts at %i and ends at %i" % (startindex,endindex))


        # In[16]:


        shoulder_colors = cm.rainbow(np.linspace(0, 1, len(shoulder_idx)))

        fig_shoulder, axs_shoulder = plt.subplots(3, constrained_layout=True)
        fig_shoulder.set_size_inches(40, 15*3)
        for u in shoulder_idx:
            axs_shoulder[0].plot(x_pruned,[n[1][u] for n in plot_setpoints_pruned], '-.',color=shoulder_colors[shoulder_idx.index(u)])
            axs_shoulder[0].plot(x_pruned,[n[1][u] for n in plot_lengths_pruned], linewidth=3, color=shoulder_colors[shoulder_idx.index(u)])
            
        for u in shoulder_idx1:
            axs_shoulder[1].plot(x_pruned,[n[1][u] for n in plot_setpoints_pruned], '-.',color=shoulder_colors[shoulder_idx.index(u)])
            axs_shoulder[1].plot(x_pruned,[n[1][u] for n in plot_lengths_pruned], linewidth=3, color=shoulder_colors[shoulder_idx.index(u)])
            
        for u in shoulder_idx2:
            axs_shoulder[2].plot(x_pruned,[n[1][u] for n in plot_setpoints_pruned], '-.',color=shoulder_colors[shoulder_idx.index(u)])
            axs_shoulder[2].plot(x_pruned,[n[1][u] for n in plot_lengths_pruned], linewidth=3, color=shoulder_colors[shoulder_idx.index(u)])

        axs_shoulder[0].set_title('All shoulder Tendons', fontsize = 30)
        axs_shoulder[1].set_title('Shoulder Tendons %s' %shoulder_idx1, fontsize = 30)
        axs_shoulder[2].set_title('Shoulder Tendons  %s' %shoulder_idx2, fontsize = 30)

        for a in axs_shoulder:
            a.set_xlabel('Time [s]', fontsize=20)
            a.set_ylabel('Length [m]', fontsize=20)

        fig_shoulder.suptitle('%s - Time from %ss to %ss' % (PIDvalues, startsec_pruned, endsec_pruned), fontsize = 30)
            
        fig_shoulder.savefig(sp_len_dir+'/shoulder_%s_s%se%s.png' %(PIDvalues, startsec_pruned, endsec_pruned) , dpi = 100)


        # In[17]:


        fig_cepses, axs_cepses = plt.subplots(1, constrained_layout=True)

        fig_cepses.set_size_inches(40, 15)

        axs_cepses.plot(x_pruned,[n[1][16] for n in plot_setpoints_pruned], '-.', color='green')
        axs_cepses.plot(x_pruned,[n[1][16] for n in plot_lengths_pruned], linewidth=3, color='green')

        axs_cepses.plot(x_pruned,[n[1][17] for n in plot_setpoints_pruned], '-.', color='red')
        axs_cepses.plot(x_pruned,[n[1][17] for n in plot_lengths_pruned], linewidth=3, color='red')

        axs_cepses.set_title('Biceps (red) and Triceps (green)', fontsize = 30)
        axs_cepses.set_xlabel('Time [s]', fontsize=20)
        axs_cepses.set_ylabel('Length [m]', fontsize=20)

        fig_cepses.suptitle('%s - Time from %ss to %ss' % (PIDvalues, startsec_pruned, endsec_pruned), fontsize = 30)

        fig_cepses.savefig(sp_len_dir+'/cepses_%s_s%se%s.png' %(PIDvalues, startsec_pruned, endsec_pruned) , dpi = 100)
    

    plt.close('all')


def PLOT_FORCE(bagdir, PIDvalues, startsec_pruned, endsec_pruned, plot_closeup):
    forces=[]
    element =[0,[0]]
    currentdir = os.path.join(bagdir,PIDvalues)
    forces_dir = os.path.join(currentdir, 'Forces')
    Path(forces_dir).mkdir(parents=True, exist_ok=True)

    filename_f = currentdir + '/forces'
    

    with open(filename_f) as f:
        line = f.readline()
        while line:
            if (line.find("1") == 0):
                element[0]=int(line.strip())
                line = f.readline()
                if (line.find("[") == 0):
                    stripped_line = line[1:].strip()
                    line = f.readline()
                    while (line.find("]") == -1):
                        stripped_line += " "+line.strip()
                        line = f.readline()
                    if (line.find("]") > 0):
                        stripped_line += " "+line[:-2].strip()
                        element[1] = [float(k) for k in stripped_line.split()]
                        forces.append(element.copy())
                        line = f.readline()
                
                    


    # In[3]:


    datanr = len(forces)
    tendonnr = len(forces[1][1])


    # In[4]:


    datanr


    # In[5]:


    #first nonzero element
    startindices = []
    for t in range(tendonnr):
        startindices.append(next((i for i, x in enumerate([n[1][t] for n in forces]) if x), None))

    Not_none_values = filter(None.__ne__, startindices)
    startindices = list(Not_none_values)


    # In[6]:


    #start at specific index?
    startindex = min(startindices)
    #start some indices before?
    endindex = datanr

    plot_forces = forces[startindex:endindex]

    plotdatanr = len(plot_forces)

    print("Data Points: %i \nTendons: %i" % (plotdatanr,tendonnr))
    print("Plot Data starts at %i and ends at %i" % (startindex,endindex))


    # In[7]:


    x = [m[0] for m in plot_forces]



    #start x axis with 0
    tmin = min([m[0] for m in plot_forces])

    x = [t - x[0] for t in x]
    x = [e*1e-9 for e in x]

    startsec = int(x[0])
    endsec = int(x[-1])


    #x is in seconds now


    # In[8]:


    shoulder_idx=[0,1,2,3,4,5,6,7]
    #biceps 17 triceps 16


    # In[9]:


    shoulder_colors = cm.rainbow(np.linspace(0, 1, len(shoulder_idx)))

    fig_shoulder, axs_shoulder = plt.subplots(3, constrained_layout=True)
    fig_shoulder.set_size_inches(40, 15*3)
    for u in shoulder_idx:
        axs_shoulder[0].plot(x,[n[1][u] for n in plot_forces], color=shoulder_colors[shoulder_idx.index(u)])
        
    for u in shoulder_idx[:4]:
        axs_shoulder[1].plot(x,[n[1][u] for n in plot_forces], color=shoulder_colors[shoulder_idx.index(u)])
        
    for u in shoulder_idx[4:]:
        axs_shoulder[2].plot(x,[n[1][u] for n in plot_forces], color=shoulder_colors[shoulder_idx.index(u)])

    axs_shoulder[0].set_title('All shoulder Tendons', fontsize = 30)
    axs_shoulder[1].set_title('Shoulder Tendons 0:4', fontsize = 30)
    axs_shoulder[2].set_title('Shoulder Tendons 4:7', fontsize = 30)

    for a in axs_shoulder:
        a.set_xlabel('Time [s]', fontsize=20)
        a.set_ylabel('Force [N]', fontsize=20)

    fig_shoulder.suptitle('%s - Time from %ss to %ss' % (PIDvalues, startsec, endsec), fontsize = 30)
        
    fig_shoulder.savefig(forces_dir+'/forces_shoulder_%s_s%se%s.png' %(PIDvalues, startsec, endsec) , dpi = 100)


    # In[10]:


    fig_cepses, axs_cepses = plt.subplots(1, constrained_layout=True)

    fig_cepses.set_size_inches(40, 15)

    axs_cepses.plot(x,[n[1][16] for n in plot_forces], color='green')
    axs_cepses.plot(x,[n[1][17] for n in plot_forces], color='red')

    axs_cepses.set_title('Biceps (red) and Triceps (green)', fontsize = 30)
    axs_cepses.set_xlabel('Time [s]', fontsize=20)
    axs_cepses.set_ylabel('Force [N]', fontsize=20)

    fig_cepses.suptitle('%s - Time from %ss to %ss' % (PIDvalues, startsec, endsec), fontsize = 30)
        
    fig_cepses.savefig(forces_dir+'/forces_cepses_%s_s%se%s.png' %(PIDvalues, startsec, endsec) , dpi = 100)


    # In[11]:


    # was to plot whole data. Now plots of relevant data follow

    if plot_closeup:
    # In[12]:

        startindex = next(z[0] for z in enumerate(x) if z[1] > startsec_pruned)
        endindex = next(z[0] for z in enumerate(x) if z[1] > endsec_pruned)


        # In[13]:


        plot_forces_pruned = plot_forces[startindex:endindex]
        x_pruned = x[startindex:endindex]

        plot_pruned_datanr = len(plot_forces_pruned)

        print("Data Points: %i \nTendons: %i" % (plot_pruned_datanr,tendonnr))
        print("Plot Data starts at %i and ends at %i" % (startindex,endindex))


        # In[14]:


        shoulder_colors = cm.rainbow(np.linspace(0, 1, len(shoulder_idx)))

        fig_shoulder, axs_shoulder = plt.subplots(3, constrained_layout=True)
        fig_shoulder.set_size_inches(40, 15*3)
        for u in shoulder_idx:
            axs_shoulder[0].plot(x_pruned,[n[1][u] for n in plot_forces_pruned], color=shoulder_colors[shoulder_idx.index(u)])
            
        for u in shoulder_idx[:4]:
            axs_shoulder[1].plot(x_pruned,[n[1][u] for n in plot_forces_pruned], color=shoulder_colors[shoulder_idx.index(u)])
            
        for u in shoulder_idx[4:]:
            axs_shoulder[2].plot(x_pruned,[n[1][u] for n in plot_forces_pruned], color=shoulder_colors[shoulder_idx.index(u)])

        axs_shoulder[0].set_title('All shoulder Tendons', fontsize = 30)
        axs_shoulder[1].set_title('Shoulder Tendons 0:4', fontsize = 30)
        axs_shoulder[2].set_title('Shoulder Tendons 4:7', fontsize = 30)

        for a in axs_shoulder:
            a.set_xlabel('Time [s]', fontsize=20)
            a.set_ylabel('Force [N]', fontsize=20)

        fig_shoulder.suptitle('%s - Time from %ss to %ss' % (PIDvalues, startsec_pruned, endsec_pruned), fontsize = 30)
            
        fig_shoulder.savefig(forces_dir+'/forces_shoulder_%s_s%se%s.png' %(PIDvalues, startsec_pruned, endsec_pruned) , dpi = 100)


        # In[15]:


        fig_cepses, axs_cepses = plt.subplots(1, constrained_layout=True)

        fig_cepses.set_size_inches(40, 15)

        axs_cepses.plot(x_pruned,[n[1][16] for n in plot_forces_pruned], color='green')
        axs_cepses.plot(x_pruned,[n[1][17] for n in plot_forces_pruned], color='red')

        axs_cepses.set_title('Biceps (red) and Triceps (green)', fontsize = 30)
        axs_cepses.set_xlabel('Time [s]', fontsize=20)
        axs_cepses.set_ylabel('Force [N]', fontsize=20)

        fig_cepses.suptitle('%s - Time from %ss to %ss' % (PIDvalues, startsec_pruned, endsec_pruned), fontsize = 30)
            
        fig_cepses.savefig(forces_dir+'/forces_cepses_%s_s%se%s.png' %(PIDvalues, startsec_pruned, endsec_pruned) , dpi = 100)


    plt.close('all')


for folders in os.listdir(bagdir):
    PIDvalues = str(folders)
    PLOT_SP_LEN(bagdir, PIDvalues, startsec_pruned, endsec_pruned, plot_closeup)
    PLOT_FORCE(bagdir, PIDvalues, startsec_pruned, endsec_pruned, plot_closeup)