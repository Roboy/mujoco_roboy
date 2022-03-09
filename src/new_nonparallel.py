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


model_xml = "/code/mujoco_models/model_physics_rolljoint.xml"
timestep_from_xml = 0.01

bagfile = 'fixed_sp_allaxis.bag'
baglocation = '/code/test_data'
add_name = 'TS_Test_parallel'

playbag = True
startBagAt = 60
playBagDuration = 60

logging = True

P = int(sys.argv[1])
I = int(sys.argv[2])
D = int(sys.argv[3])
#P = 12500
#I = 250
#D = 250


renderEveryN = 1
logEveryN = 2

n_motors = 38
warmup_step = 10
bagstart_step = 100
sim_step = 0
dt = timestep_from_xml
freq = 1/dt



bagname = bagfile[:bagfile.find('.bag')]

logdir = os.getcwd()+'/logfiles/%s_%s-s%i-u%i_F%i' % (add_name, bagname , startBagAt, playBagDuration, freq)
currlog = os.path.join(logdir, 'P%iI%iD%i' % (P, I, D))
if not os.path.exists(currlog):
    os.makedirs(currlog)

if logging:
    logfile = open('%s/logfile' % currlog, 'w')
    print(bagname, file=logfile)
    print('P=%i I=%i D=%i'%(P,I,D), file=logfile)
    print('Bagstart at %i Bag Play duration %i'%(startBagAt,playBagDuration), file=logfile)
    print('Timestep (XML):%.3f Frequency %i' % (timestep_from_xml,freq), file=logfile)
    print('', file=logfile)
    print('RECORDING START', file=logfile)



model = mujoco_py.load_model_from_path(model_xml)
sim = mujoco_py.MjSim(model)
viewer = mujoco_py.MjViewer(sim)



joint_names = [
    ['shoulder_right_axis0', 'shoulder_right_axis1', 'shoulder_right_axis2', 
    'elbow_right_axis0', 'elbow_right_axis1', 
    'wrist_right_axis0', 'wrist_right_axis1', 'wrist_right_axis2'],
    ['head_axis0', 'head_axis1', 'head_axis2'], 
    ['shoulder_left_axis0','shoulder_left_axis1','shoulder_left_axis2',
    'elbow_left_axis0', 'elbow_left_axis1',
    'wrist_left_axis0', 'wrist_left_axis1', 'wrist_left_axis2']
]

tendon_names_inner = []
for i in range(37):
    tendon_names_inner.append('motor%i' % i)
tendon_names = [tendon_names_inner]

model = mujoco_py.load_model_from_path(model_xml)
sim = mujoco_py.MjSim(model)
viewer = mujoco_py.MjViewer(sim)



Kp = float(P)* np.ones(n_motors)
Kd = float(D) * np.ones(n_motors)
Ki = float(I) * np.ones(n_motors)
setpoint = np.zeros(n_motors)
error_prev = np.zeros(n_motors)
error_int = np.zeros(n_motors)

# Head
Kp[np.arange(20, 25)] = 500.0
Kd[np.arange(20, 25)] = 10.0

def log(logfile):
    print(rospy.Time.now(), file=logfile)
    print(*setpoint, file=logfile)
    print(*sim.data.ten_length, file=logfile)
    print(*sim.data.actuator_force, file=logfile)
    print('', file=logfile)



def tendon_target_cb(data):

    global setpoint

    control_id = list(data.global_id)

    setpoint[control_id] = -np.array(data.setpoint)


def publish_joint_states(publisher, joint_states):

    start_idx = 0

    for body in joint_names:
            
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        msg.name = body
        msg.position = joint_states[start_idx:start_idx+len(body)]
        msg.velocity = [0] * len(body)
        msg.effort = [0] * len(body)
        
        publisher.publish(msg)
        start_idx += len(body)


def publish_tendon_forces(publisher, tendon_control, tendon_forces):

    start_idx = 0

    for tendon in tendon_names:
            
        msg = JointState()  #JointState is the wrong message type for this. But there is no direct Forece measurement message type.
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        msg.name = tendon
        msg.position = tendon_control[start_idx:start_idx+len(tendon)]
        msg.velocity = tendon_forces[start_idx:start_idx+len(tendon)]
        msg.effort = [0] * len(tendon)
        
        publisher.publish(msg)

        rospy.loginfo(msg)


        start_idx += len(tendon)


def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()


if __name__ == '__main__':

    topic_root = "/roboy/pinky"

    tendon_target_sub = rospy.Subscriber(f"{topic_root}/middleware/MotorCommand", MotorCommand, tendon_target_cb)
    #pub = rospy.Publisher(f"{topic_root}/external_joint_states", JointState, queue_size=1)
    #pub2 = rospy.Publisher(f"{topic_root}/tendon_forces", JointState, queue_size=1)
    rospy.init_node("mujoco_roboy")

    print("Simulation started!")

    rate = rospy.Rate(freq)

    stop_step = playBagDuration*freq + bagstart_step    
    sim_step = 0



    #XXXtime is time AFTER action XXX
    starttime = 0
    controltime = 0
    logtime = 0
    simulationtime = 0
    rendertime = 0

    maxcontroldur = 0
    maxlogdur = 0
    maxsimulationdur = 0
    maxrenderdur = 0
    maxtotaldur = 0

    while not rospy.is_shutdown() and (sim_step <= warmup_step):
        sim.step()
        sim_step += 1
        viewer.render()

    while not rospy.is_shutdown() and sim_step<=stop_step:

        if (sim_step == bagstart_step and playbag):
            bagstart_time = time.time_ns()
            bagend_time = time.time_ns() +  int(1e9*playBagDuration)
            player_proc = subprocess.Popen(['rosbag', 'play', bagfile, '-s %i' %startBagAt, '-u %i' %playBagDuration], cwd=baglocation)

        oldstarttime = starttime
        starttime = time.time_ns()

            
    #TIME DURATIONS
        currcontroldur = controltime - oldstarttime 
        currlogdur = logtime - controltime
        currsimulationdur = simulationtime - logtime
        currrenderdur = rendertime - simulationtime
        currtotaldur = rendertime - oldstarttime

            
        if (currcontroldur > maxcontroldur):
            maxcontroldur = currcontroldur
        if (currlogdur > maxlogdur):
            maxlogdur = currlogdur
        if (currsimulationdur > maxsimulationdur):
            maxsimulationdur = currsimulationdur
        if (currrenderdur > maxrenderdur):
            maxrenderdur = currrenderdur
        if (currtotaldur > maxtotaldur):
            maxtotaldur = currtotaldur
            maxtotaldur_step = sim_step


        #controller calculation
        error = setpoint - sim.data.ten_length
        error_int += Ki * error * dt
        force = Kp * error + Kd * (error - error_prev) / dt + error_int

        ctrl_idx = np.where(setpoint != 0)[0]
        sim.data.ctrl[ctrl_idx] = force[ctrl_idx]

        error_prev = error

        controltime = time.time_ns()

        #logging
        if (logging & (sim_step%logEveryN == 0)):
            log(logfile)


        logtime = time.time_ns()


        #simulation step
        sim.step()
        sim_step += 1

        simulationtime = time.time_ns()


        #render
        if (sim_step%renderEveryN==0):
            viewer.render()

        rendertime = time.time_ns()


        rate.sleep()

    loopendtime = time.time_ns()

    kill(player_proc.pid)
    stop_threads = True
    print("\nReal Time Duraion: %.2fs Loop Duarion %.2fs" %(playBagDuration, (loopendtime-bagstart_time)*1e-9))
    print("\nMaximal Time Durations")
    print("Total %.2fms at step %i" % (maxtotaldur*1e-6, maxtotaldur_step))
    print("Control %.2fms" % (maxcontroldur*1e-6))
    print("Log %.2fms" % (maxlogdur*1e-6))
    print("Simulation %.2fms" % (maxsimulationdur*1e-6))
    print("Render %.2fms" % (maxrenderdur*1e-6))
    print("\nTotal Duration must not be larger than: %.2fms\n" %(1000/freq))
    print("Current Frequency: %i\nPotential Maximal Frequency: %i \nRecommended Frequency with with 10 perc. safety margin: %i" %(freq, 1e9/maxtotaldur, 9e8/maxtotaldur))
    os._exit(1)