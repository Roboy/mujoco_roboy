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


model_xml = "/code/mujoco_models/model_physics_rolljoint.xml"
timestep_from_xml = 0.005

bagfile = 'fixed_sp_allaxis.bag'
baglocation = '/code/test_data'
add_name = 'TS_Test_nonparallel'

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

def control_runner():
    global sim

    error_prev = np.zeros(n_motors)
    error_int = np.zeros(n_motors)

    while not rospy.is_shutdown():


        error = setpoint - sim.data.ten_length
        error_int += Ki * error * dt
        force = Kp * error + Kd * (error - error_prev) / dt + error_int

        ctrl_idx = np.where(setpoint != 0)[0]
        sim.data.ctrl[ctrl_idx] = force[ctrl_idx]

        error_prev = error

        if logging:
            log(logfile)

        #publish_joint_states(pub, sim.data.qpos)
        #publish_tendon_forces(pub2, sim.data.ctrl, sim.data.actuator_force)

        #sim.data.ten_length[:15] = initial_pose[:15]
        #sim.data.ten_length[18:] = initial_pose[18:]

        if stop_threads:
            exit()

        rate.sleep()


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


    control_thread = threading.Thread(target=control_runner)
    for i in range(warmup_step):
        sim.step()
    
    control_thread.start()



    stop_step = playBagDuration*freq + bagstart_step    
    sim_step = 0
    starttime = 0
    endtime = 0
    maxtimedif = 0


    while not rospy.is_shutdown() and sim_step<=stop_step:
        if (sim_step == bagstart_step and playbag):
            player_proc = subprocess.Popen(['rosbag', 'play', bagfile, '-s %i' %startBagAt, '-u %i' %playBagDuration], cwd=baglocation)



        oldstarttime = starttime
        starttime = time.time_ns()

        

        if (sim_step > warmup_step):
            timedif = endtime-oldstarttime            
            if (timedif > maxtimedif):
                maxtimedif = timedif
                maxstep = sim_step


            if (logging & (sim_step%logEveryN == 0)):
                log(logfile)




        sim.step()
        if (sim_step%renderEveryN==0):
            viewer.render()

        sim_step += 1

        endtime = time.time_ns()
        rate.sleep()



    kill(player_proc.pid)
    stop_threads = True
    print("\nMaximal timestep: %.2fms At Simulation Step: %i \nMust not be larger than: %.2fms\n" %(maxtimedif/10e6, maxstep, 1000/freq))
    print("Current Frequency: %i\nPotential Maximal Frequency: %i \nRecommended Frequency with with 10 perc. safety margin: %i:" %(freq, 10e9/maxtimedif, 9e9/maxtimedif))
    os._exit(1)