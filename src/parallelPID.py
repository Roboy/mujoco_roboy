import numpy as np
import mujoco_py
import rospy
import os
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from roboy_middleware_msgs.msg import MotorCommand
from roboy_simulation_msgs.msg import TendonUpdate
import subprocess
import psutil
import sys
import threading
import time


model_xml = "/code/mujoco_models/model_physics_rolljoint.xml"
bagfile = 'shoulder_left.bag'
baglocation = '/code/test_data'
add_name = '_findP'

playbag = True
startBagAt = 80
playBagDuration = 10

logging = True
sp_length_logging=True
controlidfile_logging=False

P = int(sys.argv[1])
I = int(sys.argv[2])
D = int(sys.argv[3])
#P = 12500
#I = 250
#D = 250

sim_freq = 100
control_freq = 300
renderEveryN = 1


bagname = bagfile[:bagfile.find('.bag')]

logdir = os.getcwd()+'/logfiles/%s' % bagname + add_name
currlog = os.path.join(logdir, 'P%iI%iD%i' % (P, I, D))
if not os.path.exists(currlog):
    os.makedirs(currlog)

if logging:
    logfile = open('%s/forces' % currlog, 'w')
if sp_length_logging:
    logfile_setpoints = open('%s/setpoints' % currlog, 'w')
    logfile_lengths = open('%s/lengths' % currlog, 'w')
if controlidfile_logging:
    controlidfile = open('%s/controlid' % currlog, 'w')


model = mujoco_py.load_model_from_path(model_xml)
sim = mujoco_py.MjSim(model)
viewer = mujoco_py.MjViewer(sim)

n_motors = 38
warmup_step = 10
bagstart_step = 100
sim_step = 0


joint_names = [
    ['shoulder_right_axis0', 'shoulder_right_axis1', 'shoulder_right_axis2', 
    'elbow_right_axis0', 'elbow_right_axis1', 
    'wrist_right_axis0', 'wrist_right_axis1', 'wrist_right_axis2'],
    ['head_axis0', 'head_axis1', 'head_axis2'], 
    ['shoulder_left_axis0','shoulder_left_axis1','shoulder_left_axis2',
    'elbow_left_axis0', 'elbow_left_axis1',
    'wrist_left_axis0', 'wrist_left_axis1', 'wrist_left_axis2']
]

initial_pose = [0.41718248, 0.39287779, 0.40840881, 0.42272534, 0.37851663, 0.37709782,
 0.40643997, 0.39580119, 0.51552814, 0.3940247,  0.42831752, 0.44521674,
 0.39350609, 0.38703304, 0.41358078, 0.38863428, 0.11306499, 0.11068354,
 0.12320018, 0.14406976, 0.23695295, 0.17218595, 0.17111917, 0.13440764,
 0.18711481, 0.13649938, 0.04767673, 0.04649664, 0.04500692, 0.06957187,
 0.0506599,  0.07108747, 0.04672045, 0.04757878, 0.06830778, 0.04757697,
 0.06859654, 0.04595365]


tendon_names_inner = []
for i in range(37):
    tendon_names_inner.append('motor%i' % i)
tendon_names = [tendon_names_inner]


Kp = float(P)* np.ones(n_motors)
Kd = float(D) * np.ones(n_motors)
Ki = float(I) * np.ones(n_motors)
setpoint = np.zeros(n_motors)
error_prev = np.zeros(n_motors)
error_int = np.zeros(n_motors)

# Head
Kp[np.arange(20, 25)] = 7500.0
Kd[np.arange(20, 25)] = 200.0
Ki[np.arange(20, 25)] = 200.0

rate = None
dt = 1 / control_freq
publisher = None

def tendon_target_cb(data):

    global setpoint

    control_id = list(data.global_id)

    setpoint[control_id] = -np.array(data.setpoint)


    if controlidfile_logging:
        print(control_id, file=controlidfile)
        print(setpoint, file=controlidfile)

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
        if logging:
            print(msg.header.stamp, file=logfile)
            print(tendon_forces[start_idx:start_idx+len(tendon)], file=logfile)

        if sp_length_logging:
            print(msg.header.stamp, file=logfile_setpoints)
            print(setpoint, file=logfile_setpoints)

            print(msg.header.stamp, file=logfile_lengths)
            print(sim.data.ten_length, file=logfile_lengths)


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

        publish_joint_states(pub, sim.data.qpos)
        publish_tendon_forces(pub2, sim.data.ctrl, sim.data.actuator_force)

        #sim.data.ten_length[:15] = initial_pose[:15]
        #sim.data.ten_length[18:] = initial_pose[18:]
        rate.sleep()

        if stop_threads:
            exit()


def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()


if __name__ == '__main__':

    topic_root = "/roboy/pinky"

    tendon_target_sub = rospy.Subscriber(f"{topic_root}/middleware/MotorCommand", MotorCommand, tendon_target_cb)
    pub = rospy.Publisher(f"{topic_root}/external_joint_states", JointState, queue_size=1)
    pub2 = rospy.Publisher(f"{topic_root}/tendon_forces", JointState, queue_size=1)
    rospy.init_node("mujoco_roboy")

    rate = rospy.Rate(control_freq)
    stop_threads = False

    # Create control thread
    control_thread = threading.Thread(target=control_runner)
    for i in range(warmup_step):
        sim.step()
    
    control_thread.start()


    # Now start the simulation
    stop_step = playBagDuration*sim_freq + bagstart_step
    

    print("Simulation started!")
    sim_rate = rospy.Rate(sim_freq)
    sim_step = 0
    
    
    starttime = 0
    endtime = 0
    maxtimedif = 0

    while not rospy.is_shutdown():
        if (sim_step > warmup_step):
            timedif = endtime-starttime
            if (timedif > maxtimedif):
                maxtimedif = timedif
                maxstep = sim_step

        starttime = time.time_ns()

        sim.step()
        if (sim_step%renderEveryN==0):
            viewer.render()
        

        if (sim_step == bagstart_step and playbag):
                player_proc = subprocess.Popen(['rosbag', 'play', bagfile, '-s %i' %startBagAt, '-u %i' %playBagDuration], cwd=baglocation)

        if sim_step >= stop_step:
            kill(player_proc.pid)
            stop_threads = True

            print("\nMaximal Simulation timestep: %fms At Simulation Step: %i \nLast Simulation timestep: %fms \nMust not be larger than: %fms" %(maxtimedif/10e6, maxstep, timedif/10e6, 1000/sim_freq))
            os._exit(1)

        sim_step += 1

        endtime = time.time_ns()
        sim_rate.sleep()

        
        
        
        
