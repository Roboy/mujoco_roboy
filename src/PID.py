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


bagfile = 'left_elbow_ransteps_A.bag'
baglocation = '/code/test_data'

playbag = True
startBagAt = 40

P = int(sys.argv[1])
I = int(sys.argv[2])
D = int(sys.argv[3])

bagname = bagfile[:bagfile.find('.bag')]


logging = True
sp_length_logging=True

logdir = os.getcwd()+'/logfiles/%s' % bagname
currlog = os.path.join(logdir, 'P%iI%iD%i' % (P, I, D))
if not os.path.exists(currlog):
    os.makedirs(currlog)


if logging:
    logfile = open('%s/forces' % currlog, 'w')
if sp_length_logging:
    logfile_setpoints = open('%s/setpoints' % currlog, 'w')
    logfile_lengths = open('%s/lengths' % currlog, 'w')



model_xml = "/code/mujoco_models/model_physics.xml"

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

n_motors = 38

warmup_step = 10
bagstart_step = 100
stop_step = 6000

sim_step = 0



Kp = float(P)* np.ones(n_motors)
Kd = float(D) * np.ones(n_motors)
Ki = float(I) * np.ones(n_motors)
setpoint = np.zeros(n_motors)
error_prev = np.zeros(n_motors)
error_int = np.zeros(n_motors)

# Head
Kp[np.arange(20, 25)] = 500.0
Kd[np.arange(20, 25)] = 10.0

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
        if logging:
            print(msg.header.stamp, file=logfile)
            print(tendon_forces[start_idx:start_idx+len(tendon)], file=logfile)

        if sp_length_logging:
            print(msg.header.stamp, file=logfile_setpoints)
            print(setpoint, file=logfile_setpoints)

            print(msg.header.stamp, file=logfile_lengths)
            print(sim.data.ten_length, file=logfile_lengths)


        start_idx += len(tendon)


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

    print("Simulation started!")

    freq = 300
    rate = rospy.Rate(freq)
    dt = 1 / freq

    sim_step = 0
    while not rospy.is_shutdown():
        if (sim_step==bagstart_step and playbag):
                player_proc = subprocess.Popen(['rosbag', 'play', '-s %i' %startBagAt, bagfile], cwd=baglocation)
        if sim_step >= stop_step:
            kill(player_proc.pid)
            exit()
        if sim_step >= warmup_step:
            error = setpoint - sim.data.ten_length
            error_int += Ki * error * dt
            force = (Kp * error + Kd * (error - error_prev) / dt + error_int)

            ctrl_idx = np.where(setpoint != 0)[0]
            sim.data.ctrl[ctrl_idx] = force[ctrl_idx]

            error_prev = error

        sim.step()
        publish_joint_states(pub, sim.data.qpos)
        publish_tendon_forces(pub2, sim.data.ctrl, sim.data.actuator_force)


        sim_step += 1
        viewer.render()

        rate.sleep()