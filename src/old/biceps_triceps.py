import sys
import numpy as np
import mujoco_py
import rospkg
import rospy
import os
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from roboy_middleware_msgs.msg import MotorCommand
from roboy_simulation_msgs.msg import TendonUpdate
from sklearn.preprocessing import MinMaxScaler
import threading


P = 7500
I = 200
D = 250

P = int(sys.argv[1])
I = int(sys.argv[2])
D = int(sys.argv[3])

bagname = 'biceps_triceps'
add_filename = ''

logging = True
sp_length_logging=True

logdir = os.getcwd()+'/logfiles/%s' % bagname + add_filename
currlog = os.path.join(logdir, 'P%iI%iD%i' % (P, I, D))
if not os.path.exists(currlog):
    os.makedirs(currlog)


if logging:
    logfile = open('%s/forces' % currlog, 'w')
if sp_length_logging:
    logfile_setpoints = open('%s/setpoints' % currlog, 'w')
    logfile_lengths = open('%s/lengths' % currlog, 'w')



model_xml = "/code/mujoco_models/model_physics_nolimit_pull.xml"

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
    setpoint = np.zeros(38)

    biceps_ctrl = [0.08984985, 0.07165882, 0.08991515, 0.14156592, 0.13995287,
       0.1458982 , 0.06204619, 0.13818123, 0.05092336, 0.10209811,
       0.10438676, 0.09837203, 0.07257863, 0.05936345, 0.12258356,
       0.100394  , 0.06046219, 0.12035582, 0.06931697, 0.12348682]
    #16 17

    tri_bi=[[0.14178941, 0.0768353], [0.14557084, 0.07201766], [0.11295168, 0.11029518]]
    biceps_ctrl_idx=0

    while not rospy.is_shutdown():
        
            
        if sim_step == 100:
            setpoint[16:18] = tri_bi[biceps_ctrl_idx]
            biceps_ctrl_idx+=1

        if sim_step >= warmup_step:
            
            if (sim_step%1000 == 0):
                setpoint[16:18] = tri_bi[biceps_ctrl_idx]
                biceps_ctrl_idx+=1
            #   setpoint[16:18] = tri_bi[1]

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

        if (sim_step == 6010):
            exit()
