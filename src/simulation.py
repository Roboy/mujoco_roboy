import numpy as np
import mujoco_py
import rospkg
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from roboy_middleware_msgs.msg import MotorCommand
from os.path import dirname
from sklearn.preprocessing import MinMaxScaler


model_xml = "/code/mujoco_models/model_fused.xml"

joint_names = [
    ['shoulder_right_axis0', 'shoulder_right_axis1', 'shoulder_right_axis2', 
    'elbow_right_axis0', 'elbow_right_axis1', 
    'wrist_right_axis0', 'wrist_right_axis1', 'wrist_right_axis2'],
    ['head_axis0', 'head_axis1', 'head_axis2'], 
    ['shoulder_left_axis0','shoulder_left_axis1','shoulder_left_axis2',
    'elbow_left_axis0', 'elbow_left_axis1',
    'wrist_left_axis0', 'wrist_left_axis1', 'wrist_left_axis2']
]

model = mujoco_py.load_model_from_path(model_xml)
sim = mujoco_py.MjSim(model)
viewer = mujoco_py.MjViewer(sim)

n_motors = 38
warmup_step = 10
sim_step = 0
Kp = 100.0 * np.ones(n_motors)
Kd = 10.0 * np.ones(n_motors)
Ki = 0.0 # 10.0
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


if __name__ == '__main__':

    topic_root = "/roboy/pinky"

    tendon_target_sub = rospy.Subscriber(f"{topic_root}/middleware/MotorCommand", MotorCommand, tendon_target_cb)
    pub = rospy.Publisher(f"{topic_root}/external_joint_states", JointState, queue_size=1)
    rospy.init_node("mujoco_roboy")

    print("Simulation started!")

    freq = 300
    rate = rospy.Rate(freq)
    dt = 1 / freq

    sim_step = 0
    while not rospy.is_shutdown():

        if sim_step >= warmup_step:
            error = setpoint - sim.data.ten_length
            error_int += Ki * error * dt
            force = Kp * error + Kd * (error - error_prev) / dt + error_int

            ctrl_idx = np.where(setpoint != 0)[0]
            sim.data.ctrl[ctrl_idx] = force[ctrl_idx]

            error_prev = error

        sim.step()
        publish_joint_states(pub, sim.data.qpos)

        sim_step += 1
        viewer.render()

        rate.sleep()
