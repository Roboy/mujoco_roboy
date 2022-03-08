import numpy as np
import mujoco_py
import rospkg
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from roboy_middleware_msgs.msg import MotorCommand
from os.path import dirname
from sklearn.preprocessing import MinMaxScaler
import threading


model_xml = "/code/mujoco_models/model.xml"

joint_names = None 
end_effectors = None
joint_ids = None

model = mujoco_py.load_model_from_path(model_xml)
sim = mujoco_py.MjSim(model)
viewer = mujoco_py.MjViewer(sim)

n_motors = 38
warmup_step = 10
sim_step = 0
Kp = 40.0 * np.ones(n_motors) # 30
Kd = 10.0 * np.ones(n_motors) # 5
Ki = 0.0 # 10.0
setpoint = np.zeros(n_motors)

# Head
Kp[np.arange(20, 25)] = 180.0
Kd[np.arange(20, 25)] = 10.0

freq = 300
rate = None
dt = 1 / freq
publisher = None

def tendon_target_cb(data):

    global setpoint

    control_id = list(data.global_id)
    setpoint[control_id] = -np.array(data.setpoint)

def publish_joint_states(publisher, joint_states):

    start_idx = 0

    for id in joint_ids:
            
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        msg.name = np.array(joint_names)[id]
        msg.position = joint_states[id]
        msg.velocity = [0] * len(id)
        msg.effort = [0] * len(id)
        
        publisher.publish(msg)

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

        publish_joint_states(publisher, sim.data.qpos)
        rate.sleep()


if __name__ == '__main__':

    topic_root = "/roboy/pinky"

    tendon_target_sub = rospy.Subscriber(f"{topic_root}/middleware/MotorCommand", MotorCommand, tendon_target_cb)
    publisher = rospy.Publisher(f"{topic_root}/external_joint_states", JointState, queue_size=1)
    rospy.init_node("mujoco_roboy")

    joint_names = rospy.get_param("/joint_names")
    end_effectors = rospy.get_param("/endeffectors")

    joint_ids = []
    for ee in end_effectors:
        name = rospy.get_param(f'{ee}/joints')
        joint_ids.append([joint_names.index(n) for n in name])

    rate = rospy.Rate(freq)

    # Create control thread
    control_thread = threading.Thread(target=control_runner)
    for i in range(warmup_step):
        sim.step()
    control_thread.start()

    # Now start the simulation
    print("Simulation started!")
    sim_rate = rospy.Rate(1000)
    sim_step = 0
    while not rospy.is_shutdown():
        sim.step()
        viewer.render()
        sim_step += 1
        sim_rate.sleep()
