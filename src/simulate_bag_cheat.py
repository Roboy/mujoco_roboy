import numpy as np
import mujoco_py
import rospkg
import rospy
from sensor_msgs.msg import JointState
from roboy_middleware_msgs.msg import MotorCommand
from os.path import dirname
from sklearn.preprocessing import MinMaxScaler


model_xml = "/src/roboy_model/model_fused_cheat.xml"

model = mujoco_py.load_model_from_path(model_xml)
sim = mujoco_py.MjSim(model)
viewer = mujoco_py.MjViewer(sim)

n_motors = 38
warmup_step = 10
sim_step = 0
Kp = 90000.0 * np.ones(n_motors)
Kd = 1000.0 * np.ones(n_motors)
Ki = 100.0 # 10.0
setpoint = np.zeros(n_motors)
error_prev = np.zeros(n_motors)
error_int = np.zeros(n_motors)

# Head
Kp[np.arange(20, 25)] = 300000.0
Kd[np.arange(20, 25)] = 20000.0

def tendon_target_cb(data):

    global sim, setpoint, error_prev, error_int

    control_id = list(data.global_id)
    setpoint[control_id] = -np.array(data.setpoint)


if __name__ == '__main__':

    topic_root = "/roboy/pinky"

    tendon_target_sub = rospy.Subscriber(f"{topic_root}/middleware/MotorCommand", MotorCommand, tendon_target_cb)
    pub = rospy.Publisher(f"{topic_root}/sensing/external_joint_states", JointState, queue_size=1)
    rospy.init_node("mujoco_roboy")

    freq = 300
    rate = rospy.Rate(freq)
    dt = 1 / freq

    sim_step = 0
    while not rospy.is_shutdown():

        if sim_step >= warmup_step:
            error = setpoint - sim.data.ten_length
            print( error)
            error_int += Ki * error * dt
            force = (Kp * error + Kd * (error - error_prev) / dt + error_int)*1
            np.clip(force, -500, 0)

            ctrl_idx = np.where(setpoint != 0)[0]

            sim.data.ctrl[ctrl_idx] = force[ctrl_idx] 
            

            error_prev = error

        sim.step()
        sim_step += 1
        viewer.render()

        rate.sleep()