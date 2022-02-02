import numpy as np
import rospkg
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from roboy_middleware_msgs.msg import MotorCommand
from os.path import dirname
from sklearn.preprocessing import MinMaxScaler


joint_names = [
 'shoulder_right_axis0', 'shoulder_right_axis1', 'shoulder_right_axis2', 
 'elbow_right_axis0', 'elbow_right_axis1', 
 'wrist_right_axis0', 'wrist_right_axis1', 'wrist_right_axis2',
 'head_axis0', 'head_axis1', 'head_axis2', 
 'shoulder_left_axis0','shoulder_left_axis1','shoulder_left_axis2',
 'elbow_left_axis0', 'elbow_left_axis1',
 'wrist_left_axis0', 'wrist_left_axis1', 'wrist_left_axis2'
]

n_motors = 38
warmup_step = 10
sim_step = 0
topic_root = "/roboy/pinky"
pub = rospy.Publisher(f"{topic_root}/control/joint_targets", JointState, queue_size=1)

def ext_target_cb(data):

    msg = JointState()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.name = data.name
    msg.position = data.position
    msg.velocity = [0] * len(data.name)
    msg.effort = [0] * len(data.name)

    pub.publish(msg)

def simulation_target_cb(data):

    joint_states = [0.0] * len(joint_names)
    for j, j_name in enumerate(data.name):
        index = joint_names.index(j_name)
        joint_states[index] = data.position[j]

    msg = JointState()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.name = joint_names
    msg.position = joint_states
    msg.velocity = [0] * len(joint_states)
    msg.effort = [0] * len(joint_states)

    pub.publish(msg)

if __name__ == '__main__':

    # tendon_target_sub = rospy.Subscriber(f"{topic_root}/simulation/joint_targets", JointState, simulation_target_cb)
    ext_target_sub = rospy.Subscriber(f"{topic_root}/sensing/external_joint_states", JointState, ext_target_cb)
    rospy.init_node("mujoco_roboy_test")
    rospy.spin()
