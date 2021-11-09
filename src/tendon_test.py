import numpy as np
import mujoco_py
from os.path import dirname

# model_xml = "/src/model/arm26.xml"
model_xml = "/src/usc_models/nmi_leg_w_chassis_v0.xml"

model = mujoco_py.load_model_from_path(model_xml)
sim = mujoco_py.MjSim(model)

viewer = mujoco_py.MjViewer(sim)
print(model.actuator_lengthrange)

# Set initial state
# state = sim.get_state()
# state.qpos[0] = 0.5
# sim.set_state(state)

control_step = range(0, 5000, 800)

# 0: Green, 1: Red, 2: Blue

for i in range(5000):

	control_tendon = False

	for j in control_step:
		if i > j and i < j + 50:
			control_tendon = True
			break

	if control_tendon:
		sim.data.ctrl[1] = 0.5
	else:
		sim.data.ctrl[:] = 0.0
	
	sim.step()
	viewer.render()
	# print(sim.data.qpos)
