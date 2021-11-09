import numpy as np
import mujoco_py
from os.path import dirname

model_xml = "/src/opensim_models/MoBL_ARMS_model_for_mujoco_converted/MoBL_ARMS_model_for_mujoco_converted.xml"

model = mujoco_py.load_model_from_path(model_xml)
sim = mujoco_py.MjSim(model)

viewer = mujoco_py.MjViewer(sim)
print(model.actuator_lengthrange)

for i in range(5000):
	
	sim.step()
	viewer.render()
	# print(sim.data.qpos)
