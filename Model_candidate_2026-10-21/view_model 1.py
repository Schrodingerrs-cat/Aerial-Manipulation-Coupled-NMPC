import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path("aerial_manipulator_custom.xml")
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)

mujoco.viewer.launch(m, d)
