import argparse
import time
import numpy as np
import mujoco 
import mujoco.viewer
modelpth = "../quadrotor_verify_mass_inertia.xml"
model = mujoco.MjModel.from_xml_path(modelpth)
data = mujoco.MjData(model)

drone_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "drone_1")
mass = model.body_mass[drone_id]
inertia = model.body_inertia[drone_id]
weight = mass * 9.81

gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "drone_1_gyro")
gyro_adr = model.sensor_adr[gyro_id]

mode = 1 # 0 free fall , 1 hover

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()
        if mode == 0:
            data.ctrl[:] = 0.0
        else:
            data.ctrl[0] = weight
            data.ctrl[1:4] = 0.0 

        mujoco.mj_step(model, data)
        viewer.sync()

        elapsed = time.time() - step_start
        remaining = model.opt.timestep - elapsed
        if remaining > 0:
            time.sleep(remaining)

        


