import argparse
import time
import numpy as np
import mujoco 
import mujoco.viewer

modelpth = "../quadrotor_verify_mass_inertia.xml"
def hat(v):
    #3x3 skew-symmetric matrix.
    return np.array([
        [ 0.0, -v[2], v[1]],
        [ v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0 ]
    ])
def vee(M):
    #Inverse of hat. To extracts the 3-vector from a skew-symmetric matrix
    return np.array([M[2, 1], M[0, 2], M[1, 0]])

# to get the pos x_d = [cos(ωt), sin(ωt), 3] and then taking the derative to get velocity and acc
def circle_trajectory(t, r=1.0, omega=0.5, z0=3.0):
    x_d = np.array([r * np.cos(omega * t), r * np.sin(omega * t), z0])
    v_d = np.array([-r * omega * np.sin(omega * t), r * omega * np.cos(omega * t), 0.0])
    x_dd = np.array([-r * omega**2 * np.cos(omega * t), -r * omega**2 * np.sin(omega * t), 0.0])
    return x_d, v_d, x_dd

def lissajous_trajectory(t, a=1.5, b=1.0, omega=0.4, z0=3.0, z_amp=0.5):
    x_d = np.array([a * np.sin(omega * t), b * np.sin(2 * omega * t), z0 + z_amp * np.sin(0.5 * omega * t)])
    v_d = np.array([a * omega * np.cos(omega * t), 2 * b * omega * np.cos(2 * omega * t), 0.5 * z_amp * omega * np.cos(0.5 * omega * t)])
    x_dd = np.array([-a * omega**2 * np.sin(omega * t), -4 * b * omega**2 * np.sin(2 * omega * t), -0.25 * z_amp * omega**2 * np.sin(0.5 * omega * t)])
    return x_d, v_d, x_dd

def run(mode, traj_type):

    model = mujoco.MjModel.from_xml_path(modelpth)
    data = mujoco.MjData(model)

    drone_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "drone_1")
    mass = model.body_mass[drone_id]
    inertia = model.body_inertia[drone_id]
    weight = mass * 9.81

    gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "drone_1_gyro")
    gyro_adr = model.sensor_adr[gyro_id]

    def compute():
        # pose
        x= data.qpos[0:3].copy()
        x_d = np.array([1.0, 2.0, 3.0]) # target location

        # velocity
        v= data.qvel[0:3].copy()
        v_d = np.zeros(3) 

        #acceleration
        x_dd = np.zeros(3)

        if mode == "trajectory":
            x_d, v_d, x_dd = traj_func(data.time)

        #roatation
        quat = data.qpos[3:7].copy()
        # R = quat_to_rot(quat)
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        R = R.reshape(3, 3)

        #Omega
        Omega = data.sensordata[gyro_adr:gyro_adr+3].copy()
        Omega_d = np.zeros(3)
        Omega_d_dot = np.zeros(3)

        # Position PD (outer loop)
        k_v = 4.0
        k_x = 4.0 
        # Attitude PD (inner loop)
        k_R = 0.4 # proportional
        k_Omega = 0.15 # derivative

        #Outer loop
        #1 error
        e_x = x - x_d
        e_v = v - v_d

        m = mass
        g = 9.81
        J = np.diag(inertia) 
        e3 = np.array([0.0, 0.0, 1.0]) #zup in mujoco

        #Desired force vector (proportional + damper +gravit)
        A = -k_x * e_x - k_v * e_v + m * g * e3 + m * x_dd

        #Thrust magnitude
        F = np.dot(A, R @ e3)

        #inner loop to construct r_d
        A_norm = np.linalg.norm(A)
        b3_d = (A / A_norm) # no - here as the z is point up for mujoco
        #desired heading
        b1_d = np.array([1.0, 0.0, 0.0])
        b2_d = np.cross(b3_d, b1_d)
        b2_d = b2_d / np.linalg.norm(b2_d)

        R_d = np.column_stack([np.cross(b2_d, b3_d), b2_d, b3_d])

        #attitude error 
        e_R = 0.5 * vee(R_d.T @ R - R.T @ R_d)

        #Angular velocity error
        # e_Omega = Omega - R.T @ R_d @ Omega_d
        e_Omega = Omega

        #Control moment
        feedforward = J @ (hat(Omega) @ R.T @ R_d @ Omega_d- R.T @ R_d @ Omega_d_dot)

        #final moment that decides the tilt of the robot 
        M = (-k_R * e_R - k_Omega * e_Omega + np.cross(Omega, J @ Omega) - feedforward)
        return F,M,x,x_d

    trail = []
    ref_trail = []

    if mode == "trajectory":
        if traj_type == 0:
            traj_func = circle_trajectory
        else:
            traj_func = lissajous_trajectory
        x0, _, _ = traj_func(0.0)
        data.qpos[0:3] = x0
        mujoco.mj_forward(model, data)
        #save reference pos in in the ref_trail so we can visualize
        for t_ref in np.arange(0, 60, 0.2):
            pt, _, _ = traj_func(t_ref)
            ref_trail.append(pt.copy())

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start = time.time()
        while viewer.is_running() and time.time() - start < 60:
            step_start = time.time()
            F,M ,x,x_d= compute()

            #sending f and m to mujoco with clipping
            #thrust
            data.ctrl[0] = np.clip(F, 0.0, 40.0)
            #Moment
            data.ctrl[1] = np.clip(M[0], -0.5, 0.5)
            data.ctrl[2] = np.clip(M[1], -0.5, 0.5)
            data.ctrl[3] = np.clip(M[2], -0.5, 0.5)
            
            mujoco.mj_step(model, data)

            if data.time % 0.5 < model.opt.timestep:
                err = np.linalg.norm(x - x_d)
                print(f"t={data.time:6.2f}"
                    f"pos=[{x[0]:+.3f},{x[1]:+.3f},{x[2]:+.3f}]"
                    f"|e|={err:.4f} ")
            viewer.sync()
            #
            # store trail point every 0.1s
            if data.time % 0.1 < model.opt.timestep:
                trail.append(x.copy())

            # reseting scene to draw markers
            viewer.user_scn.ngeom = 0

            # reference path (green dots)
            if mode == "trajectory":
                for i, pt in enumerate(ref_trail):
                    if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom:
                        break
                    idx = viewer.user_scn.ngeom
                    #mujoco.mjv_initGeom( which slot to use,shape type, size (radius),  position (the target location),
                    #orientation (identity = no rotation),color [R, G, B, Alpha])
                    mujoco.mjv_initGeom(viewer.user_scn.geoms[idx], mujoco.mjtGeom.mjGEOM_SPHERE, [0.01, 0, 0], pt, np.eye(3).flatten(), [0, 1, 0, 0.3])
                    viewer.user_scn.ngeom += 1

            # current target
            idx = viewer.user_scn.ngeom
            mujoco.mjv_initGeom(viewer.user_scn.geoms[idx], mujoco.mjtGeom.mjGEOM_SPHERE, [0.06, 0, 0], x_d, np.eye(3).flatten(), [0, 1, 0, 1])
            viewer.user_scn.ngeom += 1

            # trail (red dots)
            for i, pt in enumerate(trail[-200:]):
                idx = i + 1
                if idx >= viewer.user_scn.maxgeom:
                    break
                idx = viewer.user_scn.ngeom
                mujoco.mjv_initGeom(viewer.user_scn.geoms[idx],mujoco.mjtGeom.mjGEOM_SPHERE, [0.02, 0, 0],pt, np.eye(3).flatten(), [1, 0, 0, 0.5])
                viewer.user_scn.ngeom = idx + 1
            #

            elapsed = time.time() - step_start
            remaining = model.opt.timestep - elapsed
            if remaining > 0:
                time.sleep(remaining)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["fixed", "trajectory"], default="fixed")
    parser.add_argument("--traj", type=int,choices=[0, 1], default=0)
    args = parser.parse_args()
    run(args.mode, args.traj)



