#!/usr/bin/env python3

from statistics import mode
from mujoco_py import load_model_from_path, MjSim
import mujoco_py
from mujoco_py.mjviewer import MjViewer

from time import sleep
import numpy as np
from time import sleep, time

from trajectory_generation.Robot import *
from trajectory_generation.DCM import *
from trajectory_generation.Ankle import *

p_gain = np.array([0.2] * 12)
d_gain = np.array([0.0001] * 12)
q_old = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
solver_frequency = 200

def set_pose(q_ref, sim):
    q = list([0] * 12)
    q[0] = sim.data.get_joint_qvel("r_hip_yaw_joint")
    q[1] = sim.data.get_joint_qvel("r_hip_roll_joint")
    q[2] = sim.data.get_joint_qvel("r_hip_pitch_joint")
    q[3] = sim.data.get_joint_qvel("r_knee_joint")
    q[4] = sim.data.get_joint_qvel("r_ankle_pitch_joint")
    q[5] = sim.data.get_joint_qvel("r_ankle_roll_joint")
    q[6] = sim.data.get_joint_qvel("l_hip_yaw_joint")
    q[7] = sim.data.get_joint_qvel("l_hip_roll_joint")
    q[8] = sim.data.get_joint_qvel("l_hip_pitch_joint")
    q[9] = sim.data.get_joint_qvel("l_knee_joint")
    q[10] = sim.data.get_joint_qvel("l_ankle_pitch_joint")
    q[11] = sim.data.get_joint_qvel("l_ankle_roll_joint")

    dq = (q - q_old) * solver_frequency
    u = (q_ref - q) * p_gain + (0 - dq) * d_gain

    sim.data.set_joint_qvel("r_hip_yaw_joint", u[0])
    sim.data.set_joint_qvel("r_hip_roll_joint", u[1])
    sim.data.set_joint_qvel("r_hip_pitch_joint", u[2])
    sim.data.set_joint_qvel("r_knee_joint", u[3])
    sim.data.set_joint_qvel("r_ankle_pitch_joint", u[4])
    sim.data.set_joint_qvel("r_ankle_roll_joint", u[5])
    sim.data.set_joint_qvel("l_hip_yaw_joint", u[6])
    sim.data.set_joint_qvel("l_hip_roll_joint", u[7])
    sim.data.set_joint_qvel("l_hip_pitch_joint", u[8])
    sim.data.set_joint_qvel("l_knee_joint", u[9])
    sim.data.set_joint_qvel("l_ankle_pitch_joint", u[10])
    sim.data.set_joint_qvel("l_ankle_roll_joint", u[11])
    pass

def test_viewer(model, sim, viewer):
    sleep(1)
    print(sim.data.sensordata)
    for _ in range(solver_frequency * 10):
        sim.step()
        set_pose(np.zeros(12), sim)
        viewer.render()
        

if __name__ == "__main__":
    model = load_model_from_path("Model/surenaV.xml")
    sim = MjSim(model)
    viewer = MjViewer(sim)
    test_viewer(model, sim, viewer)
    #print(sim.data.geom_xpos)
    pass
