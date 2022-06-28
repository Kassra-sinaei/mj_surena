#!/usr/bin/env python3

from ast import walk
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

p_gain = None
d_gain = None
q_old = None
solver_frequency = 200

model = None
sim = None
viewer = None

def set_pose(q_ref):
    global p_gain, d_gain, q_old, solver_frequency, model, sim, viewer

    q = np.empty(12)
    q[0] = sim.data.get_joint_qpos("r_hip_yaw_joint")
    q[1] = sim.data.get_joint_qpos("r_hip_roll_joint")
    q[2] = -sim.data.get_joint_qpos("r_hip_pitch_joint")
    q[3] = -sim.data.get_joint_qpos("r_knee_joint")
    q[4] = -sim.data.get_joint_qpos("r_ankle_pitch_joint")
    q[5] = -sim.data.get_joint_qpos("r_ankle_roll_joint")
    q[6] = sim.data.get_joint_qpos("l_hip_yaw_joint")
    q[7] = sim.data.get_joint_qpos("l_hip_roll_joint")
    q[8] = sim.data.get_joint_qpos("l_hip_pitch_joint")
    q[9] = -sim.data.get_joint_qpos("l_knee_joint")
    q[10] = -sim.data.get_joint_qpos("l_ankle_pitch_joint")
    q[11] = -sim.data.get_joint_qpos("l_ankle_roll_joint")

    dq = (q - q_old) * solver_frequency
    u = (q_ref - q) * p_gain + (0 - dq) * d_gain
    q_old = q

    sim.data.set_joint_qvel("r_hip_yaw_joint", u[0])
    sim.data.set_joint_qvel("r_hip_roll_joint", u[1])
    sim.data.set_joint_qvel("r_hip_pitch_joint", -u[2])
    sim.data.set_joint_qvel("r_knee_joint", -u[3])
    sim.data.set_joint_qvel("r_ankle_pitch_joint", -u[4])
    sim.data.set_joint_qvel("r_ankle_roll_joint", -u[5])
    sim.data.set_joint_qvel("l_hip_yaw_joint", u[6])
    sim.data.set_joint_qvel("l_hip_roll_joint", u[7])
    sim.data.set_joint_qvel("l_hip_pitch_joint", u[8])
    sim.data.set_joint_qvel("l_knee_joint", -u[9])
    sim.data.set_joint_qvel("l_ankle_pitch_joint", -u[10])
    sim.data.set_joint_qvel("l_ankle_roll_joint", -u[11])
    pass

def test_viewer():
    global p_gain, d_gain, q_old, solver_frequency, model, sim, viewer

    sleep(0.2)
    print(sim.data.sensordata)
    for _ in range(solver_frequency * 5):
        sim.step()
        pos = np.zeros(12)
        set_pose(pos)
        viewer.render()
        
def offline_walk():
    global p_gain, d_gain, q_old, solver_frequency, model, sim, viewer

    dt = 1/solver_frequency
    # Plan CoM Trajectory
    planner = DCMPlanner(0.68, 1.0, 0.1,dt)
    rF =np.array([[0.0,-0.115,0.0],
                [0.25,0.115,0.0],
                [0.5,-0.115,0.0],
                [0.75,0.115,0.0],
                [1.0,-0.115,0.0],
                [1.25,0.115,0.0]])
    planner.setFoot(rF)
    xi_trajectory = planner.getXiTrajectory()
    com_0 = np.array([0.0,0.0,planner.deltaZ_])
    com_trajectory = planner.getCoMTrajectory(com_0)
    # Plan Ankle Trajectory
    anklePlanner = Ankle(planner.tStep_, planner.tDS_, 0.03, dt)
    rF =np.array([[0.0,0.115,0.0],
                [0.0,-0.115,0.0],
                [0.25,0.115,0.0],
                [0.5,-0.115,0.0],
                [0.75,0.115,0.0],
                [1.0,-0.115,0.0],
                [1.25,0.115,0.0],
                [1.25,-0.115,0.0]])
    anklePlanner.updateFoot(rF)
    anklePlanner.generateTrajectory()
    left = np.array(anklePlanner.getTrajectoryL())
    right = np.array(anklePlanner.getTrajectoryR())

    surena = Robot(shank = 0.36, hip = 0.35, pelvis_lengt = 0.0975)

    # Move CoM Down
    for i in range(solver_frequency * 1):
        q = surena.doIK([0.0,0.0,0.71 - (i/solver_frequency * 1) * (0.71-planner.deltaZ_)], np.eye(3),[0.0,0.0975,0.0], np.eye(3),[0.0, -0.0975,0.0], np.eye(3))
        sim.step()
        set_pose(np.array(q))
        viewer.render()
    
    # Walk 
    for i in range (int(solver_frequency * ((rF.shape[0] - 2) * planner.tStep_))):
        q = surena.doIK(com_trajectory[i], np.eye(3),left[i], np.eye(3),right[i], np.eye(3))
        sim.step()
        set_pose(np.array(q))
        viewer.render()


if __name__ == "__main__":
    model = load_model_from_path("Model/surenaV.xml")
    sim = MjSim(model)
    viewer = MjViewer(sim)

    p_gain = np.array([50] * 12)
    d_gain = np.array([0.1] * 12)
    q_old = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    offline_walk()
    #test_viewer()
    #print(sim.data.geom_xpos)
    pass
