#!/usr/bin/env python3

from statistics import mode
from mujoco_py import load_model_from_path, MjSim
import mujoco_py
from mujoco_py.mjviewer import MjViewer
from time import sleep

def test_viewer():
    model = load_model_from_path("Model/surenaV.xml")
    sim = MjSim(model)
    sim.forward()
    viewer = MjViewer(sim)
    #print(sim.data.geom_xpos)
    sleep(1)
    print(sim.data.sensordata)
    for _ in range(1000):
        sim.step()
        sim.data.set_joint_qvel("r_hip_yaw_joint", 0)
        sim.data.set_joint_qvel("r_hip_roll_joint", 0)
        sim.data.set_joint_qvel("r_hip_pitch_joint", 0)
        sim.data.set_joint_qvel("r_knee_joint", 0)
        sim.data.set_joint_qvel("r_ankle_pitch_joint", 0)
        sim.data.set_joint_qvel("r_ankle_roll_joint", 0)
        sim.data.set_joint_qvel("l_hip_yaw_joint", 0)
        sim.data.set_joint_qvel("l_hip_roll_joint", 0)
        sim.data.set_joint_qvel("l_hip_pitch_joint", 0)
        sim.data.set_joint_qvel("l_knee_joint", 0)
        sim.data.set_joint_qvel("l_ankle_pitch_joint", 0)
        sim.data.set_joint_qvel("l_ankle_roll_joint", 0)
        viewer.render()
        
    sleep(1)
    sim.reset()
    for _ in range(1000):
        sim.step()
        sim.data.set_joint_qpos("r_hip_yaw_joint", 0)
        sim.data.set_joint_qpos("r_hip_roll_joint", 0)
        sim.data.set_joint_qpos("r_hip_pitch_joint", 0)
        sim.data.set_joint_qpos("r_knee_joint", 0)
        sim.data.set_joint_qpos("r_ankle_pitch_joint", 0)
        sim.data.set_joint_qpos("r_ankle_roll_joint", 0)
        sim.data.set_joint_qpos("l_hip_yaw_joint", 0)
        sim.data.set_joint_qpos("l_hip_roll_joint", 0)
        sim.data.set_joint_qpos("l_hip_pitch_joint", 0)
        sim.data.set_joint_qpos("l_knee_joint", 0)
        sim.data.set_joint_qpos("l_ankle_pitch_joint", 0)
        sim.data.set_joint_qpos("l_ankle_roll_joint", 0)
        viewer.render()
 
if __name__ == "__main__":
    test_viewer()
    pass
