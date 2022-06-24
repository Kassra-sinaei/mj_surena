#!/usr/bin/env python3

from mujoco_py import load_model_from_path, MjSim
import mujoco_py
from mujoco_py.mjviewer import MjViewer

def test_viewer():
    model = load_model_from_path("Model/surenaV.xml")
    sim = MjSim(model)
    viewer = MjViewer(sim)

    for _ in range(1000):
        sim.step()
        viewer.render()
 
if __name__ == "__main__":
    test_viewer()
    pass
