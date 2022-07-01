#pragma once

#include "glfw3.h"
#include "mujoco.h"
#include "iostream"

class SurenaSim
{
private:
    mjModel* model_;
    mjData* data_;
    GLFWwindow* window_;

public:
    SurenaSim(/* args */);
    void run(double time);
    ~SurenaSim();
};

