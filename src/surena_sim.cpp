#include "surena_sim.h"

SurenaSim::SurenaSim(/* args */){
    window_ = glfwCreateWindow(1200, 900, "Surena V Simulation", NULL, NULL);
    model_ = mj_loadXML("Model/surenaV.xml", NULL, NULL, NULL);
    data_ = mj_makeData(model_);
}

void SurenaSim::run(double time){
    while(data_->time < time){
        mj_step(model_, data_);
    }
}

SurenaSim::~SurenaSim(){
    mj_deleteModel(model_);
    mj_deleteData(data_);
}
