#include "reaching.h"

pandaReaching::pandaReaching(): modelTranslator(){
    filePath = "/home/davidrussell/catkin_ws/src/physicsSimSwitching/Franka-emika-panda-arm/V1/reaching_scene.xml";
    reachingNumCtrl = 7;

    vector<robot> robots;
    robot panda;
    panda.name = "panda";
    panda.jointNames = {"panda0_joint1", "panda0_joint2", "panda0_joint3", "panda0_joint4", "panda0_joint5", "panda0_joint6", "panda0_joint7"};
    panda.numActuators = 7;
    robots.push_back(panda);

    vector<bodyStateVec> bodies;

    initModelTranslator(filePath, reachingNumCtrl, robots, bodies);
    std::cout << "initialised reaching model translator" << std::endl;
}