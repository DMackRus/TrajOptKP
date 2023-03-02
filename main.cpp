#include "stdInclude/stdInclude.h"
#include "physicsSimulators/MuJoCoHelper.h"

int main() {

    // initialise robot
    vector<robot> robots;
    robot panda;
    panda.name = "panda";
    panda.jointNames = {"panda0_joint1", "panda0_joint2", "panda0_joint3", "panda0_joint4", "panda0_joint5", "panda0_joint6", "panda0_joint7"};
    panda.numActuators = 7;
    robots.push_back(panda);
    vector<string> bodies;
    bodies.push_back("goal");

    MuJoCoHelper myHelper(robots, bodies);
    myHelper.setupMuJoCoWorld(0.004, "Franka-emika-panda-arm/V1/cheezit_pushing.xml");

    bool success = myHelper.setRobotJointsPositions("panda", {1,0.5,0,-1,0,0.6,1});

    pose cheezitPose;
    cheezitPose.position = {0.5,0.5,0};
    cheezitPose.quat = {0,0,0,1};

    myHelper.setBodyPose("goal", cheezitPose);

    myHelper.render();


    return 0;
}