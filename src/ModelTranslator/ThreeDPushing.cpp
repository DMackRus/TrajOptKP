#include "ThreeDPushing.h"

ThreeDPushing::ThreeDPushing() : PushBaseClass("franka_gripper", "goal"){

    std::string yamlFilePath = "/taskConfigs/pushThreeDConfig.yaml";

    InitModelTranslator(yamlFilePath);
}

void ThreeDPushing::GenerateRandomGoalAndStartState() {
    X_start = ReturnRandomStartState();
    X_desired = ReturnRandomGoalState(X_start);
}

MatrixXd ThreeDPushing::ReturnRandomStartState(){
    MatrixXd randomStartState(state_vector_size, 1);

    float startX;
    float startY;
    float goalX;
    float goalY;

    if(clutterLevel == constrainedClutter){
        startX = randFloat(0.45, 0.46);
        startY = randFloat(-0.05, 0.05);

        goalX = randFloat(0.6, 0.65);
        goalY = randFloat(-0.2, 0.2);

    }
    else{
//        float randStartAngle = randFloat(0, PI);
//        float randStartDist = randFloat(0.05, 0.1);

        startX = 0.4;
        startY = randFloat(-0.1, 0.1);

        float randAngle = randFloat(-PI/4, PI/4);
        float randDist = randFloat(0.28, 0.3);

        goalX = startX + randDist * cos(randAngle);
        goalY = startY + randDist * sin(randAngle);
    }

    // Set start position of pushed object
    pose_6 pushedObjectStartPose;
    MuJoCo_helper->getBodyPose_angle("blueTin", pushedObjectStartPose, MuJoCo_helper->master_reset_data);
    pushedObjectStartPose.position(0) = startX;
    pushedObjectStartPose.position(1) = startY;
    pushedObjectStartPose.position(2) = 0.032;
    MuJoCo_helper->setBodyPose_angle("blueTin", pushedObjectStartPose, MuJoCo_helper->main_data);
    MuJoCo_helper->setBodyPose_angle("blueTin", pushedObjectStartPose, MuJoCo_helper->master_reset_data);
    MuJoCo_helper->forwardSimulator(MuJoCo_helper->main_data);
    MuJoCo_helper->forwardSimulator(MuJoCo_helper->master_reset_data);


    randomGoalX = goalX;
    randomGoalY = goalY;

    std::vector<double> objectXPos;
    std::vector<double> objectYPos;

    if(clutterLevel == noClutter){
        randomStartState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                startX, startY,
                0, 0, 0, 0, 0, 0, 0,
                0, 0;
    }
    else if(clutterLevel == lowClutter  || clutterLevel == constrainedClutter){

        std::string objectNames[3] = {"bigBox", "smallBox","tallCylinder"};
        int validObjectCounter = 0;

        for(int i = 0; i < 3; i++){
            bool validPlacement = false;
            float sizeX = 0.01;
            float sizeY = 0.05;
            while(!validPlacement){
                sizeX += 0.0005;
                sizeY += 0.0001;

                float randX, randY;

                if(clutterLevel == constrainedClutter){
                    randX = randFloat(startX, goalX + 0.1);
                    randY = randFloat(goalY - sizeY, goalY + sizeY);
                }
                else{
                    randX = randFloat(goalX - sizeX, goalX);
                    randY = randFloat(goalY - sizeY, goalY + sizeY);
                }

                pose_6 objectCurrentPose;
                pose_6 newObjectPose;

                MuJoCo_helper->getBodyPose_angle(objectNames[i], objectCurrentPose, MuJoCo_helper->main_data);
                newObjectPose = objectCurrentPose;
                newObjectPose.position(0) = randX;
                newObjectPose.position(1) = randY;
                MuJoCo_helper->setBodyPose_angle(objectNames[i], newObjectPose, MuJoCo_helper->main_data);
                MuJoCo_helper->setBodyPose_angle(objectNames[i], newObjectPose, MuJoCo_helper->master_reset_data);

                if(MuJoCo_helper->checkBodyForCollisions(objectNames[i], MuJoCo_helper->main_data)){
                    cout << "invalid placement at : " << randX << ", " << randY << endl;
                }
                else{
                    validPlacement = true;
                    objectXPos.push_back(randX);
                    objectYPos.push_back(randY);
                }
            }
            validObjectCounter++;
        }

        //1, -0.07, 0, -3, 0.232, 1.34, 3, 0.232,
        randomStartState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                startX, startY, objectXPos[0], objectYPos[0], objectXPos[1], objectYPos[1], objectXPos[2], objectYPos[2],
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0;

    }
    else if(clutterLevel == heavyClutter){

        bool validPlacement = false;
        float sizeX = 0.08;
        float sizeY = 0.04;
        cout << "goal position: " << goalX << ", " << goalY << endl;
        while(!validPlacement){
            sizeX += 0.001;
            sizeY += 0.0005;

            float randX = randFloat(goalX + 0.1, goalX +  + 0.1 + sizeX);
            float randY = randFloat(goalY - sizeY, goalY + sizeY);

            pose_6 objectCurrentPose;
            pose_6 newObjectPose;

            MuJoCo_helper->getBodyPose_angle("obstacle5", objectCurrentPose, MuJoCo_helper->master_reset_data);
            newObjectPose = objectCurrentPose;
            newObjectPose.position(0) = randX;
            newObjectPose.position(1) = randY;
            newObjectPose.position(2) = objectCurrentPose.position(2);
            MuJoCo_helper->setBodyPose_angle("obstacle5", newObjectPose, MuJoCo_helper->main_data);
            MuJoCo_helper->setBodyPose_angle("obstacle5", newObjectPose, MuJoCo_helper->master_reset_data);

            if(MuJoCo_helper->checkBodyForCollisions("obstacle5", MuJoCo_helper->main_data)){
                cout << "first object invalid placement : " << randX << ", " << randY << endl;
            }
            else{
                MuJoCo_helper->forwardSimulator(MuJoCo_helper->main_data);
                MuJoCo_helper->forwardSimulator(MuJoCo_helper->master_reset_data);
                validPlacement = true;
                objectXPos.push_back(randX);
                objectYPos.push_back(randY);
            }
        }

        std::string objectNames[6] = {"mediumCylinder", "bigBox", "obstacle1","obstacle2", "obstacle3", "obstacle4"};

        for(int i = 0; i < 6; i++){
            bool validPlacement = false;
            float sizeX = 0.08;
            float sizeY = 0.04;
            while(!validPlacement){
                sizeX += 0.001;
                sizeY += 0.0005;

                float randX = randFloat(goalX - sizeX, goalX + (0.5 * sizeX));
                float randY = randFloat(goalY - sizeY, goalY + sizeY);

                pose_6 objectCurrentPose;
                pose_6 newObjectPose;

                MuJoCo_helper->getBodyPose_angle(objectNames[i], objectCurrentPose, MuJoCo_helper->master_reset_data);
                newObjectPose = objectCurrentPose;
                newObjectPose.position(0) = randX;
                newObjectPose.position(1) = randY;
                newObjectPose.position(2) = objectCurrentPose.position(2);
                MuJoCo_helper->setBodyPose_angle(objectNames[i], newObjectPose, MuJoCo_helper->main_data);
                MuJoCo_helper->setBodyPose_angle(objectNames[i], newObjectPose, MuJoCo_helper->master_reset_data);

                if(MuJoCo_helper->checkBodyForCollisions(objectNames[i], MuJoCo_helper->main_data)){
                    cout << "invalid placement at : " << randX << ", " << randY << endl;
                }
                else{
                    MuJoCo_helper->forwardSimulator(MuJoCo_helper->main_data);
                    MuJoCo_helper->forwardSimulator(MuJoCo_helper->master_reset_data);
                    validPlacement = true;
                    objectXPos.push_back(randX);
                    objectYPos.push_back(randY);
                }
            }
        }

        randomStartState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                startX, startY, objectXPos[0], objectYPos[0], objectXPos[1], objectYPos[1],
                objectXPos[2], objectYPos[2], objectXPos[3], objectYPos[3],
                objectXPos[4], objectYPos[4], objectXPos[5], objectYPos[5], objectXPos[6], objectYPos[6],
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0, 0, 0;
    }

    return randomStartState;
}

MatrixXd ThreeDPushing::ReturnRandomGoalState(MatrixXd X0){
    MatrixXd randomGoalState(state_vector_size, 1);

    if(clutterLevel == noClutter){
        randomGoalState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                randomGoalX, randomGoalY,
                0, 0, 0, 0, 0, 0, 0,
                0, 0;
    }
    else if(clutterLevel == lowClutter || clutterLevel == constrainedClutter){
        randomGoalState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                randomGoalX, randomGoalY, X0(9), X0(10), X0(11), X0(12), X0(13), X0(14),
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0;
    }
    else if(clutterLevel == heavyClutter){
        randomGoalState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                randomGoalX, randomGoalY, X0(9), X0(10), X0(11), X0(12),
                X0(13), X0(14), X0(15), X0(16),
                X0(17), X0(18), X0(19), X0(20), X0(21), X0(22),
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0, 0, 0;

    }


    return randomGoalState;
}

std::vector<MatrixXd> ThreeDPushing::CreateInitSetupControls(int horizonLength){
    std::vector<MatrixXd> initSetupControls;

    MuJoCo_helper->copySystemState(MuJoCo_helper->main_data, MuJoCo_helper->master_reset_data);
    MuJoCo_helper->forwardSimulator(MuJoCo_helper->main_data);

    // Pushing create init controls borken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goal_pos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goal_pos(0) = X_desired(7);
    goal_pos(1) = X_desired(8);
    EEWayPointsSetup(goal_pos, mainWayPoints, mainWayPointsTimings, horizonLength);
//    cout << "setup mainwaypoint 0: " << mainWayPoints[0] << endl;
//    cout << "setup mainWayPoint 1: " << mainWayPoints[1] << endl;

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);

    // Step 3 - follow the points via the jacobian
    initSetupControls = JacobianEEControl(goal_pos, allWayPoints);

    return initSetupControls;
}

std::vector<MatrixXd> ThreeDPushing::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> initControls;

    // Set the goal position so that we can see where we are pushing to
    std::string goalMarkerName = "display_goal";
    pose_6 displayBodyPose;
    displayBodyPose.position[0] = X_desired(7);
    displayBodyPose.position[1] = X_desired(8);
    displayBodyPose.position[2] = 0.0f;
    MuJoCo_helper->setBodyPose_angle(goalMarkerName, displayBodyPose, MuJoCo_helper->master_reset_data);

    // Pushing create init controls broken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goal_pos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goal_pos(0) = X_desired(7);
    goal_pos(1) = X_desired(8);
    EEWayPointsPush(goal_pos, mainWayPoints, mainWayPointsTimings, horizonLength);
//    cout << mainWayPoints.size() << " waypoints created" << endl;
//    cout << "mainwaypoint 0: " << mainWayPoints[1] << endl;
//    cout << "mainWayPoint 1: " << mainWayPoints[2] << endl;

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);

    // Step 3 - follow the points via the jacobian
    initControls = JacobianEEControl(goal_pos, allWayPoints);

    return initControls;
}

bool ThreeDPushing::TaskComplete(mjData *d, double &dist){
    bool taskComplete = false;

    MatrixXd currentState = ReturnStateVector(d);

    float x_diff = currentState(7) - X_desired(7);
    float y_diff = currentState(8) - X_desired(8);

    dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

    if(dist < 0.025){
        taskComplete = true;
    }


    return taskComplete;
}