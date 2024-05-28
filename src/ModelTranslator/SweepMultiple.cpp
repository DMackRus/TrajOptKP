#include "ModelTranslator/SweepMultiple.h"

SweepMultiple::SweepMultiple(): PushBaseClass("franka_gripper", "scoop"){

    std::string yamlFilePath = "/TaskConfigs/rigid_body_manipulation/Sweep_multiple.yaml";

    InitModelTranslator(yamlFilePath);
}

void SweepMultiple::ReturnRandomStartState(){

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
        startX = 0.4;
        startY = randFloat(-0.1, 0.1);

        float randAngle = randFloat(-PI/4, PI/4);
        float randDist = randFloat(0.28, 0.3);

        goalX = startX + randDist * cos(randAngle);
        goalY = startY + randDist * sin(randAngle);
    }

    // Set start position of pushed object
    pose_6 pushedObjectStartPose;
    MuJoCo_helper->GetBodyPoseAngle("scoop", pushedObjectStartPose, MuJoCo_helper->master_reset_data);
    pushedObjectStartPose.position(0) = startX;
    pushedObjectStartPose.position(1) = startY;
    pushedObjectStartPose.position(2) = 0.032;
    MuJoCo_helper->SetBodyPoseAngle("scoop", pushedObjectStartPose, MuJoCo_helper->main_data);
    MuJoCo_helper->SetBodyPoseAngle("scoop", pushedObjectStartPose, MuJoCo_helper->master_reset_data);
    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->main_data);
    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->master_reset_data);

    randomGoalX = goalX;
    randomGoalY = goalY;

    std::vector<std::string> object_names;

    if(clutterLevel == lowClutter  || clutterLevel == constrainedClutter){
        object_names.emplace_back("obstacle_1");
        object_names.emplace_back("obstacle_2");
        object_names.emplace_back("obstacle_3");

        int validObjectCounter = 0;

        for(const auto & objectName : object_names){
            bool validPlacement = false;
            float sizeX = 0.01;
            float sizeY = 0.05;
            while(!validPlacement){
                sizeX += 0.0005;
                sizeY += 0.0001;

                float randX, randY;

                if(clutterLevel == constrainedClutter){
                    randX = randFloat(startX, goalX + 0.1f);
                    randY = randFloat(goalY - sizeY, goalY + sizeY);
                }
                else{
                    randX = randFloat(goalX - sizeX, goalX);
                    randY = randFloat(goalY - sizeY, goalY + sizeY);
                }

                pose_6 objectCurrentPose;
                pose_6 newObjectPose;

                MuJoCo_helper->GetBodyPoseAngle(objectName, objectCurrentPose, MuJoCo_helper->main_data);
                newObjectPose = objectCurrentPose;
                newObjectPose.position(0) = randX;
                newObjectPose.position(1) = randY;
                MuJoCo_helper->SetBodyPoseAngle(objectName, newObjectPose, MuJoCo_helper->main_data);
                MuJoCo_helper->SetBodyPoseAngle(objectName, newObjectPose, MuJoCo_helper->master_reset_data);

                if(MuJoCo_helper->CheckBodyForCollisions(objectName, MuJoCo_helper->main_data)){
//                    cout << "invalid placement of " << objectName << " at : " << randX << ", " << randY << endl;
                }
                else{
                    validPlacement = true;
                }
            }
            validObjectCounter++;
        }
    }
    else if(clutterLevel == heavyClutter){
        object_names.emplace_back("obstacle_1");
        object_names.emplace_back("obstacle_2");
        object_names.emplace_back("obstacle_3");
        object_names.emplace_back("obstacle_4");
        object_names.emplace_back("obstacle_5");
        object_names.emplace_back("obstacle_6");
        object_names.emplace_back("obstacle_7");


        cout << "goal position: " << goalX << ", " << goalY << endl;

        for(const auto & objectName : object_names){
            bool validPlacement = false;
            float sizeX = 0.08;
            float sizeY = 0.04;
            while(!validPlacement){
                sizeX += 0.001;
                sizeY += 0.0005;

                float randX = randFloat(goalX - sizeX, goalX + (0.5f * sizeX));
                float randY = randFloat(goalY - sizeY, goalY + sizeY);

                pose_6 objectCurrentPose;
                pose_6 newObjectPose;

                MuJoCo_helper->GetBodyPoseAngle(objectName, objectCurrentPose, MuJoCo_helper->master_reset_data);
                newObjectPose = objectCurrentPose;
                newObjectPose.position(0) = randX;
                newObjectPose.position(1) = randY;
                MuJoCo_helper->SetBodyPoseAngle(objectName, newObjectPose, MuJoCo_helper->main_data);
                MuJoCo_helper->SetBodyPoseAngle(objectName, newObjectPose, MuJoCo_helper->master_reset_data);

                if(MuJoCo_helper->CheckBodyForCollisions(objectName, MuJoCo_helper->main_data)){
                    cout << "invalid placement at : " << randX << ", " << randY << endl;
                }
                else{
                    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->main_data);
                    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->master_reset_data);
                    validPlacement = true;
                }
            }
        }
    }

    // Robot start configuration
    double robot_start_config[7] = {0, -0.183, 0, -3.1, 0, 1.34, 0};

    for(int i = 0; i < full_state_vector.robots[0].joint_names.size(); i++){
        full_state_vector.robots[0].start_pos[i] = robot_start_config[i];
    }

    full_state_vector.rigid_bodies[0].start_linear_pos[0] = startX;
    full_state_vector.rigid_bodies[0].start_linear_pos[1] = startY;
    full_state_vector.rigid_bodies[0].start_linear_pos[2] = 0.032;

    full_state_vector.rigid_bodies[0].start_angular_pos[0] = 0.0;
    full_state_vector.rigid_bodies[0].start_angular_pos[1] = 0.0;
    full_state_vector.rigid_bodies[0].start_angular_pos[2] = 0.0;

    // Distractor body poses
    for(int i = 0; i < object_names.size(); i++){
        std::cout << "object name: " << object_names[i] << "\n";
        pose_6 obstacle_pose;
        MuJoCo_helper->GetBodyPoseAngle(object_names[i], obstacle_pose, MuJoCo_helper->master_reset_data);

        for(int j = 0; j < 3; j++){
            full_state_vector.rigid_bodies[i + 1].start_linear_pos[j] = obstacle_pose.position[j];
            full_state_vector.rigid_bodies[i + 1].start_angular_pos[j] = obstacle_pose.orientation[j];
        }
    }
    std::cout << "in generation \n";
    std::cout << "body " << full_state_vector.rigid_bodies[2].name << " x: " << full_state_vector.rigid_bodies[2].start_linear_pos[0] << " y: " << full_state_vector.rigid_bodies[2].start_linear_pos[1] << std::endl;
}

void SweepMultiple::ReturnRandomGoalState(){

    // Robot configuration doesnt matter for this task
    for(int i = 0; i < full_state_vector.robots[0].joint_names.size(); i++){
        full_state_vector.robots[0].goal_pos[i] = 0.0;
        full_state_vector.robots[0].goal_vel[i] = 0.0;
    }

    // Goal object body
    std::cout << "goal x" << randomGoalX << "goal y: " << randomGoalY << std::endl;
    full_state_vector.rigid_bodies[0].goal_linear_pos[0] = randomGoalX;
    full_state_vector.rigid_bodies[0].goal_linear_pos[1] = randomGoalY;
    full_state_vector.rigid_bodies[0].goal_linear_pos[2] = 0.0;

    full_state_vector.rigid_bodies[0].goal_angular_pos[0] = 0.0;
    full_state_vector.rigid_bodies[0].goal_angular_pos[1] = 0.0;
    full_state_vector.rigid_bodies[0].goal_angular_pos[2] = 0.0;

    // Distractor objects
    for(int i = 1; i < full_state_vector.rigid_bodies.size(); i++){

        full_state_vector.rigid_bodies[i].goal_linear_pos[0] = full_state_vector.rigid_bodies[i].start_linear_pos[0];
        full_state_vector.rigid_bodies[i].goal_linear_pos[1] = full_state_vector.rigid_bodies[i].start_linear_pos[1];
        full_state_vector.rigid_bodies[i].goal_linear_pos[2] = full_state_vector.rigid_bodies[i].start_linear_pos[2];

        full_state_vector.rigid_bodies[i].goal_angular_pos[0] = full_state_vector.rigid_bodies[i].start_angular_pos[0];
        full_state_vector.rigid_bodies[i].goal_angular_pos[1] = full_state_vector.rigid_bodies[i].start_angular_pos[1];
        full_state_vector.rigid_bodies[i].goal_angular_pos[2] = full_state_vector.rigid_bodies[i].start_angular_pos[2];
    }
}

std::vector<MatrixXd> SweepMultiple::CreateInitSetupControls(int horizonLength){
    std::vector<MatrixXd> initSetupControls;

    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->master_reset_data);
    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->main_data);

    // Pushing create init controls borken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goal_pos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goal_pos(0) = current_state_vector.rigid_bodies[0].goal_linear_pos[0];
    goal_pos(1) = current_state_vector.rigid_bodies[0].goal_linear_pos[1];
    goal_pos(2) = 0.0;
    EEWayPointsSetup(goal_pos, mainWayPoints, mainWayPointsTimings, horizonLength);
//    cout << "setup mainwaypoint 0: " << mainWayPoints[0] << endl;
//    cout << "setup mainWayPoint 1: " << mainWayPoints[1] << endl;

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);

    // Compute angle of EE for push
    pose_7 goal_obj_start;
    MuJoCo_helper->GetBodyPoseQuat(body_name, goal_obj_start, MuJoCo_helper->master_reset_data);
    double diff_x = goal_pos(0) - goal_obj_start.position[0];
    double diff_y =  goal_pos(1) - goal_obj_start.position[1];
    double angle_EE_push = atan2(diff_y, diff_x);

    // Step 3 - follow the points via the jacobian
    initSetupControls = JacobianEEControl(allWayPoints, angle_EE_push);

    return initSetupControls;
}

std::vector<MatrixXd> SweepMultiple::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> initControls;

    // Set the goal position so that we can see where we are pushing to
    std::string goalMarkerName = "display_goal";
    pose_6 displayBodyPose;
    MuJoCo_helper->GetBodyPoseAngle(goalMarkerName, displayBodyPose, MuJoCo_helper->master_reset_data);
    displayBodyPose.position[0] = current_state_vector.rigid_bodies[0].goal_linear_pos[0];
    displayBodyPose.position[1] = current_state_vector.rigid_bodies[0].goal_linear_pos[1];
    displayBodyPose.position[2] = 0.0f;
    MuJoCo_helper->SetBodyPoseAngle(goalMarkerName, displayBodyPose, MuJoCo_helper->master_reset_data);

    // Pushing create init controls broken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goal_pos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goal_pos(0) = current_state_vector.rigid_bodies[0].goal_linear_pos[0];
    goal_pos(1) = current_state_vector.rigid_bodies[0].goal_linear_pos[1];
    EEWayPointsPush(goal_pos, mainWayPoints, mainWayPointsTimings, horizonLength);
//    cout << mainWayPoints.size() << " waypoints created" << endl;
//    cout << "mainwaypoint 0: " << mainWayPoints[1] << endl;
//    cout << "mainWayPoint 1: " << mainWayPoints[2] << endl;

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);

    // Compute angle of EE for push
    pose_7 goal_obj_start;
    MuJoCo_helper->GetBodyPoseQuat(body_name, goal_obj_start, MuJoCo_helper->master_reset_data);
    double diff_x = goal_pos(0) - goal_obj_start.position[0];
    double diff_y =  goal_pos(1) - goal_obj_start.position[1];
    double angle_EE_push = atan2(diff_y, diff_x);

    // Step 3 - follow the points via the jacobian
    initControls = JacobianEEControl(allWayPoints, angle_EE_push);

    return initControls;
}

bool SweepMultiple::TaskComplete(mjData *d, double &dist){
    bool taskComplete = false;

//    pose_6 goal_pose;
//    MuJoCo_helper->GetBodyPoseAngle("goal", goal_pose, d);
//
//    double x_diff = goal_pose.position(0) - current_state_vector.rigid_bodies[0].goal_linear_pos[0];
//    double y_diff = goal_pose.position(1) - current_state_vector.rigid_bodies[0].goal_linear_pos[1];
//
//    dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
//
//    if(dist < 0.025){
//        taskComplete = true;
//    }


    return taskComplete;
}