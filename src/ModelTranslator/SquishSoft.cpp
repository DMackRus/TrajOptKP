#include "ModelTranslator/SquishSoft.h"

SquishSoft::SquishSoft(): PushBaseClass("franka_gripper", "goal"){

    std::string yamlFilePath = "/taskConfigs/squish_soft.yaml";

    InitModelTranslator(yamlFilePath);
}

void SquishSoft::ReturnRandomStartState(){

    // Randomly generate a start and goal x and y position for cylinder
    // Random generate a goal x and y position for cylinder
    float startX = randFloat(0.5, 0.501);
    float startY = randFloat(0, 0.01);

    float goalX = randFloat(0.7, 0.8);
    float goalY = randFloat(-0.1, 0);

    // Place rigid object at
    randomGoalX = goalX;
    randomGoalY = goalY;

    // Initialise soft body poses to start configuration
    for(auto & soft_body : full_state_vector.soft_bodies){
        pose_6 body_pose;

        body_pose.position[0] = 0.0;
        body_pose.position[1] = 1.0;
        body_pose.position[2] = 0.0;

        for(int i = 0; i < 3; i++){
            body_pose.orientation[i] = soft_body.start_angular_pos[i];
        }

        // TODO - better way to do this where we use the spacing information and transforms
        for(int i = 0; i < soft_body.num_vertices; i++){
            MuJoCo_helper->SetSoftBodyVertexPos(soft_body.name, i, body_pose, MuJoCo_helper->main_data);
        }
    }

    // step simulator
    for(int t = 0; t < 10; t++){
        mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);
    }

    // Set up the goal positions and velocities
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

    // Soft body
    full_state_vector.soft_bodies[0].start_linear_pos[0] = -0.8;
    full_state_vector.soft_bodies[0].start_linear_pos[1] = 1.0;
    full_state_vector.soft_bodies[0].start_linear_pos[2] = 0.1;
    std::cout << "soft body x: " << full_state_vector.soft_bodies[0].start_linear_pos[0] << " y: " << full_state_vector.soft_bodies[0].start_linear_pos[1] << "\n";
    full_state_vector.soft_bodies[0].start_angular_pos[0] = 0.0;
    full_state_vector.soft_bodies[0].start_angular_pos[1] = 0.0;
    full_state_vector.soft_bodies[0].start_angular_pos[2] = 0.0;

}

void SquishSoft::ReturnRandomGoalState(){

    // Robot configuration doesnt matter for this task
    for(int i = 0; i < full_state_vector.robots[0].joint_names.size(); i++){
        full_state_vector.robots[0].goal_pos[i] = 0.0;
        full_state_vector.robots[0].goal_vel[i] = 0.0;
    }

    // Goal object body
    full_state_vector.rigid_bodies[0].goal_linear_pos[0] = randomGoalX;
    full_state_vector.rigid_bodies[0].goal_linear_pos[1] = randomGoalY;
    full_state_vector.rigid_bodies[0].goal_linear_pos[2] = 0.0;

    full_state_vector.rigid_bodies[0].goal_angular_pos[0] = 0.0;
    full_state_vector.rigid_bodies[0].goal_angular_pos[1] = 0.0;
    full_state_vector.rigid_bodies[0].goal_angular_pos[2] = 0.0;

    // Soft body distractor
    full_state_vector.soft_bodies[0].goal_linear_pos[0] = 0.0;
    full_state_vector.soft_bodies[0].goal_linear_pos[1] = 0.0;
    full_state_vector.soft_bodies[0].goal_linear_pos[2] = 0.0;

    full_state_vector.soft_bodies[0].goal_angular_pos[0] = 0.0;
    full_state_vector.soft_bodies[0].goal_angular_pos[1] = 0.0;
    full_state_vector.soft_bodies[0].goal_angular_pos[2] = 0.0;

}

//std::vector<MatrixXd> SquishSoft::CreateInitSetupControls(int horizonLength){
//    std::vector<MatrixXd> initSetupControls;
//
//    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->master_reset_data);
//    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->main_data);
//
//    // Pushing create init controls borken into three main steps
//    // Step 1 - create main waypoints we want to end-effector to pass through
//    m_point goalPos;
//    std::vector<m_point> mainWayPoints;
//    std::vector<int> mainWayPointsTimings;
//    std::vector<m_point> allWayPoints;
//    goalPos(0) = current_state_vector.rigid_bodies[0].goal_linear_pos[0];
//    goalPos(1) = current_state_vector.rigid_bodies[0].goal_linear_pos[1];
//    goalPos(2) = 0.0;
//    EEWayPointsSetup(goalPos, mainWayPoints, mainWayPointsTimings, horizonLength);
////    cout << "setup mainwaypoint 0: " << mainWayPoints[0] << endl;
////    cout << "setup mainWayPoint 1: " << mainWayPoints[1] << endl;
//
//    // Step 2 - create all subwaypoints over the entire trajectory
//    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);
//
//    // Step 3 - follow the points via the jacobian
//    initSetupControls = JacobianEEControl(goalPos, allWayPoints);
//
//    return initSetupControls;
//}

std::vector<MatrixXd> SquishSoft::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> initControls;

    // Get EE current posi

    // Pushing create init controls broken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goalPos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
//    goalPos(0) = current_state_vector.rigid_bodies[0].goal_linear_pos[0];
//    goalPos(1) = current_state_vector.rigid_bodies[0].goal_linear_pos[1];
//    goalPos(2) = 0.3;
    goalPos(0) = 0.6;
    goalPos(1) = 0.0;
    goalPos(2) = 0.3;
    EEWayPointsPush(goalPos, mainWayPoints, mainWayPointsTimings, horizonLength);
    cout << mainWayPoints.size() << " waypoints created" << endl;
    cout << "mainwaypoint 0: " << mainWayPoints[0] << endl;
    cout << "mainWayPoint 1: " << mainWayPoints[1] << endl;

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);

    // Step 3 - follow the points via the jacobian
    initControls = JacobianEEControl(goalPos, allWayPoints);

    return initControls;
}

bool SquishSoft::TaskComplete(mjData *d, double &dist){
    bool taskComplete = false;

    pose_6 goal_pose;
    MuJoCo_helper->GetBodyPoseAngle("goal", goal_pose, d);

    double x_diff = goal_pose.position(0) - current_state_vector.rigid_bodies[0].goal_linear_pos[0];
    double y_diff = goal_pose.position(1) - current_state_vector.rigid_bodies[0].goal_linear_pos[1];

    dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

//    std::cout << "dist: " << dist << "\n";
    if(dist < 0.03){
        taskComplete = true;
    }


    return taskComplete;
}
