#include "ModelTranslator/PushSoft.h"

PushSoft::PushSoft(int _task_mode): PushBaseClass("franka_gripper", "goal"){
    task_mode = _task_mode;

    std::string yamlFilePath;
    if(task_mode == PUSH_SOFT){
        yamlFilePath = "/TaskConfigs/soft_body_manipulation/push_soft.yaml";
    }
    else if(task_mode == PUSH_SOFT_RIGID){
        yamlFilePath = "/TaskConfigs/soft_body_manipulation/push_soft_into_rigid.yaml";
    }
    else{
        std::cerr << "Invalid task mode specified for push soft \n";
    }

    InitModelTranslator(yamlFilePath);
}

//void PushSoft::ReturnRandomStartState(){
//
//    // Randomly generate a start and goal x and y position for cylinder
//    // Random generate a goal x and y position for cylinder
//    float startX = randFloat(0.5, 0.501);
//    float startY = randFloat(0, 0.01);
//
//    float goalX = randFloat(0.65, 0.75);
//    float goalY = randFloat(-0.1, 0.1);
//
//    // Place rigid object at
//    randomGoalX = goalX;
//    randomGoalY = goalY;
//
//    // Initialise soft body poses to start configuration
//    for(auto & soft_body : full_state_vector.soft_bodies){
//        pose_6 body_pose;
//
//        body_pose.position[0] = 0.0;
//        body_pose.position[1] = 1.0;
//        body_pose.position[2] = 0.0;
//
//        for(int i = 0; i < 3; i++){
//            body_pose.orientation[i] = soft_body.start_angular_pos[i];
//        }
//
//        // TODO - better way to do this where we use the spacing information and transforms
//        for(int i = 0; i < soft_body.num_vertices; i++){
//            MuJoCo_helper->SetSoftBodyVertexPos(soft_body.name, i, body_pose, MuJoCo_helper->main_data);
//        }
//    }
//
//    // step simulator
//    for(int t = 0; t < 10; t++){
//        mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);
//    }
//
//    // Set up the goal positions and velocities
//    // Robot start configuration
//    double robot_start_config[7] = {0, 0.1, 0, -3, 0, 1.34, 0};
//
//    for(int i = 0; i < full_state_vector.robots[0].joint_names.size(); i++){
//        full_state_vector.robots[0].start_pos[i] = robot_start_config[i];
//    }
//
//    if(task_mode == PUSH_SOFT_RIGID){
//        full_state_vector.rigid_bodies[0].start_linear_pos[0] = startX;
//        full_state_vector.rigid_bodies[0].start_linear_pos[1] = startY;
//        full_state_vector.rigid_bodies[0].start_linear_pos[2] = 0.032;
//
//        full_state_vector.rigid_bodies[0].start_angular_pos[0] = 0.0;
//        full_state_vector.rigid_bodies[0].start_angular_pos[1] = 0.0;
//        full_state_vector.rigid_bodies[0].start_angular_pos[2] = 0.0;
//    }
//
//    // Soft body
//    full_state_vector.soft_bodies[0].start_linear_pos[0] = -0.8;
//    full_state_vector.soft_bodies[0].start_linear_pos[1] = 1.0;
//    full_state_vector.soft_bodies[0].start_linear_pos[2] = 0.1;
//    std::cout << "soft body x: " << full_state_vector.soft_bodies[0].start_linear_pos[0] << " y: " << full_state_vector.soft_bodies[0].start_linear_pos[1] << "\n";
//    full_state_vector.soft_bodies[0].start_angular_pos[0] = 0.0;
//    full_state_vector.soft_bodies[0].start_angular_pos[1] = 0.0;
//    full_state_vector.soft_bodies[0].start_angular_pos[2] = 0.0;
//
//}
//
//void PushSoft::ReturnRandomGoalState(){
//
//    // Robot configuration doesnt matter for this task
//    for(int i = 0; i < full_state_vector.robots[0].joint_names.size(); i++){
//        full_state_vector.robots[0].goal_pos[i] = 0.0;
//        full_state_vector.robots[0].goal_vel[i] = 0.0;
//    }
//
//    if(task_mode == PUSH_SOFT){
//
//        // Soft body distractor
//        full_state_vector.soft_bodies[0].goal_linear_pos[0] = randomGoalX;
//        full_state_vector.soft_bodies[0].goal_linear_pos[1] = randomGoalY;
//        full_state_vector.soft_bodies[0].goal_linear_pos[2] = 0.0;
//
//        full_state_vector.soft_bodies[0].goal_angular_pos[0] = 0.0;
//        full_state_vector.soft_bodies[0].goal_angular_pos[1] = 0.0;
//        full_state_vector.soft_bodies[0].goal_angular_pos[2] = 0.0;
//    }
//    else if(task_mode == PUSH_SOFT_RIGID){
//        // Goal object body
//        full_state_vector.rigid_bodies[0].goal_linear_pos[0] = randomGoalX;
//        full_state_vector.rigid_bodies[0].goal_linear_pos[1] = randomGoalY;
//        full_state_vector.rigid_bodies[0].goal_linear_pos[2] = 0.0;
//
//        full_state_vector.rigid_bodies[0].goal_angular_pos[0] = 0.0;
//        full_state_vector.rigid_bodies[0].goal_angular_pos[1] = 0.0;
//        full_state_vector.rigid_bodies[0].goal_angular_pos[2] = 0.0;
//
//        // Soft body distractor
//        full_state_vector.soft_bodies[0].goal_linear_pos[0] = 0.0;
//        full_state_vector.soft_bodies[0].goal_linear_pos[1] = 0.0;
//        full_state_vector.soft_bodies[0].goal_linear_pos[2] = 0.0;
//
//        full_state_vector.soft_bodies[0].goal_angular_pos[0] = 0.0;
//        full_state_vector.soft_bodies[0].goal_angular_pos[1] = 0.0;
//        full_state_vector.soft_bodies[0].goal_angular_pos[2] = 0.0;
//    }
//
//
//
//}
//
////std::vector<MatrixXd> PushSoft::CreateInitSetupControls(int horizonLength){
////    std::vector<MatrixXd> initSetupControls;
////
////    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->master_reset_data);
////    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->main_data);
////
////    // Pushing create init controls borken into three main steps
////    // Step 1 - create main waypoints we want to end-effector to pass through
////    m_point goalPos;
////    std::vector<m_point> mainWayPoints;
////    std::vector<int> mainWayPointsTimings;
////    std::vector<m_point> allWayPoints;
////    goalPos(0) = current_state_vector.rigid_bodies[0].goal_linear_pos[0];
////    goalPos(1) = current_state_vector.rigid_bodies[0].goal_linear_pos[1];
////    goalPos(2) = 0.0;
////    EEWayPointsSetup(goalPos, mainWayPoints, mainWayPointsTimings, horizonLength);
//////    cout << "setup mainwaypoint 0: " << mainWayPoints[0] << endl;
//////    cout << "setup mainWayPoint 1: " << mainWayPoints[1] << endl;
////
////    // Step 2 - create all subwaypoints over the entire trajectory
////    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);
////
////    // Step 3 - follow the points via the jacobian
////    initSetupControls = JacobianEEControl(goalPos, allWayPoints);
////
////    return initSetupControls;
////}
//
//std::vector<MatrixXd> PushSoft::CreateInitOptimisationControls(int horizonLength){
//    std::vector<MatrixXd> initControls;
//
//    // Get EE current posi
//
//    // Pushing create init controls broken into three main steps
//    // Step 1 - create main waypoints we want to end-effector to pass through
//    m_point goal_pos;
//    std::vector<m_point> mainWayPoints;
//    std::vector<int> mainWayPointsTimings;
//    std::vector<m_point> allWayPoints;
//    double angle_EE_push = 0.0;
//
//    if(task_mode == PUSH_SOFT_RIGID){
//        goal_pos(0) = current_state_vector.rigid_bodies[0].goal_linear_pos[0];
//        goal_pos(1) = current_state_vector.rigid_bodies[0].goal_linear_pos[1];
//        goal_pos(2) = 0.3;
//
//        angle_EE_push = 0.0;
//    }
//    else{
//        goal_pos(0) = current_state_vector.soft_bodies[0].goal_linear_pos[0];
//        goal_pos(1) = current_state_vector.soft_bodies[0].goal_linear_pos[1];
//        goal_pos(2) = 0.3;
//
//        pose_6 goal_obj_start;
//        MuJoCo_helper->GetSoftBodyVertexPos(current_state_vector.soft_bodies[0].name, 0, goal_obj_start, MuJoCo_helper->master_reset_data);
//        double diff_x = goal_pos(0) - goal_obj_start.position[0];
//        double diff_y =  goal_pos(1) - goal_obj_start.position[1];
//
//        angle_EE_push = atan2(diff_y, diff_x);
//    }
//    EEWayPointsPush(goal_pos, mainWayPoints, mainWayPointsTimings, horizonLength);
//    cout << mainWayPoints.size() << " waypoints created" << endl;
//    cout << "mainwaypoint 0: " << mainWayPoints[0] << endl;
//    cout << "mainWayPoint 1: " << mainWayPoints[1] << endl;
//
//    // Step 2 - create all subwaypoints over the entire trajectory
//    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);
//
//    // Step 3 - follow the points via the jacobian
//    initControls = JacobianEEControl(allWayPoints, angle_EE_push);
//
//    return initControls;
//}

//bool PushSoft::TaskComplete(mjData *d, double &dist){
//    bool taskComplete = false;
//
//    if(task_mode == PUSH_SOFT){
//        // Minimise the sum of all vertices from the goal position
//        dist = 0.0;
//        pose_6 vertex_pose;
//        for(int i = 0; i < full_state_vector.soft_bodies[0].num_vertices; i++){
//            MuJoCo_helper->GetSoftBodyVertexPosGlobal(full_state_vector.soft_bodies[0].name, i, vertex_pose, d);
//            // TODO - fix this!
////            double diffX = full_state_vector.soft_bodies[0].goal_linear_pos[0] - vertex_pose.position[0];
////            double diffY = full_state_vector.soft_bodies[0].goal_linear_pos[1] - vertex_pose.position[1];
////            dist += sqrt(pow(diffX, 2) + pow(diffY, 2));
//        }
//
//        // dist = 15
////        std::cout << "dist: " << dist << "\n";
////        if(dist < 3.0){
////            taskComplete = true;
////        }
//
//    }
//    else if(task_mode == PUSH_SOFT_RIGID){
//        pose_6 goal_pose;
//        MuJoCo_helper->GetBodyPoseAngle("goal", goal_pose, d);
//
//        // TODO - fix this!
////        double x_diff = goal_pose.position(0) - full_state_vector.rigid_bodies[0].goal_linear_pos[0];
////        double y_diff = goal_pose.position(1) - full_state_vector.rigid_bodies[0].goal_linear_pos[1];
////        dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
//
////    std::cout << "dist: " << dist << "\n";
//        if(dist < 0.03){
//            taskComplete = true;
//        }
//
//        // if weve pushed too far in x direction, the task should stop
////        if(x_diff > 0.08){
////            taskComplete = true;
////            std::cout << "pushed too far \n";
////        }
//    }
//
//    return taskComplete;
//}
