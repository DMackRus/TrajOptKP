//
// Created by dave on 26/06/23.
//
#include "ModelTranslator/Walker.h"

walker::walker(int terrain, int locomotion_type): ModelTranslator(){
    std::string yaml_file_path;

    if(locomotion_type == WALK){
        low_bound_velocity = 0.1;
        high_bound_velocity = 0.6;
        if(terrain == PLANE)
            yaml_file_path = "/TaskConfigs/locomotion/walk_plane.yaml";
        else if(terrain == UNEVEN)
            yaml_file_path = "/TaskConfigs/locomotion/walk_uneven.yaml";

    }
    else if(locomotion_type == RUN){
        low_bound_velocity = 0.9;
        high_bound_velocity = 1.3;
        yaml_file_path = "/TaskConfigs/locomotion/run_plane.yaml";
    }

    InitModelTranslator(yaml_file_path);
}

bool walker::TaskComplete(mjData *d, double &dist){
    dist = 0.0;
    return false;
}

void walker::ReturnRandomStartState(){

    double start_config[9] = {0, 0, 0, 1, -1, 0.2, 0, 0, 0};

    for(int i = 0; i < 9; i++){
        current_state_vector.robots[0].start_pos[i] = start_config[i];
    }
}

void walker::ReturnRandomGoalState(){
    for(int i = 0; i < 9; i++){
        current_state_vector.robots[0].goal_pos[i] = 0.0;
        current_state_vector.robots[0].goal_vel[i] = 0.0;
    }

    // Random body velocity between low_bound_vel and hig_bound_vel
    float rand_body_vel = randFloat(low_bound_velocity, high_bound_velocity);
    current_state_vector.robots[0].goal_vel[1] = rand_body_vel;
}

void walker::InstantiateResiduals(){
    num_residual_terms = 9;
    residual_weights.resize(num_residual_terms);
    residual_weights_terminal.resize(num_residual_terms);

    residual_weights[0] = 1.0;
    residual_weights[1] = 0.1;
    residual_weights[2] = 0.1;

    residual_weights[3] = 0.001;
    residual_weights[4] = 0.001;
    residual_weights[5] = 0.001;
    residual_weights[6] = 0.001;
    residual_weights[7] = 0.001;
    residual_weights[8] = 0.001;

    residual_weights_terminal[0] = 100.0;
    residual_weights_terminal[1] = 0.0;
    residual_weights_terminal[2] = 0.0;

    residual_weights_terminal[3] = 0.001;
    residual_weights_terminal[4] = 0.001;
    residual_weights_terminal[5] = 0.001;
    residual_weights_terminal[6] = 0.001;
    residual_weights_terminal[7] = 0.001;
    residual_weights_terminal[8] = 0.001;
}

MatrixXd walker::Residuals(mjData *d){
    MatrixXd residuals(9, 1);

    std::vector<double> walker_joints;
    std::vector<double> walker_velocities;
    std::vector<double> walker_controls;
    MuJoCo_helper->GetRobotJointsPositions("walker", walker_joints, d);
    MuJoCo_helper->GetRobotJointsVelocities("walker", walker_velocities, d);
    MuJoCo_helper->GetRobotJointsControls("walker", walker_controls, d);

    // --------------- Residual 0: Body height -----------------
    residuals(0, 0) = walker_joints[0];

    // --------------- Residual 1: Body rotation ---------------
    residuals(1, 0) = walker_joints[2];

    // --------------- Residual 2: Body velocity ---------------
    residuals(2, 0) = walker_velocities[1];

    // --------------- Residual 3: Joints controls -------------
    for(int i = 0; i < walker_controls.size(); i++){
        residuals(3+i, 0) = walker_controls[i];
    }
//    residuals(3, 0) = 0.0;
//    for(double walker_control : walker_controls){
//        residuals(3, 0) += walker_control;
//    }

    return residuals;
}

//double walker::CostFunction(mjData *d, bool terminal){
//    double cost;
//    MatrixXd Xt = ReturnStateVector(d);
//    MatrixXd Ut = ReturnControlVector(d);
//
//    MatrixXd X_diff = Xt - X_desired;
//
//    for(int i = 0; i < 9; i++){
//        //0 = height, 1 = x, 2 = rotation
//        if(i == 0){
//            // Only penalise if height below expected.
//            if(X_diff(i) < 0){
//                if(terminal) cost += X_diff(i) * X_diff(i) * Q_terminal.diagonal()(i);
//                else cost += X_diff(i) * X_diff(i) * Q.diagonal()(i);
//            }
//
////            if(terminal) cost += exp(-Q_terminal.diagonal()(i) * X_diff(i));
////            else cost += exp(-Q.diagonal()(i) * X_diff(i));
//        }
//        else{
//            if(terminal) cost += X_diff(i) * X_diff(i) * Q_terminal.diagonal()(i);
//            else cost += X_diff(i) * X_diff(i) * Q.diagonal()(i);
//        }
//    }
//
//    return cost;
//}
//
//void walker::CostDerivatives(mjData *d, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal){
//    MatrixXd Xt = ReturnStateVector(d);
//    MatrixXd Ut = ReturnControlVector(d);
//    MatrixXd X_diff = Xt - X_desired;
//
//    double height_gradient = 0.0f;
//    double height_hessian = 0.0f;
//
//    if(terminal){
//        l_x = 2 * Q_terminal * X_diff;
//        l_xx = 2 * Q_terminal;
//        if(X_diff(0) < 0){
//            height_gradient = 2 * Q_terminal.diagonal()[0] * X_diff(0);
//            height_hessian = 2 * Q_terminal.diagonal()[0];
//        }
////        height_gradient = -Q_terminal.diagonal()[0] * X_diff(0) * exp(-Q_terminal.diagonal()[0] * X_diff(0));
////        height_hessian = Q_terminal.diagonal()[0] * exp(-Q_terminal.diagonal()[0] * X_diff(0)) * (Q_terminal.diagonal()[0] * X_diff(0));
//    }
//    else{
//        l_x = 2 * Q * X_diff;
//        l_xx = 2 * Q;
//        if(X_diff(0) < 0){
//            height_gradient = 2 * Q.diagonal()[0] * X_diff(0);
//            height_hessian = 2 * Q.diagonal()[0];
//        }
////        height_gradient = -Q.diagonal()[0] * X_diff(0) * exp(-Q.diagonal()[0] * X_diff(0));
////        height_hessian = Q.diagonal()[0] * exp(-Q.diagonal()[0] * X_diff(0)) * (Q.diagonal()[0] * X_diff(0));
//    }
//
//    l_x(0) = l_x(0) + height_gradient;
//    l_xx(0,0) = l_xx(0,0) + height_hessian;
//
//    l_u = 2 * R * Ut;
//    l_uu = 2 * R;
//}
