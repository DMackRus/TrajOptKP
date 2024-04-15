//
// Created by dave on 26/06/23.
//
#include "Walker.h"

walker::walker(int terrain, int locomotion_type): ModelTranslator(){
    std::string yaml_file_path;

    if(locomotion_type == WALK){
        low_bound_velocity = 0.1;
        high_bound_velocity = 0.6;
        if(terrain == PLANE)
            yaml_file_path = "/taskConfigs/walk_plane_config.yaml";
        else if(terrain == UNEVEN)
            yaml_file_path = "/taskConfigs/walk_uneven_config.yaml";

    }
    else if(locomotion_type == RUN){
        low_bound_velocity = 0.9;
        high_bound_velocity = 1.3;
        yaml_file_path = "/taskConfigs/run_plane_config.yaml";
    }

    InitModelTranslator(yaml_file_path);
}

bool walker::TaskComplete(mjData *d, double &dist){
    dist = 0.0f;
    return false;
}

void walker::ReturnRandomStartState(){

    double start_config[9] = {0, 0, 0, 1, -1, 0.2, 0, 0, 0};

    for(int i = 0; i < 9; i++){
        active_state_vector.robots[0].startPos[i] = start_config[i];
    }
}

void walker::ReturnRandomGoalState(){
    for(int i = 0; i < 9; i++){
        active_state_vector.robots[0].goalPos[i] = 0.0;
        active_state_vector.robots[0].goalVel[i] = 0.0;
    }

    // Random body velocity between low_bound_vel and hig_bound_vel
    float rand_body_vel = randFloat(low_bound_velocity, high_bound_velocity);
    active_state_vector.robots[0].goalVel[1] = rand_body_vel;
}

std::vector<MatrixXd> walker::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> initControls;

    MatrixXd control(num_ctrl, 1);
    for(int i = 0; i < horizonLength; i++){

        for(int j = 0; j < num_ctrl; j++){
            control(j) = 0.0f;
        }
        initControls.push_back(control);
    }
    return initControls;
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
