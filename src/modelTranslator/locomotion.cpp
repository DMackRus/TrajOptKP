//
// Created by dave on 26/06/23.
//
#include "locomotion.h"

locomotion_anymal::locomotion_anymal(): modelTranslator(){
    std::string yamlFilePath = "/taskConfigs/locomotionConfig.yaml";
    initModelTranslator(yamlFilePath);
}

//double locomotion_anymal::costFunction(int dataIndex, bool terminal){
//    double cost = 0.0f;
//
////    MatrixXd Ut = returnControlVector(dataIndex);
////    for(int i = 0; i < num_ctrl; i++){
////        cost += 1e-1 * pow(Ut(i), 2);
////    }
//
//    double height = activePhysicsSimulator->sensorState(dataIndex, "torso_position")[2];
//    cost += 10.0 * pow((height - 1.3), 2);
////    cout << "height: " << height << endl;
//
//    // ---------- Residual (2) ----------
//    double torso_up = activePhysicsSimulator->sensorState(dataIndex, "torso_zaxis")[2];
//    cost += 3.0 * pow((torso_up - 1.0), 2);
//
//    double com_vel = activePhysicsSimulator->sensorState(dataIndex, "torso_subtreelinvel")[0];
//    cost += 0.0 * pow((com_vel - 2.0), 2);
//
////    cout << "height: " << height << " torso_up: " << torso_up << " com_vel: " << com_vel << "cost: " << cost << endl;
//
////    MatrixXd Xt = returnStateVector(dataIndex);
////    MatrixXd Ut = returnControlVector(dataIndex);
////    MatrixXd X_diff = Xt - X_desired;
////
////    MatrixXd result = X_diff.transpose() * Q * X_diff + Ut.transpose() * R * Ut;
////    double cost = result(0);
//
//    return cost;
//}
//
//void locomotion_anymal::costDerivatives(int dataIndex, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal){
////    MatrixXd Xt = returnStateVector(dataIndex);
////    MatrixXd Ut = returnControlVector(dataIndex);
////    MatrixXd X_diff = Xt - X_desired;
////
////    l_x = 2 * Q * X_diff;
////    l_xx = 2 * Q;
////
////    l_u = 2 * R * Ut;
////    l_uu = 2 * R;
//}

bool locomotion_anymal::taskComplete(int dataIndex, double &dist){
    return false;
}

void locomotion_anymal::generateRandomGoalAndStartState(){
    MatrixXd test;
}

MatrixXd locomotion_anymal::returnRandomStartState(){
    MatrixXd test;
    return test;
}

MatrixXd locomotion_anymal::returnRandomGoalState(MatrixXd X0){
    MatrixXd test;
    return test;
}

std::vector<MatrixXd> locomotion_anymal::createInitOptimisationControls(int horizonLength){
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
