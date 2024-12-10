#include "ModelTranslator/FloatingCube.h"

FloatingCube::FloatingCube(): ModelTranslator(){
    std::string yamlFilePath = "/TaskConfigs/toys/floating_cube.yaml";
    InitModelTranslator(yamlFilePath);
}

void FloatingCube::SetGoalVisuals(mjData *d){
    pose_6 box_goal;
    box_goal.position(0) = residual_list[0].target[0];
    box_goal.position(1) = residual_list[0].target[1];

    box_goal.position(2) = 0.0;

    MuJoCo_helper->SetBodyPoseAngle("display_goal", box_goal, d);
}

void FloatingCube::Residuals(mjData *d, MatrixXd &residuals){
    int resid_index = 0;

    // Compute kinematics chain to compute site poses
    mj_kinematics(MuJoCo_helper->model, d);

    pose_6 goal_pose;
    MuJoCo_helper->GetBodyPoseAngle("floating_cube", goal_pose, d);

    // --------------- Residual 0: Body goal orientation -----------------
    Eigen::Matrix3d current_rot_mat = eul2RotMat(goal_pose.orientation);

//    Eigen::Matrix3d desired_rot_mat = eul2RotMat(desired_eul);

    m_point desired_axis;
    desired_axis(0) = 1;
    desired_axis(1) = 1;
    desired_axis(2) = 1;

    m_quat current, desired, inv_current, diff;
    current = axis2Quat(goal_pose.orientation);
    desired = axis2Quat(desired_axis);

    inv_current = invQuat(current);
    diff = multQuat(inv_current, desired);

    // temp mujoco quat2vel methodology?
    double axis[3] = {diff[1], diff[2], diff[3]};
    double sin_a_2 = sin(sqrt(pow(axis[0], 2) + pow(axis[1], 2) + pow(axis[2], 2)));
    double speed = 2 * atan2(sin_a_2, diff[0]);

    // When axis angle > pi rot is in other direction
    if(speed > PI) speed -= 2*PI;

    axis[0] *= speed;
    axis[1] *= speed;
    axis[2] *= speed;

    m_point axis_diff = quat2Axis(diff);

//    double dot_x = current_rot_mat(0, 0) * desired_rot_mat(0, 0) +
//        current_rot_mat(1, 0) * desired_rot_mat(1, 0) +
//        current_rot_mat(2, 0) * desired_rot_mat(2, 0);

//    residuals(resid_index++, 0) = acos(dot_x);
    residuals(resid_index++, 0) = axis_diff(2);


    if(resid_index != residual_list.size()){
        std::cerr << "Error: Residuals size mismatch\n";
        exit(1);
    }
}

bool FloatingCube::TaskComplete(mjData *d, double &dist){
    bool taskComplete = false;

    pose_6 goal_pose;
    MuJoCo_helper->GetBodyPoseAngle("floating_cube", goal_pose, d);

//    double x_diff = goal_pose.position(0) - residual_list[0].target[0];
//    double y_diff = goal_pose.position(1) - residual_list[0].target[1];
//
//    dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
//
//    if(dist < 0.035){
//        taskComplete = true;
//    }
//
    return taskComplete;
}

//class FloatingCube: public ModelTranslator{
//public:
//    FloatingCube();
////
////    void ReturnRandomStartState() override;
////    void ReturnRandomGoalState() override;
//
//    void SetGoalVisuals(mjData *d) override;
//    void Residuals(mjData *d, MatrixXd &residuals) override;
//
//    bool TaskComplete(mjData *d, double &dist) override;
//
//private:
//
//};