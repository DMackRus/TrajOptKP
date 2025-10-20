#include <gtest/gtest.h>

#include "ModelTranslator/ModelTranslator.h"
#include "test_acrobot.h"
#include "3D_test_class.h"
#include "ModelTranslator/anyMal.h"
#include "test_bimanual_pickup.h"

std::shared_ptr<ModelTranslator> model_translator;

bool check_state_vectors_match(std::vector<std::string> expected, std::vector<std::string> actual){
    bool match = true;

    if(expected.size() != actual.size()){
        return false;
    }

    for(int i = 0; i < expected.size(); i++){
        if(expected[i] != actual[i]){
            std::cerr << "Expected: " << expected[i] << " ,actual: " << actual[i] << "\n";
            match = false;
            break;
        }
    }

    return match;
}

TEST(ModelTranslator, default_state_vector_names){

    std::cout << "start of test default state vector names \n";
    std::shared_ptr<threeDTestClass> threeD_test = std::make_shared<threeDTestClass>();
    model_translator = threeD_test;

    std::vector<std::string> expected_state_names = {"panda0_joint1", "panda0_joint2", "panda0_joint3",
                                                     "panda0_joint4", "panda0_joint5", "panda0_joint6",
                                                     "panda0_joint7", "panda0_finger_joint1", "panda0_finger_joint2",
                                                     "goal_x", "goal_y", "goal_z", "goal_roll", "goal_pitch", "goal_yaw",
                                                     "obstacle_1_x", "obstacle_1_y", "obstacle_1_z", "obstacle_1_roll", "obstacle_1_pitch", "obstacle_1_yaw",
                                                     "obstacle_2_x", "obstacle_2_y", "obstacle_2_z", "obstacle_2_roll", "obstacle_2_pitch", "obstacle_2_yaw"};

    std::vector<std::string> actual_state_names = model_translator->current_state_vector.state_names;

    ASSERT_TRUE(check_state_vectors_match(expected_state_names, actual_state_names));
}

TEST(ModelTranslator, set_state_vector){

    std::shared_ptr<threeDTestClass> threeD_test = std::make_shared<threeDTestClass>();
    model_translator = threeD_test;

    MatrixXd test_state_vector(model_translator->current_state_vector.dof*2, 1);

    test_state_vector << 0, -0.183, 0, -3.1, 0, 1.34, 0, 0, 0,
            0.5, 0.2, 0.1, 0, 0, 0,
            0.6, 0.2, 0.1, 0, 0, 0,
            0.7, 0.1, 0.1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0;

    std::shared_ptr<MuJoCoHelper> MuJoCo_helper = model_translator->MuJoCo_helper;

    model_translator->SetStateVector(test_state_vector, MuJoCo_helper->master_reset_data,
                                     model_translator->current_state_vector);

    MatrixXd return_state_vector = model_translator->ReturnStateVector(MuJoCo_helper->master_reset_data,
                                                                       model_translator->current_state_vector);

    for(int i = 0; i < model_translator->current_state_vector.dof*2; i++){
        EXPECT_EQ(test_state_vector(i), return_state_vector(i));
    }
}

TEST(ModelTranslator, remove_elements_check_names){
    std::shared_ptr<threeDTestClass> threeD_test = std::make_shared<threeDTestClass>();
    model_translator = threeD_test;

    std::vector<std::string> expected_state_names = {"panda0_joint1", "panda0_joint2", "panda0_joint3",
                                                     "panda0_joint4", "panda0_joint5", "panda0_joint6",
                                                     "panda0_joint7", "panda0_finger_joint1", "panda0_finger_joint2",
                                                     "goal_x", "goal_y", "goal_z",
                                                     "obstacle_1_roll", "obstacle_1_pitch", "obstacle_1_yaw",
                                                     "obstacle_2_y", "obstacle_2_z", "obstacle_2_yaw"};

    std::vector<std::string> remove_names = {"goal_roll", "goal_pitch", "goal_yaw",
                                             "obstacle_1_x", "obstacle_1_y", "obstacle_1_z",
                                             "obstacle_2_roll", "obstacle_2_x", "obstacle_2_pitch"};

    model_translator->UpdateCurrentStateVector(remove_names, false);

    std::vector<std::string> actual_state_names = model_translator->current_state_vector.state_names;

    ASSERT_TRUE(check_state_vectors_match(expected_state_names, actual_state_names));
}

TEST(ModelTranslator, remove_elements_then_add_check_names){
    std::shared_ptr<threeDTestClass> threeD_test = std::make_shared<threeDTestClass>();
    model_translator = threeD_test;

    std::vector<std::string> expected_state_names = {"panda0_joint1", "panda0_joint2", "panda0_joint3",
                                                     "panda0_joint4", "panda0_joint5", "panda0_joint6",
                                                     "panda0_joint7", "panda0_finger_joint1", "panda0_finger_joint2",
                                                     "goal_x", "goal_y", "goal_z", "goal_roll",
                                                     "obstacle_1_x", "obstacle_1_roll", "obstacle_1_pitch", "obstacle_1_yaw",
                                                     "obstacle_2_y", "obstacle_2_z", "obstacle_2_yaw"};

    std::vector<std::string> remove_names = {"goal_roll", "goal_pitch", "goal_yaw",
                                             "obstacle_1_x", "obstacle_1_y", "obstacle_1_z",
                                             "obstacle_2_roll", "obstacle_2_x", "obstacle_2_pitch"};

    model_translator->UpdateCurrentStateVector(remove_names, false);

    std::vector<std::string> re_add_names = {"goal_roll", "obstacle_1_x"};

    model_translator->UpdateCurrentStateVector(re_add_names, true);

    std::vector<std::string> actual_state_names = model_translator->current_state_vector.state_names;

    std::cout << "current state vector names: ";
    for(const auto & state_name : model_translator->current_state_vector.state_names){
        std::cout << " " << state_name;
    }
    std::cout << "\n";

    ASSERT_TRUE(check_state_vectors_match(expected_state_names, actual_state_names));
}

TEST(ModelTranslator, set_state_remove_names_return_state){

    std::shared_ptr<threeDTestClass> threeD_test = std::make_shared<threeDTestClass>();
    model_translator = threeD_test;

    MatrixXd test_state_vector(model_translator->current_state_vector.dof*2, 1);

    test_state_vector << 0, -0.183, 0, -3.1, 0, 1.34, 0, 0, 0,
            0.5, 0.2, 0.1, 0, 0, 0,
            0.6, 0.2, 0.1, 0, 0, 0,
            0.7, 0.1, 0.1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0;

    std::shared_ptr<MuJoCoHelper> MuJoCo_helper = model_translator->MuJoCo_helper;

    model_translator->SetStateVector(test_state_vector, MuJoCo_helper->master_reset_data,
                                     model_translator->current_state_vector);

    // ---------------------- remove state vector elements ----------------------------
    std::vector<std::string> remove_names = {"goal_roll", "goal_pitch", "goal_yaw",
                                             "obstacle_1_x", "obstacle_1_y", "obstacle_1_z",
                                             "obstacle_2_roll", "obstacle_2_x", "obstacle_2_pitch"};

    model_translator->UpdateCurrentStateVector(remove_names, false);

    MatrixXd expected_return_state_vector(model_translator->current_state_vector.dof*2, 1);

    expected_return_state_vector << 0, -0.183, 0, -3.1, 0, 1.34, 0, 0, 0,
            0.5, 0.2, 0.1,
            0, 0, 0,
            0.1, 0.1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0;

    MatrixXd return_state_vector = model_translator->ReturnStateVector(MuJoCo_helper->master_reset_data,
                                                                       model_translator->current_state_vector);

    for(int i = 0; i < model_translator->current_state_vector.dof*2; i++){
        EXPECT_EQ(expected_return_state_vector(i), return_state_vector(i));
    }
}

TEST(model_translator, anyMal_SetReturnState){

    // Create state vector for anymal, and then set it then return it and check we get the same thing
    std::cout << "Begin test - Compare derivatives anyMal \n";
    std::shared_ptr<anyMal> anymal = std::make_shared<anyMal>();
    model_translator = anymal;
    std::cout << "Initialising system to start state \n";

    model_translator->InitialiseSystemToStartState(model_translator->MuJoCo_helper->master_reset_data);

    std::shared_ptr<MuJoCoHelper> MuJoCo_helper = model_translator->MuJoCo_helper;

    MatrixXd test_state_vector(model_translator->current_state_vector.dof_quat + model_translator->current_state_vector.dof, 1);
    test_state_vector << 1, 2, 3, 1, 0, 0, 0,
                        0.1, 0.1, 0.1,
                        0.2, 0.2, 0.2,
                        0.1, 0.1, 0.1,
                        0.2, 0.2, 0.2,
                        0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
                        0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01;

    model_translator->SetStateVectorQuat(test_state_vector, MuJoCo_helper->master_reset_data,
                                     model_translator->current_state_vector);

    MatrixXd return_state_vector = model_translator->ReturnStateVectorQuaternions(MuJoCo_helper->master_reset_data,
                                                                       model_translator->current_state_vector);

    std::cout << "Test state vector: \n" << test_state_vector.transpose() << "\n";
    std::cout << "Return state vector: \n" << return_state_vector.transpose() << "\n";
    for(int i = 0; i < model_translator->current_state_vector.dof_quat + model_translator->current_state_vector.dof; i++){
        EXPECT_NEAR(test_state_vector(i), return_state_vector(i), 1e-9);
    }

}

TEST(model_translator, anyMal_SetReturnControlVector){

    // Create state vector for anymal, and then set it then return it and check we get the same thing
    std::cout << "Begin test - Set Return control vectors \n";
    std::shared_ptr<anyMal> anymal = std::make_shared<anyMal>();
    model_translator = anymal;
    std::cout << "Initialising system to start state \n";

    model_translator->InitialiseSystemToStartState(model_translator->MuJoCo_helper->master_reset_data);

    std::shared_ptr<MuJoCoHelper> MuJoCo_helper = model_translator->MuJoCo_helper;

    MatrixXd test_control_vector(model_translator->current_state_vector.num_ctrl, 1);
    test_control_vector << 0.5, 0.4, 0.3, 0.2, 0.1, 0.0,
                            0.0, 0.1, 0.2, 0.3, 0.4, 0.5;

    model_translator->SetControlVector(test_control_vector, MuJoCo_helper->master_reset_data,
                                         model_translator->current_state_vector);

    MatrixXd return_control_vector = model_translator->ReturnControlVector(MuJoCo_helper->master_reset_data,
                                                                                  model_translator->current_state_vector);

    std::cout << "Test state vector: \n" << test_control_vector.transpose() << "\n";
    std::cout << "Return state vector: \n" << return_control_vector.transpose() << "\n";

    std::vector<double> anyMal_controls;
//    MuJoCo_helper->GetRobotJointsControls("anyMal", anyMal_controls, MuJoCo_helper->master_reset_data);

//    mj_forwardSkip(MuJoCo_helper->model, MuJoCo_helper->master_reset_data, , 0);
    mj_fwdActuation(MuJoCo_helper->model, MuJoCo_helper->master_reset_data);
    for(int i = 0; i < MuJoCo_helper->model->nu; i++){
        anyMal_controls.push_back(MuJoCo_helper->master_reset_data->actuator_force[i]);
    }
    std::cout << "Control values in mjData: \n";
    for(const auto & control_val : anyMal_controls){
        std::cout << control_val << " ";
    }
    std::cout << "\n";

    for(int i = 0; i < model_translator->current_state_vector.num_ctrl; i++){
        EXPECT_NEAR(test_control_vector(i), return_control_vector(i), 1e-9);
    }
}

TEST(model_translator, bimanual_SetReturnControlVector){

    // Create state vector for anymal, and then set it then return it and check we get the same thing
    std::cout << "Begin test - Set Return control vectors (Bimanual)\n";
    std::shared_ptr<BimanualPickup> bimanual = std::make_shared<BimanualPickup>();
    model_translator = bimanual;
    std::cout << "Initialising system to start state \n";

    model_translator->InitialiseSystemToStartState(model_translator->MuJoCo_helper->master_reset_data);

    std::shared_ptr<MuJoCoHelper> MuJoCo_helper = model_translator->MuJoCo_helper;

    MatrixXd test_control_vector(model_translator->current_state_vector.num_ctrl, 1);
    test_control_vector << 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.0,
            0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;

    model_translator->SetControlVector(test_control_vector, MuJoCo_helper->master_reset_data,
                                       model_translator->current_state_vector);

    MatrixXd return_control_vector = model_translator->ReturnControlVector(MuJoCo_helper->master_reset_data,
                                                                           model_translator->current_state_vector);

    std::cout << "Test control vector: \n" << test_control_vector.transpose() << "\n";
    std::cout << "Return control vector: \n" << return_control_vector.transpose() << "\n";

//    std::vector<double> anyMal_controls;
////    MuJoCo_helper->GetRobotJointsControls("anyMal", anyMal_controls, MuJoCo_helper->master_reset_data);
//
////    mj_forwardSkip(MuJoCo_helper->model, MuJoCo_helper->master_reset_data, , 0);
//    mj_fwdActuation(MuJoCo_helper->model, MuJoCo_helper->master_reset_data);
//    for(int i = 0; i < MuJoCo_helper->model->nu; i++){
//        anyMal_controls.push_back(MuJoCo_helper->master_reset_data->actuator_force[i]);
//    }
//    std::cout << "Control values in mjData: \n";
//    for(const auto & control_val : anyMal_controls){
//        std::cout << control_val << " ";
//    }
//    std::cout << "\n";

    for(int i = 0; i < model_translator->current_state_vector.num_ctrl; i++){
        EXPECT_NEAR(test_control_vector(i), return_control_vector(i), 1e-9);
    }

}

TEST(model_translator, bimanual_SetReturnStateVector){

    // Create state vector for anymal, and then set it then return it and check we get the same thing
    std::cout << "Begin test - Set Return control vectors (Bimanual)\n";
    std::shared_ptr<BimanualPickup> bimanual = std::make_shared<BimanualPickup>();
    model_translator = bimanual;
    std::cout << "Initialising system to start state \n";

    model_translator->InitialiseSystemToStartState(model_translator->MuJoCo_helper->master_reset_data);

    std::shared_ptr<MuJoCoHelper> MuJoCo_helper = model_translator->MuJoCo_helper;

    MatrixXd test_state_vector(model_translator->current_state_vector.dof + model_translator->current_state_vector.dof_quat, 1);
    test_state_vector << 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.0,
            0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6,
            0.5, 0.4, 0.3, 1.0, 0.0, 0.0, 0.0,
            0, 0, 0, 0, 0, 0, 0,
            0 ,0 ,0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0;

    model_translator->SetStateVectorQuat(test_state_vector, MuJoCo_helper->master_reset_data,
                                       model_translator->current_state_vector);

    MatrixXd return_state_vector = model_translator->ReturnStateVectorQuaternions(MuJoCo_helper->master_reset_data,
                                                                           model_translator->current_state_vector);

    std::cout << "Test state vector: \n" << test_state_vector.transpose() << "\n";
    std::cout << "Return state vector: \n" << return_state_vector.transpose() << "\n";

    for(int i = 0; i < model_translator->current_state_vector.dof + model_translator->current_state_vector.dof_quat; i++){
        EXPECT_NEAR(test_state_vector(i), return_state_vector(i), 1e-9);
    }
}

TEST(model_translator, bimanual_ControlLims){

    // Create state vector for anymal, and then set it then return it and check we get the same thing
    std::cout << "Begin test - Set Return control vectors (Bimanual)\n";
    std::shared_ptr<BimanualPickup> bimanual = std::make_shared<BimanualPickup>();
    model_translator = bimanual;
    std::cout << "Initialising system to start state \n";

    model_translator->InitialiseSystemToStartState(model_translator->MuJoCo_helper->master_reset_data);

    std::shared_ptr<MuJoCoHelper> MuJoCo_helper = model_translator->MuJoCo_helper;

    MatrixXd control_lims = model_translator->ReturnControlLimits(model_translator->current_state_vector);

    MatrixXd control_lims_robot(14, 1);
    control_lims_robot << -87, 87, -87, 87, -87, 87, -87, 87,
            -12, 12, -12, 12, -12, 12;

    std::cout << "control lims: \n" << control_lims << "\n";

//    MatrixXd test_state_vector(model_translator->current_state_vector.dof + model_translator->current_state_vector.dof_quat, 1);
//    test_state_vector << 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.0,
//            0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6,
//            0.5, 0.4, 0.3, 1.0, 0.0, 0.0, 0.0,
//            0, 0, 0, 0, 0, 0, 0,
//            0 ,0 ,0, 0, 0, 0, 0,
//            0, 0, 0, 0, 0, 0;
//
//    model_translator->SetStateVectorQuat(test_state_vector, MuJoCo_helper->master_reset_data,
//                                         model_translator->current_state_vector);
//
//    MatrixXd return_state_vector = model_translator->ReturnStateVectorQuaternions(MuJoCo_helper->master_reset_data,
//                                                                                  model_translator->current_state_vector);
//
//    std::cout << "Test state vector: \n" << test_state_vector.transpose() << "\n";
//    std::cout << "Return state vector: \n" << return_state_vector.transpose() << "\n";
//

    // Robot 1
    for(int i = 0; i < 14; i++){
        EXPECT_NEAR(control_lims(i), control_lims_robot(i), 1e-9);
    }

    // Robot 2
    for(int i = 0; i < 14; i++){
        EXPECT_NEAR(control_lims(i + 14), control_lims_robot(i), 1e-9);
    }
}

int main(int argc, char* argv[]){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
