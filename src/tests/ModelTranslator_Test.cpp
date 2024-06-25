#include <gtest/gtest.h>

#include "ModelTranslator/ModelTranslator.h"
#include "test_acrobot.h"
#include "3D_test_class.h"

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

int main(int argc, char* argv[]){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
