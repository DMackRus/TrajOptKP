#include <gtest/gtest.h>

#include "KeyPointGenerator.h"
#include "Differentiator.h"
#include "ModelTranslator.h"
#include "Acrobot.h"

std::shared_ptr<ModelTranslator> model_translator;

void AssertKeypoints(std::vector<std::vector<int>> keypoints, int T, int max_N){
    // Assert all keypoints at time index 0
    for(int i = 0; i < model_translator->dof; i++){
        ASSERT_EQ(i, keypoints[0][i]);
    }

    // Assert all keypoints at time index T - 1
    for(int i = 0; i < model_translator->dof; i++){
        ASSERT_EQ(i, keypoints[T-1][i]);
    }

    // Assert that keypoints arent located more than maxN apart?
    std::vector<int> last_time_indices(model_translator->dof, 0);
    for(int t = 0; t < T; t++){

        for(int i : keypoints[t]){
            // Check for every keypoint, that the last keypoint was no more than maxN steps ago.
            ASSERT_LE((t - last_time_indices[i]), max_N);

            last_time_indices[i] = t;
        }
    }

}

void CreateTrajectory(std::vector<MatrixXd> &trajectory_states, int T){

    for(int t = 0; t < T; t++){
        MatrixXd current_state = model_translator->ReturnStateVector(model_translator->MuJoCo_helper->master_reset_data);

        trajectory_states.push_back(current_state);

        mj_step(model_translator->MuJoCo_helper->model, model_translator->MuJoCo_helper->master_reset_data);
    }

}

TEST(keypoints, set_interval){

    std::shared_ptr<Acrobot> acrobot = std::make_shared<Acrobot>();
    model_translator = acrobot;

    int T = 100;

    std::shared_ptr<Differentiator> differentiator =
            std::make_shared<Differentiator>(model_translator, model_translator->MuJoCo_helper);

    std::shared_ptr<KeypointGenerator> keypoint_generator =
            std::make_shared<KeypointGenerator>(differentiator,
                                                model_translator->MuJoCo_helper,
                                                model_translator->dof, T);

    keypoint_method keypoint_method;
    keypoint_method.name = "set_interval";
    keypoint_method.min_N = 2;
    keypoint_method.auto_adjust = false;

    keypoint_generator->SetKeypointMethod(keypoint_method);

    std::vector<MatrixXd> trajectory_states;
    std::vector<MatrixXd> A;
    std::vector<MatrixXd> B;

    keypoint_generator->GenerateKeyPoints(trajectory_states, A, B);

    // Test time index 0
    for(int i = 0; i < model_translator->dof; i++){
        ASSERT_EQ(i, keypoint_generator->keypoints[0][i]);
    }

    // Test time index n
    for(int i = 0; i < model_translator->dof; i++){
        ASSERT_EQ(i, keypoint_generator->keypoints[keypoint_method.min_N][i]);
    }

    // Test time index T
    for(int i = 0; i < model_translator->dof; i++){
        ASSERT_EQ(i, keypoint_generator->keypoints[T-1][i]);
    }

    keypoint_method.min_N = 3;
    keypoint_generator->SetKeypointMethod(keypoint_method);

    keypoint_generator->GenerateKeyPoints(trajectory_states, A, B);

    // Test time index 0
    for(int i = 0; i < model_translator->dof; i++){
        ASSERT_EQ(i, keypoint_generator->keypoints[0][i]);
    }

    // Test time index n
    for(int i = 0; i < model_translator->dof; i++){
        ASSERT_EQ(i, keypoint_generator->keypoints[keypoint_method.min_N][i]);
    }

    // Test time index T
    for(int i = 0; i < model_translator->dof; i++){
        ASSERT_EQ(i, keypoint_generator->keypoints[T-1][i]);
    }

}

TEST(keypoints, adaptive_jerk){

    std::shared_ptr<Acrobot> acrobot = std::make_shared<Acrobot>();
    model_translator = acrobot;

    int T = 100;

    std::shared_ptr<Differentiator> differentiator =
            std::make_shared<Differentiator>(model_translator, model_translator->MuJoCo_helper);

    std::shared_ptr<KeypointGenerator> keypoint_generator =
            std::make_shared<KeypointGenerator>(differentiator,
                                                model_translator->MuJoCo_helper,
                                                model_translator->dof, T);

    MatrixXd start_state(model_translator->state_vector_size, 1);
    start_state << 0.5, 0.1, 0, 0;
    model_translator->SetStateVector(start_state, model_translator->MuJoCo_helper->master_reset_data);

    keypoint_method keypoint_method;
    keypoint_method.name = "adaptive_jerk";
    keypoint_method.min_N = 1;
    keypoint_method.max_N = 5;
    double jerk_threshold = 0.0005;
    for(int i = 0; i < model_translator->dof; i++){
        keypoint_method.jerk_thresholds.push_back(jerk_threshold);
    }
    keypoint_method.auto_adjust = false;

    keypoint_generator->SetKeypointMethod(keypoint_method);

    std::vector<MatrixXd> trajectory_states;
    std::vector<MatrixXd> A;
    std::vector<MatrixXd> B;

    // Create a trajectory
    CreateTrajectory(trajectory_states, T);

    keypoint_generator->GenerateKeyPoints(trajectory_states, A, B);

    AssertKeypoints(keypoint_generator->keypoints, T, keypoint_method.max_N);

}

TEST(keypoints, velocity_change){
    std::shared_ptr<Acrobot> acrobot = std::make_shared<Acrobot>();
    model_translator = acrobot;

    int T = 100;

    std::shared_ptr<Differentiator> differentiator =
            std::make_shared<Differentiator>(model_translator, model_translator->MuJoCo_helper);

    std::shared_ptr<KeypointGenerator> keypoint_generator =
            std::make_shared<KeypointGenerator>(differentiator,
                                                model_translator->MuJoCo_helper,
                                                model_translator->dof, T);

    MatrixXd start_state(model_translator->state_vector_size, 1);
    start_state << 0.5, 0.1, 0, 0;
    model_translator->SetStateVector(start_state, model_translator->MuJoCo_helper->master_reset_data);

    keypoint_method keypoint_method;
    keypoint_method.name = "velocity_change";
    keypoint_method.min_N = 1;
    keypoint_method.max_N = 5;
    double velocity_change_threshold = 0.1;
    for(int i = 0; i < model_translator->dof; i++){
        keypoint_method.velocity_change_thresholds.push_back(velocity_change_threshold);
    }
    keypoint_method.auto_adjust = false;

    keypoint_generator->SetKeypointMethod(keypoint_method);

    std::vector<MatrixXd> trajectory_states;
    std::vector<MatrixXd> A;
    std::vector<MatrixXd> B;

    // Create a trajectory
    CreateTrajectory(trajectory_states, T);

    keypoint_generator->GenerateKeyPoints(trajectory_states, A, B);

    AssertKeypoints(keypoint_generator->keypoints, T, keypoint_method.max_N);
}

// TODO - Write a test for auto adjust keypoint methods.
//TEST(keypoints, auto_adjust){
//
//}

int main(int argc, char* argv[]){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}