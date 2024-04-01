#include <gtest/gtest.h>

#include "KeyPointGenerator.h"
#include "Differentiator.h"
#include "ModelTranslator.h"
#include "Acrobot.h"

TEST(keypoints, set_interval){

    std::shared_ptr<ModelTranslator> model_translator;
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
    }    std::cout << "keypoints";

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
    }    std::cout << "keypoints";

    // Test time index T
    for(int i = 0; i < model_translator->dof; i++){
        ASSERT_EQ(i, keypoint_generator->keypoints[T-1][i]);
    }

//    for(int t = 0; t < T; t++){
//        std::cout << "time index " << t << ": ";
//        for(int i = 0; i < keypoint_generator->keypoints[t].size(); i++){
//            std::cout << keypoint_generator->keypoints[t][i] << " ";
//        }
//        std::cout << "\n";
//    }
}

TEST(keypoints, adaptive_jerk){

}

int main(int argc, char* argv[]){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}