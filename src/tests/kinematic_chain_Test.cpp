#include <gtest/gtest.h>

#include "test_walker.h"
#include "3D_test_class.h"

std::shared_ptr<ModelTranslator> model_translator;

// Helper function to make sure kinematic chains are as expected.
void checkChainsMatch(const std::vector<std::vector<int>>& actual_chains,
                      const std::vector<std::vector<int>>& correct_chains) {

    for (size_t i = 0; i < correct_chains.size(); ++i) {
        // Skip empty chains in actual
        if (actual_chains[i].empty()) continue;

        const auto& actual = actual_chains[i];
        const auto& correct = correct_chains[i];

        std::set<int> actual_set(actual.begin(), actual.end());
        std::set<int> correct_set(correct.begin(), correct.end());

        EXPECT_EQ(actual_set, correct_set) << "Mismatch in chain " << i;
    }
}


TEST(kinematic_chain, walker){

    std::shared_ptr<Walker> walker = std::make_shared<Walker>();
    model_translator = walker;

//    //Print qpos adresses
//    std::cout << "State vector qpos addresses: \n";
//    for(int i = 0; i < model_translator->current_state_vector.q_pos_adr.size(); i++){
//        std::cout << model_translator->current_state_vector.q_pos_adr[i] << " ";
//    }
//    std::cout << "\n";
//
//    // Print state vector names
//    std::cout << "State vector names: \n";
//    for(int i = 0; i < model_translator->current_state_vector.state_names.size(); i++){
//        std::cout << model_translator->current_state_vector.state_names[i] << " ";
//    }
//    std::cout << "\n";
//
//    // Print out kinematic chain
//    std::cout << "Kinematic chain bodies: \n";
//    for(int i = 0; i < model_translator->current_state_vector.kinematic_chains_bodies.size(); i++){
//        std::cout << "Chain " << i << ": ";
//        for(int j = 0; j < model_translator->current_state_vector.kinematic_chains_bodies[i].size(); j++){
//            std::cout << model_translator->current_state_vector.kinematic_chains_bodies[i][j] << " ";
//        }
//        std::cout << "\n";
//    }
//
//    // Print out kinematic chain state indices
//    std::cout << "Kinematic chains state indices: \n";
//    for(int i = 0; i < model_translator->current_state_vector.kinematic_chain_state_indices.size(); i++){
//        std::cout << "Chain " << i << ": ";
//        for(int j = 0; j < model_translator->current_state_vector.kinematic_chain_state_indices[i].size(); j++){
//            std::cout << model_translator->current_state_vector.kinematic_chain_state_indices[i][j] << " ";
//        }
//        std::cout << "\n";
//    }

    std::vector<std::vector<int>> correct_chains = {
        {0, 1, 2, 3, 4, 5, 6, 7, 8}
    };

    checkChainsMatch(
        model_translator->current_state_vector.kinematic_chain_state_indices,
        correct_chains);

}

TEST(kinematic_chain, 3D_test_class){

    std::shared_ptr<threeDTestClass> test_class = std::make_shared<threeDTestClass>();
    model_translator = test_class;

    std::vector<std::vector<int>> correct_chains = {
        {0, 1, 2, 3, 4, 5, 6, 7, 8},
        {9, 10, 11, 12, 13, 14},
        {15, 16, 17, 18, 19, 20},
        {21, 22, 23, 24, 25, 26}
    };

    checkChainsMatch(
        model_translator->current_state_vector.kinematic_chain_state_indices,
        correct_chains);

}

int main(int argc, char* argv[]){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}