#include <gtest/gtest.h>

#include "Differentiator.h"
#include "ModelTranslator.h"
#include "Acrobot.h"
#include "FileHandler.h"
#include "Visualiser.h"

TEST(temp, temp2)
{
    std::shared_ptr<ModelTranslator> model_translator;
    std::shared_ptr<Differentiator> differentiator;
    std::shared_ptr<FileHandler> file_handler;
    std::shared_ptr<Visualiser> visualiser;


    std::shared_ptr<Acrobot> acrobot = std::make_shared<Acrobot>();

    model_translator = acrobot;
    differentiator = std::make_shared<Differentiator>(model_translator, model_translator->MuJoCo_helper);
    visualiser = std::make_shared<Visualiser>(model_translator);

    // Initialise a state for the simulator
    MatrixXd start_state(model_translator->state_vector_size, 1);
    start_state << 0, 0, 0, 0;
    model_translator->SetStateVector(start_state, model_translator->MuJoCo_helper->master_reset_data);

    // Step the similar to stabilise things
    for(int j = 0; j < 5; j++){
        mj_step(model_translator->MuJoCo_helper->model, model_translator->MuJoCo_helper->master_reset_data);
    }
    // Append data to save systems state list
    model_translator->MuJoCo_helper->appendSystemStateToEnd(model_translator->MuJoCo_helper->master_reset_data);

    // Compute the A, B, C and D matrices via mjd_transitionFD
    // - Allocate A, B, C and D matrices.
    int dim_state_derivative = model_translator->MuJoCo_helper->model->nv * 2;
    int dof = dim_state_derivative / 2;
    int dim_action = model_translator->MuJoCo_helper->model->nu;
    int dim_sensor = model_translator->MuJoCo_helper->model->nsensordata;
    int T = 1;
    std::vector<double> A;
    std::vector<double> B;
    std::vector<double> C;
    std::vector<double> D;

    A.resize(dim_state_derivative * dim_state_derivative * T);
    B.resize(dim_state_derivative * dim_action * T);
    C.resize(dim_sensor * dim_state_derivative * T);
    D.resize(dim_sensor * dim_action * T);

    int t = 0;

//    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
//        MatrixXd control_vector = MatrixXd::Zero(dim_action, 1);
//        control_vector << -1, -1, -1, -1, -1, -1;
//        activeModelTranslator->SetControlVector(control_vector, activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0]);
    bool flg_centred = false;

    std::cout << "start of mjd_transitionFD \n";
    auto start = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < T; i++){
        mjd_transitionFD(
                model_translator->MuJoCo_helper->model, model_translator->MuJoCo_helper->master_reset_data, 1e-6, flg_centred,
                DataAt(A, t * (dim_state_derivative * dim_state_derivative)),
                DataAt(B, t * (dim_state_derivative * dim_action)),
                DataAt(C, t * (dim_sensor * dim_state_derivative)),
                DataAt(D, t * (dim_sensor * dim_action)));
    }
    std::cout << "time taken for mjd_transitionFD " << std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start).count() / 1000.0f << "ms\n";

    // Now my code
    std::vector<MatrixXd> A_mine;
    std::vector<MatrixXd> B_mine;

    int dof_model_translator = model_translator->dof;

    A_mine.push_back(MatrixXd(dof_model_translator*2, dof_model_translator*2));
    B_mine.push_back(MatrixXd(dof_model_translator*2, dim_action));

    std::vector<int> cols(dof_model_translator, 0);
    for (int i = 0; i < dof_model_translator; i++) {
        cols[i] = i;
    }

    MatrixXd l_x, l_xx, l_u, l_uu;
    double time = 0.0f;
    start = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < T; i++){
        differentiator->ComputeDerivatives(A_mine[0], B_mine[0], cols, l_x, l_u, l_xx, l_uu,
                                                 0, 0, false, false, flg_centred, 1e-6);
        time += differentiator->time_mj_forwards;
    }
    std::cout << "time of mj_forwards calls " << (time / 1000.0f) << "ms\n";
    std::cout << "time taken for my code " << (std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start).count()) / 1000.0f << "ms\n";


    // compute difference and print
    MatrixXd A_diff;
    MatrixXd B_diff;
    A_diff.resize(dim_state_derivative, dim_state_derivative);
    B_diff.resize(dim_state_derivative, dim_action);
    std::cout << "A_mine[0] \n";
    std::cout << A_mine[0] << std::endl;

    for(int i = 0; i < dim_state_derivative; i++){
        for(int j = 0; j < dim_state_derivative; j++){
            A_diff(i, j) = abs(A[i * dim_state_derivative + j] - A_mine[0](i, j));
            if(A_diff(i, j) < 1e-6) A_diff(i, j) = 0;
        }
    }

    std::cout << "A_diff \n";
    std::cout << A_diff << std::endl;

    for(int i = 0; i < dim_state_derivative; i++){
        for(int j = 0; j < dim_action; j++){
            B_diff(i, j) = abs(B[i * dim_action + j] - B_mine[0](i, j));
            if(B_diff(i, j) < 1e-6) B_diff(i, j) = 0;
        }
    }

    std::cout << "B_diff \n";
    std::cout << B_diff << std::endl;

    EXPECT_NEAR(A_mine[0](0, 0), A[0], 1.0e-5);


}

int main(int argc, char* argv[]){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}