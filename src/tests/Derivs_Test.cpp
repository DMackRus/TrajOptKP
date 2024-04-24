#include <gtest/gtest.h>

#include "Differentiator.h"
#include "ModelTranslator/ModelTranslator.h"
#include "test_acrobot.h"

#include "3D_test_class.h"

std::shared_ptr<ModelTranslator> model_translator;
std::shared_ptr<Differentiator> differentiator;

void compare_derivs(){
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

    bool flg_centred = false;

    std::cout << "start of mjd_transitionFD \n";
    auto start = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < T; i++){
        mjd_transitionFD(
                model_translator->MuJoCo_helper->model, model_translator->MuJoCo_helper->master_reset_data, 1e-6, flg_centred,
                DataAt(A, 0 * (dim_state_derivative * dim_state_derivative)),
                DataAt(B, 0 * (dim_state_derivative * dim_action)),
                DataAt(C, 0 * (dim_sensor * dim_state_derivative)),
                DataAt(D, 0 * (dim_sensor * dim_action)));
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

    MatrixXd A_diff;
    MatrixXd B_diff;
    A_diff.resize(dim_state_derivative, dim_state_derivative);
    B_diff.resize(dim_state_derivative, dim_action);

    for(int i = 0; i < dim_state_derivative; i++){
        for(int j = 0; j < dim_state_derivative; j++){
            EXPECT_NEAR(A_mine[0](i, j), A[i * dim_state_derivative + j], 1.0e-5);
            A_diff(i, j) = abs(A[i * dim_state_derivative + j] - A_mine[0](i, j));
            if(A_diff(i, j) < 1e-6) A_diff(i, j) = 0;
        }
    }

    std::cout << "A diff \n";
    std::cout << A_diff << "\n";

    for(int i = 0; i < dim_state_derivative; i++){
        for(int j = 0; j < dim_action; j++){
            EXPECT_NEAR(B_mine[0](i, j), B[i * dim_action + j], 1.0e-5);
            B_diff(i, j) = abs(B[i * dim_action + j] - B_mine[0](i, j));
            if(B_diff(i, j) < 1e-6) B_diff(i, j) = 0;
        }
    }
}

TEST(Derivatives, acrobot)
{
    std::cout << "Begin test - Compare derivatives acrobot \n";
    std::shared_ptr<Acrobot> acrobot = std::make_shared<Acrobot>();
    model_translator = acrobot;

    differentiator = std::make_shared<Differentiator>(model_translator, model_translator->MuJoCo_helper);

    // Initialise a state for the simulator
    MatrixXd start_state(model_translator->state_vector_size, 1);
    start_state << 0, 0, 0, 0;
    model_translator->SetStateVector(start_state, model_translator->MuJoCo_helper->master_reset_data);

    // Step the similar to stabilise things
    for(int j = 0; j < 5; j++){
        mj_step(model_translator->MuJoCo_helper->model, model_translator->MuJoCo_helper->master_reset_data);
    }
    // Append data to save systems state list
    model_translator->MuJoCo_helper->AppendSystemStateToEnd(model_translator->MuJoCo_helper->master_reset_data);

    compare_derivs();

}

TEST(Derivatives, pushing_3D)
{
    std::cout << "Begin test - Compare derivatives 3D - not rotated \n";
    std::shared_ptr<threeDTestClass> pushing_3D = std::make_shared<threeDTestClass>();
    model_translator = pushing_3D;

    differentiator = std::make_shared<Differentiator>(model_translator, model_translator->MuJoCo_helper);

    // Initialise a state for the simulator
    MatrixXd start_state(model_translator->state_vector_size, 1);
    start_state << 0, -0.183, 0, -3.1, 0, 1.34, 0, 0, 0,
                    0.5, 0.2, 0.1, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0;
    model_translator->SetStateVector(start_state, model_translator->MuJoCo_helper->master_reset_data);

    // Append data to save systems state list
    model_translator->MuJoCo_helper->AppendSystemStateToEnd(model_translator->MuJoCo_helper->master_reset_data);

    compare_derivs();
}

TEST(Derivatives, pushing_3D_rotated)
{
    std::cout << "Begin test - Compare derivatives 3D - rotated \n";
    std::shared_ptr<threeDTestClass> pushing_3D = std::make_shared<threeDTestClass>();
    model_translator = pushing_3D;

    differentiator = std::make_shared<Differentiator>(model_translator, model_translator->MuJoCo_helper);

    // Initialise a state for the simulator
    MatrixXd start_state(model_translator->state_vector_size, 1);
    start_state << 0, -0.183, 0, -3.1, 0, 1.34, 0, 0, 0,
            0.5, 0.2, 0.1, 0.5, 0.2, 0.1,
            0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0;
    model_translator->SetStateVector(start_state, model_translator->MuJoCo_helper->master_reset_data);

    // Append data to save systems state list
    model_translator->MuJoCo_helper->AppendSystemStateToEnd(model_translator->MuJoCo_helper->master_reset_data);

    compare_derivs();
}

int main(int argc, char* argv[]){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}