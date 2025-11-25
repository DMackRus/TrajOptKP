#include "StdInclude.h"
#include "FileHandler.h"
#include "Visualiser.h"
#include "MuJoCoHelper.h"

// --------------------- different scenes -----------------------
#include "ModelTranslator/PistonBlock.h"
#include "ModelTranslator/Acrobot.h"
#include "ModelTranslator/ArticulatedContact.h"
#include "ModelTranslator/BoxSweep.h"

#include "Optimiser/Optimiser.h"
#include "Optimiser/iLQR.h"

// --------------------- other -----------------------
#include <mutex>
//#include <queue>

// --------------------- Global variables -----------------------
std::shared_ptr<ModelTranslator> activeModelTranslator;
std::shared_ptr<Differentiator> activeDifferentiator;
std::shared_ptr<iLQR> iLQROptimiser;
std::shared_ptr<Visualiser> activeVisualiser;
std::shared_ptr<FileHandler> yamlReader;

void ApproximationError(
        const std::vector<Eigen::MatrixXd> &A_exact,
        const std::vector<Eigen::MatrixXd> &B_exact,
        const std::vector<Eigen::MatrixXd> &A_approx,
        const std::vector<Eigen::MatrixXd> &B_approx,
        double &mse,
        double &elementnorm_mse_error,
        double &max_abs_error)
{
    assert(A_exact.size() == A_approx.size());
    assert(B_exact.size() == B_approx.size());
    size_t T = A_exact.size();

    const double eps = 1e-8;

    mse = 0.0;
    elementnorm_mse_error = 0.0;
    max_abs_error = 0.0;

    // 1. Compute global MSE and max absolute error
    size_t total_elements = 0;
    for (size_t t = 0; t < T; ++t) {
        Eigen::MatrixXd diffA = A_exact[t] - A_approx[t];
        Eigen::MatrixXd diffB = B_exact[t] - B_approx[t];
        double step_mse = diffA.squaredNorm() + diffB.squaredNorm();
        mse += step_mse;

        if(step_mse > max_abs_error){
            max_abs_error = step_mse;
        }
    }
    // Normalise MSE by total number of elements across all timesteps
    mse /= static_cast<double>((A_exact[0].size() + B_exact[0].size()) * T);
    max_abs_error /= static_cast<double>(A_exact[0].size() + B_exact[0].size());

    // 2. Compute elementwise norm matrices for A and B based on exact values
    Eigen::MatrixXd minA = Eigen::MatrixXd::Zero(A_exact[0].rows(), A_exact[0].cols());
    Eigen::MatrixXd maxA = Eigen::MatrixXd::Zero(A_exact[0].rows(), A_exact[0].cols());
    Eigen::MatrixXd minB = Eigen::MatrixXd::Zero(B_exact[0].rows(), B_exact[0].cols());
    Eigen::MatrixXd maxB = Eigen::MatrixXd::Zero(B_exact[0].rows(), B_exact[0].cols());

    // Find the min and max values for each element across all timesteps
    for(int t = 0; t < T; t++){
        for(int i = 0; i < A_exact[t].rows(); i++){
            for(int j = 0; j < A_exact[t].cols(); j++){
                if(t == 0){
                    minA(i, j) = A_exact[t](i, j);
                    maxA(i, j) = A_exact[t](i, j);
                }
                else{
                    if(A_exact[t](i, j) < minA(i, j)) minA(i, j) = A_exact[t](i, j);
                    if(A_exact[t](i, j) > maxA(i, j)) maxA(i, j) = A_exact[t](i, j);
                }
            }
        }
        for(int i = 0; i < B_exact[t].rows(); i++){
            for(int j = 0; j < B_exact[t].cols(); j++){
                if(t == 0){
                    minB(i, j) = B_exact[t](i, j);
                    maxB(i, j) = B_exact[t](i, j);
                }
                else{
                    if(B_exact[t](i, j) < minB(i, j)) minB(i, j) = B_exact[t](i, j);
                    if(B_exact[t](i, j) > maxB(i, j)) maxB(i, j) = B_exact[t](i, j);
                }
            }
        }
    }

    // Normalise the exact and approximate matrices using these min and max values
    for(int t = 0; t < T; t++){
        Eigen::MatrixXd normA_exact = Eigen::MatrixXd::Zero(A_exact[t].rows(), A_exact[t].cols());
        Eigen::MatrixXd normA_approx = Eigen::MatrixXd::Zero(A_approx[t].rows(), A_approx[t].cols());
        Eigen::MatrixXd normB_exact = Eigen::MatrixXd::Zero(B_exact[t].rows(), B_exact[t].cols());
        Eigen::MatrixXd normB_approx = Eigen::MatrixXd::Zero(B_approx[t].rows(), B_approx[t].cols());

        for(int i = 0; i < A_exact[t].rows(); i++){
            for(int j = 0; j < A_exact[t].cols(); j++){
                normA_exact(i, j) = (A_exact[t](i, j) - minA(i, j)) / (maxA(i, j) - minA(i, j) + eps);
                normA_approx(i, j) = (A_approx[t](i, j) - minA(i, j)) / (maxA(i, j) - minA(i, j) + eps);
            }
        }
        for(int i = 0; i < B_exact[t].rows(); i++){
            for(int j = 0; j < B_exact[t].cols(); j++){
                normB_exact(i, j) = (B_exact[t](i, j) - minB(i, j)) / (maxB(i, j) - minB(i, j) + eps);
                normB_approx(i, j) = (B_approx[t](i, j) - minB(i, j)) / (maxB(i, j) - minB(i, j) + eps);
            }
        }

        // Compute the MSE between the normalised exact and approximate matrices
        double step_mse = (normA_exact - normA_approx).squaredNorm() + (normB_exact - normB_approx).squaredNorm();
        elementnorm_mse_error += step_mse;
    }
    elementnorm_mse_error /= static_cast<double>((A_exact[0].size() + B_exact[0].size()) * T);
}

Eigen::MatrixXd getMassMatrix(const mjModel* m, mjData* d) {
    // Allocate a buffer to store the mass matrix
    int nv = m->nv;
    std::vector<mjtNum> M(nv * nv);

    // Fill the buffer with the mass matrix
    mj_fullM(m, M.data(), d->qM);

    // Copy into an Eigen matrix
    Eigen::MatrixXd M_eigen(nv, nv);
    for (int i = 0; i < nv; ++i) {
        for (int j = 0; j < nv; ++j) {
            M_eigen(i, j) = M[i * nv + j];  // row-major access
        }
    }

    return M_eigen;
}

Eigen::MatrixXd getCoriolisMatrix(const mjModel* m, mjData* d){
    mj_forward(m, d);

    double* cqdq = new double[m->nv];
    for (int i = 0; i < m->nv; i++) {
        cqdq[i] = d->qfrc_bias[i] - d->qfrc_gravcomp[i];
    }

    int nv = m->nv;
    double h = 1e-6;
    Eigen::MatrixXd C(nv, nv);  // using Eigen for convenience

    for (int j = 0; j < nv; ++j) {
        // Save original
        std::vector<double> orig_dq(d->qvel, d->qvel + nv);

        // Perturb
        d->qvel[j] += h;
        mj_forward(m, d);
        Eigen::VectorXd plus(nv);
        for (int i = 0; i < nv; ++i)
            plus[i] = d->qfrc_bias[i] - d->qfrc_gravcomp[i];

        // Restore and perturb in negative direction
        std::copy(orig_dq.begin(), orig_dq.end(), d->qvel);
        d->qvel[j] -= h;
        mj_forward(m, d);
        Eigen::VectorXd minus(nv);
        for (int i = 0; i < nv; ++i)
            minus[i] = d->qfrc_bias[i] - d->qfrc_gravcomp[i];

        // Central difference
        Eigen::VectorXd column = (plus - minus) / (2.0 * h);
        C.col(j) = column;

        // Reset velocity
        std::copy(orig_dq.begin(), orig_dq.end(), d->qvel);
    }

    return C;
}

//int main(){
//    std::cout << "Toy Derivative Analysis" << std::endl;
//
//    // Doesnt actually do anything for this program
//    yamlReader = std::make_shared<FileHandler>();
//
//    // Instantiate the model translator
//    std::shared_ptr<Acrobot> acrobot = std::make_shared<Acrobot>();
//    activeModelTranslator = acrobot;
//
//    // Instantiate the differentiator
//    activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);
//
//    activeModelTranslator->MuJoCo_helper->AppendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
//    //Instantiate the visualiser
//    activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);
//
//    // Setup the initial horizon, based on open loop or mpc method
//    int opt_horizon = 2000;
//
//    iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator,
//                                           activeModelTranslator->MuJoCo_helper,
//                                           activeDifferentiator,
//                                           opt_horizon, activeVisualiser, yamlReader);
//
//    iLQROptimiser->Resize(activeModelTranslator->current_state_vector.dof,
//                          activeModelTranslator->current_state_vector.num_ctrl,
//                          opt_horizon);
//
//    // Initialise storage for A and B matrices
//    std::vector<MatrixXd> A;
//    std::vector<MatrixXd> B;
//
//    int dof_model_translator = activeModelTranslator->current_state_vector.dof;
//    int dim_action = activeModelTranslator->current_state_vector.num_ctrl;
//    int dim_sensor = activeModelTranslator->MuJoCo_helper->model->nsensordata;
//    int dim_state_derivative = dof_model_translator*2;
//
//    A.push_back(MatrixXd(dim_state_derivative, dim_state_derivative));
//    B.push_back(MatrixXd(dim_state_derivative, dim_action));
//
//    std::vector<int> cols(dof_model_translator, 0);
//    for (int i = 0; i < dof_model_translator; i++) {
//        cols[i] = i;
//    }
//
//    // Initialise scene such that the piston is in contact with the block
//    activeModelTranslator->InitialiseSystemToStartState(activeModelTranslator->MuJoCo_helper->master_reset_data);
//    MatrixXd state_vector = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->master_reset_data,
//                                                              activeModelTranslator->current_state_vector);
//    state_vector(0) = 0.1;  //qpos0
//    state_vector(1) = 0.3;  //qpos1
//    state_vector(2) = 0.0;  //qvel0
//    state_vector(3) = 0.0;  //qvel1
//    activeModelTranslator->SetStateVector(state_vector, activeModelTranslator->MuJoCo_helper->master_reset_data,
//                                          activeModelTranslator->current_state_vector);
//    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
//                                                          activeModelTranslator->MuJoCo_helper->master_reset_data
//                                                          );
//
//    //-------------------- Test 1: Alter control Signal and alter control signal and compute dynamics derivatives ------
//
//    // Open the file
//    std::string projectParentPath = __FILE__;
//    projectParentPath = projectParentPath.substr(0, projectParentPath.find_last_of("/\\"));
//    projectParentPath = projectParentPath.substr(0, projectParentPath.find_last_of("/\\"));
//    std::string dir_name = projectParentPath + "/TestingData/" + activeModelTranslator->model_name;
//
//    //Create directory if it does not exist
//    if(!std::filesystem::exists(dir_name)) {
//        std::filesystem::create_directories(dir_name);
//    }
//
//    // Loops iterate u[0], q[0], q[1], dotq[0], dotq[1]
//    std::string test_name_suffixes[5] = {"u0", "q0", "q1", "dotq0", "dotq1"};
//
//    for(int i = 3; i < 5; i++){
//        std::string filename = dir_name + "/" + test_name_suffixes[i] + ".csv";
//        std::cout << "filename: " << filename << std::endl;
//        ofstream file_output;
//        file_output.open(filename);
//
//        // Create the headers - dependant on outerloop iteration
//        if(i == 0){
//            file_output << "u0" << ",";
//        }
//        else if(i == 1){
//            file_output << "q0" << ",";
//        }
//        else if(i == 2){
//            file_output << "q1" << ",";
//        }
//        else if(i == 3){
//            file_output << "dotq0" << ",";
//        }
//        else if(i == 4){
//            file_output << "dotq1" << ",";
//        }
//
//        for(int j = 0; j < dof_model_translator*2; j++){
//            for(int k = 0; k < dof_model_translator*2; k++){
//                file_output << "A" << j << k << ",";
//            }
//        }
//        for(int j = 0; j < dim_action; j++){
//            for(int k = 0; k < dof_model_translator*2; k++){
//                file_output << "B" << j << k << ",";
//            }
//        }
//        file_output << endl;
//
//        for(int j = 0; j < 100; j++){
//
//            // Alter the state / control signal
//            if(i == 0){
//                activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]->ctrl[0] += 0.01; // Alter the first control signal
//            }
//            else if(i == 1){
//                activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]->qpos[0] += 0.01; // Alter the first state variable
//            }
//            else if(i == 2){
//                activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]->qpos[1] += 0.01; // Alter the second state variable
//            }
//            else if(i == 3){
//                activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]->qvel[0] += 0.01; // Alter the first velocity variable
//            }
//            else if(i == 4){
//                activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]->qvel[1] += 0.01; // Alter the second velocity variable
//            }
//
//            // Compute the dynamics derivatives
//            activeDifferentiator->DynamicsDerivatives(A[0], B[0], cols, 0, 0, false, 1e-6);
//
//            // Save the data
//            if(i == 0){
//                file_output << activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]->ctrl[0] << ",";
//            }
//            else if(i == 1){
//                file_output << activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]->qpos[0] << ",";
//            }
//            else if(i == 2){
//                file_output << activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]->qpos[1] << ",";
//            }
//            else if(i == 3){
//                file_output << activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]->qvel[0] << ",";
//            }
//            else if(i == 4){
//                file_output << activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]->qvel[1] << ",";
//            }
//
//            // Write the A and B matrices to the file
//            for(int k = 0; k < dof_model_translator*2; k++){
//                for(int m = 0; m < dof_model_translator*2; m++){
//                    file_output << A[0](k,m) << ",";
//                }
//            }
//            for(int k = 0; k < dof_model_translator*2; k++){
//                for(int m = 0; m < dim_action; m++){
//                    file_output << B[0](k,m) << ",";
//                }
//            }
//            file_output << endl;
//            // Compute mass matrix
////        mj_forward(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]);
////        Eigen::MatrixXd mass_matrix = getMassMatrix(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]);
////
////        std::cout << "Mass Matrix: " << mass_matrix << endl;
//
//        std::cout << "state vector: " << activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], activeModelTranslator->current_state_vector) << std::endl;
//        Eigen::MatrixXd coriolis_matrix = getCoriolisMatrix(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]);
//        std::cout << "Coriolis Matrix: \n" << coriolis_matrix << std::endl;
//
//
//            //Render and sleep
////        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]);
////        mj_forward(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);
////        activeVisualiser->render("");
////        std::this_thread::sleep_for(std::chrono::milliseconds(100));
//        }
//        file_output.close();
//    }
////    std::vector<double> A_m;
////    std::vector<double> B_m;
////    std::vector<double> C;
////    std::vector<double> D;
////
////    std::cout << "dim state_derivative: " << dim_state_derivative << std::endl;
////    std::cout << "dim action: " << dim_action << std::endl;
////    std::cout << "dim sensor: " << dim_sensor << std::endl;
////
////    A_m.resize(dim_state_derivative * dim_state_derivative * 1);
////    B_m.resize(dim_state_derivative * dim_action * 1);
////    C.resize(dim_sensor * dim_state_derivative * 1);
////    D.resize(dim_sensor * dim_action * 1);
//    return 0;
//}

void TestKeypointMethod(){
    std::cout << "Keypoint method testing" << std::endl;

    // Doesnt actually do anything for this program
    yamlReader = std::make_shared<FileHandler>();

    // Instantiate the model translator
    std::shared_ptr<ArticulatedContact> articulated_contact = std::make_shared<ArticulatedContact>();
    activeModelTranslator = articulated_contact;

    // Instantiate the differentiator
    activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);

    activeModelTranslator->MuJoCo_helper->AppendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
    //Instantiate the visualiser
    activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);

    // Setup the initial horizon, based on open loop or mpc method
    int opt_horizon = 2000;

    iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator,
                                           activeModelTranslator->MuJoCo_helper,
                                           activeDifferentiator,
                                           opt_horizon, activeVisualiser, yamlReader);

    iLQROptimiser->Resize(activeModelTranslator->current_state_vector.dof,
                          activeModelTranslator->current_state_vector.num_ctrl,
                          opt_horizon);

    vector<MatrixXd> init_controls = activeModelTranslator->CreateInitOptimisationControls(opt_horizon);

    MatrixXd state_vector = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->master_reset_data,
                                                                     activeModelTranslator->current_state_vector);

    std::cout << "State vector: " << state_vector << std::endl;
    state_vector(0) = PI;   //qpos0
    state_vector(1) = 0.2;  //qpos1
    state_vector(2) = 0.0;  //qpos2
    state_vector(3) = 0.0;  //qvel0
    state_vector(4) = 0.0;  //qvel1
    state_vector(5) = 0.0;  //qvel2
    activeModelTranslator->SetStateVector(state_vector, activeModelTranslator->MuJoCo_helper->master_reset_data,
                                          activeModelTranslator->current_state_vector);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
                                                          activeModelTranslator->MuJoCo_helper->master_reset_data
    );

    // Test contact list generation
    iLQROptimiser->RolloutTrajectory(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], true, init_controls);
//    for(int i = 0; i < opt_horizon; i++){
//        std::cout << "Contact list at step " << i << ": ";
//        for( const auto& contact_pair : iLQROptimiser->contact_list[i] ) {
//            std::cout << "(" << contact_pair.first << ", " << contact_pair.second << ") ";
//        }
//        std::cout << "\n";
//    }

    // Print the state vector names and q pos addresses
    std::cout << "State vector names: \n";
    for(int i = 0; i < activeModelTranslator->current_state_vector.state_names.size(); i++){
        std::cout << activeModelTranslator->current_state_vector.state_names[i] << " ";
    }
    std::cout << "\n";
    std::cout << "State vector qpos addresses: \n";
    for(int i = 0; i < activeModelTranslator->current_state_vector.q_pos_adr.size(); i++){
        std::cout << activeModelTranslator->current_state_vector.q_pos_adr[i] << " ";
    }
    std::cout << "\n";

    // Print kinematic chains
    std::cout << "Kinematic chains: \n";
    for(int i = 0; i < activeModelTranslator->current_state_vector.kinematic_chains_bodies.size(); i++){
        std::cout << "Chain " << i << ": ";
        for(int j = 0; j < activeModelTranslator->current_state_vector.kinematic_chains_bodies[i].size(); j++){
            std::cout << activeModelTranslator->current_state_vector.kinematic_chains_bodies[i][j] << " ";
        }
        std::cout << "\n";
    }

    // Test keypoint generation
//    iLQROptimiser->keypoint_generator->ContactChangeDyn(iLQROptimiser->X_old,iLQROptimiser->U_old,
//                                                             iLQROptimiser->contact_list, activeModelTranslator->current_state_vector, false);

//    iLQROptimiser->keypoint_generator->ContactAwareKeypointsSep(iLQROptimiser->X_old,iLQROptimiser->U_old,
//                                                        iLQROptimiser->contact_list, activeModelTranslator->current_state_vector);

    //Print out the key points
    std::cout << "Keypoints: \n";
    for(int t = 0; t < opt_horizon; t++){
        if(!iLQROptimiser->keypoint_generator->keypoints[t].empty()){
            std::cout << "time " << t << " :";
            for(int i = 0; i < iLQROptimiser->keypoint_generator->keypoints[t].size(); i++){
                std::cout << iLQROptimiser->keypoint_generator->keypoints[t][i] << " ";
            }
            std::cout << "\n";
        }
    }

    // Playback the trajectory
    for(int t = 0; t < opt_horizon; t++){
        // Copy the system state to the visualiser
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data,
                                                              activeModelTranslator->MuJoCo_helper->saved_systems_state_list[t]);
        // Forward the model
        mj_forward(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);
        // Render the visualiser
        activeVisualiser->render("Keypoint Method Test");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void BoxSweepTest(){
    std::cout << "Keypoint method testing" << std::endl;

    // Doesnt actually do anything for this program
    yamlReader = std::make_shared<FileHandler>();

    // Instantiate the model translator
    std::shared_ptr<BoxSweep> box_sweep = std::make_shared<BoxSweep>();
    activeModelTranslator = box_sweep;

    // Instantiate the differentiator
    activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);

    activeModelTranslator->MuJoCo_helper->AppendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
    //Instantiate the visualiser
    activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);

    // Setup the initial horizon, based on open loop or mpc method
    int opt_horizon = 2000;

    iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator,
                                           activeModelTranslator->MuJoCo_helper,
                                           activeDifferentiator,
                                           opt_horizon, activeVisualiser, yamlReader);

    iLQROptimiser->Resize(activeModelTranslator->current_state_vector.dof,
                          activeModelTranslator->current_state_vector.num_ctrl,
                          opt_horizon);

    // Initialise scene to random state
    std::string task_prefix = activeModelTranslator->model_name;
    yamlReader->LoadTaskFromFile(task_prefix, yamlReader->csvRow, activeModelTranslator->full_state_vector,
                                 activeModelTranslator->residual_list);
    activeModelTranslator->InitialiseSystemToStartState(activeModelTranslator->MuJoCo_helper->master_reset_data);

    // Do any setup
    std::vector<MatrixXd> init_setup_controls = activeModelTranslator->CreateInitSetupControls(1000);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);

    std::vector<MatrixXd> init_opt_controls = activeModelTranslator->CreateInitOptimisationControls(opt_horizon);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

    vector<MatrixXd> init_controls = activeModelTranslator->CreateInitOptimisationControls(opt_horizon);


    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
                                                          activeModelTranslator->MuJoCo_helper->master_reset_data
    );

    // Test contact list generation
    iLQROptimiser->RolloutTrajectory(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], true, init_controls);

    // Print the state vector names and q pos addresses
    std::cout << "State vector names: \n";
    for(int i = 0; i < activeModelTranslator->current_state_vector.state_names.size(); i++){
        std::cout << activeModelTranslator->current_state_vector.state_names[i] << " ";
    }
    std::cout << "\n";
    std::cout << "State vector qpos addresses: \n";
    for(int i = 0; i < activeModelTranslator->current_state_vector.q_pos_adr.size(); i++){
        std::cout << activeModelTranslator->current_state_vector.q_pos_adr[i] << " ";
    }
    std::cout << "\n";

    // Print kinematic chains
    std::cout << "Kinematic chain bodies: \n";
    for(int i = 0; i < activeModelTranslator->current_state_vector.kinematic_chains_bodies.size(); i++){
        std::cout << "Chain " << i << ": ";
        for(int j = 0; j < activeModelTranslator->current_state_vector.kinematic_chains_bodies[i].size(); j++){
            std::cout << activeModelTranslator->current_state_vector.kinematic_chains_bodies[i][j] << " ";
        }
        std::cout << "\n";
    }

    std::cout << "Kinematic chain state indices: \n";
    for(int i = 0; i < activeModelTranslator->current_state_vector.kinematic_chain_state_indices.size(); i++){
        std::cout << "Chain " << i << ": ";
        for(int j = 0; j < activeModelTranslator->current_state_vector.kinematic_chain_state_indices[i].size(); j++){
            std::cout << activeModelTranslator->current_state_vector.kinematic_chain_state_indices[i][j] << " ";
        }
        std::cout << "\n";
    }

    // Print out the contact sequence
    std::cout << "Contact sequence: \n";

    for(int i = 0; i < iLQROptimiser->contact_list.size(); i++){
        std::cout << "Contact " << i << ": ";
        for(int j = 0; j < iLQROptimiser->contact_list[i].size(); j++){
            std::cout << "[" << iLQROptimiser->contact_list[i][j].first << " " << iLQROptimiser->contact_list[i][j].second << "] ";
        }
        std::cout << "\n";
    }

    // Test keypoint generation
    iLQROptimiser->keypoint_generator->ContactChangeDyn(iLQROptimiser->contact_list, activeModelTranslator->current_state_vector, true , false, false);

    //Print out the key points
    std::cout << "Keypoints: \n";
    for(int t = 0; t < opt_horizon; t++){
        if(!iLQROptimiser->keypoint_generator->keypoints[t].empty()){
            std::cout << "time " << t << ": ";
            for(int i = 0; i < iLQROptimiser->keypoint_generator->keypoints[t].size(); i++){
                std::cout << iLQROptimiser->keypoint_generator->keypoints[t][i] << " ";
            }
            std::cout << "\n";
        }
    }

    // Playback the trajectory
    for(int t = 0; t < opt_horizon; t++){
        // Copy the system state to the visualiser
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data,
                                                              activeModelTranslator->MuJoCo_helper->saved_systems_state_list[t]);
        // Forward the model
        mj_forward(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);
        // Render the visualiser
        activeVisualiser->render("Keypoint Method Test");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void ArticulatedContactSaveDerivs(){
    std::cout << "Articulated Contact Derivative Analysis" << std::endl;

    // Doesnt actually do anything for this program
    yamlReader = std::make_shared<FileHandler>();

    // Instantiate the model translator
    std::shared_ptr<ArticulatedContact> articulated_contact = std::make_shared<ArticulatedContact>();
    activeModelTranslator = articulated_contact;

    // Instantiate the differentiator
    activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);

    activeModelTranslator->MuJoCo_helper->AppendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
    //Instantiate the visualiser
    activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);

    // Setup the initial horizon, based on open loop or mpc method
    int opt_horizon = 2000;

    iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator,
                                           activeModelTranslator->MuJoCo_helper,
                                           activeDifferentiator,
                                           opt_horizon, activeVisualiser, yamlReader);

    iLQROptimiser->Resize(activeModelTranslator->current_state_vector.dof,
                          activeModelTranslator->current_state_vector.num_ctrl,
                          opt_horizon);

    // Initialise storage for A and B matrices
    std::vector<MatrixXd> A;
    std::vector<MatrixXd> B;

    int dof_model_translator = activeModelTranslator->current_state_vector.dof;
    int dim_action = activeModelTranslator->current_state_vector.num_ctrl;
    int dim_sensor = activeModelTranslator->MuJoCo_helper->model->nsensordata;
    int dim_state_derivative = dof_model_translator*2;

    A.push_back(MatrixXd(dim_state_derivative, dim_state_derivative));
    B.push_back(MatrixXd(dim_state_derivative, dim_action));

    std::vector<int> cols(dof_model_translator, 0);
    for (int i = 0; i < dof_model_translator; i++) {
        cols[i] = i;
    }

    vector<MatrixXd> init_controls = activeModelTranslator->CreateInitOptimisationControls(opt_horizon);

    MatrixXd state_vector = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->master_reset_data,
                                                                     activeModelTranslator->current_state_vector);

    std::cout << "model integrator: " << activeModelTranslator->MuJoCo_helper->model->opt.integrator << std::endl;
    if (activeModelTranslator->MuJoCo_helper->model->opt.integrator == mjINT_EULER) {
        std::cout << "Anitescu integrator is used, setting the state vector to zero." << std::endl;
        // Set the state vector to zero for Anitescu integrator
        state_vector.setZero();
    } else {
        std::cout << "Using default integrator, setting the state vector to non-zero values." << std::endl;
    }

    std::cout << "State vector: " << state_vector << std::endl;
    state_vector(0) = PI;  //qpos0
    state_vector(1) = 0.2;  //qpos1
    state_vector(2) = 0.0;  //qpos2
    state_vector(3) = 0.0;  //qvel0
    state_vector(4) = 0.0;  //qvel1
    state_vector(5) = 0.0;  //qvel2
    activeModelTranslator->SetStateVector(state_vector, activeModelTranslator->MuJoCo_helper->master_reset_data,
                                          activeModelTranslator->current_state_vector);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
                                                          activeModelTranslator->MuJoCo_helper->master_reset_data
    );

    // Test contact list generation
    iLQROptimiser->RolloutTrajectory(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], false, init_controls);
    for(int i = 0; i < opt_horizon; i++){
        std::cout << "Contact list at step " << i << ": ";
        for( const auto& contact_pair : iLQROptimiser->contact_list[i] ) {
            std::cout << "(" << contact_pair.first << ", " << contact_pair.second << ") ";
        }
        std::cout << "\n";
    }

    // Open the file
    std::string projectParentPath = __FILE__;
    projectParentPath = projectParentPath.substr(0, projectParentPath.find_last_of("/\\"));
    projectParentPath = projectParentPath.substr(0, projectParentPath.find_last_of("/\\"));
    std::string dir_name = projectParentPath + "/TestingData/" + activeModelTranslator->model_name;

    //Create directory if it does not exist
    if(!std::filesystem::exists(dir_name)) {
        std::filesystem::create_directories(dir_name);
    }

    // Loops iterate u[0], q[0], q[1], dotq[0], dotq[1]
    std::string test_name_suffixes[7] = {"u0", "q0", "q1", "q2", "dotq0", "dotq1", "dotq2"};

    for(int i = 0; i < 7; i++){
        // Reset system state
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
                                                              activeModelTranslator->MuJoCo_helper->master_reset_data);
        // File directory creation
        std::string filename = dir_name + "/" + test_name_suffixes[i] + ".csv";
        std::cout << "filename: " << filename << std::endl;
        ofstream file_output;
        file_output.open(filename);

        // Create the headers - dependant on outerloop iteration
        if(i == 0){
            file_output << "u0" << ",";
        }
        else if(i == 1){
            file_output << "q0" << ",";
        }
        else if(i == 2){
            file_output << "q1" << ",";
        }
        else if(i == 3){
            file_output << "q2" << ",";
        }
        else if(i == 4){
            file_output << "dotq0" << ",";
        }
        else if(i == 5){
            file_output << "dotq1" << ",";
        }
        else if(i == 6){
            file_output << "dotq2" << ",";
        }

        for(int j = 0; j < dof_model_translator*2; j++){
            for(int k = 0; k < dof_model_translator*2; k++){
                file_output << "A" << j << k << ",";
            }
        }
        for(int j = 0; j < dim_action; j++){
            for(int k = 0; k < dof_model_translator*2; k++){
                file_output << "B" << j << k << ",";
            }
        }
        file_output << endl;
        for(int j = 0; j < 100; j++){
            // Alter the state / control signal
            if(i == 0){
                activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]->ctrl[0] += 0.01; // Alter the first control signal
                file_output << activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]->ctrl[0] << ","; // Save the control variable
            }
            else{
                // Alter the state vector
                MatrixXd new_state_vector = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
                                                                                     activeModelTranslator->current_state_vector);
                new_state_vector(i-1) += 0.01; // Alter the i-th state variable
                activeModelTranslator->SetStateVector(new_state_vector, activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
                                                      activeModelTranslator->current_state_vector);
                file_output << new_state_vector(i-1) << ",";    // Save the control variable
            }

            // Compute the dynamics derivatives
            activeDifferentiator->DynamicsDerivatives(A[0], B[0], cols, 0, 0, false, 1e-6);

            // Write the A and B matrices to the file
            for(int k = 0; k < dof_model_translator*2; k++){
                for(int m = 0; m < dof_model_translator*2; m++){
                    file_output << A[0](k,m) << ",";
                }
            }
            for(int k = 0; k < dof_model_translator*2; k++){
                for(int m = 0; m < dim_action; m++){
                    file_output << B[0](k,m) << ",";
                }
            }
            file_output << endl;

            //Render and sleep
            activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]);
            mj_forward(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);
            activeVisualiser->render("");
//            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        file_output.close();
    }
}

void PistonBlockTest(){
    // This test I want to setup the piston block task - compute dynamics derivatives about the nominal trajectory
    // for SI1 case. Save those to a file. Generate keypoints for a contact change method and save to file.
    // As well as saving the raw values to files, I want to compute the error metrics, the same as I do in another
    // script and save that to a yaml file as well in same directory.

    std::cout << "Articulated Contact Derivative Analysis" << std::endl;

    // Doesnt actually do anything for this program
    yamlReader = std::make_shared<FileHandler>();

    // Instantiate the model translator
    std::shared_ptr<PistonBlock> piston_block = std::make_shared<PistonBlock>();
    activeModelTranslator = piston_block;

    // Instantiate the differentiator
    activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);

    activeModelTranslator->MuJoCo_helper->AppendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
    //Instantiate the visualiser
    activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);

    // Setup the initial horizon, based on open loop or mpc method
    int opt_horizon = 2000;

    iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator,
                                           activeModelTranslator->MuJoCo_helper,
                                           activeDifferentiator,
                                           opt_horizon, activeVisualiser, yamlReader);

    iLQROptimiser->Resize(activeModelTranslator->current_state_vector.dof,
                          activeModelTranslator->current_state_vector.num_ctrl,
                          opt_horizon);



    std::vector<MatrixXd> init_opt_controls;
    std::vector<MatrixXd> optimised_controls;

    // Load new task instance
    std::string task_prefix = activeModelTranslator->model_name;
    yamlReader->LoadTaskFromFile(task_prefix, 0, activeModelTranslator->full_state_vector,
                                 activeModelTranslator->residual_list);
    activeModelTranslator->full_state_vector.Update();
    activeModelTranslator->current_state_vector = activeModelTranslator->full_state_vector;
    activeModelTranslator->UpdateSceneVisualisation();

    activeModelTranslator->InitialiseSystemToStartState(activeModelTranslator->MuJoCo_helper->master_reset_data);

    std::vector<MatrixXd> init_setup_controls = activeModelTranslator->CreateInitSetupControls(1000);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);

    init_opt_controls = activeModelTranslator->CreateInitOptimisationControls(opt_horizon);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

    optimised_controls = init_opt_controls;

    // Rollout the initial controls of the trajectory to give a sequence of states to compute dynamics derivatives from
    iLQROptimiser->RolloutTrajectory(activeModelTranslator->MuJoCo_helper->master_reset_data, true, init_opt_controls);

    // Visualise the controls
//    for(int t = 0; t < opt_horizon; t++) {
//        // Copy the system state to the visualiser
//        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data,
//                                                              activeModelTranslator->MuJoCo_helper->saved_systems_state_list[t]);
//        // Forward the model
//        mj_forward(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);
//        // Render the visualiser
//        activeVisualiser->render("Piston Block Test");
//        std::this_thread::sleep_for(std::chrono::milliseconds(20));
//    }


    // Define key-point methods to test
    std::vector<MatrixXd> A_matrices_SI1, B_matrices_SI1;
    std::vector<std::string> methods = {"SI20", "contact_change", "contact_change_maxN"};
    std::vector<std::string> keypoint_methods = {"set_interval", "contact_change", "contact_change_maxN"};
    std::vector<int> min_n_values = {20, 1, 1};
    std::vector<int> max_n_values = {1, 1, 20};

    // Make sure all vectors are the same size
    assert(methods.size() == keypoint_methods.size());
    assert(methods.size() == min_n_values.size());
    assert(methods.size() == max_n_values.size());

    // Data storage
    std::vector<std::vector<double>> mean_squared_error,
            elementnorm_mse_errors,
            max_abs_error,
            percentage_derivatives;

    mean_squared_error.resize(methods.size());
    elementnorm_mse_errors.resize(methods.size());
    max_abs_error.resize(methods.size());

    // Create Vectors of MatrixXd to store A and B matrices for each method
    std::vector<std::vector<MatrixXd>> A_matrices(methods.size()), B_matrices(methods.size());

    // Create vectors to store the key-points per method
    std::vector<std::vector<std::vector<int>>> keypoints_per_method(methods.size());

    // Size matrices appropriately
    for(int i = 0; i < methods.size(); i++){
        A_matrices[i].resize(opt_horizon, MatrixXd::Zero(activeModelTranslator->current_state_vector.dof, activeModelTranslator->current_state_vector.dof));
        B_matrices[i].resize(opt_horizon, MatrixXd::Zero(activeModelTranslator->current_state_vector.dof, activeModelTranslator->current_state_vector.num_ctrl));
    }

    // First compute key-points for SI1
    keypoint_method method;
    method = iLQROptimiser->ReturnCurrentKeypointMethod();
    method.min_N = 1;
    method.name = "set_interval";
    iLQROptimiser->SetCurrentKeypointMethod(method);
    iLQROptimiser->GenerateDerivatives();

    // Save the data
    A_matrices_SI1 = iLQROptimiser->A;
    B_matrices_SI1 = iLQROptimiser->B;

    // Loop through key-point methods - compute dynamics derivatives for each method
    for(int i = 0; i < methods.size(); i++){
        method.min_N = min_n_values[i];
        method.max_N = max_n_values[i];
        method.name = keypoint_methods[i];
        iLQROptimiser->SetCurrentKeypointMethod(method);

        iLQROptimiser->GenerateDerivatives();

        // Save the data
        A_matrices[i] = iLQROptimiser->A;
        B_matrices[i] = iLQROptimiser->B;
        std::vector<std::vector<int>> kp = iLQROptimiser->keypoint_generator->keypoints;
        keypoints_per_method[i] = kp;
    }

    // Compute error metrics for all methods between approximated dynamics derivatives and accurate ones
    for(int i = 0; i < methods.size(); i++){
        double mse, elementnorm_mse, max_error;
        ApproximationError(A_matrices_SI1, B_matrices_SI1, A_matrices[i], B_matrices[i],
                           mse, elementnorm_mse, max_error);

        mean_squared_error[i].push_back(mse);
        elementnorm_mse_errors[i].push_back(elementnorm_mse);
        max_abs_error[i].push_back(max_error);
    }


    // Save all data to files - each method will be a directory - with a A_matrices.csv, B_matrices.csv, error_metrics.yaml files
    // --------------- Save the results to a file --------------------------
    // directory "DerivativeErrorData /{task_name}/{method_name}/

    std::string project_parent_path = __FILE__;
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));

    std::string task_name = activeModelTranslator->model_name;

    // Check folder exists, if it does not create one
    std::string folder_name = project_parent_path + "/DerivativeErrorData/" + task_name + + "/";
    if(!std::filesystem::exists(folder_name)){
        std::filesystem::create_directories(folder_name);
    }

    // Loop through all methods and save the data
    for(int i = 0; i < methods.size(); i++){
        std::string method_folder_name = folder_name + methods[i] + "/";
        if(!std::filesystem::exists(method_folder_name)){
            std::filesystem::create_directories(method_folder_name);
        }

        // Save A matrices
        std::string A_filename = method_folder_name + "A_matrices.csv";
        ofstream A_file_output;
        A_file_output.open(A_filename);
        for(int t = 0; t < opt_horizon; t++){
            for(int r = 0; r < A_matrices[i][t].rows(); r++){
                for(int c = 0; c < A_matrices[i][t].cols(); c++){
                    A_file_output << A_matrices[i][t](r,c);
                    if(c < A_matrices[i][t].cols() - 1){
                        A_file_output << ",";
                    }
                }
                A_file_output << "\n";
            }
        }
        A_file_output.close();

        // Save B matrices
        std::string B_filename = method_folder_name + "B_matrices.csv";
        ofstream B_file_output;
        B_file_output.open(B_filename);
        for(int t = 0; t < opt_horizon; t++){
            for(int r = 0; r < B_matrices[i][t].rows(); r++){
                for(int c = 0; c < B_matrices[i][t].cols(); c++){
                    B_file_output << B_matrices[i][t](r,c);
                    if(c < B_matrices[i][t].cols() - 1){
                        B_file_output << ",";
                    }
                }
                B_file_output << "\n";
            }
        }
        B_file_output.close();

        // Save error metrics to yaml file
        std::string error_filename = method_folder_name + "error_metrics.yaml";
        ofstream error_file_output;
        error_file_output.open(error_filename);
        error_file_output << "mean_squared_error: " << mean_squared_error[i][0] << "\n";
        error_file_output << "elementnorm_mse_error: " << elementnorm_mse_errors[i][0] << "\n";
        error_file_output << "max_absolute_error: " << max_abs_error[i][0] << "\n";
        error_file_output.close();

        // Save the keypoints to a file
        std::string keypoints_filename = method_folder_name + "keypoints.csv";
        ofstream keypoints_file_output;
        keypoints_file_output.open(keypoints_filename);
        for(int t = 0; t < opt_horizon; t++){
            if(!keypoints_per_method[i][t].empty()){
                for(int k = 0; k < keypoints_per_method[i][t].size(); k++){
                    keypoints_file_output << keypoints_per_method[i][t][k];
                    if(k < keypoints_per_method[i][t].size() - 1){
                        keypoints_file_output << ",";
                    }
                }
            }
            keypoints_file_output << "\n";
        }
    }

    // Save the SI1 data as well - no error metrics however
    std::string method_folder_name = folder_name + "SI1/";
    if(!std::filesystem::exists(method_folder_name)){
        std::filesystem::create_directories(method_folder_name);
    }
    // Save A matrices
    std::string A_filename = method_folder_name + "A_matrices.csv";
    ofstream A_file_output;
    A_file_output.open(A_filename);
    for(int t = 0; t < opt_horizon; t++){
        for(int r = 0; r < A_matrices_SI1[t].rows(); r++){
            for(int c = 0; c < A_matrices_SI1[t].cols(); c++){
                A_file_output << A_matrices_SI1[t](r,c);
                if(c < A_matrices_SI1[t].cols() - 1){
                    A_file_output << ",";
                }
            }
            A_file_output << "\n";
        }
    }
    A_file_output.close();
    // Save B matrices
    std::string B_filename = method_folder_name + "B_matrices.csv";
    ofstream B_file_output;
    B_file_output.open(B_filename);
    for(int t = 0; t < opt_horizon; t++){
        for(int r = 0; r < B_matrices_SI1[t].rows(); r++){
            for(int c = 0; c < B_matrices_SI1[t].cols(); c++){
                B_file_output << B_matrices_SI1[t](r,c);
                if(c < B_matrices_SI1[t].cols() - 1){
                    B_file_output << ",";
                }
            }
            B_file_output << "\n";
        }
    }

}

// Articulated Contact Script
int main(){

//    TestKeypointMethod();

    PistonBlockTest();

//    BoxSweepTest();

//    ArticulatedContactSaveDerivs();
    return 0;
}