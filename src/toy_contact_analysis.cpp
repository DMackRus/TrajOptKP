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
    iLQROptimiser->keypoint_generator->ContactAwareKeyPoints(iLQROptimiser->X_old,iLQROptimiser->U_old,
                                                             iLQROptimiser->contact_list, activeModelTranslator->current_state_vector);

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


    // Test keypoint generation
    iLQROptimiser->keypoint_generator->ContactAwareKeyPoints(iLQROptimiser->X_old,iLQROptimiser->U_old,
                                                             iLQROptimiser->contact_list, activeModelTranslator->current_state_vector);

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

// Articulated Contact Script
int main(){

//    TestKeypointMethod();

    BoxSweepTest();

//    ArticulatedContactSaveDerivs();
    return 0;
}