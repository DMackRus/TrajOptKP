#include "StdInclude.h"
#include "FileHandler.h"
#include "Visualiser.h"
#include "MuJoCoHelper.h"

// --------------------- different scenes -----------------------
#include "ModelTranslator/PistonBlock.h"
#include "ModelTranslator/Acrobot.h"

#include "Optimiser/Optimiser.h"
#include "Optimiser/iLQR.h"

// --------------------- other -----------------------
#include <mutex>

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

int main(){
    std::cout << "Toy Contact Analysis Program" << std::endl;

    // Doesnt actually do anything for this program
    yamlReader = std::make_shared<FileHandler>();

    // Instantiate the model translator
    std::shared_ptr<Acrobot> acrobot = std::make_shared<Acrobot>();
    activeModelTranslator = acrobot;

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

    // Evaluate the parallelisation effectiveness of the dynamics derivatives computation
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

    // Initialise scene such that the piston is in contact with the block

    activeModelTranslator->InitialiseSystemToStartState(activeModelTranslator->MuJoCo_helper->master_reset_data);
    MatrixXd state_vector = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->master_reset_data,
                                                              activeModelTranslator->current_state_vector);
    state_vector(0) = 0.1;
    state_vector(1) = 0.3;
    state_vector(2) = 0.0;
    state_vector(3) = 0.0;
    activeModelTranslator->SetStateVector(state_vector, activeModelTranslator->MuJoCo_helper->master_reset_data,
                                          activeModelTranslator->current_state_vector);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
                                                          activeModelTranslator->MuJoCo_helper->master_reset_data
                                                          );

    //-------------------- Test 1: Alter control Signal and alter control signal and compute dynamics derivatives ------

    // Open the file
    std::string projectParentPath = __FILE__;
    projectParentPath = projectParentPath.substr(0, projectParentPath.find_last_of("/\\"));
    projectParentPath = projectParentPath.substr(0, projectParentPath.find_last_of("/\\"));
//    projectParentPath = projectParentPath.substr(0, projectParentPath.find_last_of("/\\"));
    std::string filename = projectParentPath + "/Gah.csv";
    std::cout << "filename: " << filename << std::endl;
    ofstream file_output;
    file_output.open(filename);

    // Create the headers
    for(int i = 0; i < dim_action; i++){
        file_output << "u" << i << ",";
    }
    for(int i = 0; i < dof_model_translator*2; i++){
        for(int j = 0; j < dof_model_translator*2; j++){
            file_output << "A" << i << j << ",";
        }
    }
    for(int i = 0; i < dim_action; i++){
        for(int j = 0; j < dof_model_translator*2; j++){
            file_output << "B" << i << j << ",";
        }
    }
    file_output << endl;

    std::vector<double> A_m;
    std::vector<double> B_m;
    std::vector<double> C;
    std::vector<double> D;

    std::cout << "dim state_derivative: " << dim_state_derivative << std::endl;
    std::cout << "dim action: " << dim_action << std::endl;
    std::cout << "dim sensor: " << dim_sensor << std::endl;

    A_m.resize(dim_state_derivative * dim_state_derivative * 1);
    B_m.resize(dim_state_derivative * dim_action * 1);
    C.resize(dim_sensor * dim_state_derivative * 1);
    D.resize(dim_sensor * dim_action * 1);

    for(int k = 0; k < 100; k++){
        // Alter the control signal u
//        activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]->ctrl[0] += 0.01; // Alter the first control signal
        activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]->qpos[1] += 0.01; // Alter the first control signal
        // Compute the dynamics derivatives
        activeDifferentiator->DynamicsDerivatives(A[0], B[0], cols, 0, 0, false, 1e-6);
        std::cout << "State vector: " << activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
                                                          activeModelTranslator->current_state_vector) << endl;

//        bool flg_centred = false;

//        mjd_transitionFD( activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], 1e-6, flg_centred,
//                DataAt(A_m, 0 * (dim_state_derivative * dim_state_derivative)),
//                DataAt(B_m, 0 * (dim_state_derivative * dim_action)),
//                nullptr,
//                nullptr);


        std::cout << "iteration " << k << " - control signal: " << activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]->ctrl[0] << endl;
        std::cout << "A: " << A[0] << endl;
//        std::cout << "A_m: ";
//        for(int i = 0; i < dim_state_derivative; i++){
//            for(int j = 0; j < dim_state_derivative; j++){
//                std::cout << A_m[i * dim_state_derivative + j] << ", ";
//            }
//            std::cout << endl;
//        }
        std::cout << "B: " << B[0] << endl;

        // Save the data
        file_output << activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]->ctrl[0] << ",";
        for(int i = 0; i < dof_model_translator*2; i++){
            for(int j = 0; j < dof_model_translator*2; j++){
                file_output << A[0](i,j) << ",";
            }
        }
        for(int i = 0; i < dof_model_translator*2; i++){
            for(int j = 0; j < dim_action; j++){
                file_output << B[0](i,j) << ",";
            }
        }
        file_output << endl;

        // Compute mass matrix
//        mj_forward(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]);
//        Eigen::MatrixXd mass_matrix = getMassMatrix(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]);
//
//        std::cout << "Mass Matrix: " << mass_matrix << endl;




        //Render and sleep
//        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0]);
//        mj_forward(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);
//        activeVisualiser->render("");

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    file_output.close();

    return 0;
}
