#include "StdInclude.h"
#include "FileHandler.h"

// --------------------- different scenes -----------------------
#include "DoublePendulum.h"
#include "Acrobot.h"
#include "Reaching.h"
#include "TwoDPushing.h"
#include "ThreeDPushing.h"
#include "BoxFlick.h"
#include "Walker.h"
#include "Hopper.h"
#include "humanoid.h"
#include "BoxSweep.h"

#include "Visualiser.h"
#include "MuJoCoHelper.h"

// --------------------- different optimisers -----------------------
#include "iLQR.h"
#include "PredictiveSampling.h"
#include "GradDescent.h"

//----------------------- Testing methods ---------------------------
#include "Testing.h"

// --------------------- other -----------------------
#include <mutex>
#include <atomic>

// ------------ MODES OF OPERATION -------------------------------
#define ASYNC_MPC   true

// --------------------- Global class instances --------------------------------
std::shared_ptr<ModelTranslator> activeModelTranslator;
std::shared_ptr<Differentiator> activeDifferentiator;
std::shared_ptr<Optimiser> activeOptimiser;
std::shared_ptr<iLQR> iLQROptimiser;
std::shared_ptr<PredictiveSampling> stompOptimiser;
std::shared_ptr<GradDescent> gradDescentOptimiser;
std::shared_ptr<Visualiser> activeVisualiser;
std::shared_ptr<FileHandler> yamlReader;

std::string task;
bool mpcVisualise = true;
bool playback = true;
std::vector<std::string> testingMethods;

int assign_task();

void showInitControls();
void optimiseOnceandShow();
void MPCUntilComplete(double &trajecCost, double &avgHz, double &avgTimeGettingDerivs, double &avgPercentDerivs, double &avgTimeBP, double &avgTimeFP,
                      int MAX_TASK_TIME, int REPLAN_TIME, int OPT_HORIZON);

void generateTestScenes();

void generateTestingData_MPC();
int generateTestingData_MPCHorizons();
int generateTestingData_MPC_asynchronous();
void generateTestingData();
void generateFilteringData();

void genericTesting();

void async_MPC_testing();
void worker();

double avg_opt_time, avg_percent_derivs, avg_time_derivs, avg_time_bp, avg_time_fp;

bool stopMPC = false;

int main(int argc, char **argv) {

//    mjModel *temp_model;
//    mjData *temp_data;
//    char error[1000];
//    temp_model = mj_loadXML("/home/davidrussell/catkin_ws/src/TrajOptKP/mujoco_models/Franka_emika_scenes_V1/cylinder_pushing.xml", NULL, error, 1000);
//    if( !temp_model ) {
//        printf("%s\n", error);
//    }
//    temp_data = mj_makeData(temp_model);
//
//    for(int j = 0; j < 5; j++){
//        mj_step(temp_model, temp_data);
//    }
//
//    mj_deleteData(temp_data);
//    mj_deleteModel(temp_model);
//
//
//    return -1;

//    vector<robot> temp1;
//    vector<string> temp2;
//    MuJoCoHelper temp(temp1, temp2);
//    temp._mjdTransitionFD();

//    return -1;

    // Expected arguments
    // 1. Program name
    // 2. Task name
    if(argc < 2){
        std::cout << "No task name provided, exiting" << endl;
        return -1;
    }

    std::string configFileName = argv[1];
//    std::cout << "config file name: " << configFileName << endl;

    // Optional arguments
    // 3. key-point method (only used for generating testing data)
    // Only used for generating testing data for different key-point methods
    if(argc > 2){
        for (int i = 1; i < argc; i++) {
            testingMethods.push_back(argv[i]);
        }
    }

    std::string optimiser;
    std::string runMode;

    std::string taskInitMode;

    yamlReader = std::make_shared<FileHandler>();
    yamlReader->readSettingsFile("/generalConfigs/" + configFileName + ".yaml");
    optimiser = yamlReader->optimiser;
    runMode = yamlReader->project_run_mode;
    task = yamlReader->taskName;
    taskInitMode = yamlReader->taskInitMode;

    std::cout << "task: " << task << endl;

    MatrixXd startStateVector(1, 1);

    // Instantiate model translator as specified by the config file.
    if(assign_task() == EXIT_FAILURE){
        return EXIT_FAILURE;
    }

    if(runMode == "Generate_testing_data"){
//    	 return generateTestingData_MPCHorizons();
//         return generateTestingData_MPC_asynchronous();
        Testing myTestingObject(iLQROptimiser, activeModelTranslator,
                                activeDifferentiator, activeVisualiser, yamlReader);
        return myTestingObject.testing_different_velocity_change_asynchronus_mpc();

    }

    startStateVector.resize(activeModelTranslator->state_vector_size, 1);
    std::cout << "X start: " << activeModelTranslator->X_start << "\n";
    startStateVector = activeModelTranslator->X_start;

    // random start and goal state
    std::string taskPrefix = activeModelTranslator->model_name;
    if(taskInitMode == "random"){
        activeModelTranslator->GenerateRandomGoalAndStartState();
    }
    else if(taskInitMode == "fromCSV"){
        yamlReader->loadTaskFromFile(taskPrefix, yamlReader->csvRow, startStateVector, activeModelTranslator->X_desired);
        activeModelTranslator->X_start = startStateVector;
    }

    cout << "start state " << startStateVector << endl;
    cout << "desired state " << activeModelTranslator->X_desired.transpose() << endl;

    activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);
    activeModelTranslator->SetStateVector(startStateVector, activeModelTranslator->MuJoCo_helper->master_reset_data);
    for(int j = 0; j < 5; j++){
        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->master_reset_data);
    }
    activeModelTranslator->MuJoCo_helper->appendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);

    //Instantiate my visualiser
    std::cout << "before make visualiser \n";
    activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);

    // Choose an Optimiser
    if(optimiser == "interpolated_iLQR"){
        std::cout << "before make optimiser \n";
        iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator, activeModelTranslator->MuJoCo_helper, activeDifferentiator, yamlReader->maxHorizon, activeVisualiser, yamlReader);
        activeOptimiser = iLQROptimiser;
        std::cout << "after make optimiser \n";
    }
    else if(optimiser == "PredictiveSampling"){
        stompOptimiser = std::make_shared<PredictiveSampling>(activeModelTranslator, activeModelTranslator->MuJoCo_helper, yamlReader, activeDifferentiator, yamlReader->maxHorizon, 8);
        activeOptimiser = stompOptimiser;
    }
    else if(optimiser == "GradDescent"){
        gradDescentOptimiser = std::make_shared<GradDescent>(activeModelTranslator, activeModelTranslator->MuJoCo_helper, activeDifferentiator, activeVisualiser, yamlReader->maxHorizon, yamlReader);
        activeOptimiser = gradDescentOptimiser;
    }
    else{
        cout << "invalid Optimiser selected, exiting" << endl;
        return -1;
    }

    // Methods of control / visualisation
    if(runMode == "Init_controls"){
        cout << "SHOWING INIT CONTROLS MODE \n";
        showInitControls();
    }
    else if(runMode == "Optimise_once"){
        cout << "OPTIMISE TRAJECTORY ONCE AND DISPLAY MODE \n";
        activeOptimiser->verbose_output = true;
        optimiseOnceandShow();
    }
    else if(runMode == "MPC_until_completion"){
        cout << "MPC UNTIL TASK COMPLETE MODE \n";
        async_MPC_testing();
    }
    else if(runMode == "Generate_test_scenes"){
        cout << "TASK INIT MODE \n";
        generateTestScenes();
    }
    else if(runMode == "GENERATE_FILTERING_DATA"){
        cout << "GENERATE FILTERING DATA MODE \n";
        generateFilteringData();
    }
    else{
        cout << "INVALID MODE OF OPERATION OF PROGRAM \n";

        // Test ability to convert state vector to q pos index.
        std::cout << "indices: ";
        for(int i = 0; i < activeModelTranslator->state_vector_names.size(); i++){
            int index = activeModelTranslator->StateIndexToQposIndex(i);
            std::cout << index << " ";
        }
        std::cout << "\n";

        // Compare my fd code versus mjd_transitionFD;

        // Start with mjd_transitionFD

        // - Allocate A, B, C and D matrices.
        int dim_state_derivative = activeModelTranslator->MuJoCo_helper->model->nv * 2;
        int dof = dim_state_derivative / 2;
        int dim_action = activeModelTranslator->MuJoCo_helper->model->nu;
        int dim_sensor = activeModelTranslator->MuJoCo_helper->model->nsensordata;
        int T = 200;
        std::vector<double> A;
        std::vector<double> B;
        std::vector<double> C;
        std::vector<double> D;

        std::cout << "dim state derivative: " << dim_state_derivative << "\n";
        std::cout << "dim action: " << dim_action << "\n";
        std::cout << "dim sensor: " << dim_sensor << "\n";

        A.resize(dim_state_derivative * dim_state_derivative * T);
        B.resize(dim_state_derivative * dim_action * T);
        C.resize(dim_sensor * dim_state_derivative * T);
        D.resize(dim_sensor * dim_action * T);

        int t = 0;

        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
//        MatrixXd control_vector = MatrixXd::Zero(dim_action, 1);
//        control_vector << -1, -1, -1, -1, -1, -1;
//        activeModelTranslator->SetControlVector(control_vector, activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0]);

        std::cout << "start of mjd_transitionFD \n";
        auto start = std::chrono::high_resolution_clock::now();
        for(int i = 0; i < T; i++){
            mjd_transitionFD(
                    activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], 1e-6, 1,
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

        int dof_model_translator = activeModelTranslator->dof;

        A_mine.push_back(MatrixXd(dof_model_translator*2, dof_model_translator*2));
        B_mine.push_back(MatrixXd(dof_model_translator*2, dim_action));

        A_mine[0].block(0, 0, dof_model_translator, dof_model_translator).setIdentity();
        A_mine[0].block(0, dof_model_translator, dof_model_translator, dof_model_translator).setIdentity();
        A_mine[0].block(0, dof_model_translator, dof_model_translator, dof_model_translator) *= activeModelTranslator->MuJoCo_helper->returnModelTimeStep();
        B_mine[0].setZero();

        std::vector<int> cols(dof_model_translator, 0);
        for (int i = 0; i < dof_model_translator; i++) {
            cols[i] = i;
        }

        MatrixXd l_x, l_xx, l_u, l_uu;
        double time = 0.0f;
        start = std::chrono::high_resolution_clock::now();
        for(int i = 0; i < T; i++){
            activeDifferentiator->ComputeDerivatives(A_mine[0], B_mine[0], cols, l_x, l_xx, l_u, l_uu, 0, 0, false, false, true, 1e-6);
            time += activeDifferentiator->time_mj_forwards;
        }
        std::cout << "time of mj_forwards calls " << (time / 1000.0f) << "ms\n";
        std::cout << "time taken for my code " << (std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - start).count()) / 1000.0f << "ms\n";


        if(dim_state_derivative != dof_model_translator*2){
            // Print theirs and print mine
            std::cout << "A from mjd_transition \n";
            for(int i = 0; i < dim_state_derivative; i++){
                for(int j = 0; j < dim_state_derivative; j++){
                    std::cout << setw(6) << A[i * dim_state_derivative + j] << " ";
                }
                std::cout << std::endl;
            }

            std::cout << "A_mine[0] \n";
            std::cout << A_mine[0] << std::endl;

            // Print their B matrix
            std::cout << "B from mjd_transition \n";
            for(int i = 0; i < dim_state_derivative; i++){
                for(int j = 0; j < dim_action; j++){
                    std::cout << setw(6) << B[i * dim_action + j] << " ";
                }
                std::cout << std::endl;
            }

            // Print our B matrix
            std::cout << "B_mine[0] \n";
            std::cout << B_mine[0] << std::endl;


        }
        else{
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
        }

        return EXIT_FAILURE;
    }
    std::cout << "before program exit \n";

    return EXIT_SUCCESS;
}

void onetaskGenerateTestingData(){
    int setupHorizon = 1000;
    int optHorizon = 2800;

    MatrixXd startStateVector;
    startStateVector.resize(activeModelTranslator->state_vector_size, 1);

    std::vector<std::vector<double>> optTimes;
    std::vector<double> optTimesRow;

    std::vector<std::vector<double>> costReductions;
    std::vector<double> costReductionsRow;

    std::vector<std::vector<double>> avgPercentageDerivs;
    std::vector<double> avgPercentageDerivsRow;

    std::vector<std::vector<double>> avgTimeForDerivs;
    std::vector<double> avgTimeForDerivsRow;

    std::vector<std::vector<int>> numIterations;
    std::vector<int> numIterationsRow;

    // Object manipulation?
//    std::vector<std::string> methodNames = {"baseline", "SI5", "SI1000", "adaptive_jerk_5", "iterative_error_5", "magvel_change_5"};
//    int numMethods = methodNames.size();
//    std::vector<string> keyPointMethods = {"setInterval", "setInterval", "setInterval", "adaptive_jerk", "iterative_error", "magvel_change"};
//    std::vector<int> min_N = {1, 5, 1000, 5, 5, 5};
//    std::vector<int> max_N = {1, 5, 1000, 100, 100, 100};

    std::vector<std::string> methodNames = {"baseline", "SI5", "SI1000", "adaptive_jerk_5", "iterative_error_5", "magvel_change_5"};
    int numMethods = methodNames.size();
    std::vector<string> keyPointMethods = {"setInterval", "setInterval", "setInterval", "adaptive_jerk", "iterative_error", "magvel_change"};
    std::vector<int> minN = {1, 5, 1000, 2, 2, 2};
    std::vector<int> maxN = {1, 5, 1000, 10, 10, 10};

//    std::vector<std::string> methodNames = {"iterative_error"};
//    int numMethods = methodNames.size();
//    std::vector<std::string> keyPointMethods = {"iterative_error"};
//    std::vector<int> min_N = {5};
//    std::vector<int> max_N = {20};

    // Loop through saved trajectories
    for(int i = 0; i < 100; i++){
        cout << "------------------------------------ Trajec " << i << " ------------------------------------\n";
        cout << "dof: " << activeModelTranslator->dof << endl;

        // Loop through our interpolating derivatives methods
        optTimesRow.clear();
        costReductionsRow.clear();
        avgPercentageDerivsRow.clear();
        avgTimeForDerivsRow.clear();
        numIterationsRow.clear();

        yamlReader->loadTaskFromFile(activeModelTranslator->model_name, i, startStateVector, activeModelTranslator->X_desired);
        activeModelTranslator->X_start = startStateVector;
        activeModelTranslator->SetStateVector(startStateVector, activeModelTranslator->MuJoCo_helper->master_reset_data);
        for(int j = 0; j < 5; j++){
            mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->master_reset_data);
        }


        if(activeModelTranslator->MuJoCo_helper->checkIfDataIndexExists(0) == false){
            activeModelTranslator->MuJoCo_helper->appendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
        }

        // Move the end-effector to a decent starting position
        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
        std::vector<MatrixXd> setupControls = activeModelTranslator->CreateInitSetupControls(1000);
        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);

        std::vector<MatrixXd> initOptimisationControls = activeModelTranslator->CreateInitOptimisationControls(optHorizon);

        for(int j = 0; j < numMethods; j++){
            double optTime;
            double costReduction;
            double avgPercentageDerivs;
            double avgTimeForDerivs;
            int numIterationsForConvergence;

            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], activeModelTranslator->MuJoCo_helper->master_reset_data);

            // Setup interpolation method
            keypoint_method currentInterpolator = activeOptimiser->ReturnCurrentKeypointMethod();
            currentInterpolator.name = keyPointMethods[j];
            currentInterpolator.min_N = minN[j];
            currentInterpolator.max_N = maxN[j];
            activeOptimiser->SetCurrentKeypointMethod(currentInterpolator);

            std::vector<MatrixXd> optimisedControls = activeOptimiser->Optimise(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], initOptimisationControls, 8, 2, optHorizon);

//            yamlReader->save_trajec_information(activeOptimiser->A, activeOptimiser->B, activeOptimiser->X_old, activeOptimiser->U_old, activeModelTranslator->model_name, i, optHorizon);

            // Return testing data and append appropriately
            activeOptimiser->ReturnOptimisationData(optTime, costReduction, avgPercentageDerivs, avgTimeForDerivs, numIterationsForConvergence);
            optTimesRow.push_back(optTime);
            costReductionsRow.push_back(costReduction);
            avgPercentageDerivsRow.push_back(avgPercentageDerivs);
            avgTimeForDerivsRow.push_back(avgTimeForDerivs);
            numIterationsRow.push_back(numIterationsForConvergence);

//            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, 0);

//            int controlCounter = 0;
//            int visualCounter = 0;
//            cout << "final controls size: " << optimisedControls.size() << endl;
//
//            while(controlCounter < initOptimisationControls.size()){
//
//                activeModelTranslator->SetControlVector(initOptimisationControls[controlCounter], activeModelTranslator->MuJoCo_helper->main_data);
//
//                activeModelTranslator->MuJoCo_helper->stepSimulator(1, activeModelTranslator->MuJoCo_helper->main_data);
//
//                controlCounter++;
//                visualCounter++;
//
//                if(visualCounter > 5){
//
//                    activeVisualiser->render("show init controls");
//                    visualCounter = 0;
//                }
//            }

        }
        optTimes.push_back(optTimesRow);
        costReductions.push_back(costReductionsRow);
        avgPercentageDerivs.push_back(avgPercentageDerivsRow);
        avgTimeForDerivs.push_back(avgTimeForDerivsRow);
        numIterations.push_back(numIterationsRow);

        cout << "average percent derivs: " << avgPercentageDerivsRow[0] << endl;
        cout << "row " << i << " done \n";
    }

    // Save data to file
    cout << "save data to file \n";
    yamlReader->saveResultsDataForMethods(activeModelTranslator->model_name, methodNames, optTimes, costReductions, avgPercentageDerivs, avgTimeForDerivs, numIterations);
}

void generateTestingData(){
    activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);

    MatrixXd startStateVector;
    startStateVector.resize(activeModelTranslator->state_vector_size, 1);
    startStateVector = activeModelTranslator->X_start;
    activeModelTranslator->SetStateVector(startStateVector, activeModelTranslator->MuJoCo_helper->master_reset_data);
    mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->main_data);
    activeModelTranslator->MuJoCo_helper->appendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->main_data);

    activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);
    iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator, activeModelTranslator->MuJoCo_helper, activeDifferentiator, yamlReader->maxHorizon, activeVisualiser, yamlReader);
    activeOptimiser = iLQROptimiser;

    onetaskGenerateTestingData();
}

void generateFilteringData(){
    int setupHorizon = 1000;
    int optHorizon = 2200;
    int numTests = 100;

    MatrixXd startStateVector;
    startStateVector.resize(activeModelTranslator->state_vector_size, 1);

    std::vector<double> lowPassTests = {0.05, 0.1, 0.15, 0.2, 0.25, 0.3};
    std::vector<std::vector<double>> FIRTests;
    FIRTests.push_back({0.25, 0.5, 0.25});
    FIRTests.push_back({0.1, 0.15, 0.5, 0.15, 0.1});
    FIRTests.push_back({0.05, 0.1, 0.15, 0.3, 0.15, 0.1, 0.05});
    FIRTests.push_back({0.05, 0.05, 0.15, 0.2, 0.2, 0.2, 0.15, 0.05, 0.05});
    FIRTests.push_back({0.05, 0.1, 0.15, 0.2, 0.2, 0.15, 0.1, 0.05});

    keypoint_method currentInterpolator = activeOptimiser->ReturnCurrentKeypointMethod();
    currentInterpolator.name = "setInterval";
    currentInterpolator.min_N = 1;
    activeOptimiser->SetCurrentKeypointMethod(currentInterpolator);

    for (int i = 0; i < numTests; i++) {
        yamlReader->loadTaskFromFile(activeModelTranslator->model_name, i, startStateVector,
                                     activeModelTranslator->X_desired);
        activeModelTranslator->X_start = startStateVector;
        activeModelTranslator->SetStateVector(startStateVector, activeModelTranslator->MuJoCo_helper->master_reset_data);
        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->master_reset_data);

        std::vector<MatrixXd> initSetupControls = activeModelTranslator->CreateInitSetupControls(setupHorizon);
        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);

        std::vector<MatrixXd> initOptimisationControls = activeModelTranslator->CreateInitOptimisationControls(optHorizon);

        // Initialise task for optimisation by here

        // ---------------- unfiltered tests ------------------------------
        activeOptimiser->filteringMethod = "none";
        // Load a task from saved tasks

        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], activeModelTranslator->MuJoCo_helper->master_reset_data);

        std::vector<MatrixXd> optimisedControls = activeOptimiser->Optimise(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0],
                                                                            initOptimisationControls,
                                                                            yamlReader->maxIter,
                                                                            yamlReader->minIter,
                                                                            optHorizon);
        // Save cost history to file
        std::string filePrefix;
        filePrefix = activeModelTranslator->model_name + "/none/";

        yamlReader->saveCostHistory(activeOptimiser->costHistory, filePrefix, i);

        // ---------------------- Low pass filter tests ----------------------

        for(int j = 0; j < lowPassTests.size(); j++){
            activeOptimiser->filteringMethod = "low_pass";
            activeOptimiser->lowPassACoefficient = lowPassTests[j];
            // Load a task from saved tasks

            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], activeModelTranslator->MuJoCo_helper->master_reset_data);

            std::vector<MatrixXd> optimisedControls = activeOptimiser->Optimise(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0],
                                                                                initOptimisationControls,
                                                                                yamlReader->maxIter,
                                                                                yamlReader->minIter,
                                                                                optHorizon);
            // Save cost history to file
            std::string filePrefix;

            filePrefix = activeModelTranslator->model_name + "/lowPass" + std::to_string(lowPassTests[j]) + "/";

            yamlReader->saveCostHistory(activeOptimiser->costHistory, filePrefix, i);
        }

        // ---------------------- FIR filter tests ----------------------
        for(int j = 0; j < FIRTests.size(); j++){
            activeOptimiser->filteringMethod = "FIR";
            activeOptimiser->setFIRFilter(FIRTests[j]);
            // Load a task from saved tasks

            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], activeModelTranslator->MuJoCo_helper->master_reset_data);

            std::vector<MatrixXd> optimisedControls = activeOptimiser->Optimise(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0],
                                                                                initOptimisationControls,
                                                                                yamlReader->maxIter,
                                                                                yamlReader->minIter,
                                                                                optHorizon);
            // Save cost history to file
            std::string filePrefix;

            filePrefix = activeModelTranslator->model_name + "/FIR_" + std::to_string(j) + "/";

            yamlReader->saveCostHistory(activeOptimiser->costHistory, filePrefix, i);
        }


    }
}

void generateTestScenes(){
    for(int i = 0; i < 200; i++){
        activeModelTranslator->GenerateRandomGoalAndStartState();
        activeModelTranslator->SetStateVector(activeModelTranslator->X_start, activeModelTranslator->MuJoCo_helper->main_data);
        activeVisualiser->render("init state");
        std::cout << "X_desired: " << activeModelTranslator->X_desired << std::endl;
        yamlReader->saveTaskToFile(activeModelTranslator->model_name, i, activeModelTranslator->X_start, activeModelTranslator->X_desired);
    }
}

void showInitControls(){
    int setupHorizon = 1000;
    int optHorizon = 2500;
    int controlCounter = 0;
    int visualCounter = 0;

    std::vector<MatrixXd> initControls;

//    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
    std::vector<MatrixXd> initSetupControls = activeModelTranslator->CreateInitSetupControls(setupHorizon);
    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);
    std::vector<MatrixXd> initOptimisationControls = activeModelTranslator->CreateInitOptimisationControls(optHorizon);
    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

    //Stitch setup and optimisation controls together
//    initControls.insert(initControls.end(), initSetupControls.begin(), initSetupControls.end());
    initControls.insert(initControls.end(), initOptimisationControls.begin(), initOptimisationControls.end());

    while(activeVisualiser->windowOpen()){

        activeModelTranslator->SetControlVector(initControls[controlCounter], activeModelTranslator->MuJoCo_helper->main_data);
        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->main_data);


        controlCounter++;
        visualCounter++;

        if(controlCounter == setupHorizon){
            int a = 1;

        }

        if(controlCounter == setupHorizon + optHorizon/2){
            int a = 1;

        }

        if(controlCounter == setupHorizon + optHorizon){
            int a = 1;

        }

        if(controlCounter >= initControls.size()){
            controlCounter = 0;
            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
        }

        if(visualCounter > 5){
            visualCounter = 0;
            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->main_data);
            activeModelTranslator->MuJoCo_helper->forwardSimulator(activeModelTranslator->MuJoCo_helper->vis_data);
            activeVisualiser->render("show init controls");
        }
    }
}

void optimiseOnceandShow(){
//    int setupHorizon = 1000;
    int optHorizon = 3000;
    int controlCounter = 0;
    int visualCounter = 0;
    bool showFinalControls = true;
    const char* label = "Final trajectory after optimisation";
//    char label[50] = "Final trajectory after optimisation";
//    const char* label_init = "Initial trajectory";
//    const char* label_final = "Final trajectory";

    std::vector<MatrixXd> initControls;
    std::vector<MatrixXd> finalControls;

//    activeModelTranslator->MuJoCo_helper->copySystemState(0, activeModelTranslator->MuJoCo_helper->main_data);
//    MatrixXd test = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->main_data);
//    cout << "test: " << test << endl;

    std::vector<MatrixXd> initSetupControls = activeModelTranslator->CreateInitSetupControls(1000);
    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);

    std::vector<MatrixXd> initOptimisationControls = activeModelTranslator->CreateInitOptimisationControls(optHorizon);
    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

    auto start = high_resolution_clock::now();
    std::vector<MatrixXd> optimisedControls = activeOptimiser->Optimise(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], initOptimisationControls, yamlReader->maxIter, yamlReader->minIter, optHorizon);
    auto stop = high_resolution_clock::now();
    auto linDuration = duration_cast<microseconds>(stop - start);

    // Stitch together setup controls with init control + optimised controls
    initControls.insert(initControls.end(), initOptimisationControls.begin(), initOptimisationControls.end());
    finalControls.insert(finalControls.end(), optimisedControls.begin(), optimisedControls.end());

    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

    while(activeVisualiser->windowOpen()){

        if(showFinalControls){
            activeModelTranslator->SetControlVector(finalControls[controlCounter], activeModelTranslator->MuJoCo_helper->main_data);
        }
        else{
            activeModelTranslator->SetControlVector(initControls[controlCounter], activeModelTranslator->MuJoCo_helper->main_data);
        }

        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->main_data);

        controlCounter++;
        visualCounter++;

        if(controlCounter >= finalControls.size()){
            controlCounter = 0;
            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
            showFinalControls = !showFinalControls;
            if(showFinalControls){
                label = "Final trajectory after optimisation";
            }
            else{
                label = "Initial trajectory before optimisation";
            }
        }

        if(visualCounter >= 5){
            visualCounter = 0;
            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->main_data);
            activeModelTranslator->MuJoCo_helper->forwardSimulator(activeModelTranslator->MuJoCo_helper->vis_data);
            activeVisualiser->render(label);
        }
    }
}

void worker(){
    double trajecCost, avgHz, avgPercentDerivs, avgTimeDerivs, avgTimeBP, avgTimeFP;
    MPCUntilComplete(trajecCost, avgHz, avgPercentDerivs, avgTimeDerivs, avgTimeBP, avgTimeFP, 3000, 1, 150);
}

void async_MPC_testing(){

    std::vector<MatrixXd> initSetupControls = activeModelTranslator->CreateInitSetupControls(1000);
    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);

    // Whether Optimiser will output useful information
    activeOptimiser->verbose_output = true;
    // Visualise MPC trajectory live
    mpcVisualise = true;

    if(ASYNC_MPC){
        std::thread MPC_controls_thread;
        MPC_controls_thread = std::thread(&worker);
        int vis_counter = 0;
        MatrixXd next_control;
        // timer variables
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point end;

        int MAX_TASK_TIME = 2500;
        int task_time = 0;

        while(task_time++ < MAX_TASK_TIME){
            begin = std::chrono::steady_clock::now();

            if(activeVisualiser->current_control_index < activeVisualiser->controlBuffer.size()){

                next_control = activeVisualiser->controlBuffer[activeVisualiser->current_control_index];
                // Increment the current control index
                activeVisualiser->current_control_index++;
            }
            else{
                MatrixXd empty_control(activeModelTranslator->num_ctrl, 1);
                empty_control.setZero();
                next_control = empty_control;
            }

            // Store latest control and state in a replay buffer
            activeVisualiser->trajectory_controls.push_back(next_control);
            activeVisualiser->trajectory_states.push_back(activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->vis_data));

            // Set the latest control
            activeModelTranslator->SetControlVector(next_control, activeModelTranslator->MuJoCo_helper->vis_data);

            // Update the simulation
            mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);

            // Check if task complete
            double dist;
            if(activeModelTranslator->TaskComplete(activeModelTranslator->MuJoCo_helper->vis_data, dist)){
                cout << "task complete - dist: " << dist  << endl;
                break;
            }

            // Update the visualisation
            // Unsure why rendering every time causes it to lag so much more???
            vis_counter++;
            if(vis_counter > 5){
                activeVisualiser->render("live-MPC");
                vis_counter = 0;
            }

            end = std::chrono::steady_clock::now();
            // time taken
            auto time_taken = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

            // compare how long we took versus the timestep of the model
            int difference_ms = (activeModelTranslator->MuJoCo_helper->returnModelTimeStep() * 1000) - (time_taken / 1000.0f) + 1;

            if(difference_ms > 0) {
//                difference_ms = 20;
                std::this_thread::sleep_for(std::chrono::milliseconds(difference_ms));
            }
        }

        std::mutex mtx;
        mtx.lock();
        stopMPC = true;
        mtx.unlock();
        MPC_controls_thread.join();

        double cost = 0.0f;
        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
        for(int i = 0; i < activeVisualiser->trajectory_states.size(); i++){
            activeModelTranslator->SetControlVector(activeVisualiser->trajectory_controls[i], activeModelTranslator->MuJoCo_helper->vis_data);
            activeModelTranslator->SetStateVector(activeVisualiser->trajectory_states[i], activeModelTranslator->MuJoCo_helper->vis_data);
            activeModelTranslator->MuJoCo_helper->forwardSimulator(activeModelTranslator->MuJoCo_helper->vis_data);
//            activeModelTranslator->MuJoCo_helper->stepSimulator(1, activeModelTranslator->MuJoCo_helper->vis_data);
            cost += (activeModelTranslator->CostFunction(activeModelTranslator->MuJoCo_helper->vis_data, false) * activeModelTranslator->MuJoCo_helper->returnModelTimeStep());

//            activeVisualiser->render("live-MPC");

        }

        std::cout << "final cost of entire MPC trajectory was: " << cost << "\n";
        std::cout << "avg opt time: " << avg_opt_time << " ms \n";
        std::cout << "avg percent derivs: " << avg_percent_derivs << " % \n";
        std::cout << "avg time derivs: " << avg_time_derivs << " ms \n";
        std::cout << "avg time BP: " << avg_time_bp << " ms \n";
        std::cout << "avg time FP: " << avg_time_fp << " ms \n";

    }
    else{
        double trajecCost, avgHz;
        double avgPercentDerivs, avgTimeDerivs, avgTimeBP, avgTimeFP;
        MPCUntilComplete(trajecCost, avgHz, avgPercentDerivs, avgTimeDerivs, avgTimeBP, avgTimeFP, 2500, 1, 100);
    }
}

// Before calling this function, we should setup the activeModelTranslator with the correct initial state and the
// Optimiser settings. This function can then return necessary testing data for us to store
void MPCUntilComplete(double &trajecCost, double &avgHZ, double &avgTimeGettingDerivs, double &avgPercentDerivs, double &avgTimeBP, double &avgTimeFP,
                      int MAX_TASK_TIME, int REPLAN_TIME, int OPT_HORIZON){
    bool taskComplete = false;
    int visualCounter = 0;
    int overallTaskCounter = 0;
    int reInitialiseCounter = 0;
    const char* label = "MPC until complete";

    activeVisualiser->replayControls.clear();

    std::vector<double> timeGettingDerivs;
    std::vector<double> timeBackwardsPass;
    std::vector<double> timeForwardsPass;
    std::vector<double> percentagesDerivsCalculated;

    std::vector<MatrixXd> optimisedControls;

    // Instantiate init controls
    std::vector<MatrixXd> initOptimisationControls;

    int horizon = OPT_HORIZON;

    initOptimisationControls = activeModelTranslator->CreateInitOptimisationControls(horizon);
    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

    optimisedControls = activeOptimiser->Optimise(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], initOptimisationControls, 1, 1, OPT_HORIZON);

    MatrixXd currState;
    activeOptimiser->verbose_output = true;

    while(!taskComplete){

        if(stopMPC){
            taskComplete = true;
            break;
        }

        if(!ASYNC_MPC){
//            currState = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->main_data);

            MatrixXd nextControl = optimisedControls[0].replicate(1, 1);
            activeVisualiser->replayControls.push_back(nextControl.replicate(1, 1));

            optimisedControls.erase(optimisedControls.begin());

            optimisedControls.push_back(optimisedControls.at(optimisedControls.size() - 1));

            activeModelTranslator->SetControlVector(nextControl, activeModelTranslator->MuJoCo_helper->vis_data);
            mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);
        }

        reInitialiseCounter++;
        visualCounter++;

        // Re-Optimise evert REPLAN_TIME steps
        if(reInitialiseCounter >= REPLAN_TIME){
            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->vis_data);
            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], activeModelTranslator->MuJoCo_helper->main_data);

            if(ASYNC_MPC){
                int current_control_index = activeVisualiser->current_control_index;

                // Slice the optimised controls to get the current control
                for(int i = 0; i < current_control_index; i++){
                    optimisedControls.erase(optimisedControls.begin());
                    optimisedControls.push_back(optimisedControls.at(optimisedControls.size() - 1));
                }
            }
            optimisedControls = activeOptimiser->Optimise(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], optimisedControls, 1, 1, OPT_HORIZON);
            reInitialiseCounter = 0;

            timeGettingDerivs.push_back(activeOptimiser->avg_time_get_derivs_ms);
            timeBackwardsPass.push_back(activeOptimiser->avg_time_backwards_pass_ms);
            timeForwardsPass.push_back(activeOptimiser->avg_time_forwards_pass_ms);
            percentagesDerivsCalculated.push_back(activeOptimiser->avg_percent_derivs);

        }

        if(!ASYNC_MPC){
            if(mpcVisualise){
                if(visualCounter > 10){
//                    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->main_data);
//                    activeModelTranslator->MuJoCo_helper->forwardSimulator(activeModelTranslator->MuJoCo_helper->vis_data);
                    activeVisualiser->render(label);
                    visualCounter = 0;
                }
            }
        }
        else{
            // If we are use Async visualisation, need to copy our control vector to internal control vector for
            // visualisation class
            std::mutex mtx;
            mtx.lock();

            int optTimeToTimeSteps = activeOptimiser->opt_time_ms / (activeModelTranslator->MuJoCo_helper->returnModelTimeStep() * 1000);

            int low_bound = optTimeToTimeSteps - 3;
            if (low_bound < 0) low_bound = 0;

            int high_bound = optTimeToTimeSteps + 3;

            // By the time we have computed optimal controls, main visualisation will be some number
            // of time-steps ahead. We need to find the correct control to apply.

            MatrixXd current_vis_state = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->vis_data);

            double smallestError = 1000.00;
            int bestMatchingStateIndex = 0;
            // TODO - possible issue if optimisation time > horizon
            for(int i = low_bound; i < high_bound; i++){
//                std::cout << "i: " << i << " state: " << activeOptimiser->X_old[i].transpose() << std::endl;
//                std::cout << "correct state: " << current_vis_state.transpose() << std::endl;
                double currError = 0.0f;
                for(int j = 0; j < activeModelTranslator->state_vector_size; j++){
                    currError += abs(activeOptimiser->X_old[i](j) - current_vis_state(j));
                }

                if(currError < smallestError){
                    smallestError = currError;
                    bestMatchingStateIndex = i;
                }
            }

            activeVisualiser->controlBuffer = optimisedControls;

            // Set the current control index to the best matching state index
            activeVisualiser->current_control_index = bestMatchingStateIndex;
            std::cout << "best matching state index: " << bestMatchingStateIndex << std::endl;

            activeVisualiser->new_controls_flag = true;

            mtx.unlock();
        }

        if(!ASYNC_MPC){
            overallTaskCounter++;

            if(overallTaskCounter >= MAX_TASK_TIME){
                cout << "task time out" << endl;
                taskComplete = true;
            }
        }
    }

    if(!ASYNC_MPC){
        trajecCost = 0.0f;
        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

        for(int i = 0; i < activeVisualiser->replayControls.size(); i++){
            MatrixXd startState = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->vis_data);
            MatrixXd nextControl = activeVisualiser->replayControls[i].replicate(1, 1);
            double stateCost = activeModelTranslator->CostFunction(activeModelTranslator->MuJoCo_helper->vis_data, false);
            trajecCost += stateCost  * activeModelTranslator->MuJoCo_helper->returnModelTimeStep();

            activeModelTranslator->SetControlVector(nextControl, activeModelTranslator->MuJoCo_helper->vis_data);
            mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);
        }
    }


    //cout << "trajec cost: " << trajecCost << endl;
    avgTimeGettingDerivs = 0.0f;
    avgTimeBP = 0.0f;
    avgTimeFP = 0.0f;
    avgPercentDerivs = 0.0f;

    for(int i = 0; i < timeGettingDerivs.size(); i++){
        avgTimeGettingDerivs += timeGettingDerivs[i];
        avgTimeBP += timeBackwardsPass[i];
        avgTimeFP += timeForwardsPass[i];
        avgPercentDerivs += percentagesDerivsCalculated[i];
    }

    avgTimeGettingDerivs /= timeGettingDerivs.size();
    avgTimeBP /= timeBackwardsPass.size();
    avgTimeFP /= timeForwardsPass.size();
    avgPercentDerivs /= percentagesDerivsCalculated.size();

    avgHZ = 1000.0f / (avgTimeGettingDerivs + avgTimeBP + avgTimeFP);

    cout << "| Avg percentage of derivatives calculated: " << avgPercentDerivs << "\n";
    cout << "| avg time derivs: " << avgTimeGettingDerivs << " bp: " << avgTimeBP << " fp: " << avgTimeFP << " ms |\n";
    cout << "average control frequency is: " << avgHZ << endl;

    if(playback && !ASYNC_MPC){
        while(activeVisualiser->windowOpen()){

            if(activeVisualiser->replayTriggered){
                activeVisualiser->replayTriggered = false;

                activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
                int controlCounter = 0;
                while(controlCounter < activeVisualiser->replayControls.size()){
                    MatrixXd nextControl = activeVisualiser->replayControls[controlCounter].replicate(1, 1);

                    activeModelTranslator->SetControlVector(nextControl, activeModelTranslator->MuJoCo_helper->vis_data);

                    mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);

                    controlCounter++;

                    if(controlCounter % 5 == 0){

                        activeVisualiser->render("replaying");
                    }
                }
            }
            activeVisualiser->render("replay_mode - (PRESS BACKSPACE)");
        }
    }
    else{
        activeVisualiser->task_finished = true;
    }

    avg_opt_time = avgTimeGettingDerivs + avgTimeBP + avgTimeFP;
    avg_time_bp = avgTimeBP;
    avg_time_fp = avgTimeFP;
    avg_time_derivs = avgTimeGettingDerivs;
    avg_percent_derivs = avgPercentDerivs;
}

void generateTestingData_MPC(){
    playback = false;
    mpcVisualise = false;

    // start timer here
    auto startTime = std::chrono::high_resolution_clock::now();

    for(int k = 0; k < 1; k ++) {
        activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);

        activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);
        iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator, activeModelTranslator->MuJoCo_helper,
                                               activeDifferentiator, yamlReader->maxHorizon, activeVisualiser,
                                               yamlReader);
        activeOptimiser = iLQROptimiser;

        // ------------------------- data storage -------------------------------------
        std::vector<std::vector<double>> finalCosts;
        std::vector<double> finalCostsRow;

        std::vector<std::vector<double>> avgHzs;
        std::vector<double> avgHZRow;

        std::vector<std::vector<double>> avgTimeForDerivs;
        std::vector<double> avgTimeForDerivsRow;

        std::vector<std::vector<double>> avgTimeBP;
        std::vector<double> avgTimeBPRow;

        std::vector<std::vector<double>> avgTimeFP;
        std::vector<double> avgTimeFPRow;

        std::vector<std::vector<double>> avgPercentDerivs;
        std::vector<double> avgPercentDerivsRow;

//        std::vector<std::string> methodNames = {"baseline", "SI5", "SI20", "adapJerk", "iter_error", "magvel"};
//        int numMethods = methodNames.size();
//        std::vector<string> keypointMethods = {"setInterval", "setInterval", "setInterval", "adaptive_jerk", "iterative_error", "magvel_change"};
//        std::vector<int> min_N = {1, 5, 20, 1, 1, 1};
//        std::vector<int> max_N = {1, 2, 5, 5, 5, 5};

        std::vector<std::string> methodNames = {"magvel", "iter_error"};
        int numMethods = methodNames.size();
        std::vector<string> keypointMethods = {"magvel_change", "iterative_error"};
        std::vector<int> minN = {1, 1};
        std::vector<int> maxN = {5, 5};

        std::vector<double> targetVelocities;
        double minTarget = 0.1;
        double maxTarget = 0.3;
        int numTests = 100;
        double currentVel = minTarget;

        for(int i = 0; i < numTests; i++){
            targetVelocities.push_back(currentVel);
            currentVel += (maxTarget - minTarget) / (numTests - 1);
        }

        auto startTimer = std::chrono::high_resolution_clock::now();
        activeOptimiser->verbose_output = false;

        for (int i = 0; i < targetVelocities.size(); i++) {
            cout << "------------------------------------ Trajec " << i << " ------------------------------------\n";

            // Loop through our interpolating derivatives methods
            finalCostsRow.clear();
            avgHZRow.clear();
            avgTimeForDerivsRow.clear();
            avgTimeBPRow.clear();
            avgTimeFPRow.clear();
            avgPercentDerivsRow.clear();

            MatrixXd startStateVector;
            startStateVector.resize(activeModelTranslator->state_vector_size, 1);

            yamlReader->loadTaskFromFile(activeModelTranslator->model_name, i, startStateVector,
                                         activeModelTranslator->X_desired);

            // Walker model where were trying to match a velocity
            if(task == "walker"){
                activeModelTranslator->X_desired(10) = targetVelocities[i];
            }


            activeModelTranslator->X_start = startStateVector;
            activeModelTranslator->SetStateVector(startStateVector, activeModelTranslator->MuJoCo_helper->master_reset_data);
            for(int j = 0; j < 5; j++){
                mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->master_reset_data);
            }

            if(activeModelTranslator->MuJoCo_helper->checkIfDataIndexExists(0) == false){
                activeModelTranslator->MuJoCo_helper->appendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
            }

            // Move the end-effector to a decent starting position
            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
            std::vector<MatrixXd> setupControls = activeModelTranslator->CreateInitSetupControls(1000);
            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);


//            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
//            activeModelTranslator->MuJoCo_helper->copySystemState(0, activeModelTranslator->MuJoCo_helper->master_reset_data);

            for (int j = 0; j < numMethods; j++) {
                double avgHz = 0.0f;
                double finalCost = 0.0f;
                double avgPercentageDerivs = 0.0f;
                double avgTimeForDerivs = 0.0f;
                double avgTimeBP = 0.0f;
                double avgTimeFP = 0.0f;

                cout << "--------------------------------------------------------------------------------\n";
                cout << "current method: " << methodNames[j] << "\n";

                // Setup the keypoint method
                keypoint_method currentInterpolator = activeOptimiser->ReturnCurrentKeypointMethod();
                currentInterpolator.min_N = minN[j];
                currentInterpolator.max_N = maxN[j];
                currentInterpolator.name = keypointMethods[j];
                activeOptimiser->SetCurrentKeypointMethod(currentInterpolator);

                activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
                MPCUntilComplete(finalCost, avgHz, avgTimeForDerivs, avgPercentageDerivs, avgTimeBP, avgTimeFP, 1000, 1, 50);

                finalCostsRow.push_back(finalCost);
                avgHZRow.push_back(avgHz);
                avgTimeForDerivsRow.push_back(avgTimeForDerivs);
                avgTimeBPRow.push_back(avgTimeBP);
                avgTimeFPRow.push_back(avgTimeFP);
                avgPercentDerivsRow.push_back(avgPercentageDerivs);
            }

            // New row of data added
            finalCosts.push_back(finalCostsRow);
            avgHzs.push_back(avgHZRow);
            avgTimeForDerivs.push_back(avgTimeForDerivsRow);
            avgTimeBP.push_back(avgTimeBPRow);
            avgTimeFP.push_back(avgTimeFPRow);
            avgPercentDerivs.push_back(avgPercentDerivsRow);

            auto currentTime = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTimer).count();


            cout << "Time taken so far: " << duration/ 1000.0f << " s" << endl;
        }
        // Save data to csv
        cout << "save data to file\n";
        yamlReader->saveResultsData_MPC(activeModelTranslator->model_name, methodNames, finalCosts, avgHzs,
                                        avgTimeForDerivs, avgTimeBP, avgTimeFP, avgPercentDerivs);
    }
}

int generateTestingData_MPCHorizons(){
    playback = false;
    mpcVisualise = false;

    // start timer here
    auto startTime = std::chrono::high_resolution_clock::now();

    activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);

    activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);
    iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator, activeModelTranslator->MuJoCo_helper,
                                           activeDifferentiator, yamlReader->maxHorizon, activeVisualiser,
                                           yamlReader);
    activeOptimiser = iLQROptimiser;

    // ------------------------- data storage -------------------------------------
    std::vector<std::vector<double>> finalCosts;
    std::vector<double> finalCostsRow;

    std::vector<std::vector<double>> avgHzs;
    std::vector<double> avgHZRow;

    std::vector<std::vector<double>> avgTimeForDerivs;
    std::vector<double> avgTimeForDerivsRow;

    std::vector<std::vector<double>> avgTimeBP;
    std::vector<double> avgTimeBPRow;

    std::vector<std::vector<double>> avgTimeFP;
    std::vector<double> avgTimeFPRow;

    std::vector<std::vector<double>> avgPercentDerivs;
    std::vector<double> avgPercentDerivsRow;

    std::vector<int> horizons = {20, 30, 40, 50, 60, 70, 80};
    std::vector<std::string> horizonNames;
    int numHorizons = horizons.size();

    for(int i = 0; i < horizons.size(); i++){
        horizonNames.push_back(std::to_string(horizons[i]));
    }

    if(testingMethods.size() == 0){
        return 0;
    }

    std::vector<std::string> methodNames = {"baseline", "SI5", "SI10", "SI20", "adaptive_jerk2", "iterative_error", "magvel_change2"};
    std::vector<int> testIndices;
    bool anyMatch = false;
    for(int i = 0; i < testingMethods.size(); i++){
        for(int j = 0; j < methodNames.size(); j++){
            if(testingMethods[i] == methodNames[j]){
                anyMatch = true;
                testIndices.push_back(j);
            }
        }

    }

    if(anyMatch == false){
        cout << "passed testing arguments didnt match any allowed methods \n";
        return 0;
    }

    std::vector<int> minN = {1, 5, 10, 20, 2, 2, 2};
    std::vector<int> maxN = {1, 5, 10, 20, 20, 5, 10};
    std::vector<std::string> keypoint_method = {"setInterval", "setInterval", "setInterval", "setInterval", "adaptive_jerk", "iterative_error", "magvel_change"};

    std::vector<double> targetVelocities;
    double minTarget = 0.1;
    double maxTarget = 0.3;
    int numTests = 100;
    double currentVel = minTarget;

    for(int i = 0; i < numTests; i++){
        targetVelocities.push_back(currentVel);
        currentVel += (maxTarget - minTarget) / (numTests - 1);
    }

    auto startTimer = std::chrono::high_resolution_clock::now();
    activeOptimiser->verbose_output = false;

    for(int k = 0; k < testIndices.size(); k++) {
        int testIndex = testIndices[k];
        cout << "---------- current method " << methodNames[testIndex] << " ----------------" << endl;

        struct keypoint_method currentInterpolator = activeOptimiser->ReturnCurrentKeypointMethod();
        currentInterpolator.min_N = minN[testIndex];
        currentInterpolator.max_N = maxN[testIndex];
        currentInterpolator.name = keypoint_method[testIndex];
        activeOptimiser->SetCurrentKeypointMethod(currentInterpolator);

        finalCosts.clear();
        avgHzs.clear();
        avgTimeForDerivs.clear();
        avgTimeBP.clear();
        avgTimeFP.clear();
        avgPercentDerivs.clear();

        for (int i = 0; i < targetVelocities.size(); i++) {
            cout << "------------------------------------ Trajec " << i << " ------------------------------------\n";

            // Loop through our interpolating derivatives methods
            finalCostsRow.clear();
            avgHZRow.clear();
            avgTimeForDerivsRow.clear();
            avgTimeBPRow.clear();
            avgTimeFPRow.clear();
            avgPercentDerivsRow.clear();

            MatrixXd startStateVector;
            startStateVector.resize(activeModelTranslator->state_vector_size, 1);

            yamlReader->loadTaskFromFile(activeModelTranslator->model_name, i, startStateVector,
                                         activeModelTranslator->X_desired);

            // Walker model where were trying to match a velocity
            if(task == "walker"){
                activeModelTranslator->X_desired(10) = targetVelocities[i];
            }

            activeModelTranslator->X_start = startStateVector;
            activeModelTranslator->SetStateVector(startStateVector, activeModelTranslator->MuJoCo_helper->master_reset_data);
            for(int j = 0; j < 5; j++){
                mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->master_reset_data);
            }

            if(activeModelTranslator->MuJoCo_helper->checkIfDataIndexExists(0) == false){
                activeModelTranslator->MuJoCo_helper->appendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
            }

            // Move the end-effector to a decent starting position
            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
            std::vector<MatrixXd> setupControls = activeModelTranslator->CreateInitSetupControls(1000);
            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);

            for (int j = 0; j < numHorizons; j++) {
                double avgHz = 0.0f;
                double finalCost = 0.0f;
                double avgPercentageDerivs = 0.0f;
                double avgTimeForDerivs = 0.0f;
                double avgTimeBP = 0.0f;
                double avgTimeFP = 0.0f;

                cout << "--------------------------------------------------------------------------------\n";
                cout << "current horizon: " << horizonNames[j] << "\n";

                activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
                MPCUntilComplete(finalCost, avgHz, avgTimeForDerivs, avgPercentageDerivs, avgTimeBP, avgTimeFP, 1500, 1, horizons[j]);

                finalCostsRow.push_back(finalCost);
                avgHZRow.push_back(avgHz);
                avgTimeForDerivsRow.push_back(avgTimeForDerivs);
                avgTimeBPRow.push_back(avgTimeBP);
                avgTimeFPRow.push_back(avgTimeFP);
                avgPercentDerivsRow.push_back(avgPercentageDerivs);
            }

            // New row of data added
            finalCosts.push_back(finalCostsRow);
            avgHzs.push_back(avgHZRow);
            avgTimeForDerivs.push_back(avgTimeForDerivsRow);
            avgTimeBP.push_back(avgTimeBPRow);
            avgTimeFP.push_back(avgTimeFPRow);
            avgPercentDerivs.push_back(avgPercentDerivsRow);

            auto currentTime = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTimer).count();

            cout << "Time taken so far: " << duration/ 1000.0f << " s" << endl;
        }
        // Save data to csv
        cout << "save data to file for " << methodNames[testIndex] << endl;
        std::string taskPrefix = activeModelTranslator->model_name + "_" + methodNames[testIndex];
        yamlReader->saveResultsData_MPC(taskPrefix, horizonNames, finalCosts, avgHzs,
                                        avgTimeForDerivs, avgTimeBP, avgTimeFP, avgPercentDerivs);
    }

    cout << "tests exited correctly \n";
    return 1;
}

int assign_task(){
    if(task == "double_pendulum"){
        std::shared_ptr<DoublePendulum> myDoublePendulum = std::make_shared<DoublePendulum>();
        activeModelTranslator = myDoublePendulum;
    }
    else if(task == "acrobot"){
        std::shared_ptr<Acrobot> myAcrobot = std::make_shared<Acrobot>();
        activeModelTranslator = myAcrobot;

    }
    else if(task == "reaching"){
        std::shared_ptr<pandaReaching> myReaching = std::make_shared<pandaReaching>();
        activeModelTranslator = myReaching;
    }
    else if(task == "pushing_no_clutter"){
        std::shared_ptr<TwoDPushing> myTwoDPushing = std::make_shared<TwoDPushing>(noClutter);
        activeModelTranslator = myTwoDPushing;

    }
    else if(task == "pushing_low_clutter"){
        std::shared_ptr<TwoDPushing> myTwoDPushing = std::make_shared<TwoDPushing>(lowClutter);
        activeModelTranslator = myTwoDPushing;

    }
    else if(task == "pushing_moderate_clutter"){
        std::shared_ptr<TwoDPushing> myTwoDPushing = std::make_shared<TwoDPushing>(heavyClutter);
        activeModelTranslator = myTwoDPushing;

    }
    else if(task == "pushing_moderate_clutter_constrained"){
        std::shared_ptr<TwoDPushing> myTwoDPushing = std::make_shared<TwoDPushing>(constrainedClutter);
        activeModelTranslator = myTwoDPushing;
    }
    else if(task == "3D_pushing"){
        std::shared_ptr<ThreeDPushing> myThreeDPushing = std::make_shared<ThreeDPushing>();
        activeModelTranslator = myThreeDPushing;
    }
    else if(task == "box_push_toppling"){
        cout << "not implemented task yet " << endl;
        return EXIT_FAILURE;
    }
    else if(task == "box_flick_no_clutter"){
        std::shared_ptr<BoxFlick> myBoxFlick = std::make_shared<BoxFlick>(noClutter);
        activeModelTranslator = myBoxFlick;
    }
    else if(task == "box_flick_low_clutter"){
        std::shared_ptr<BoxFlick> myBoxFlick = std::make_shared<BoxFlick>(lowClutter);
        activeModelTranslator = myBoxFlick;
    }
    else if(task == "box_flick_moderate_clutter"){
        std::shared_ptr<BoxFlick> myBoxFlick = std::make_shared<BoxFlick>(heavyClutter);
        activeModelTranslator = myBoxFlick;
    }
    else if(task == "walker_walk"){
        std::shared_ptr<walker> myLocomotion = std::make_shared<walker>(PLANE, WALK);
        activeModelTranslator = myLocomotion;
    }
    else if(task == "walker_run"){
        std::shared_ptr<walker> myLocomotion = std::make_shared<walker>(UNEVEN, RUN);
        activeModelTranslator = myLocomotion;
    }
    else if(task == "walker_uneven"){
        std::shared_ptr<walker> myLocomotion = std::make_shared<walker>(UNEVEN, WALK);
        activeModelTranslator = myLocomotion;
    }
    else if(task == "Hopper"){
        cout << "not implemented task yet " << endl;
        return EXIT_FAILURE;
    }
    else if(task == "box_sweep"){
        std::shared_ptr<BoxSweep> myBoxSweep = std::make_shared<BoxSweep>();
        activeModelTranslator = myBoxSweep;
    }
    else{
        std::cout << "invalid scene selected, exiting" << std::endl;
    }
    return EXIT_SUCCESS;
}
