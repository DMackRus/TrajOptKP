#include "stdInclude.h"
#include "fileHandler.h"

// --------------------- different scenes -----------------------
#include "doublePendulum.h"
#include "acrobot.h"
#include "reaching.h"
#include "twoDPushing.h"
#include "boxFlick.h"
#include "locomotion.h"
#include "hopper.h"
#include "humanoid.h"
#include "boxSweep.h"

#include "visualizer.h"
#include "MuJoCoHelper.h"

// --------------------- different optimisers -----------------------
#include "interpolated_iLQR.h"
#include "stomp.h"
#include "gradDescent.h"

// --------------------- other -----------------------
#include <mutex>

// ------------ MODES OF OPERATION -------------------------------
#define ASYNC_MPC   true

// --------------------- Global class instances --------------------------------
std::shared_ptr<modelTranslator> activeModelTranslator;
std::shared_ptr<differentiator> activeDifferentiator;
std::shared_ptr<optimiser> activeOptimiser;
std::shared_ptr<interpolatediLQR> iLQROptimiser;
std::shared_ptr<stomp> stompOptimiser;
std::shared_ptr<gradDescent> gradDescentOptimiser;
std::shared_ptr<visualizer> activeVisualiser;
std::shared_ptr<fileHandler> yamlReader;

std::string task;
bool mpcVisualise = true;
bool playback = true;
std::vector<std::string> testingMethods;

void showInitControls();
void optimiseOnceandShow();
void MPCUntilComplete(double &trajecCost, double &avgHz, double &avgTimeGettingDerivs, double &avgPercentDerivs, double &avgTimeBP, double &avgTimeFP,
                      int MAX_TASK_TIME, int REPLAN_TIME, int OPT_HORIZON);

void generateTestScenes();

void generateTestingData_MPC();
int generateTestingData_MPCHorizons();
void generateTestingData();
void generateFilteringData();

void genericTesting();
void worker();

int main(int argc, char **argv) {

    // Expected arguments
    // 1. Program name
    // 2. Task name
    if(argc < 2){
        std::cout << "No task name provided, exiting" << endl;
        return -1;
    }

    std::string configFileName = argv[1];
    std::cout << "config file name: " << configFileName << endl;

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

    yamlReader = std::make_shared<fileHandler>();
    yamlReader->readSettingsFile("/generalConfigs/" + configFileName + ".yaml");
    optimiser = yamlReader->optimiser;
    runMode = yamlReader->project_run_mode;
    task = yamlReader->taskName;
    taskInitMode = yamlReader->taskInitMode;

    MatrixXd startStateVector(1, 1);

    if(task == "double_pendulum"){
        std::shared_ptr<doublePendulum> myDoublePendulum = std::make_shared<doublePendulum>();
        activeModelTranslator = myDoublePendulum;
    }
    else if(task == "acrobot"){
        std::shared_ptr<acrobot> myAcrobot = std::make_shared<acrobot>();
        activeModelTranslator = myAcrobot;

    }
    else if(task == "reaching"){
        std::shared_ptr<pandaReaching> myReaching = std::make_shared<pandaReaching>();
        activeModelTranslator = myReaching;
    }
    else if(task == "pushing_no_clutter"){
        std::shared_ptr<twoDPushing> myTwoDPushing = std::make_shared<twoDPushing>(noClutter);
        activeModelTranslator = myTwoDPushing;

    }
    else if(task == "pushing_low_clutter"){
        std::shared_ptr<twoDPushing> myTwoDPushing = std::make_shared<twoDPushing>(lowClutter);
        activeModelTranslator = myTwoDPushing;

    }
    else if(task == "pushing_moderate_clutter"){
        std::shared_ptr<twoDPushing> myTwoDPushing = std::make_shared<twoDPushing>(heavyClutter);
        activeModelTranslator = myTwoDPushing;

    }
    else if(task == "pushing_moderate_clutter_constrained"){
        std::shared_ptr<twoDPushing> myTwoDPushing = std::make_shared<twoDPushing>(constrainedClutter);
        activeModelTranslator = myTwoDPushing;
    }
    else if(task == "box_push_toppling"){
        cout << "not implemented task yet " << endl;
        return -1;
    }
    else if(task == "box_flick_no_clutter"){
        std::shared_ptr<boxFlick> myBoxFlick = std::make_shared<boxFlick>(noClutter);
        activeModelTranslator = myBoxFlick;
    }
    else if(task == "box_flick_low_clutter"){
        std::shared_ptr<boxFlick> myBoxFlick = std::make_shared<boxFlick>(lowClutter);
        activeModelTranslator = myBoxFlick;
    }
    else if(task == "box_flick_moderate_clutter"){
        std::shared_ptr<boxFlick> myBoxFlick = std::make_shared<boxFlick>(heavyClutter);
        activeModelTranslator = myBoxFlick;
    }
    else if(task == "walker"){
        std::shared_ptr<walker> myLocomotion = std::make_shared<walker>();
        activeModelTranslator = myLocomotion;
    }
    else if(task == "hopper"){
//        std::shared_ptr<hopper> myHopper = std::make_shared<hopper>();
//        activeModelTranslator = myHopper;
        cout << "not implemented task yet " << endl;
        return -1;
    }
    else if(task == "box_sweep"){
        std::shared_ptr<boxSweep> myBoxSweep = std::make_shared<boxSweep>();
        activeModelTranslator = myBoxSweep;
    }
    else{
        std::cout << "invalid scene selected, exiting" << std::endl;
    }

    if(runMode == "Generate_testing_data"){
    	 return generateTestingData_MPCHorizons();
//        generateTestingData_MPC();
        //generateTestingData();
        //return 0;
    }

    startStateVector.resize(activeModelTranslator->stateVectorSize, 1);
    startStateVector = activeModelTranslator->X_start;

    // random start and goal state
    std::string taskPrefix = activeModelTranslator->modelName;
    if(taskInitMode == "random"){
        activeModelTranslator->generateRandomGoalAndStartState();
    }
    else if(taskInitMode == "fromCSV"){
        yamlReader->loadTaskFromFile(taskPrefix, yamlReader->csvRow, startStateVector, activeModelTranslator->X_desired);
        activeModelTranslator->X_start = startStateVector;
    }

    cout << "start state " << activeModelTranslator->X_start << endl;
    cout << "desired state " << activeModelTranslator->X_desired << endl;

    activeDifferentiator = std::make_shared<differentiator>(activeModelTranslator, activeModelTranslator->myHelper);
    activeModelTranslator->setStateVector(startStateVector, MASTER_RESET_DATA);
    activeModelTranslator->activePhysicsSimulator->stepSimulator(5, MASTER_RESET_DATA);
    activeModelTranslator->activePhysicsSimulator->appendSystemStateToEnd(MASTER_RESET_DATA);

    //Instantiate my visualiser
    activeVisualiser = std::make_shared<visualizer>(activeModelTranslator);

    // Choose an optimiser
    if(optimiser == "interpolated_iLQR"){
        iLQROptimiser = std::make_shared<interpolatediLQR>(activeModelTranslator, activeModelTranslator->activePhysicsSimulator, activeDifferentiator, yamlReader->maxHorizon, activeVisualiser, yamlReader);
        activeOptimiser = iLQROptimiser;
    }
    else if(optimiser == "stomp"){
        stompOptimiser = std::make_shared<stomp>(activeModelTranslator, activeModelTranslator->activePhysicsSimulator, yamlReader, activeDifferentiator, yamlReader->maxHorizon, 8);
        activeOptimiser = stompOptimiser;
    }
    else if(optimiser == "gradDescent"){
        gradDescentOptimiser = std::make_shared<gradDescent>(activeModelTranslator, activeModelTranslator->activePhysicsSimulator, activeDifferentiator, activeVisualiser, yamlReader->maxHorizon, yamlReader);
        activeOptimiser = gradDescentOptimiser;
    }
    else{
        cout << "invalid optimiser selected, exiting" << endl;
        return -1;
    }

    // Methods of control / visualisation
    if(runMode == "Init_controls"){
        cout << "SHOWING INIT CONTROLS MODE \n";
        showInitControls();
    }
    else if(runMode == "Optimise_once"){
        cout << "OPTIMISE TRAJECTORY ONCE AND DISPLAY MODE \n";
        activeOptimiser->verboseOutput = true;
        optimiseOnceandShow();
    }
    else if(runMode == "MPC_until_completion"){
        cout << "MPC UNTIL TASK COMPLETE MODE \n";
        double trajecCost, avgHz;
        double avgPercentDerivs, avgTimeDerivs, avgTimeBP, avgTimeFP;
        activeOptimiser->setTrajecNumber(1000);

        // Some tasks need setup controls to be generated and executed
        std::vector<MatrixXd> initSetupControls = activeModelTranslator->createInitSetupControls(1000);
        activeModelTranslator->activePhysicsSimulator->copySystemState(MASTER_RESET_DATA, MAIN_DATA_STATE);

        // Whether optimiser will output useful information
        activeOptimiser->verboseOutput = true;
        // Visualise MPC trajectory live
        mpcVisualise = true;

        if(task == "walker"){
            // Setting lateral desired speed
            activeModelTranslator->X_desired(10) = 0.3;
        }

        if(ASYNC_MPC){
            std::thread MPC_controls_thread;
            MPC_controls_thread = std::thread(&worker);
            int vis_counter = 0;
            MatrixXd next_control;
            // timer variables
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            std::chrono::steady_clock::time_point end;

            while(activeVisualiser->windowOpen()){
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
                activeVisualiser->trajectory_states.push_back(activeModelTranslator->returnStateVector(VISUALISATION_DATA));

                // Set the latest control
                activeModelTranslator->setControlVector(next_control, VISUALISATION_DATA);

                // Update the simulation
                activeModelTranslator->activePhysicsSimulator->stepSimulator(1, VISUALISATION_DATA);

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
                int difference_ms = (activeModelTranslator->activePhysicsSimulator->returnModelTimeStep() * 1000) - (time_taken / 1000.0f) + 1;

                if(difference_ms > 0) {
                    std::cout << "visualisation took " << (time_taken / 1000.0f) << " ms, sleeping for "
                              << difference_ms << " ms \n";
                    std::this_thread::sleep_for(std::chrono::milliseconds(difference_ms));
                }
                else
                    std::cout << "visualisation took " << (time_taken / 1000.0f) << " ms, longer than time-step, skipping sleep \n";


                activeModelTranslator->X_desired(1) = activeModelTranslator->returnStateVector(VISUALISATION_DATA)(1) + 0.15;

//                std::this_thread::sleep_for(std::chrono::milliseconds(5));

                if(activeVisualiser->task_finished){
                    activeVisualiser->task_finished = false;
                    // Replay states through cost function

                    double cost = 0.0f;
                    activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
                    for(int i = 0; i < activeVisualiser->trajectory_states.size(); i++){
                        activeModelTranslator->setControlVector(activeVisualiser->trajectory_controls[i], MAIN_DATA_STATE);
                        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);
                        cost += (activeModelTranslator->costFunction(MAIN_DATA_STATE, false) * activeModelTranslator->activePhysicsSimulator->returnModelTimeStep());

                    }
                    std::cout << "final cost of entire MPC trajectory was: " << cost << "\n";

                }

            }
            MPC_controls_thread.join();

        }
        else{
            MPCUntilComplete(trajecCost, avgHz, avgPercentDerivs, avgTimeDerivs, avgTimeBP, avgTimeFP, 2500, 1, 100);
        }
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
    }
    return 0;
}

void onetaskGenerateTestingData(){
    int setupHorizon = 1000;
    int optHorizon = 2800;

    MatrixXd startStateVector;
    startStateVector.resize(activeModelTranslator->stateVectorSize, 1);

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
//    std::vector<int> minN = {1, 5, 1000, 5, 5, 5};
//    std::vector<int> maxN = {1, 5, 1000, 100, 100, 100};

    std::vector<std::string> methodNames = {"baseline", "SI5", "SI1000", "adaptive_jerk_5", "iterative_error_5", "magvel_change_5"};
    int numMethods = methodNames.size();
    std::vector<string> keyPointMethods = {"setInterval", "setInterval", "setInterval", "adaptive_jerk", "iterative_error", "magvel_change"};
    std::vector<int> minN = {1, 5, 1000, 2, 2, 2};
    std::vector<int> maxN = {1, 5, 1000, 10, 10, 10};

//    std::vector<std::string> methodNames = {"iterative_error"};
//    int numMethods = methodNames.size();
//    std::vector<std::string> keyPointMethods = {"iterative_error"};
//    std::vector<int> minN = {5};
//    std::vector<int> maxN = {20};

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

        yamlReader->loadTaskFromFile(activeModelTranslator->modelName, i, startStateVector, activeModelTranslator->X_desired);
        activeModelTranslator->X_start = startStateVector;
        activeModelTranslator->setStateVector(startStateVector, MASTER_RESET_DATA);
        activeModelTranslator->activePhysicsSimulator->stepSimulator(5, MASTER_RESET_DATA);

        if(activeModelTranslator->activePhysicsSimulator->checkIfDataIndexExists(0) == false){
            activeModelTranslator->activePhysicsSimulator->appendSystemStateToEnd(MASTER_RESET_DATA);
        }

        // Move the end-effector to a decent starting position
        activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
        std::vector<MatrixXd> setupControls = activeModelTranslator->createInitSetupControls(1000);
        activeModelTranslator->activePhysicsSimulator->copySystemState(MASTER_RESET_DATA, MAIN_DATA_STATE);

        std::vector<MatrixXd> initOptimisationControls = activeModelTranslator->createInitOptimisationControls(optHorizon);

        for(int j = 0; j < numMethods; j++){
            double optTime;
            double costReduction;
            double avgPercentageDerivs;
            double avgTimeForDerivs;
            int numIterationsForConvergence;

            activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
            activeModelTranslator->activePhysicsSimulator->copySystemState(0, MASTER_RESET_DATA);

            // Setup interpolation method
            derivative_interpolator currentInterpolator = activeOptimiser->returnDerivativeInterpolator();
            currentInterpolator.keypoint_method = keyPointMethods[j];
            currentInterpolator.minN = minN[j];
            currentInterpolator.maxN = maxN[j];
            activeOptimiser->setDerivativeInterpolator(currentInterpolator);

            std::vector<MatrixXd> optimisedControls = activeOptimiser->optimise(0, initOptimisationControls, 8, 2, optHorizon);

//            yamlReader->saveTrajecInfomation(activeOptimiser->A, activeOptimiser->B, activeOptimiser->X_old, activeOptimiser->U_old, activeModelTranslator->modelName, i, optHorizon);

            // Return testing data and append appropriately
            activeOptimiser->returnOptimisationData(optTime, costReduction, avgPercentageDerivs, avgTimeForDerivs, numIterationsForConvergence);
            optTimesRow.push_back(optTime);
            costReductionsRow.push_back(costReduction);
            avgPercentageDerivsRow.push_back(avgPercentageDerivs);
            avgTimeForDerivsRow.push_back(avgTimeForDerivs);
            numIterationsRow.push_back(numIterationsForConvergence);

//            activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

//            int controlCounter = 0;
//            int visualCounter = 0;
//            cout << "final controls size: " << optimisedControls.size() << endl;
//
//            while(controlCounter < initOptimisationControls.size()){
//
//                activeModelTranslator->setControlVector(initOptimisationControls[controlCounter], MAIN_DATA_STATE);
//
//                activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);
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
    yamlReader->saveResultsDataForMethods(activeModelTranslator->modelName, methodNames,optTimes, costReductions, avgPercentageDerivs, avgTimeForDerivs, numIterations);
}

void generateTestingData(){
    activeDifferentiator = std::make_shared<differentiator>(activeModelTranslator, activeModelTranslator->myHelper);

    MatrixXd startStateVector;
    startStateVector.resize(activeModelTranslator->stateVectorSize, 1);
    startStateVector = activeModelTranslator->X_start;
    activeModelTranslator->setStateVector(startStateVector, MASTER_RESET_DATA);
    activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);
    activeModelTranslator->activePhysicsSimulator->appendSystemStateToEnd(MAIN_DATA_STATE);

    activeVisualiser = std::make_shared<visualizer>(activeModelTranslator);
    iLQROptimiser = std::make_shared<interpolatediLQR>(activeModelTranslator, activeModelTranslator->activePhysicsSimulator, activeDifferentiator, yamlReader->maxHorizon, activeVisualiser, yamlReader);
    activeOptimiser = iLQROptimiser;

    onetaskGenerateTestingData();
}

void generateFilteringData(){
    int setupHorizon = 1000;
    int optHorizon = 2200;
    int numTests = 100;

    MatrixXd startStateVector;
    startStateVector.resize(activeModelTranslator->stateVectorSize, 1);

    std::vector<double> lowPassTests = {0.05, 0.1, 0.15, 0.2, 0.25, 0.3};
    std::vector<std::vector<double>> FIRTests;
    FIRTests.push_back({0.25, 0.5, 0.25});
    FIRTests.push_back({0.1, 0.15, 0.5, 0.15, 0.1});
    FIRTests.push_back({0.05, 0.1, 0.15, 0.3, 0.15, 0.1, 0.05});
    FIRTests.push_back({0.05, 0.05, 0.15, 0.2, 0.2, 0.2, 0.15, 0.05, 0.05});
    FIRTests.push_back({0.05, 0.1, 0.15, 0.2, 0.2, 0.15, 0.1, 0.05});

    derivative_interpolator currentInterpolator = activeOptimiser->returnDerivativeInterpolator();
    currentInterpolator.keypoint_method = "setInterval";
    currentInterpolator.minN = 1;
    activeOptimiser->setDerivativeInterpolator(currentInterpolator);

    for (int i = 0; i < numTests; i++) {
        yamlReader->loadTaskFromFile(activeModelTranslator->modelName, i, startStateVector,
                                     activeModelTranslator->X_desired);
        activeModelTranslator->X_start = startStateVector;
        activeModelTranslator->setStateVector(startStateVector, MASTER_RESET_DATA);
        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MASTER_RESET_DATA);

        std::vector<MatrixXd> initSetupControls = activeModelTranslator->createInitSetupControls(setupHorizon);
        activeModelTranslator->activePhysicsSimulator->copySystemState(MASTER_RESET_DATA, MAIN_DATA_STATE);

        std::vector<MatrixXd> initOptimisationControls = activeModelTranslator->createInitOptimisationControls(optHorizon);

        // Initialise task for optimisation by here

        // ---------------- unfiltered tests ------------------------------
        activeOptimiser->filteringMethod = "none";
        // Load a task from saved tasks

        activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
        activeModelTranslator->activePhysicsSimulator->copySystemState(0, MASTER_RESET_DATA);

        std::vector<MatrixXd> optimisedControls = activeOptimiser->optimise(0,
                                                                            initOptimisationControls,
                                                                            yamlReader->maxIter,
                                                                            yamlReader->minIter,
                                                                            optHorizon);
        // Save cost history to file
        std::string filePrefix;
        filePrefix = activeModelTranslator->modelName + "/none/";

        yamlReader->saveCostHistory(activeOptimiser->costHistory, filePrefix, i);

        // ---------------------- Low pass filter tests ----------------------

        for(int j = 0; j < lowPassTests.size(); j++){
            activeOptimiser->filteringMethod = "low_pass";
            activeOptimiser->lowPassACoefficient = lowPassTests[j];
            // Load a task from saved tasks

            activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
            activeModelTranslator->activePhysicsSimulator->copySystemState(0, MASTER_RESET_DATA);

            std::vector<MatrixXd> optimisedControls = activeOptimiser->optimise(0,
                                                                                initOptimisationControls,
                                                                                yamlReader->maxIter,
                                                                                yamlReader->minIter,
                                                                                optHorizon);
            // Save cost history to file
            std::string filePrefix;

            filePrefix = activeModelTranslator->modelName + "/lowPass" + std::to_string(lowPassTests[j]) +"/";

            yamlReader->saveCostHistory(activeOptimiser->costHistory, filePrefix, i);
        }

        // ---------------------- FIR filter tests ----------------------
        for(int j = 0; j < FIRTests.size(); j++){
            activeOptimiser->filteringMethod = "FIR";
            activeOptimiser->setFIRFilter(FIRTests[j]);
            // Load a task from saved tasks

            activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
            activeModelTranslator->activePhysicsSimulator->copySystemState(0, MASTER_RESET_DATA);

            std::vector<MatrixXd> optimisedControls = activeOptimiser->optimise(0,
                                                                                initOptimisationControls,
                                                                                yamlReader->maxIter,
                                                                                yamlReader->minIter,
                                                                                optHorizon);
            // Save cost history to file
            std::string filePrefix;

            filePrefix = activeModelTranslator->modelName + "/FIR_" + std::to_string(j) +"/";

            yamlReader->saveCostHistory(activeOptimiser->costHistory, filePrefix, i);
        }


    }
}

void generateTestScenes(){
    for(int i = 0; i < 100; i++){
        activeModelTranslator->generateRandomGoalAndStartState();
        activeModelTranslator->setStateVector(activeModelTranslator->X_start, MAIN_DATA_STATE);
        activeVisualiser->render("init state");
        cout << "starting state: " << activeModelTranslator->X_start.transpose() << endl;

        yamlReader->saveTaskToFile(activeModelTranslator->modelName, i, activeModelTranslator->X_start, activeModelTranslator->X_desired);
    }
}

void showInitControls(){
    int setupHorizon = 1000;
    int optHorizon = 2500;
    int controlCounter = 0;
    int visualCounter = 0;

    std::vector<MatrixXd> initControls;

//    activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
    std::vector<MatrixXd> initSetupControls = activeModelTranslator->createInitSetupControls(setupHorizon);
    activeModelTranslator->activePhysicsSimulator->copySystemState(MASTER_RESET_DATA, MAIN_DATA_STATE);
    std::vector<MatrixXd> initOptimisationControls = activeModelTranslator->createInitOptimisationControls(optHorizon);
    activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);

    //Stitch setup and optimisation controls together
//    initControls.insert(initControls.end(), initSetupControls.begin(), initSetupControls.end());
    initControls.insert(initControls.end(), initOptimisationControls.begin(), initOptimisationControls.end());

    while(activeVisualiser->windowOpen()){

        activeModelTranslator->setControlVector(initControls[controlCounter], MAIN_DATA_STATE);
        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);


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
            activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
        }

        if(visualCounter > 5){
            visualCounter = 0;
            activeModelTranslator->activePhysicsSimulator->copySystemState(VISUALISATION_DATA, MAIN_DATA_STATE);
            activeModelTranslator->activePhysicsSimulator->forwardSimulator(VISUALISATION_DATA);
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
    char* label = "Final trajectory after optimisation";

    std::vector<MatrixXd> initControls;
    std::vector<MatrixXd> finalControls;

//    activeModelTranslator->activePhysicsSimulator->copySystemState(0, MAIN_DATA_STATE);
//    MatrixXd test = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);
//    cout << "test: " << test << endl;

    std::vector<MatrixXd> initSetupControls = activeModelTranslator->createInitSetupControls(1000);
    activeModelTranslator->activePhysicsSimulator->copySystemState(MASTER_RESET_DATA, MAIN_DATA_STATE);

    std::vector<MatrixXd> initOptimisationControls = activeModelTranslator->createInitOptimisationControls(optHorizon);
    activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
    activeModelTranslator->activePhysicsSimulator->copySystemState(0, MASTER_RESET_DATA);

//    activeOptimiser->setupTestingExtras(1000, keyPointMethod, activeOptimiser->min_interval);

    auto start = high_resolution_clock::now();
    std::vector<MatrixXd> optimisedControls = activeOptimiser->optimise(0, initOptimisationControls, yamlReader->maxIter, yamlReader->minIter, optHorizon);
    auto stop = high_resolution_clock::now();
    auto linDuration = duration_cast<microseconds>(stop - start);
    cout << "optimisation took: " << linDuration.count() / 1000000.0f << " ms\n";

    // Stitch together setup controls with init control + optimised controls
//    initControls.insert(initControls.end(), initSetupControls.begin(), initSetupControls.end());
    initControls.insert(initControls.end(), initOptimisationControls.begin(), initOptimisationControls.end());
//    finalControls.insert(finalControls.end(), initSetupControls.begin(), initSetupControls.end());
    finalControls.insert(finalControls.end(), optimisedControls.begin(), optimisedControls.end());

    activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);

    while(activeVisualiser->windowOpen()){

        if(showFinalControls){
            activeModelTranslator->setControlVector(finalControls[controlCounter], MAIN_DATA_STATE);
        }
        else{
            activeModelTranslator->setControlVector(initControls[controlCounter], MAIN_DATA_STATE);
        }

        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

        controlCounter++;
        visualCounter++;

        if(controlCounter >= finalControls.size()){
            controlCounter = 0;
            activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
            showFinalControls = !showFinalControls;
            if(showFinalControls){
                label = "Final trajectory after optimisation";
            }
            else{
                label = "Intial trajectory before optimisation";
            }
        }

        if(visualCounter >= 5){
            visualCounter = 0;
            activeModelTranslator->activePhysicsSimulator->copySystemState(VISUALISATION_DATA, MAIN_DATA_STATE);
            activeModelTranslator->activePhysicsSimulator->forwardSimulator(VISUALISATION_DATA);
            activeVisualiser->render(label);
        }
    }
}

void worker(){
    double trajecCost, avgHz, avgPercentDerivs, avgTimeDerivs, avgTimeBP, avgTimeFP;
    MPCUntilComplete(trajecCost, avgHz, avgPercentDerivs, avgTimeDerivs, avgTimeBP, avgTimeFP, 3000, 1, 80);
}

// Before calling this function, we should setup the activeModelTranslator with the correct initial state and the
// optimiser settings. This function can then return necessary testing data for us to store
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

    initOptimisationControls = activeModelTranslator->createInitOptimisationControls(horizon);
    activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
    activeModelTranslator->activePhysicsSimulator->copySystemState(0, MASTER_RESET_DATA);
    activeModelTranslator->activePhysicsSimulator->copySystemState(VISUALISATION_DATA, MASTER_RESET_DATA);

    optimisedControls = activeOptimiser->optimise(0, initOptimisationControls, 5, 4, OPT_HORIZON);

    MatrixXd currState;
    activeOptimiser->verboseOutput = true;

    while(!taskComplete){
        if(!ASYNC_MPC){
//            currState = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);

            MatrixXd nextControl = optimisedControls[0].replicate(1, 1);
            activeVisualiser->replayControls.push_back(nextControl.replicate(1, 1));

            optimisedControls.erase(optimisedControls.begin());

            optimisedControls.push_back(optimisedControls.at(optimisedControls.size() - 1));

            activeModelTranslator->setControlVector(nextControl, VISUALISATION_DATA);
            activeModelTranslator->activePhysicsSimulator->stepSimulator(1, VISUALISATION_DATA);
        }


        reInitialiseCounter++;
        visualCounter++;

        // Re-optimise evert REPLAN_TIME steps
        if(reInitialiseCounter >= REPLAN_TIME){
            activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, VISUALISATION_DATA);
            activeModelTranslator->activePhysicsSimulator->copySystemState(0, MAIN_DATA_STATE);

            if(ASYNC_MPC){
                int current_control_index = activeVisualiser->current_control_index;

                // Slice the optimised controls to get the current control
                for(int i = 0; i < current_control_index; i++){
                    optimisedControls.erase(optimisedControls.begin());
                    optimisedControls.push_back(optimisedControls.at(optimisedControls.size() - 1));
                }
            }
            optimisedControls = activeOptimiser->optimise(0, optimisedControls, 1, 1, OPT_HORIZON);
            reInitialiseCounter = 0;

            timeGettingDerivs.push_back(activeOptimiser->avgTime_getDerivs_ms);
            timeBackwardsPass.push_back(activeOptimiser->avgTime_backwardsPass_ms);
            timeForwardsPass.push_back(activeOptimiser->avgTime_forwardsPass_ms);
            percentagesDerivsCalculated.push_back(activeOptimiser->avgPercentDerivs);

            std::cout << "optimise iteration complete \n";

        }

        if(!ASYNC_MPC){
            if(mpcVisualise){
                if(visualCounter > 10){
//                    activeModelTranslator->activePhysicsSimulator->copySystemState(VISUALISATION_DATA, MAIN_DATA_STATE);
//                    activeModelTranslator->activePhysicsSimulator->forwardSimulator(VISUALISATION_DATA);
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

            int optTimeToTimeSteps = activeOptimiser->optTime / (activeModelTranslator->activePhysicsSimulator->returnModelTimeStep() * 1000);
            std::cout << "opt time to time steps " << optTimeToTimeSteps << std::endl;

            int low_bound = optTimeToTimeSteps - 3;
            if (low_bound < 0) low_bound = 0;

            int high_bound = optTimeToTimeSteps + 3;

            // By the time we have computed optimal controls, main visualisation will be some number
            // of time-steps ahead. We need to find the correct control to apply.

            MatrixXd current_vis_state = activeModelTranslator->returnStateVector(VISUALISATION_DATA);

            double smallestError = 1000.00;
            int bestMatchingStateIndex = 0;
            // TODO - possible issue if optimisation time > horizon
            for(int i = low_bound; i < high_bound; i++){
//                std::cout << "i: " << i << " state: " << activeOptimiser->X_old[i].transpose() << std::endl;
//                std::cout << "correct state: " << current_vis_state.transpose() << std::endl;
                double currError = 0.0f;
                for(int j = 0; j < activeModelTranslator->stateVectorSize; j++){
                    currError += abs(activeOptimiser->X_old[i](j) - current_vis_state(j));
                }

                if(currError < smallestError){
                    smallestError = currError;
                    bestMatchingStateIndex = i;
                }
            }

            activeVisualiser->controlBuffer = optimisedControls;
            activeVisualiser->current_control_index = bestMatchingStateIndex;
            activeVisualiser->new_controls_flag = true;

            mtx.unlock();

        }

        overallTaskCounter++;

        if(overallTaskCounter >= MAX_TASK_TIME){
            cout << "task time out" << endl;
            taskComplete = true;
        }
    }

    trajecCost = 0.0f;
    activeModelTranslator->activePhysicsSimulator->copySystemState(VISUALISATION_DATA, MASTER_RESET_DATA);

    for(int i = 0; i < activeVisualiser->replayControls.size(); i++){
        MatrixXd startState = activeModelTranslator->returnStateVector(VISUALISATION_DATA);
//        std::cout << "start state: " << startState.transpose() << std::endl;
        MatrixXd nextControl = activeVisualiser->replayControls[i].replicate(1, 1);
        double stateCost = activeModelTranslator->costFunction(VISUALISATION_DATA, false);
        trajecCost += stateCost  * activeModelTranslator->activePhysicsSimulator->returnModelTimeStep();

        activeModelTranslator->setControlVector(nextControl, VISUALISATION_DATA);
        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, VISUALISATION_DATA);

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

                activeModelTranslator->activePhysicsSimulator->copySystemState(VISUALISATION_DATA, MASTER_RESET_DATA);
                int controlCounter = 0;
                while(controlCounter < activeVisualiser->replayControls.size()){
                    MatrixXd nextControl = activeVisualiser->replayControls[controlCounter].replicate(1, 1);

                    activeModelTranslator->setControlVector(nextControl, VISUALISATION_DATA);

                    activeModelTranslator->activePhysicsSimulator->stepSimulator(1, VISUALISATION_DATA);

                    controlCounter++;

                    if(controlCounter % 5 == 0){

//                        activeModelTranslator->activePhysicsSimulator->copySystemState(VISUALISATION_DATA, MAIN_DATA_STATE);
//                        activeModelTranslator->activePhysicsSimulator->forwardSimulator(VISUALISATION_DATA);
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
}

void generateTestingData_MPC(){
    playback = false;
    mpcVisualise = false;

    // start timer here
    auto startTime = std::chrono::high_resolution_clock::now();

    for(int k = 0; k < 1; k ++) {
        activeDifferentiator = std::make_shared<differentiator>(activeModelTranslator, activeModelTranslator->myHelper);

        activeVisualiser = std::make_shared<visualizer>(activeModelTranslator);
        iLQROptimiser = std::make_shared<interpolatediLQR>(activeModelTranslator, activeModelTranslator->activePhysicsSimulator,
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
//        std::vector<int> minN = {1, 5, 20, 1, 1, 1};
//        std::vector<int> maxN = {1, 2, 5, 5, 5, 5};

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
        activeOptimiser->verboseOutput = false;

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
            startStateVector.resize(activeModelTranslator->stateVectorSize, 1);

            yamlReader->loadTaskFromFile(activeModelTranslator->modelName, i, startStateVector,
                                         activeModelTranslator->X_desired);

            // Walker model where were trying to match a velocity
            if(task == "walker"){
                activeModelTranslator->X_desired(10) = targetVelocities[i];
            }


            activeModelTranslator->X_start = startStateVector;
            activeModelTranslator->setStateVector(startStateVector, MASTER_RESET_DATA);
            activeModelTranslator->activePhysicsSimulator->stepSimulator(5, MASTER_RESET_DATA);

            if(activeModelTranslator->activePhysicsSimulator->checkIfDataIndexExists(0) == false){
                activeModelTranslator->activePhysicsSimulator->appendSystemStateToEnd(MASTER_RESET_DATA);
            }

            // Move the end-effector to a decent starting position
            activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
            std::vector<MatrixXd> setupControls = activeModelTranslator->createInitSetupControls(1000);
            activeModelTranslator->activePhysicsSimulator->copySystemState(MASTER_RESET_DATA, MAIN_DATA_STATE);


//            activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
//            activeModelTranslator->activePhysicsSimulator->copySystemState(0, MASTER_RESET_DATA);

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
                derivative_interpolator currentInterpolator = activeOptimiser->returnDerivativeInterpolator();
                currentInterpolator.minN = minN[j];
                currentInterpolator.maxN = maxN[j];
                currentInterpolator.keypoint_method = keypointMethods[j];
                activeOptimiser->setDerivativeInterpolator(currentInterpolator);

                activeModelTranslator->activePhysicsSimulator->copySystemState( MAIN_DATA_STATE, MASTER_RESET_DATA);
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
        yamlReader->saveResultsData_MPC(activeModelTranslator->modelName, methodNames, finalCosts, avgHzs,
                                        avgTimeForDerivs, avgTimeBP, avgTimeFP, avgPercentDerivs);
    }
}

int generateTestingData_MPCHorizons(){
    playback = false;
    mpcVisualise = false;

    // start timer here
    auto startTime = std::chrono::high_resolution_clock::now();

    activeDifferentiator = std::make_shared<differentiator>(activeModelTranslator, activeModelTranslator->myHelper);

    activeVisualiser = std::make_shared<visualizer>(activeModelTranslator);
    iLQROptimiser = std::make_shared<interpolatediLQR>(activeModelTranslator, activeModelTranslator->activePhysicsSimulator,
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
    activeOptimiser->verboseOutput = false;

    for(int k = 0; k < testIndices.size(); k++) {
        int testIndex = testIndices[k];
        cout << "---------- current method " << methodNames[testIndex] << " ----------------" << endl;

        derivative_interpolator currentInterpolator = activeOptimiser->returnDerivativeInterpolator();
        currentInterpolator.minN = minN[testIndex];
        currentInterpolator.maxN = maxN[testIndex];
        currentInterpolator.keypoint_method = keypoint_method[testIndex];
        activeOptimiser->setDerivativeInterpolator(currentInterpolator);

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
            startStateVector.resize(activeModelTranslator->stateVectorSize, 1);

            yamlReader->loadTaskFromFile(activeModelTranslator->modelName, i, startStateVector,
                                         activeModelTranslator->X_desired);

            // Walker model where were trying to match a velocity
            if(task == "walker"){
                activeModelTranslator->X_desired(10) = targetVelocities[i];
            }

            activeModelTranslator->X_start = startStateVector;
            activeModelTranslator->setStateVector(startStateVector, MASTER_RESET_DATA);
            activeModelTranslator->activePhysicsSimulator->stepSimulator(5, MASTER_RESET_DATA);

            if(activeModelTranslator->activePhysicsSimulator->checkIfDataIndexExists(0) == false){
                activeModelTranslator->activePhysicsSimulator->appendSystemStateToEnd(MASTER_RESET_DATA);
            }

            // Move the end-effector to a decent starting position
            activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
            std::vector<MatrixXd> setupControls = activeModelTranslator->createInitSetupControls(1000);
            activeModelTranslator->activePhysicsSimulator->copySystemState(MASTER_RESET_DATA, MAIN_DATA_STATE);

            for (int j = 0; j < numHorizons; j++) {
                double avgHz = 0.0f;
                double finalCost = 0.0f;
                double avgPercentageDerivs = 0.0f;
                double avgTimeForDerivs = 0.0f;
                double avgTimeBP = 0.0f;
                double avgTimeFP = 0.0f;

                cout << "--------------------------------------------------------------------------------\n";
                cout << "current horizon: " << horizonNames[j] << "\n";

                activeModelTranslator->activePhysicsSimulator->copySystemState( MAIN_DATA_STATE, MASTER_RESET_DATA);
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
        std::string taskPrefix = activeModelTranslator->modelName + "_" + methodNames[testIndex];
        yamlReader->saveResultsData_MPC(taskPrefix, horizonNames, finalCosts, avgHzs,
                                        avgTimeForDerivs, avgTimeBP, avgTimeFP, avgPercentDerivs);
    }

    cout << "tests exited correctly \n";
    return 1;
}

//void genericTesting(){
//    // Initialise trajectory from csv and genrate A, B, steta and controls
//    std::string taskPrefix = activeModelTranslator->modelName;
//    MatrixXd startStateVector;
//    startStateVector.resize(activeModelTranslator->stateVectorSize, 1);
//
//    derivative_interpolator derivInterpolator = activeOptimiser->returnDerivativeInterpolator();
//    derivInterpolator.keypoint_method = "setInterval";
//    derivInterpolator.minN = 1;
//    derivInterpolator.maxN = 1;
//    activeOptimiser->setDerivativeInterpolator(derivInterpolator);
//
//
//    for(int i = 0; i < 100; i++){
//
//        yamlReader->loadTaskFromFile(taskPrefix, i, startStateVector, activeModelTranslator->X_desired);
//        activeModelTranslator->X_start = startStateVector;
//
//        cout << "start state " << activeModelTranslator->X_start << endl;
//        cout << "modelName: " << activeModelTranslator->modelName << endl;
//
//        activeModelTranslator->setStateVector(startStateVector, MASTER_RESET_DATA);
//        activeModelTranslator->activePhysicsSimulator->stepSimulator(5, MASTER_RESET_DATA);
//        if(activeModelTranslator->activePhysicsSimulator->checkIfDataIndexExists(0)){
//            activeModelTranslator->activePhysicsSimulator->copySystemState(0, MASTER_RESET_DATA);
//        }
//        else{
//
//            activeModelTranslator->activePhysicsSimulator->appendSystemStateToEnd(MASTER_RESET_DATA);
//        }
//
//        int setupHorizon = 1000;
//        int optHorizon = 2980;
//        std::vector<MatrixXd> initSetupControls = activeModelTranslator->createInitSetupControls(setupHorizon);
//        activeModelTranslator->activePhysicsSimulator->copySystemState(MASTER_RESET_DATA, MAIN_DATA_STATE);
//        std::vector<MatrixXd> initOptimisationControls = activeModelTranslator->createInitOptimisationControls(optHorizon);
//
//        activeOptimiser->horizonLength = optHorizon;
//        activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
//
//        activeOptimiser->rolloutTrajectory(0, true, initOptimisationControls);
//        activeOptimiser->generateDerivatives();
//
//        yamlReader->saveTrajecInfomation(activeOptimiser->A, activeOptimiser->B, activeOptimiser->X_old, activeOptimiser->U_old, activeModelTranslator->modelName, i, optHorizon);
//    }
//}
