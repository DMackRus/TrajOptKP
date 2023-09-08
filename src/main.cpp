#include "stdInclude.h"
#include "ros/ros.h"
#include "fileHandler.h"

// --------------------- different scenes -----------------------
#include "doublePendulum.h"
#include "acrobot.h"
#include "reaching.h"
#include "twoDPushing.h"
#include "boxFlick.h"
#include "locomotion.h"
#include "spherePush.h"
#include "boxSweep.h"

#include "visualizer.h"
#include "MuJoCoHelper.h"

#include "interpolated_iLQR.h"
#include "stomp.h"
#include "gradDescent.h"

// ------------ MODES OF OEPRATION -------------------------------
#define SHOW_INIT_CONTROLS          0
#define ILQR_ONCE                   1
#define MPC_CONTINOUS               2
#define MPC_UNTIL_COMPLETE          3
#define GENERATE_TEST_SCENES        4
#define GENERATE_TESTING_DATA       5
#define GENERATE_FILTERING_DATA     6
#define DEFAULT_KEYBOARD_CONTROL    7
#define GENERIC_TESTING             9

enum scenes{
    double_pendulum = 0,
    acrobot_swing = 1,
    reaching = 2,
    cylinder_pushing = 3,
    cylinder_pushing_mild_clutter = 4,
    cylinder_pushing_heavy_clutter = 5,
    cylinder_pushing_mild_clutter_constrained = 6,
    box_push_toppling = 7,
    box_flicking = 8,
    box_flicking_mild_clutter = 9,
    box_flicking_heavy_clutter = 10,
    walker = 11,
    sphere_push = 12,
    box_sweep = 13
};

// --------------------- Global class instances --------------------------------
std::shared_ptr<modelTranslator> activeModelTranslator;
std::shared_ptr<differentiator> activeDifferentiator;
std::shared_ptr<optimiser> activeOptimiser;
std::shared_ptr<interpolatediLQR> iLQROptimiser;
std::shared_ptr<stomp> stompOptimiser;
std::shared_ptr<gradDescent> gradDescentOptimiser;
std::shared_ptr<visualizer> activeVisualiser;
std::shared_ptr<fileHandler> yamlReader;

int task;
bool mpcVisualise = true;
bool playback = true;

void showInitControls();
void optimiseOnceandShow();
void MPCUntilComplete(double &trajecCost, double &avgHz, double &avgTimeGettingDerivs, double &avgPercentDerivs, double &avgTimeBP, double &avgTimeFP,
                      int MAX_TASK_TIME, int REPLAN_TIME, int OPT_HORIZON);
void MPCContinous();
void generateTestScenes();
void keyboardControl();

void generateTestingData_MPC();
void generateTestingData_MPCHorizons();
void generateTestingData();
void generateFilteringData();

void genericTesting();

int main(int argc, char **argv) {
    cout << "program started \n";
    // TODO - figure out what this does
//    omp_set_dynamic(0);     // Explicitly disable dynamic teams
    std::string optimiser;
    int mode;

    std::string taskInitMode;

    yamlReader = std::make_shared<fileHandler>();
    yamlReader->readSettingsFile("/generalConfig.yaml");
    optimiser = yamlReader->optimiser;
    mode = yamlReader->project_display_mode;
    task = yamlReader->taskNumber;
    taskInitMode = yamlReader->taskInitMode;

    MatrixXd startStateVector(1, 1);

    if(task == double_pendulum){
        std::shared_ptr<doublePendulum> myDoublePendulum = std::make_shared<doublePendulum>();
        activeModelTranslator = myDoublePendulum;
    }
    else if(task == acrobot_swing){
        std::shared_ptr<acrobot> myAcrobot = std::make_shared<acrobot>();
        activeModelTranslator = myAcrobot;

    }
    else if(task == reaching){
        std::shared_ptr<pandaReaching> myReaching = std::make_shared<pandaReaching>();
        activeModelTranslator = myReaching;
    }
    else if(task == cylinder_pushing){
        std::shared_ptr<twoDPushing> myTwoDPushing = std::make_shared<twoDPushing>(noClutter);
        activeModelTranslator = myTwoDPushing;

    }
    else if(task == cylinder_pushing_mild_clutter){
        std::shared_ptr<twoDPushing> myTwoDPushing = std::make_shared<twoDPushing>(lowClutter);
        activeModelTranslator = myTwoDPushing;

    }
    else if(task == cylinder_pushing_heavy_clutter){
        std::shared_ptr<twoDPushing> myTwoDPushing = std::make_shared<twoDPushing>(heavyClutter);
        activeModelTranslator = myTwoDPushing;

    }
    else if(task == cylinder_pushing_mild_clutter_constrained){
        std::shared_ptr<twoDPushing> myTwoDPushing = std::make_shared<twoDPushing>(constrainedClutter);
        activeModelTranslator = myTwoDPushing;
    }
    else if(task == box_push_toppling){
        cout << "not implemented task yet " << endl;
        return -1;
    }
    else if(task == box_flicking){
        std::shared_ptr<boxFlick> myBoxFlick = std::make_shared<boxFlick>(noClutter);
        activeModelTranslator = myBoxFlick;
    }
    else if(task == box_flicking_mild_clutter){
        std::shared_ptr<boxFlick> myBoxFlick = std::make_shared<boxFlick>(lowClutter);
        activeModelTranslator = myBoxFlick;
    }
    else if(task == box_flicking_heavy_clutter){
        std::shared_ptr<boxFlick> myBoxFlick = std::make_shared<boxFlick>(heavyClutter);
        activeModelTranslator = myBoxFlick;
    }
    else if(task == walker){
        std::shared_ptr<locomotion_anymal> myLocomotion = std::make_shared<locomotion_anymal>();
        activeModelTranslator = myLocomotion;
    }
    else if(task == sphere_push){
        std::shared_ptr<spherePush> mySpherePush = std::make_shared<spherePush>(noClutter);
        activeModelTranslator = mySpherePush;
    }
    else if(task == box_sweep){
        std::shared_ptr<boxSweep> myBoxSweep = std::make_shared<boxSweep>();
        activeModelTranslator = myBoxSweep;
    }
    else{
        std::cout << "invalid scene selected, exiting" << std::endl;
    }

    if(mode == GENERATE_TESTING_DATA){
        generateTestingData_MPCHorizons();
//        generateTestingData_MPC();
//        generateTestingData();
        return 1;
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

    //Instantiate my optimiser
    activeVisualiser = std::make_shared<visualizer>(activeModelTranslator);

//    activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
//    activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);
//    activeVisualiser->render("test");
//
//    activeModelTranslator->setStateVector(activeModelTranslator->X_desired, MAIN_DATA_STATE);
//    activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);
//    activeVisualiser->render("test");

    if(optimiser == "interpolated_iLQR"){
        yamlReader->readOptimisationSettingsFile(opt_iLQR);
        iLQROptimiser = std::make_shared<interpolatediLQR>(activeModelTranslator, activeModelTranslator->activePhysicsSimulator, activeDifferentiator, yamlReader->maxHorizon, activeVisualiser, yamlReader);
        activeOptimiser = iLQROptimiser;
    }
    else if(optimiser == "stomp"){
        yamlReader->readOptimisationSettingsFile(opt_stomp);
        stompOptimiser = std::make_shared<stomp>(activeModelTranslator, activeModelTranslator->activePhysicsSimulator, yamlReader, activeDifferentiator, yamlReader->maxHorizon, 8);
        activeOptimiser = stompOptimiser;
    }
    else if(optimiser == "gradDescent"){
        yamlReader->readOptimisationSettingsFile(opt_gradDescent);
        gradDescentOptimiser = std::make_shared<gradDescent>(activeModelTranslator, activeModelTranslator->activePhysicsSimulator, activeDifferentiator, activeVisualiser, yamlReader->maxHorizon, yamlReader);
        activeOptimiser = gradDescentOptimiser;
    }
    else{
        cout << "invalid optimiser selected, exiting" << endl;
        return -1;
    }

    if(mode == SHOW_INIT_CONTROLS){
        cout << "SHOWING INIT CONTROLS MODE \n";
        showInitControls();
    }
    else if(mode == ILQR_ONCE){
        cout << "OPTIMISE TRAJECTORY ONCE AND DISPLAY MODE \n";
        activeOptimiser->verboseOutput = true;
        optimiseOnceandShow();
    }
    else if(mode == MPC_CONTINOUS){
        cout << "CONTINOUS MPC MODE \n";
        cout << "mode is disabled for now \n";
        return 0;
//        MPCContinous();
    }
    else if(mode == MPC_UNTIL_COMPLETE){
        cout << "MPC UNTIL TASK COMPLETE MODE \n";
        double trajecCost, avgHz;
        double avgPercentDerivs, avgTimeDerivs, avgTimeBP, avgTimeFP;
        activeOptimiser->setTrajecNumber(1000);
        std::vector<MatrixXd> initSetupControls = activeModelTranslator->createInitSetupControls(1000);
        activeModelTranslator->activePhysicsSimulator->copySystemState(MASTER_RESET_DATA, MAIN_DATA_STATE);
        activeOptimiser->verboseOutput = true;
        mpcVisualise = true;

        // No clutter - 1800 - 500 - 1800

        if(task == walker){
            activeModelTranslator->X_desired(10) = 0.25;
        }

        cout << "X_desired: " << activeModelTranslator->X_desired << endl;
        MPCUntilComplete(trajecCost, avgHz, avgPercentDerivs, avgTimeDerivs, avgTimeBP, avgTimeFP, 1200, 1, 50);
    }
    else if(mode == GENERATE_TEST_SCENES){
        cout << "TASK INIT MODE \n";
        generateTestScenes();
    }
    else if(mode == GENERATE_FILTERING_DATA){
        cout << "GENERATE FILTERING DATA MODE \n";
        generateFilteringData();
    }
    else if(mode == DEFAULT_KEYBOARD_CONTROL){
        cout << "KEYBOARD TESTING MODE \n";
        keyboardControl();
    }
    else if(mode == GENERIC_TESTING){
        genericTesting();
    }
    else{
        cout << "INVALID MODE OF OPERATION OF PROGRAM \n";
    }
    return 0;
}

void genericTesting(){
    // Initialise trajectory from csv and genrate A, B, steta and controls
    std::string taskPrefix = activeModelTranslator->modelName;
    MatrixXd startStateVector;
    startStateVector.resize(activeModelTranslator->stateVectorSize, 1);

    derivative_interpolator derivInterpolator = activeOptimiser->returnDerivativeInterpolator();
    derivInterpolator.keypoint_method = "setInterval";
    derivInterpolator.minN = 1;
    derivInterpolator.maxN = 1;
    activeOptimiser->setDerivativeInterpolator(derivInterpolator);


    for(int i = 0; i < 100; i++){

        yamlReader->loadTaskFromFile(taskPrefix, i, startStateVector, activeModelTranslator->X_desired);
        activeModelTranslator->X_start = startStateVector;

        cout << "start state " << activeModelTranslator->X_start << endl;
        cout << "modelName: " << activeModelTranslator->modelName << endl;

        activeModelTranslator->setStateVector(startStateVector, MASTER_RESET_DATA);
        activeModelTranslator->activePhysicsSimulator->stepSimulator(5, MASTER_RESET_DATA);
        if(activeModelTranslator->activePhysicsSimulator->checkIfDataIndexExists(0)){
            activeModelTranslator->activePhysicsSimulator->copySystemState(0, MASTER_RESET_DATA);
        }
        else{

            activeModelTranslator->activePhysicsSimulator->appendSystemStateToEnd(MASTER_RESET_DATA);
        }

        int setupHorizon = 1000;
        int optHorizon = 2980;
        std::vector<MatrixXd> initSetupControls = activeModelTranslator->createInitSetupControls(setupHorizon);
        activeModelTranslator->activePhysicsSimulator->copySystemState(MASTER_RESET_DATA, MAIN_DATA_STATE);
        std::vector<MatrixXd> initOptimisationControls = activeModelTranslator->createInitOptimisationControls(optHorizon);

        activeOptimiser->horizonLength = optHorizon;
        activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);

        activeOptimiser->rolloutTrajectory(0, true, initOptimisationControls);
        activeOptimiser->generateDerivatives();

        yamlReader->saveTrajecInfomation(activeOptimiser->A, activeOptimiser->B, activeOptimiser->X_old, activeOptimiser->U_old, activeModelTranslator->modelName, i, optHorizon);
    }
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

    std::vector<std::string> methodNames = {"baseline", "SI5", "SI1000", "adaptive_jerk_5", "iterative_error_5", "magvel_change_5"};
    int numMethods = methodNames.size();
    std::vector<string> keyPointMethods = {"setInterval", "setInterval", "setInterval", "adaptive_jerk", "iterative_error", "magvel_change"};
    std::vector<int> minN = {1, 5, 1000, 5, 5, 5};
    std::vector<int> maxN = {1, 5, 1000, 100, 100, 100};

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

            yamlReader->saveTrajecInfomation(activeOptimiser->A, activeOptimiser->B, activeOptimiser->X_old, activeOptimiser->U_old, activeModelTranslator->modelName, i, optHorizon);

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

//    int configs[3] = {noClutter, lowClutter, heavyClutter};
    int configs[1] = {heavyClutter};


    // start timer here
    auto startTime = std::chrono::high_resolution_clock::now();

    activeDifferentiator = std::make_shared<differentiator>(activeModelTranslator, activeModelTranslator->myHelper);

    MatrixXd startStateVector;
    startStateVector.resize(activeModelTranslator->stateVectorSize, 1);
    startStateVector = activeModelTranslator->X_start;
    activeModelTranslator->setStateVector(startStateVector, MASTER_RESET_DATA);
    activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);
    activeModelTranslator->activePhysicsSimulator->appendSystemStateToEnd(MAIN_DATA_STATE);

    activeVisualiser = std::make_shared<visualizer>(activeModelTranslator);
    yamlReader->readOptimisationSettingsFile(opt_iLQR);
    iLQROptimiser = std::make_shared<interpolatediLQR>(activeModelTranslator, activeModelTranslator->activePhysicsSimulator, activeDifferentiator, yamlReader->maxHorizon, activeVisualiser, yamlReader);
    activeOptimiser = iLQROptimiser;

    cout << "before one task \n";
    onetaskGenerateTestingData();



//    for(int i = 0; i < 1; i ++){
////        std::shared_ptr<pandaReaching> myPandaReaching = std::make_shared<pandaReaching>();
////        activeModelTranslator = myPandaReaching;
////        std::shared_ptr<doublePendulum> myDoublePendulum = std::make_shared<doublePendulum>();
////        activeModelTranslator = myDoublePendulum;
//
//
//    }

//    for(int i = 0; i < 1; i ++){
////        twoDPushing *myTwoDPushing = new twoDPushing(heavyClutter);
//        std::shared_ptr<twoDPushing> myTwoDPushing = std::make_shared<twoDPushing>(noClutter);
//        activeModelTranslator = myTwoDPushing;
//        activeDifferentiator = std::make_shared<differentiator>(activeModelTranslator, activeModelTranslator->myHelper);
//
//        MatrixXd startStateVector;
//        startStateVector.resize(activeModelTranslator->stateVectorSize, 1);
//        startStateVector = activeModelTranslator->X_start;
//        activeModelTranslator->setStateVector(startStateVector, MAIN_DATA_STATE);
//        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);
//        activeModelTranslator->activePhysicsSimulator->appendSystemStateToEnd(MAIN_DATA_STATE);
//
//        activeVisualiser = std::make_shared<visualizer>(activeModelTranslator);
//        yamlReader->readOptimisationSettingsFile(opt_iLQR);
//        iLQROptimiser = std::make_shared<interpolatediLQR>(activeModelTranslator, activeModelTranslator->activePhysicsSimulator, activeDifferentiator, yamlReader->maxHorizon, activeVisualiser, yamlReader);
//        activeOptimiser = iLQROptimiser;
//
//        onetaskGenerateTestingData();
//
//    }

//    for(int i = 0; i < 3; i ++){
//        boxFlick *myBoxFlicking = new boxFlick(configs[i]);
//        activeModelTranslator = myBoxFlicking;
//        activeDifferentiator = new differentiator(activeModelTranslator, activeModelTranslator->myHelper);
//
//        MatrixXd startStateVector;
//        startStateVector.resize(activeModelTranslator->stateVectorSize, 1);
//        startStateVector = activeModelTranslator->X_start;
//        activeModelTranslator->setStateVector(startStateVector, MAIN_DATA_STATE);
//        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);
//        activeModelTranslator->activePhysicsSimulator->appendSystemStateToEnd(MAIN_DATA_STATE);
//
//        activeVisualiser = new visualizer(activeModelTranslator);
//        yamlReader->readOptimisationSettingsFile(opt_iLQR);
//        iLQROptimiser = new interpolatediLQR(activeModelTranslator, activeModelTranslator->activePhysicsSimulator, activeDifferentiator, yamlReader->maxHorizon, activeVisualiser, yamlReader);
//        activeOptimiser = iLQROptimiser;
//
//        onetaskGenerateTestingData();
//
//    }

    auto stopTimer = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stopTimer - startTime);
    double timeForTesting = duration.count() / 1000.0;
    timeForTesting = timeForTesting / 3600.0;
    cout << "time taken: " << timeForTesting << " h" << endl;
//    double predictedTimeFor100Scenes = timeForTesting * (100.0 / 3600.0);
//    cout << "predicted time taken: " << predictedTimeFor100Scenes << " h" << endl;
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
    char* label = "Final controls";

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
                label = "Final controls";
            }
            else{
                label = "Init Controls";
            }
        }

        if(visualCounter > 5){
            visualCounter = 0;
            activeVisualiser->render(label);
        }
    }
}

void MPCContinous(){
    int horizon = 200;
    bool taskComplete = false;
    int currentControlCounter = 0;
    int visualCounter = 0;
    int overallTaskCounter = 0;
    int reInitialiseCounter = 0;
    const char* label = "MPC Continous";

    // Instantiate init controls
    std::vector<MatrixXd> initControls;
    initControls = activeModelTranslator->createInitOptimisationControls(horizon);
    activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

    std::vector<MatrixXd> optimisedControls = activeOptimiser->optimise(0, initControls, yamlReader->maxIter, yamlReader->minIter, horizon);
    MatrixXd initState = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);
    cout << "init state in MPC continous: " << initState << endl;

    while(!taskComplete){
        MatrixXd nextControl = optimisedControls[0].replicate(1, 1);

        optimisedControls.erase(optimisedControls.begin());

        optimisedControls.push_back(optimisedControls.at(optimisedControls.size() - 1));

        activeModelTranslator->setControlVector(nextControl, MAIN_DATA_STATE);

        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

        reInitialiseCounter++;
        visualCounter++;

        if(reInitialiseCounter > 10){
            //initControls = activeModelTranslator->createInitControls(horizon);
            optimisedControls = activeOptimiser->optimise(MAIN_DATA_STATE, optimisedControls, yamlReader->maxIter, yamlReader->minIter, horizon);
            //initState = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);
            //cout << "init state in MPC continous: " << initState << endl;
            reInitialiseCounter = 0;
        }

        if(visualCounter > 5){
            activeVisualiser->render(label);
            visualCounter = 0;
        }
    }
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

    optimisedControls = activeOptimiser->optimise(0, initOptimisationControls, 5, 4, OPT_HORIZON);

    while(!taskComplete){
        MatrixXd nextControl = optimisedControls[0].replicate(1, 1);
        activeVisualiser->replayControls.push_back(nextControl.replicate(1, 1));

        optimisedControls.erase(optimisedControls.begin());

        optimisedControls.push_back(optimisedControls.at(optimisedControls.size() - 1));

        activeModelTranslator->setControlVector(nextControl, MAIN_DATA_STATE);

        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

        reInitialiseCounter++;
        visualCounter++;

        if(reInitialiseCounter >= REPLAN_TIME){
            activeModelTranslator->activePhysicsSimulator->copySystemState(0, MAIN_DATA_STATE);

            optimisedControls = activeOptimiser->optimise(0, optimisedControls, 1, 1, OPT_HORIZON);
            reInitialiseCounter = 0;
//            yamlReader->saveTrajecInfomation(activeOptimiser->A, activeOptimiser->B,
//                                             activeOptimiser->X_old, activeOptimiser->U_old, activeModelTranslator->modelName,
//                                             overallTaskCounter, horizon);

//                totalOptimisationTime += activeOptimiser->optTime / 1000.0f;
                timeGettingDerivs.push_back(activeOptimiser->avgTime_getDerivs_ms);
                timeBackwardsPass.push_back(activeOptimiser->avgTime_backwardsPass_ms);
                timeForwardsPass.push_back(activeOptimiser->avgTime_forwardsPass_ms);
                percentagesDerivsCalculated.push_back(activeOptimiser->avgPercentDerivs);

        }

        if(mpcVisualise){
            if(visualCounter > 10){
                activeVisualiser->render(label);
                visualCounter = 0;
            }
        }

        overallTaskCounter++;
//        cout << "overall task counter: " << overallTaskCounter << endl;

        if(overallTaskCounter >= MAX_TASK_TIME){
            cout << "task time out" << endl;
            taskComplete = true;
        }
    }

    trajecCost = 0.0f;
    activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
    for(int i = 0; i < activeVisualiser->replayControls.size(); i++){
        MatrixXd nextControl = activeVisualiser->replayControls[i].replicate(1, 1);
        double stateCost = activeModelTranslator->costFunction(MAIN_DATA_STATE, false);
        trajecCost += stateCost * activeModelTranslator->activePhysicsSimulator->returnModelTimeStep();

        activeModelTranslator->setControlVector(nextControl, MAIN_DATA_STATE);
        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

    }

    cout << "trajec cost: " << trajecCost << endl;
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

    if(playback){
        while(activeVisualiser->windowOpen()){

            if(activeVisualiser->replayTriggered){
                activeVisualiser->replayTriggered = false;

                activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
                int controlCounter = 0;
                while(controlCounter < activeVisualiser->replayControls.size()){
                    MatrixXd nextControl = activeVisualiser->replayControls[controlCounter].replicate(1, 1);
                    double stateCost = activeModelTranslator->costFunction(MAIN_DATA_STATE, false);

                    activeModelTranslator->setControlVector(nextControl, MAIN_DATA_STATE);

                    activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

                    controlCounter++;

                    if(controlCounter % 5 == 0){
                        activeVisualiser->render("replaying");
                    }
                }
            }
            activeVisualiser->render("replay_mode");
        }
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
        yamlReader->readOptimisationSettingsFile(opt_iLQR);
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

        std::vector<std::string> methodNames = {"baseline", "SI5", "SI20", "adapJerk", "iter_error", "magvel"};
        int numMethods = methodNames.size();
        std::vector<string> keypointMethods = {"setInterval", "setInterval", "setInterval", "adaptive_jerk", "iterative_error", "magvel_change"};
        std::vector<int> minN = {1, 5, 20, 1, 1, 1};
        std::vector<int> maxN = {1, 2, 5, 5, 5, 5};

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
            if(task == walker){
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

void generateTestingData_MPCHorizons(){
    playback = false;
    mpcVisualise = false;

    // start timer here
    auto startTime = std::chrono::high_resolution_clock::now();

    activeDifferentiator = std::make_shared<differentiator>(activeModelTranslator, activeModelTranslator->myHelper);

    activeVisualiser = std::make_shared<visualizer>(activeModelTranslator);
    yamlReader->readOptimisationSettingsFile(opt_iLQR);
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

    std::vector<int> horizons = {5, 10, 20, 30, 40, 50, 60};
    std::vector<std::string> horizonNames;
    int numHorizons = horizons.size();

    for(int i = 0; i < horizons.size(); i++){
        horizonNames.push_back(std::to_string(horizons[i]));
    }

//    std::vector<std::string> methodNames = {"baseline", "SI5", "SI10", "adaptive_jerk", "iterative_error", "magvel_change"};
//    int numMethods = methodNames.size();
//    std::vector<int> minN = {1, 5, 10, 1, 1, 1};
//    std::vector<int> maxN = {1, 5, 10, 5, 5, 5};
//    std::vector<std::string> keypoint_method = {"setInterval", "setInterval", "setInterval", "adaptive_jerk", "iterative_error", "magvel_change"};

    std::vector<std::string> methodNames = {"iterative_error"};
    int numMethods = methodNames.size();
    std::vector<int> minN = {1};
    std::vector<int> maxN = {5};
    std::vector<std::string> keypoint_method = {"iterative_error"};

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

    for(int k = 0; k < numMethods; k++) {
        cout << "---------- current method " << methodNames[k] << " ----------------" << endl;

        derivative_interpolator currentInterpolator = activeOptimiser->returnDerivativeInterpolator();
        currentInterpolator.minN = minN[k];
        currentInterpolator.maxN = maxN[k];
        currentInterpolator.keypoint_method = keypoint_method[k];
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
            if(task == walker){
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
                MPCUntilComplete(finalCost, avgHz, avgTimeForDerivs, avgPercentageDerivs, avgTimeBP, avgTimeFP, 1200, 1, horizons[j]);

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
        cout << "save data to file for " << methodNames[k] << endl;
        std::string taskPrefix = activeModelTranslator->modelName + "_" + methodNames[k];
        yamlReader->saveResultsData_MPC(taskPrefix, horizonNames, finalCosts, avgHzs,
                                        avgTimeForDerivs, avgTimeBP, avgTimeFP, avgPercentDerivs);
    }
}

void keyboardControl(){

    MatrixXd startStateVector;
    startStateVector.resize(activeModelTranslator->stateVectorSize, 1);
    int dof = activeModelTranslator->stateVectorSize / 2;

//    startStateVector << 0, 1, 0, 0.3, 0, -0.3, -0.3, 0, 0.3,
//            0, 0, 0, 0, 0, 0, 0, 0, 0;
//
//
//
//    activeModelTranslator->X_start = startStateVector;
//    activeModelTranslator->setStateVector(startStateVector, MASTER_RESET_DATA);
//    activeModelTranslator->activePhysicsSimulator->forwardSimulator(MASTER_RESET_DATA);

//    double cost = activeModelTranslator->costFunction(MASTER_RESET_DATA, false);
//
//    startStateVector << 0, 0, 0, 0, 0, 0,
//            0, 0, 0, 0, 0, 0;
//
//    activeModelTranslator->setStateVector(startStateVector, MASTER_RESET_DATA);
//
//    activeModelTranslator->activePhysicsSimulator->forwardSimulator(MASTER_RESET_DATA);
//
//    double cost2 = activeModelTranslator->costFunction(MASTER_RESET_DATA, false);

    activeModelTranslator->activePhysicsSimulator->stepSimulator(5, MASTER_RESET_DATA);
    activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);

    activeModelTranslator->activePhysicsSimulator->forwardSimulator(MAIN_DATA_STATE);

    while(activeVisualiser->windowOpen()){
        vector<double> gravCompensation;
//        MatrixXd returncontrol = activeModelTranslator->returnControlVector(MAIN_DATA_STATE);
//        cout << "return control: " << returncontrol << endl;

        activeVisualiser->render("keyboard control");
    }
}