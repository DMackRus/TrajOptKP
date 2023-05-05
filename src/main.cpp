#include "stdInclude.h"
#include "ros/ros.h"
#include "fileHandler.h"

// --------------------- different scenes -----------------------
#include "doublePendulum.h"
#include "reaching.h"
#include "twoDPushing.h"
#include "twoDPushingClutter.h"

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
#define DEFAULT_KEYBOARD_CONTROL    6

enum scenes{
    pendulum = 0,
    reaching = 1,
    twoReaching = 2,
    cylinderPushing = 3,
    threeDPushing = 4,
    boxFlicking = 5,
    cylinderPushingClutter = 6
};

// --------------------- Global class instances --------------------------------
modelTranslator *activeModelTranslator;
differentiator *activeDifferentiator;
optimiser *activeOptimiser;
interpolatediLQR *iLQROptimiser;
stomp *stompOptimiser;
gradDescent *gradDescentOptimiser;
visualizer *activeVisualiser;
fileHandler *yamlReader;

int interpolationMethod = linear;
int keyPointMethod = iterative_error;

void showInitControls();
void iLQROnce();
void MPCUntilComplete();
void MPCContinous();
void generateTestScenes();
void keyboardControl();
void generateTestingData();

int main(int argc, char **argv) {
    cout << "program started \n";
    std::string optimiser;
    int mode;
    int task;
    std::string taskInitMode;

    yamlReader = new fileHandler();
    yamlReader->readSettingsFile("/generalConfig.yaml");
    optimiser = yamlReader->optimiser;
    mode = yamlReader->project_display_mode;
    task = yamlReader->taskNumber;
    taskInitMode = yamlReader->taskInitMode;

    MatrixXd startStateVector(1, 1);

    if(task == pendulum){
        doublePendulum *myDoublePendulum = new doublePendulum();
        activeModelTranslator = myDoublePendulum;
    }
    else if(task == reaching){
        // std::cout << "before creating reaching problem" << std::endl;
        pandaReaching *myReaching = new pandaReaching();
        activeModelTranslator = myReaching;
    }
    else if(task == twoReaching){
        cout << "not implemented task yet " << endl;
        return -1;
    }
    else if(task == cylinderPushing){
        twoDPushing *myTwoDPushing = new twoDPushing();
        activeModelTranslator = myTwoDPushing;

    }
    else if(task == threeDPushing){
        cout << "not implemented task yet " << endl;
        return -1;

    }
    else if(task == boxFlicking){
        cout << "not implemented task yet " << endl;
        return -1;
    }
    else if(task == cylinderPushingClutter){
        twoDPushingClutter *myTwoDPushingClutter = new twoDPushingClutter();
        activeModelTranslator = myTwoDPushingClutter;
    }
    else{
        std::cout << "invalid scene selected, exiting" << std::endl;
    }

    startStateVector.resize(activeModelTranslator->stateVectorSize, 1);
    startStateVector = activeModelTranslator->X_start;

    // random start and goal state
    std::string taskPrefix = activeModelTranslator->modelName;
    if(taskInitMode == "random"){
        startStateVector = activeModelTranslator->returnRandomStartState();
        activeModelTranslator->X_start = startStateVector;
        activeModelTranslator->X_desired = activeModelTranslator->returnRandomGoalState(startStateVector);

        yamlReader->saveTaskToFile(taskPrefix, 0, activeModelTranslator->X_start, activeModelTranslator->X_desired);
    }
    else if(taskInitMode == "fromCSV"){
        yamlReader->loadTaskFromFile(taskPrefix, yamlReader->csvRow, startStateVector, activeModelTranslator->X_desired);
        activeModelTranslator->X_start = startStateVector;
    }

    activeDifferentiator = new differentiator(activeModelTranslator, activeModelTranslator->myHelper);
    activeModelTranslator->setStateVector(startStateVector, MAIN_DATA_STATE);
    cout << "starting state: " << startStateVector << endl;
    cout << "desired state: " << activeModelTranslator->X_desired << endl;

    //Instantiate my optimiser
    activeVisualiser = new visualizer(activeModelTranslator);

    if(optimiser == "interpolated_iLQR"){
        yamlReader->readOptimisationSettingsFile(opt_iLQR);
        iLQROptimiser = new interpolatediLQR(activeModelTranslator, activeModelTranslator->activePhysicsSimulator, activeDifferentiator, yamlReader->maxHorizon, activeVisualiser, yamlReader);
        activeOptimiser = iLQROptimiser;
    }
    else if(optimiser == "stomp"){
        yamlReader->readOptimisationSettingsFile(opt_stomp);
        stompOptimiser = new stomp(activeModelTranslator, activeModelTranslator->activePhysicsSimulator, yamlReader->maxHorizon, 50);
        activeOptimiser = stompOptimiser;
    }
    else if(optimiser == "gradDescent"){
        yamlReader->readOptimisationSettingsFile(opt_gradDescent);
        gradDescentOptimiser = new gradDescent(activeModelTranslator, activeModelTranslator->activePhysicsSimulator, activeDifferentiator, activeVisualiser, yamlReader->maxHorizon, yamlReader);
        activeOptimiser = gradDescentOptimiser;
    }
    else{
        cout << "invalid optimiser selected, exiting" << endl;
        return -1;
    }
    
    activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);
    activeModelTranslator->activePhysicsSimulator->appendSystemStateToEnd(MAIN_DATA_STATE);

    if(mode == SHOW_INIT_CONTROLS){
        cout << "SHOWING INIT CONTROLS MODE \n";
        showInitControls();
    }
    else if(mode == ILQR_ONCE){
        cout << "OPTIMISE TRAJECTORY ONCE AND DISPLAY MODE \n";
        iLQROnce();
    }
    else if(mode == MPC_CONTINOUS){
        cout << "CONTINOUS MPC MODE \n";
        MPCContinous();
    }
    else if(mode == MPC_UNTIL_COMPLETE){
        cout << "MPC UNTIL TASK COMPLETE MODE \n";
        MPCUntilComplete();
    }
    else if(mode == GENERATE_TEST_SCENES){
        cout << "TASK INIT MODE \n";
        generateTestScenes();
    }
    else if(mode == GENERATE_TESTING_DATA){
        cout << "GENERATE TESTING DATA MODE \n";
        generateTestingData();
    }
    else if(mode == DEFAULT_KEYBOARD_CONTROL){
        cout << "KEYBOARD TESTING MODE \n";
        keyboardControl();
    }
    else{
        cout << "INVALID MODE OF OPERATION OF PROGRAM \n";
    }
    return 0;
}

void generateTestingData(){
    int setupHorizon = 1000;
    int optHorizon = 1500;

    MatrixXd startStateVector;
    startStateVector.resize(activeModelTranslator->stateVectorSize, 1);

    for(int i = 0; i < 50; i++){
        // Load a task from saved tasks

        yamlReader->loadTaskFromFile(activeModelTranslator->modelName, i, startStateVector, activeModelTranslator->X_desired);
        activeModelTranslator->X_start = startStateVector;
        cout << "starting state: " << startStateVector << endl;
        cout << "desired state: " << activeModelTranslator->X_desired << endl;
        activeModelTranslator->setStateVector(startStateVector, MAIN_DATA_STATE);
        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

        // Load optimiser

        // Reset optimisers variables as required
        activeOptimiser->setupTestingExtras(i, interpolationMethod, keyPointMethod);

        // Generate init controls
        std::vector<MatrixXd> initControls;
        std::vector<MatrixXd> finalControls;

        activeModelTranslator->activePhysicsSimulator->copySystemState(MASTER_RESET_DATA, MAIN_DATA_STATE);
        std::vector<MatrixXd> initSetupControls = activeModelTranslator->createInitSetupControls(setupHorizon);
        activeModelTranslator->activePhysicsSimulator->copySystemState(0, MAIN_DATA_STATE);
        std::vector<MatrixXd> initOptimisationControls = activeModelTranslator->createInitOptimisationControls(optHorizon);
        activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

        auto start = high_resolution_clock::now();
        std::vector<MatrixXd> optimisedControls = activeOptimiser->optimise(MAIN_DATA_STATE, initOptimisationControls, yamlReader->maxIter, yamlReader->minIter, optHorizon);
        auto stop = high_resolution_clock::now();
        auto linDuration = duration_cast<microseconds>(stop - start);
        cout << "iLQR once took: " << linDuration.count() / 1000000.0f << " s\n";

        initControls.insert(initControls.end(), initSetupControls.begin(), initSetupControls.end());
        initControls.insert(initControls.end(), initOptimisationControls.begin(), initOptimisationControls.end());
        finalControls.insert(finalControls.end(), initSetupControls.begin(), initSetupControls.end());
        finalControls.insert(finalControls.end(), optimisedControls.begin(), optimisedControls.end());

        activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);

        int controlCounter = 0;
        int visualCounter = 0;
        cout << "final controls size: " << finalControls.size() << endl;

//        while(controlCounter < finalControls.size()){
//
//            activeModelTranslator->setControlVector(initControls[controlCounter], MAIN_DATA_STATE);
//
//            activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);
//
//            controlCounter++;
//            visualCounter++;
//
//            if(visualCounter > 5){
//
//                activeVisualiser->render("show init controls");
//                visualCounter = 0;
//            }
//        }

    }
}

void generateTestScenes(){
    for(int i = 0; i < 50; i++){
        MatrixXd startStateVector = activeModelTranslator->returnRandomStartState();
        activeModelTranslator->X_start = startStateVector;
        activeModelTranslator->X_desired = activeModelTranslator->returnRandomGoalState(startStateVector);
        yamlReader->saveTaskToFile(activeModelTranslator->modelName, i, activeModelTranslator->X_start, activeModelTranslator->X_desired);
    }
}

void showInitControls(){
    int setupHorizon = 1000;
    int optHorizon = 2000;
    int controlCounter = 0;
    int visualCounter = 0;

    std::vector<MatrixXd> initControls;

    activeModelTranslator->activePhysicsSimulator->copySystemState(MASTER_RESET_DATA, MAIN_DATA_STATE);
    std::vector<MatrixXd> initSetupControls = activeModelTranslator->createInitSetupControls(setupHorizon);
    activeModelTranslator->activePhysicsSimulator->copySystemState(0, MAIN_DATA_STATE);
    std::vector<MatrixXd> initOptimisationControls = activeModelTranslator->createInitOptimisationControls(optHorizon);
    activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);

    //Stitch setup and optimisation controls together
    initControls.insert(initControls.end(), initSetupControls.begin(), initSetupControls.end());
    initControls.insert(initControls.end(), initOptimisationControls.begin(), initOptimisationControls.end());


    while(activeVisualiser->windowOpen()){

        activeModelTranslator->setControlVector(initControls[controlCounter], MAIN_DATA_STATE);

        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

        controlCounter++;
        visualCounter++;

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

void iLQROnce(){
    int setupHorizon = 1000;
    int optHorizon = 2200;
    int controlCounter = 0;
    int visualCounter = 0;
    bool showFinalControls = true;
    char* label = "Final controls";

    std::vector<MatrixXd> initControls;
    std::vector<MatrixXd> finalControls;

    activeModelTranslator->activePhysicsSimulator->copySystemState(MASTER_RESET_DATA, MAIN_DATA_STATE);
    std::vector<MatrixXd> initSetupControls = activeModelTranslator->createInitSetupControls(setupHorizon);
    activeModelTranslator->activePhysicsSimulator->copySystemState(0, MAIN_DATA_STATE);
    std::vector<MatrixXd> initOptimisationControls = activeModelTranslator->createInitOptimisationControls(optHorizon);
    activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);

    activeOptimiser->setupTestingExtras(1000, interpolationMethod, keyPointMethod);

    activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);
    auto start = high_resolution_clock::now();
    std::vector<MatrixXd> optimisedControls = activeOptimiser->optimise(0, initOptimisationControls, yamlReader->maxIter, yamlReader->minIter, optHorizon);
    auto stop = high_resolution_clock::now();
    auto linDuration = duration_cast<microseconds>(stop - start);
    cout << "iLQR once took: " << linDuration.count() / 1000000.0f << " ms\n";

    // Stitch together setup controls with init control + optimised controls
    initControls.insert(initControls.end(), initSetupControls.begin(), initSetupControls.end());
    initControls.insert(initControls.end(), initOptimisationControls.begin(), initOptimisationControls.end());
    finalControls.insert(finalControls.end(), initSetupControls.begin(), initSetupControls.end());
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
    int horizon = 500;
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

        if(reInitialiseCounter > 50){
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
    //finalControls = optimiser->optimise(d_init, initControls, 2, MUJ_STEPS_HORIZON_LENGTH, 5, predicted_States, K_feedback);



    // Initialise optimiser - creates all the data objects
    // cout << "X desired: " << modelTranslator->X_desired << endl;
    // optimiser->updateNumStepsPerDeriv(5);
    

    // cpMjData(model, mdata, d_init);
    // cpMjData(model, d_init_master, d_init);

    // auto MPCStart = high_resolution_clock::now();

    // while(!taskComplete){

    //     if(movingToStart){

    //     }
    //     else{

    //     }

    //     m_ctrl nextControl = finalControls.at(0);
    //     // Delete control we have applied
    //     finalControls.erase(finalControls.begin());
    //     // add control to back - replicate last control for now
    //     finalControls.push_back(finalControls.at(finalControls.size() - 1));

    //     // Store applied control in a std::vector for re-playability
    //     MPCControls.push_back(nextControl);
    //     modelTranslator->setControls(mdata, nextControl, false);
    //     modelTranslator->stepModel(mdata, 1);
    //     currentControlCounter++;
    //     overallTaskCounter++;
    //     reInitialiseCounter++;

    //     // check if problem is solved?
    //     if(modelTranslator->taskCompleted(mdata)){
    //         cout << "task completed" << endl;
    //         taskComplete = true;
    //     }

    //     // timeout of problem solution
    //     if(overallTaskCounter > 3000){
    //         cout << "task timeout" << endl;
    //         taskComplete = true;
    //     }

    //     // State we predicted we would be at at this point. TODO - always true at the moment as no noise in system
    //     m_state predictedState = modelTranslator->returnState(mdata);
    //     //Check states mismatched
    //     bool replanNeeded = false;
    //     // todo - fix this!!!!!!!!!!!!!!!!!
    //     if(modelTranslator->predictiveStateMismatch(mdata, mdata)){
    //         replanNeeded = true;
    //     }

    //     if(currentControlCounter > 300){
    //         replanNeeded = true;
    //         currentControlCounter = 0;

    //     }

    //     if(replanNeeded){
    //         cpMjData(model, d_init, mdata);
    //         if(modelTranslator->newControlInitialisationNeeded(d_init, reInitialiseCounter)){
    //             cout << "re initialise needed" << endl;
    //             reInitialiseCounter = 0;
    //             initControls = modelTranslator->initOptimisationControls(mdata, d_init);
    //             finalControls = optimiser->optimise(d_init, initControls, 2, MUJ_STEPS_HORIZON_LENGTH, 5, predicted_States, K_feedback);
    //         }
    //         else{
    //             finalControls = optimiser->optimise(d_init, finalControls, 2, MUJ_STEPS_HORIZON_LENGTH, 5, predicted_States, K_feedback);
    //         }
    //     }

    //     visualCounter++;
    //     if(visualCounter >= 20){
    //         renderOnce(mdata);
    //         visualCounter = 0;
    //     }
    // }

    // auto MPCStop = high_resolution_clock::now();
    // auto MPCDuration = duration_cast<microseconds>(MPCStop - MPCStart);
    // float trajecTime = MPCControls.size() * MUJOCO_DT;
    // cout << "duration of MPC was: " << MPCDuration.count()/1000 << " ms. Trajec length of " << trajecTime << " s" << endl;

    // renderMPCAfter();
}

void MPCUntilComplete(){
    int setupHorizon = 1000;
    int optHorizon = 100;
    bool taskComplete = false;
    int currentControlCounter = 0;
    int visualCounter = 0;
    int overallTaskCounter = 0;
    int reInitialiseCounter = 0;
    const char* label = "MPC until complete";
    bool applyingSetupControls = false;

    std::vector<MatrixXd> setupControls;
    std::vector<MatrixXd> optimisedControls;

    // Instantiate init controls
    std::vector<MatrixXd> initOptimisationControls;
    activeModelTranslator->activePhysicsSimulator->copySystemState(0, MAIN_DATA_STATE);
    activeModelTranslator->activePhysicsSimulator->copySystemState(MASTER_RESET_DATA, MAIN_DATA_STATE);
    setupControls = activeModelTranslator->createInitSetupControls(setupHorizon);
    activeModelTranslator->activePhysicsSimulator->copySystemState(0, MAIN_DATA_STATE);
    if(setupControls.size() > 0){
        applyingSetupControls = true;
    }
    else{
        activeModelTranslator->activePhysicsSimulator->copySystemState(0, MAIN_DATA_STATE);
        initOptimisationControls = activeModelTranslator->createInitOptimisationControls(optHorizon);
        activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

        cout << "init controls: " << initOptimisationControls.size() << endl;
        optimisedControls = activeOptimiser->optimise(0, initOptimisationControls, yamlReader->maxIter, yamlReader->minIter, optHorizon);
        activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);
    }

    while(!taskComplete){
        if(applyingSetupControls){
            activeModelTranslator->activePhysicsSimulator->copySystemState(0, MAIN_DATA_STATE);
            std::vector<MatrixXd> setupControls = activeModelTranslator->createInitSetupControls(setupHorizon);
            activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

            while(setupControls.size() > 0){
                MatrixXd nextControl = setupControls[0].replicate(1, 1);
                activeVisualiser->replayControls.push_back(nextControl.replicate(1, 1));
                setupControls.erase(setupControls.begin());
                activeModelTranslator->setControlVector(nextControl, MAIN_DATA_STATE);
                activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);
                visualCounter++;
                if(visualCounter >= 20){
                    activeVisualiser->render("MPC Until Complete");
                    visualCounter = 0;
                }
            }
            applyingSetupControls = false;
            activeModelTranslator->activePhysicsSimulator->copySystemState(0, MAIN_DATA_STATE);
            initOptimisationControls = activeModelTranslator->createInitOptimisationControls(optHorizon);
            activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

            cout << "init controls: " << initOptimisationControls.size() << endl;
            optimisedControls = activeOptimiser->optimise(0, initOptimisationControls, yamlReader->maxIter, yamlReader->minIter, optHorizon);
            activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

        }
        else{
            MatrixXd nextControl = optimisedControls[0].replicate(1, 1);
            activeVisualiser->replayControls.push_back(nextControl.replicate(1, 1));

            optimisedControls.erase(optimisedControls.begin());

            optimisedControls.push_back(optimisedControls.at(optimisedControls.size() - 1));

            activeModelTranslator->setControlVector(nextControl, MAIN_DATA_STATE);

            activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

            reInitialiseCounter++;
            visualCounter++;

            if(activeModelTranslator->taskComplete(MAIN_DATA_STATE)){
                taskComplete = true;
            }
            else{
                if(reInitialiseCounter > 2){
                    //initControls = activeModelTranslator->createInitOptimisationControls(horizon);
                    optimisedControls = activeOptimiser->optimise(MAIN_DATA_STATE, optimisedControls, yamlReader->maxIter, yamlReader->minIter, optHorizon);
                    reInitialiseCounter = 0;
                }
            }

            if(visualCounter > 5){
                activeVisualiser->render(label);
                visualCounter = 0;
            }
        }

    }
    cout << "finished \n";

    while(activeVisualiser->windowOpen()){
        if(activeVisualiser->replayTriggered){
            activeVisualiser->replayTriggered = false;

            activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
            int controlCounter = 0;
            while(controlCounter < activeVisualiser->replayControls.size()){
                MatrixXd nextControl = activeVisualiser->replayControls[controlCounter].replicate(1, 1);

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

void keyboardControl(){
    
    while(activeVisualiser->windowOpen()){
        vector<double> gravCompensation;
        activeModelTranslator->activePhysicsSimulator->getRobotJointsGravityCompensaionControls("panda", gravCompensation, MAIN_DATA_STATE);
        MatrixXd control(activeModelTranslator->num_ctrl, 1);
        for(int i = 0; i < activeModelTranslator->num_ctrl; i++){
            control(i) = gravCompensation[i];
        }
        cout << "control: " << control << endl;
        activeModelTranslator->setControlVector(control, MAIN_DATA_STATE);
        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

        activeVisualiser->render("keyboard control");
    }
}