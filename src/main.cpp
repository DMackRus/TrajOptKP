#include "stdInclude.h"
#include <yaml-cpp/yaml.h>


// --------------------- different scenes -----------------------
#include "doublePendulum.h"
#include "reaching.h"


#include "visualizer.h"
#include "MuJoCoHelper.h"

#include "interpolated_iLQR.h"


#define SHOW_INIT_CONTROLS          0
#define ILQR_ONCE                   1
#define MPC_CONTINOUS               0
#define MPC_UNTIL_COMPLETE          0
#define DEFAULT_KEYBOARD_CONTROL    0

enum scenes{
    pendulum = 0,
    reaching = 1,
    twoReaching = 2,
    twoDPushing = 3,
    threeDPushing = 4,
    boxFlicking = 5
};

modelTranslator *activeModelTranslator;
differentiator *activeDifferentiator;
interpolatediLQR *activeOptimiser;
visualizer *activeVisualiser;

void showInitControls();
void iLQROnce();
void MPCUntilComplete();
void MPCContinous();
void keyboardControl();

int main() {

    scenes myScene = pendulum;
    MatrixXd startStateVector(1, 1);

    if(myScene == pendulum){
        doublePendulum *myDoublePendulum = new doublePendulum();
        activeModelTranslator = myDoublePendulum;
        startStateVector.resize(activeModelTranslator->stateVectorSize, 1);

        startStateVector = activeModelTranslator->returnRandomStartState();
        //startStateVector << 3.14, 0, 0, 0;
        startStateVector << 0.2, 1.9, 0, 0;
    }
    else if(myScene == reaching){
        // std::cout << "before creating reaching problem" << std::endl;
        pandaReaching *myReaching = new pandaReaching();
        activeModelTranslator = myReaching;
        startStateVector.resize(activeModelTranslator->stateVectorSize, 1);
        startStateVector = activeModelTranslator->returnRandomStartState();

        // startStateVector << -1, 0.5, 0, -1, 0, 0.6, 1,
        //     0, 0, 0, 0, 0, 0, 0;
        

    }
    else if(myScene == twoReaching){

    }
    else if(myScene == twoDPushing){
        

    }
    else if(myScene == threeDPushing){
        // startStateVector << -1, 0.5, 0, -1, 0, 0.6, 1,
    //         0.5, 0.5, 0.4, 0, 0, 0,
    //         0, 0, 0, 0, 0, 0, 0,
    //         0, 0, 0, 0, 0, 0;

    }
    else if(myScene == boxFlicking){

    }
    else{
        std::cout << "invalid scene selected, exiting" << std::endl;
    }

    activeDifferentiator = new differentiator(activeModelTranslator, activeModelTranslator->myHelper);
    activeModelTranslator->setStateVector(startStateVector, MAIN_DATA_STATE);

    //Instantiate my optimiser
    activeOptimiser = new interpolatediLQR(activeModelTranslator, activeModelTranslator->activePhysicsSimulator, activeDifferentiator, 3000);

    activeVisualiser = new visualizer(activeModelTranslator, activeOptimiser);

    if(SHOW_INIT_CONTROLS){
        showInitControls();
    }
    else if(ILQR_ONCE){
        iLQROnce();
    }
    else if(MPC_CONTINOUS){
        MPCContinous();
    }
    else if(MPC_UNTIL_COMPLETE){
        MPCUntilComplete();
    }
    else if(DEFAULT_KEYBOARD_CONTROL){
        keyboardControl();
    }
    else{
        cout << "INVALID MODE OF OPERATION OF PROGRAM \n";
    }

    //activeVisualiser->render();

    return 0;
}

void showInitControls(){
    int horizon = 2000;
    int controlCounter = 0;
    int visualCounter = 0;

    std::vector<MatrixXd> initControls = activeModelTranslator->createInitControls(horizon);

    activeModelTranslator->activePhysicsSimulator->appendSystemStateToEnd(MAIN_DATA_STATE);
    cout << "init controls created \n";

    while(activeVisualiser->windowOpen()){

        activeModelTranslator->setControlVector(initControls[controlCounter], MAIN_DATA_STATE);

        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

        controlCounter++;
        visualCounter++;

        if(controlCounter >= horizon){
            controlCounter = 0;
            activeModelTranslator->activePhysicsSimulator->loadSystemStateFromIndex(MAIN_DATA_STATE, 0);
        }

        if(visualCounter > 5){
            visualCounter = 0;
            activeVisualiser->render("show init controls");
        }
    }
}

void iLQROnce(){
    int horizon = 3000;
    int controlCounter = 0;
    int visualCounter = 0;
    bool showFinalControls = true;
    char* label = "Final controls";

    std::vector<MatrixXd> initControls = activeModelTranslator->createInitControls(horizon);
    std::vector<MatrixXd> optimisedControls = activeOptimiser->optimise(0, initControls, 10, horizon);

    activeModelTranslator->activePhysicsSimulator->appendSystemStateToEnd(MAIN_DATA_STATE);

    while(activeVisualiser->windowOpen()){

        if(showFinalControls){
            activeModelTranslator->setControlVector(optimisedControls[controlCounter], MAIN_DATA_STATE);
        }
        else{
            activeModelTranslator->setControlVector(initControls[controlCounter], MAIN_DATA_STATE);
        }
        

        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

        controlCounter++;
        visualCounter++;

        if(controlCounter >= horizon){
            controlCounter = 0;
            activeModelTranslator->activePhysicsSimulator->loadSystemStateFromIndex(MAIN_DATA_STATE, 0);
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


    int horizon = 2000;
    bool taskComplete = false;
    int currentControlCounter = 0;
    int visualCounter = 0;
    int overallTaskCounter = 0;
    int reInitialiseCounter = 0;
    char* label = "MPC Continous";

    // Instantiate init controls
    std::vector<MatrixXd> initControls;
    initControls = activeModelTranslator->createInitControls(horizon);
    cout << "init controls: " << initControls.size() << endl;
    std::vector<MatrixXd> optimisedControls = activeOptimiser->optimise(0, initControls, 10, horizon);

    while(!taskComplete){
        MatrixXd nextControl = optimisedControls[0].replicate(1, 1);

        optimisedControls.erase(optimisedControls.begin());

        optimisedControls.push_back(optimisedControls.at(optimisedControls.size() - 1));

        activeModelTranslator->setControlVector(nextControl, MAIN_DATA_STATE);

        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

        reInitialiseCounter++;
        visualCounter++;

        if(reInitialiseCounter > 500){
            reInitialiseCounter = 0;
            optimisedControls = activeOptimiser->optimise(reInitialiseCounter, optimisedControls, 5, horizon);
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

}

void keyboardControl(){
    activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);
    while(activeVisualiser->windowOpen()){
        activeVisualiser->render("keyboard control");
    }
}