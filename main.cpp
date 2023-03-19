#include "stdInclude/stdInclude.h"


// --------------------- different scenes -----------------------
#include "modelTranslator/doublePendulum.h"
#include "modelTranslator/reaching.h"


#include "visualizer/visualizer.h"
#include "physicsSimulators/MuJoCoHelper.h"

#include "optimiser/interpolated_iLQR.h"


#define PENDULUM_SCENE              0
#define REACHING_SCENE              0
#define TWOD_PUSHING_SCENE          0
#define THREED_PUSHING_SCENE        0
#define TWOD_PUSHING_CLUTTER_SCENE  0
#define BOX_FLICK_SCENE             0

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


void MPCControl();

int main() {

    scenes myScene = pendulum;
    MatrixXd startStateVector(1, 1);
    
    if(myScene == pendulum){
        doublePendulum *myDoublePendulum = new doublePendulum();
        activeModelTranslator = myDoublePendulum;
        startStateVector.resize(activeModelTranslator->stateVectorSize, 1);

        startStateVector = activeModelTranslator->returnRandomStartState();
        //startStateVector << 3.14, 0, 0, 0;
    }
    else if(myScene == reaching){
        // std::cout << "before creating reaching problem" << std::endl;
        // pandaReaching *myReaching = new pandaReaching();
        // activeModelTranslator = myReaching;
        // startStateVector.resize(activeModelTranslator->stateVectorSize, 1);

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
    activeOptimiser = new interpolatediLQR(activeModelTranslator, activeModelTranslator->activePhysicsSimulator, activeDifferentiator, 2000);

    activeVisualiser = new visualizer(activeModelTranslator, activeOptimiser);

    MPCControl();
    //activeVisualiser->render();

    return 0;
}

void MPCControl(){


    int horizon = 2000;
    bool taskComplete = false;
    int currentControlCounter = 0;
    int visualCounter = 0;
    int overallTaskCounter = 0;
    int reInitialiseCounter = 0;
    cout << "before init dhfdkjsdhkdfshkd \n";

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
            activeVisualiser->render();
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