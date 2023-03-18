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


int main() {

    scenes myScene = pendulum;
    modelTranslator *activeModelTranslator;
    MatrixXd startStateVector(1, 1);
    
    std::cout << "before scene check" << std::endl;
    if(myScene == pendulum){
        std::cout << "before creating double pendulum" << std::endl;
        doublePendulum *myDoublePendulum = new doublePendulum();
        activeModelTranslator = myDoublePendulum;
        startStateVector.resize(activeModelTranslator->stateVectorSize, 1);

        startStateVector = activeModelTranslator->returnRandomStartState();
        MatrixXd controlVec(2,1);
        controlVec << 10, 10;
        //activeModelTranslator->setControlVector(controlVec);
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

    }
    else if(myScene == boxFlicking){

    }
    else{
        std::cout << "invalid scene selected, exiting" << std::endl;
    }

    differentiator *myDifferentiator = new differentiator(activeModelTranslator, activeModelTranslator->myHelper);

    


    //differentiator *myDifferentiator = new differentiator();

    // startStateVector << -1, 0.5, 0, -1, 0, 0.6, 1,
    //         0.5, 0.5, 0.4, 0, 0, 0,
    //         0, 0, 0, 0, 0, 0, 0,
    //         0, 0, 0, 0, 0, 0;
    std::cout << "before set start vector" << std::endl;
    
    activeModelTranslator->setStateVector(startStateVector, MAIN_DATA_STATE);

    cout << " -------------- Set State vector -------------------- \n";

    //Instantiate my optimiser
    interpolatediLQR *myOptimiser = new interpolatediLQR(activeModelTranslator, activeModelTranslator->activePhysicsSimulator, myDifferentiator, 2000);
    vector<MatrixXd> initControls;
    int horizon = 1500;
    for(int i = 0; i < horizon; i++){
        MatrixXd controlVec(activeModelTranslator->num_ctrl, 1);
        controlVec << 0, 0;
        initControls.push_back(controlVec);
    }
    cout << "made init controls " << endl;
    //activeModelTranslator->activePhysicsSimulator->appendSystemStateToEnd(MAIN_DATA_STATE);
    //double initCost = myOptimiser->rolloutTrajectory(MAIN_DATA_STATE, true, initControls);
    //cout << "init cost: " << initCost << endl;

    vector<MatrixXd> optimisedControls;
    optimisedControls = myOptimiser->optimise(0, initControls, 5, horizon);

    

    activeModelTranslator->activePhysicsSimulator->loadSystemStateFromIndex(MAIN_DATA_STATE, 0);
    activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

    visualizer myVisualizer(activeModelTranslator, myDifferentiator);
    myVisualizer.render();

    return 0;
}