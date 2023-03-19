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
    
    activeModelTranslator->setStateVector(startStateVector, MAIN_DATA_STATE);

    //Instantiate my optimiser
    interpolatediLQR *myOptimiser = new interpolatediLQR(activeModelTranslator, activeModelTranslator->activePhysicsSimulator, myDifferentiator, 2000);

    // activeModelTranslator->activePhysicsSimulator->loadSystemStateFromIndex(MAIN_DATA_STATE, 0);
    // activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

    visualizer myVisualizer(activeModelTranslator, myOptimiser);
    myVisualizer.render();

    return 0;
}