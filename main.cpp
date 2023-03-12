#include "stdInclude/stdInclude.h"
#include "physicsSimulators/MuJoCoHelper.h"


// --------------------- different scenes -----------------------
#include "modelTranslator/doublePendulum.h"
#include "modelTranslator/reaching.h"


#include "visualizer/visualizer.h"


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

    scenes myScene = reaching;
    modelTranslator *activeModelTranslator;

    std::cout << "before scene check" << std::endl;
    if(myScene == pendulum){
        std::cout << "before creating double pendulum" << std::endl;
        doublePendulum *myDoublePendulum = new doublePendulum();
        activeModelTranslator = myDoublePendulum;
    }
    else if(myScene == reaching){
        std::cout << "before creating reaching problem" << std::endl;
        pandaReaching *myReaching = new pandaReaching();
        activeModelTranslator = myReaching;

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

    MatrixXd startStateVector(activeModelTranslator->stateVectorSize, 1);

    startStateVector << -1, 0.5, 0, -1, 0, 0.6, 1,
            0, 0, 0, 0, 0, 0, 0;

    // startStateVector << -1, 0.5, 0, -1, 0, 0.6, 1,
    //         0.5, 0.5, 0.4, 0, 0, 0,
    //         0, 0, 0, 0, 0, 0, 0,
    //         0, 0, 0, 0, 0, 0;

    //startStateVector << 0.1, 0.1, 0, 0;
    activeModelTranslator->setStateVector(startStateVector);

    activeModelTranslator->activePhysicsSimulator->stepSimulator(1);

    visualizer myVisualizer(activeModelTranslator);
    myVisualizer.render();

    return 0;
}