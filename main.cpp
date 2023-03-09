#include "stdInclude/stdInclude.h"
#include "physicsSimulators/MuJoCoHelper.h"
#include "modelTranslator/modelTranslator.h"
#include "visualizer/visualizer.h"

int main() {

    int taskNumber = 1;
    modelTranslator *myModelTranslator = new modelTranslator(taskNumber);

    MatrixXd startStateVector(myModelTranslator->stateVectorSize, 1);
    startStateVector << -1, 0.5, 0, -1, 0, 0.6, 1,
            0.5, 0.5, 0.4, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0;
    myModelTranslator->setStateVector(startStateVector);
    MatrixXd stateVector = myModelTranslator->returnStateVector();
    cout << "stateVector: " << stateVector << endl;

    myModelTranslator->activePhysicsSimulator->stepSimulator(1);

    visualizer myVisualizer(myModelTranslator);
    myVisualizer.render();

    return 0;
}