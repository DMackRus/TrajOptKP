#include "stdInclude/stdInclude.h"
#include "physicsSimulators/MuJoCoHelper.h"

int main() {

    std::cout << "Hello, World!" << std::endl;

    MuJoCoHelper myHelper;
    myHelper.setupMuJoCoWorld(0.004, "Franka-emika-panda-arm/V1/reaching_scene.xml");
    myHelper.render();

    return 0;
}