//
// Created by dave on 09/03/23.
//

#ifndef PHYSICSSIMSWITCHING_REACHING_H
#define PHYSICSSIMSWITCHING_REACHING_H

#include "modelTranslator.h"

class pandaReaching : public modelTranslator {
public:
    pandaReaching();

    char* filePath; 
    int reachingDOF;
    int reachingNumCtrl;

};

#endif //PHYSICSSIMSWITCHING_REACHING_H
