#pragma once

#include "ModelTranslator/ModelTranslator.h"

class Walker : virtual public ModelTranslator{
public:

    Walker(){
        std::string yamlFilePath = "/src/tests/test_configs/walker.yaml";

        InitModelTranslator(yamlFilePath);
    }

    void Residuals(mjData *d, MatrixXd &residual){

    }

};