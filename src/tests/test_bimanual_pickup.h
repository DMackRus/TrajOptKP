#pragma once

#include "ModelTranslator/ModelTranslator.h"

class BimanualPickup : virtual public ModelTranslator{
public:

    BimanualPickup(){
        std::string yamlFilePath = "/src/tests/test_configs/bimanualPickup.yaml";

        InitModelTranslator(yamlFilePath);
    }

    void Residuals(mjData *d, MatrixXd &residual){

    }

};
