#pragma once

#include "ModelTranslator.h"

class Acrobot : virtual public ModelTranslator{
public:

    Acrobot(){
        std::string yamlFilePath = "/src/tests/test_configs/acrobot.yaml";

        InitModelTranslator(yamlFilePath);
    }

    MatrixXd ReturnRandomStartState() override{
        return MatrixXd(1,1);
    }

    MatrixXd ReturnRandomGoalState(MatrixXd X0) override{
        return MatrixXd(1,1);
    }

    void GenerateRandomGoalAndStartState() override{

    }
};
