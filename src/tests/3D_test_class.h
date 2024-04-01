#include "ModelTranslator.h"

class threeDTestClass : virtual public ModelTranslator{
public:

    MatrixXd ReturnRandomStartState() override{
        return MatrixXd(1,1);
    }

    MatrixXd ReturnRandomGoalState(MatrixXd X0) override{
        return MatrixXd(1,1);
    }

    void GenerateRandomGoalAndStartState() override{

    }

public:
    threeDTestClass(){
        std::string yamlFilePath = "/taskConfigs/threeDPushTest.yaml";

        InitModelTranslator(yamlFilePath);
    }
};