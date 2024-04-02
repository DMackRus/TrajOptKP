#include "ModelTranslator.h"

class threeDTestClass : virtual public ModelTranslator{
public:

    threeDTestClass(){
        std::string yamlFilePath = "/src/tests/test_configs/threeDPushTest.yaml";

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