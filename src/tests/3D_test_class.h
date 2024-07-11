#include "ModelTranslator/ModelTranslator.h"

class threeDTestClass : virtual public ModelTranslator{
public:

    threeDTestClass(){
        std::string yamlFilePath = "/src/tests/test_configs/threeDPushTest.yaml";

        InitModelTranslator(yamlFilePath);
    }

    void Residuals(mjData *d, MatrixXd &residual){

    }
};