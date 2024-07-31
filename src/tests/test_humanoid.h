#pragma once

class Humanoid : virtual public ModelTranslator{
public:

    Humanoid(){
        std::string yamlFilePath = "/src/tests/test_configs/humanoid.yaml";

        InitModelTranslator(yamlFilePath);
    }

    void Residuals(mjData *d, MatrixXd &residual){

    }
};