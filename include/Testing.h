//
// Created by davidrussell on 1/17/24.
//

#pragma once

#include "stdInclude.h"
#include "interpolated_iLQR.h"
#include <thread>
#include <mutex>

class Testing{

public:
    Testing(std::shared_ptr<interpolatediLQR> iLQROptimiser_,
            std::shared_ptr<modelTranslator> activeModelTranslator_,
            std::shared_ptr<differentiator> activeDifferentiator_,
            std::shared_ptr<visualizer> activeVisualiser_,
            std::shared_ptr<fileHandler> yamlReader_);


    int testing_asynchronus_mpc(std::vector<std::string> keypoint_methods);
    int single_asynchronus_run(bool visualise);

    void asynchronus_optimiser_worker();


    std::shared_ptr<interpolatediLQR> iLQROptimiser;
    std::shared_ptr<modelTranslator> activeModelTranslator;
    std::shared_ptr<differentiator> activeDifferentiator;
    std::shared_ptr<visualizer> activeVisualiser;
    std::shared_ptr<fileHandler> yamlReader;


private:

    bool stop_opt_thread = false;

    double average_opt_time_ms = 0.0f;
    double average_percent_derivs = 0.0f;
    double average_time_derivs_ms = 0.0f;
    double average_time_fp_ms = 0.0f;
    double average_time_bp_ms = 0.0f;

};