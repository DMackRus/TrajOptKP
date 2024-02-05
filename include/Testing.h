//
// Created by davidrussell on 1/17/24.
//

#pragma once

#include "StdInclude.h"
#include "iLQR.h"
#include <thread>
#include <mutex>

class Testing{

public:
    Testing(std::shared_ptr<iLQR> iLQROptimiser_,
            std::shared_ptr<ModelTranslator> activeModelTranslator_,
            std::shared_ptr<Differentiator> activeDifferentiator_,
            std::shared_ptr<Visualiser> activeVisualiser_,
            std::shared_ptr<FileHandler> yamlReader_);


    /**
     * Tests different minN parmeters for setInterval keypoint method for asycnhrnous MPC
     *
     * @Param lowest_minN: Smallest interval to test
     * @Param higherst_minN: Largest interval to test
     * @Param step_size: Step size between intervals
     *
     * @Return: 1 if successful, 0 if not
     */
    int testing_different_minN_asynchronus_mpc(int lowest_minN, int higherst_minN, int step_size);

    /**
     * This function tests a particular keypoint method for a static number
     * of trials for the same tasks. The task is set in the config file.
     *
     * @Param keypoint_method: The keypoint method to be tested
     *
     * @Return: 1 if successful, 0 if not
     */
    int testing_asynchronus_mpc(keypoint_method keypoint_method);

    /**
     * This function performs an asynchronus MPC optimisation for a set number of time-steps
     * with the simulation running in one thread and calles'asycnhronus_optimiser_worker' for
     * the optimisation in another thread.
     *
     * @Param keypoint_method: The keypoint method to be tested
     *
     * @Return: 1 if successful, 0 if not
     */
    int single_asynchronus_run(bool visualise);

    /**
     * This function performs an asynchronus MPC optimisation with another thread doing simulation
     * and visualisation. This thread will keep performing MPC optimisation until the main thread
     * terminates after task timeout.
     *
     */
    void asynchronus_optimiser_worker();


    std::shared_ptr<iLQR> iLQROptimiser;
    std::shared_ptr<ModelTranslator> activeModelTranslator;
    std::shared_ptr<Differentiator> activeDifferentiator;
    std::shared_ptr<Visualiser> activeVisualiser;
    std::shared_ptr<FileHandler> yamlReader;


private:

    bool stop_opt_thread = false;

    double final_cost = 0.0f;
    double average_opt_time_ms = 0.0f;
    double average_percent_derivs = 0.0f;
    double average_time_derivs_ms = 0.0f;
    double average_time_fp_ms = 0.0f;
    double average_time_bp_ms = 0.0f;

};