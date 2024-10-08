#pragma once

#include "StdInclude.h"
#include "Optimiser/iLQR.h"
#include <thread>
#include <mutex>
#include <filesystem>
#include <yaml-cpp/yaml.h>

class GenTestingData{

public:
    GenTestingData(std::shared_ptr<Optimiser> optimiser,
                   std::shared_ptr<ModelTranslator> activeModelTranslator_,
                   std::shared_ptr<Differentiator> activeDifferentiator_,
                   std::shared_ptr<Visualiser> activeVisualiser_,
                   std::shared_ptr<FileHandler> yamlReader_);

    /**
     * Tests different parametrisations of velocity change methods (different minN, maxN
     * and velocity change thresholds. for asynchronous MPC.
     *
     * @return 1 if successful, 0 if not
     */
    int GenDataAsyncMPC(int task_horizon, int task_timeout);

    int GenDataOpenLoopMultipleMethods(int task_horizon);

    int GenDataOpenloopOptimisation(int task_horizon);

    int GenDataMPCHorizons(int task_timeout);

    /**
     * This function tests a particular keypoint method for a static number
     * of trials for the same tasks. The task is set in the config file.
     *
     * @Param keypoint_method: The keypoint method to be tested
     * @Param asynchronus: Whether to perform the optimisation asynchronusly
     * @Param num_trials: The number of trials to be run
     * @Param task_horizon: Optimisation horizon of the task
     * @Param task_timeout: When to stop the task, if no other exit condition
     *
     * @Return: 1 if successful, 0 if not
     */
    int TestingMPC(const keypoint_method& keypoint_method, bool asynchronus,
                   int num_trials, int task_horzion, int task_timeout);

    /**
     * This function performs an asynchronus MPC optimisation for a set number of time-steps
     * with the simulation running in one thread and calles'asycnhronus_optimiser_worker' for
     * the optimisation in another thread.
     *
     * @Param visualise: Whether to visualise the results
     * @param asynchronus: Whether to perform the optimisation asynchronusly
     * @Param method_directory: The directory to save the results
     * @Param task_number: The task number to be performed
     * @Param task_horizon: The optimisation horizon
     * @Param TASK_TIMEOUT: When to stop the task, if no other exit condition
     *
     * @Return: 1 if successful, 0 if not
     */
    int SingleMPCRun(bool visualise, bool asynchronus,
                     const std::string& method_directory,
                     int task_number, int task_horizon, int TASK_TIMEOUT);

    /**
     * This function performs an asynchronus MPC optimisation with another thread doing simulation
     * and visualisation. This thread will keep performing MPC optimisation until the main thread
     * terminates after task timeout.
     *
     * @Param method_directory: The directory to save the results
     * @Param task_number: The task number to be performed
     *
     */
    void AsyncronusMPCWorker(const std::string& method_directory, int task_number, int task_horizon);

    int GenerateDynamicsDerivsData(int num_trajecs, int num_iters_per_task);

    int GenerateTestScenes(int num_scenes);

    int AnalyseToyContact(int horizon);

    int AnalyseToyContactKeypoints(int horizon);

    void SetParamsiLQR_SVR(int re_add_dofs, double threshold){
        optimiser->num_dofs_readd = re_add_dofs;
        optimiser->K_matrix_threshold = threshold;
    }

    std::shared_ptr<Optimiser> optimiser;
    std::shared_ptr<ModelTranslator> activeModelTranslator;
    std::shared_ptr<Differentiator> activeDifferentiator;
    std::shared_ptr<Visualiser> activeVisualiser;
    std::shared_ptr<FileHandler> yamlReader;

private:

    std::string CreateTestName(const std::string& testing_method);

    void SaveTestSummaryData(keypoint_method keypoint_method,
                             int opt_horizon,
                             double control_noise,
                             const std::string& optimiser_name,
                             const std::string& testing_directory);

    double controls_noise = 0.5;

    std::mutex mtx;

    volatile bool stop_opt_thread = false;
    volatile bool apply_next_control = false;
//    bool async_mpc = true;

    int num_controls_apply = 80;
    int num_steps_replan = 1;
    volatile bool reoptimise = true;

    double final_cost = 0.0;
    double final_dist = 0.0;
    double average_dof = 0.0;
    double average_opt_time_ms = 0.0;
    double average_percent_derivs = 0.0;
    double average_time_derivs_ms = 0.0;
    double average_time_fp_ms = 0.0;
    double average_time_bp_ms = 0.0;
    double average_surprise = 0.0;
};