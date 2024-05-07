/*
================================================================================
    File: Optimiser.h
    Author: David Russell
    Date: January 16, 2024
    Description:
        Optimiser is a default class for optimisation algorithms to inherit from.
        Provides basic utility functions used for most optimisers, like computing
        gradients, performing rollouts, checking for convergence, etc.

================================================================================
*/
#pragma once

#include "StdInclude.h"
#include "ModelTranslator/ModelTranslator.h"
#include "Differentiator.h"
#include "KeyPointGenerator.h"
#include <atomic>

class Optimiser{
public:
    /**
     * Construct a new Optimiser  object.
     *
     * @param _modelTranslator - ModelTranslator object used to translate between the optimiser and current system
     * @param _MuJoCoHelper - PhysicsSimulator object used to simulate the system
     * @param _yamlReader - FileHandler object used to read in YAML files
     * @param _differentiator - Differentiator object used to compute derivatives
     *
     */
    Optimiser(std::shared_ptr<ModelTranslator> _modelTranslator,
              std::shared_ptr<MuJoCoHelper> _MuJoCoHelper,
              std::shared_ptr<FileHandler> _yamlReader,
              std::shared_ptr<Differentiator> _differentiator);

    // -----------------------------------------------------------------------------------------------------------
    // -------------------------------------- PURE virtual functions ---------------------------------------------
    // -----------------------------------------------------------------------------------------------------------
    /**
     * Rollout a trajectory from a given initial data state with a sequence of controls.
     *
     * @param initial_data_index - The data index that the rollout should start from.
     * @param save_states - Whether the rollout should be saved in the internal data buffer as well as the state vectors.
     * @param control_sequence - The sequence of controls to rollout.
     *
     * @return double The cost of the rollout after evaluating it with the cost function. (Cost function from the model translator)
     */
    virtual double RolloutTrajectory(mjData *d, bool save_states, std::vector<MatrixXd> control_sequence) = 0;

    /**
     * Optimise a trajectory from a given initial data state and a given set of initial controls. Will return the new optimised
     * control sequence, when either convergence is detected, or the maximum number of iterations is reached.
     *
     * This functions NEEDS to be overwritten by the optimiser that inherits from this class.
     *
     * @param initial_data_index - The data index that the rollout should start from.
     * @param initial_controls - The initial guess of controls to optimise from.
     * @param max_iterations - Maximum number of optimisation iterations.
     * @param min_iterations - Minimum number of optimisation iterations.
     * @param _horizonLength - The length of the horizon to optimise over.
     *
     * @return std::vector<MatrixXd> The optimised control sequence.
     */
    virtual std::vector<MatrixXd> Optimise(mjData *d, std::vector<MatrixXd> initial_controls, int max_iterations, int min_iterations, int _horizonLength) = 0;

    // -----------------------------------------------------------------------------------------------------------
    // -------------------------------------- OPTIONAL virtual function ------------------------------------------
    // -----------------------------------------------------------------------------------------------------------

    /**
     * This function compares the old iteration cost with the new iteration cost and computes the gradient.
     * If the cost didnt change significantly, then the optimisation has converged.
     *
     * @param old_cost - Last iterations cumulative cost.
     * @param new_cost - Current iterations cumulative cost.
     *
     * @return bool Whether the optimisation has converged or not.
     */
    virtual bool CheckForConvergence(double old_cost, double new_cost);

    virtual std::string ReturnName(){
        return "general";
    }

    /**
     * Returns optimisation data about the last optimisation performed from function "Optimise". The data it returns
     * is timing data about the optimisation, the cost reduction, the average percentage of derivatives computed, and
     * the number of iterations it took to converge.
     *
     * @param _optTime - The time it took to optimise the trajectory. Passed by reference.
     * @param _costReduction - The cost reduction from the initial cost to the final cost. Passed by reference.
     * @param _avgPercentageDerivs - The average percentage of derivatives computed over the entire trajectory. Passed by reference.
     * @param _avgTimeGettingDerivs - The average time it took to compute the derivatives. Passed by reference.
     * @param _numIterations - The number of iterations it took to converge. Passed by reference.
     *
     */
    void ReturnOptimisationData(double &_optTime, double &_costReduction, double &_avgPercentageDerivs, double &_avgTimeGettingDerivs, int &_numIterations);

    /**
     * Resize variables that are dependant on the size of the state vector.
     *
     * @param new_num_dofs - The new number of degrees of freedom in the state vector.
     * @param new_num_ctrl - The new number of controls
     * @param new_horizon - New horizon of the trajectory
     */
    virtual void Resize(int new_num_dofs, int new_num_ctrl, int new_horizon);

    /**
     * Returns the current active keypoint method, and its associating parameters.
     *
     * @return keypoint_method The current active keypoint method.
     */
    keypoint_method ReturnCurrentKeypointMethod();

    /**
     * Sets the current active keypoint method, and its associating parameters.
     *
     * @param _derivativeInterpolator - The keypoint method to set as the current active keypoint method.
     *
     */
    void SetCurrentKeypointMethod(keypoint_method _derivativeInterpolator);

    /**
     * Worker function for computing dynamics derivatives in parallel. Uses a global variable of the keypoints
     * to know which columns of the dynamics matrices need computing at which time indices.
     *
     * @param threadId - The thread id of the worker thread.
     *
     */
    void WorkerComputeDerivatives(int threadId);

    /**
     * This functions generates all the dynamic derivatives and cost derivatives over the entire trajectory.
     *
     */
    void GenerateDerivatives();

    /**
     * This function sets the current FIR filter coefficients.
     *
     * @param _FIRCoefficients - The FIR filter coefficients to set.
     */
    void setFIRFilter(std::vector<double> _FIRCoefficients);

    // List of differentiator function callbacks, for parallelisation.
    std::vector<void (Differentiator::*)(MatrixXd &A, MatrixXd &B, const std::vector<int> &cols,
                                        MatrixXd &l_x, MatrixXd &l_u, MatrixXd &l_xx, MatrixXd &l_uu,
                                        int dataIndex, int threadId, bool terminal, bool costDerivs,
                                        bool central_diff, double eps)> tasks;

    // current_iteration used for parallelisation of dynamics derivatives
    std::atomic<int> current_iteration;
    int num_threads_iterations;
    std::vector<int> timeIndicesGlobal;

    double initial_cost = 0.0;
    double cost_reduction = 0.0;

    int numberOfTotalDerivs = 0;

    int num_iterations;

    std::string filteringMethod = "none";


    // ------- Timing variables --------------
    double opt_time_ms;
    std::vector<double> time_get_derivs_ms;
    double avg_time_get_derivs_ms = 0.0;
    std::vector<double> time_backwards_pass_ms;
    double avg_time_backwards_pass_ms = 0.0;
    std::vector<double> time_forwardsPass_ms;
    double avg_time_forwards_pass_ms = 0.0;
    std::vector<double> percentage_derivs_per_iteration;
    double avg_percent_derivs = 0.0;
    std::vector<int> num_dofs;
    double avg_dofs = 0.0;
    bool verbose_output = true;

    // - Top level function - ensures all derivatives are calculated over an entire trajectory by some method

    // -------------- Vectors of matrices for gradient information about the trajectory -------------
    // First order dynamics
    vector<MatrixXd> A;
    vector<MatrixXd> B;

    // First and second order cost derivatives
    vector<MatrixXd> l_x;
    vector<MatrixXd> l_xx;
    vector<MatrixXd> l_u;
    vector<MatrixXd> l_uu;

    // Saved states and controls
//    vector<MatrixXd> U_new;
    vector<MatrixXd> U_old;
    vector<MatrixXd> X_new;
    vector<MatrixXd> X_old;

    int horizon_length = 0;

    std::vector<double> cost_history;
    double lowPassACoefficient = 0.25;
    std::vector<double> FIRCoefficients = {0.1, 0.15, 0.5, 0.15, 0.1};

    std::shared_ptr<KeypointGenerator> keypoint_generator;

    int sampling_k_interval = 1;
    int num_dofs_readd = 2;
    double K_matrix_threshold = 1.5; // maybe 0.001 or 0.0001
//    double threshold_k_eigenvectors = 0.1;

    keypoint_method activeKeyPointMethod;

    int dof = 0;
    int num_ctrl = 0;

protected:
    std::shared_ptr<ModelTranslator> activeModelTranslator;
    std::shared_ptr<MuJoCoHelper> MuJoCo_helper;

    std::vector<std::vector<int>> keypointsGlobal;

    std::shared_ptr<FileHandler> activeYamlReader;
    std::shared_ptr<Differentiator> activeDifferentiator;

    std::vector<std::vector<mujoco_data_min>> rollout_data;
    int num_parallel_rollouts = 6;

    /**
     * Computes the dynamics derivatives at the specified indices. This function is used after computing a set of keypoints
     * based on the previous trajectory.
     *
     * @param keyPoints - The keypoints to compute the derivatives at. A list of lists, where each sublist refers to a different
     * degree of freedom. The elements in the list are the time indices to compute the derivatives at.
     */
    void ComputeDerivativesAtSpecifiedIndices(std::vector<std::vector<int>> keyPoints);

    /**
     * Computes the cost derivatives over the entire trajectory.
     */
    void ComputeCostDerivatives();

    /**
     * Applies a filter to the internal dynamics derivatives.
     */
    void FilterDynamicsMatrices();

    /**
     * Applies a FIR filter to the vector of unfiltered values.
     *
     * @param unfiltered - The unfiltered values.
     * @param filterCoefficients - The filter coefficients.
     *
     * @return filtered - The filtered values.
     */
    std::vector<double> FilterIndValFIRFilter(std::vector<double> unfiltered, std::vector<double> filterCoefficients);

    /**
     * Applies a low pass filter to the vector of unfiltered values.
     *
     * @param unfiltered - The unfiltered values.
     *
     * @return filtered - The filtered values.
     */
    std::vector<double> FilterIndValLowPass(std::vector<double> unfiltered);

    void SaveSystemStateToRolloutData(mjData *d, int thread_id, int data_index);
    void SaveBestRollout(int thread_id);

private:
    double epsConverge = 0.02;

};