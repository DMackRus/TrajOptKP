/*
================================================================================
    File: ModelTranslator.h
    Author: David Russell
    Date: January 16, 2024
    Description:
        Model translator is an abstract class that is used to provide high level
        functions to interface between trajectory optimisation and a physics
        simulator. Provides utility functions like setting and getting the
        state of the system. Also provides functions for calculating the cost
        function and its derivatives.

        Some of the functions can be overwritten to provide custom behaviour
        ,especially useful for defining custom cost functions and initial controls.
================================================================================
*/
#pragma once

#include "StdInclude.h"
#include "MuJoCoHelper.h"
#include "FileHandler.h"
#include <random>

enum clutterLevels{
    noClutter = 0,
    lowClutter = 1,
    heavyClutter = 2,
    constrainedClutter = 3,
    clutter_realWorld = 4
};

struct color{
    float r;
    float g;
    float b;
    float a;
};

class ModelTranslator {
public:
    /**
     * Construct a new Model Translator object.
     *
     */
    ModelTranslator() = default;

    /**
     * Update the state vector of the system by either adding or removing elements from
     * the state vector.
     *
     * @param state_vector_names The names of the state vector elements to add or remove.
     * @param add_extra_states Whether to add or remove the state vector elements.
     *
     */
    void UpdateCurrentStateVector(std::vector<std::string> state_vector_names, bool add_extra_states);

    /**
     * Randomly sample a number of unused dofs from the full state vector
     *
     * @param num_dofs The number of dofs to resample.
     *
     */
    std::vector<std::string> RandomSampleUnusedDofs(int num_dofs) const;

    /**
     * Returns a random start state for the system. This is used mainly for generating
     * mass testing data.
     *
     * @return MatrixXd A random start state for the system.
     */
    virtual void ReturnRandomStartState();

    /**
     * Returns a random goal state for the system. This is used mainly for generating
     * mass testing data.
     *
     * @return MatrixXd A random goal state for the system.
     */
    virtual void ReturnRandomGoalState();

    /**
     * Generates a random goal and start state for the system. This is used mainly for
     * generating mass testing data.
     *
     */
    virtual void GenerateRandomGoalAndStartState();

    //--------------------------------------------------------------------------------
    // Virtual functions that can be overwritten by the child class
    //--------------------------------------------------------------------------------
    virtual void Residuals(mjData *d, MatrixXd &residual);

    /**
     * Returns the current cost of the system at the given data index.
     *
     * @param residuals The residuals of the system.
     * @param state_vector The state vector object to use for cost function computations
     * @param terminal Whether or not this is the terminal state or not.
     *
     * @return double The cost of the system at the given data index.
     */
    virtual double CostFunction(const MatrixXd &residuals, const struct stateVectorList &state_vector, bool terminal);

    virtual void UpdateSceneVisualisation();

    double CostFunctionBody(const rigid_body& body, mjData *d, bool terminal);

    double CostFuntionSoftBody(const soft_body& soft_body, mjData *d, bool terminal);

    /**
     * Returns the current cost derivatives (1st and 2nd order) of the system with respect to the
     * state and control vectors at the given data index.
     *
     * @param data_index The data index of the system to calculate the cost for.
     * @param state_vector The state vector object to use to get cost elements
     * @param l_x The first order cost derivative with respect to the state vector. Passed by reference.
     * @param l_xx The second order cost derivative with respect to the state vector. Passed by reference.
     * @param l_u The first order cost derivative with respect to the control vector. Passed by reference.
     * @param l_uu The second order cost derivative with respect to the control vector. Passed by reference.
     * @param terminal Whether or not this is the terminal state or not.
     *
     */
    virtual void CostDerivatives(mjData* d, const struct stateVectorList &state_vector,
            MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal);

    /**
     * Returns the current cost derivatives (1st and 2nd order) of the system with respect to the
     * state and control vectors at the given data index. For the given state vector representation.
     * Uses the Gauss newton approximation to compute second order derivatives. I.e ignoring
     * Hessian of the residuals.
     *
     * @param state_vector The state vector object to use to get cost elements
     * @param l_x The first order cost derivative with respect to the state vector. Passed by reference.
     * @param l_xx The second order cost derivative with respect to the state vector. Passed by reference.
     * @param l_u The first order cost derivative with respect to the control vector. Passed by reference.
     * @param l_uu The second order cost derivative with respect to the control vector. Passed by reference.
     * @param residuals The residuals of the system at the given data index. Passed by reference.
     * @param r_x The first order residual derivative with respect to the state vector. Passed by reference.
     * @param r_u The first order residual derivative with respect to the control vector. Passed by reference.
     * @param terminal Whether or not this is the terminal state or not.
     *
     */
    void CostDerivativesFromResiduals(const struct stateVectorList &state_vector,
                                        MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu,
                                        const MatrixXd &residuals, const vector<MatrixXd> r_x, const vector<MatrixXd> r_u, bool terminal);

    /**
     * Returns whether the task has been completed yet. Distance is sometimes useful depending on the task. E.g. for
     * distance between goal object and goal position.
     *
     * @param  data_index The data index of the system.
     * @param  dist The distance between the goal and the current state of the system.
     *
     * @return bool Whether the task is complete or not.
     */
    virtual bool TaskComplete(mjData* d, double &dist);

    /**
     * Computes an initial sequence of controls for the system to setup the task to a use-able state.
     *
     * @param  horizon_length The length of the horizon to compute initial setup controls for.
     *
     * @return std::vector<MatrixXd> The sequence of control vectors to setup the task.
     *
     */
    virtual std::vector<MatrixXd> CreateInitSetupControls(int horizon_length);

    /**
     * Computes an initial sequence of controls for the system to Optimise from. This is used to
     * setup the optimisation problem.
     *
     * @param  horizon_length The length of the horizon of the optimisation problem.
     *
     * @return std::vector<MatrixXd> The sequence of control vectors to Optimise from.
     *
     */
    virtual std::vector<MatrixXd> CreateInitOptimisationControls(int horizon_length);

    /**
     * Initialise the model translator class by setting up the state and control vectors bsed on
     * the provided YAML file.
     *
     * @param  file_path File path to the YAML configuration file. This file provides high-level
     * information about the system. These files can be found in the taskConfigs folder.
     *
     */
    void InitModelTranslator(const std::string& file_path);

    /**
     * Returns the current state vector of the system in the specified data index.
     *
     * @param data_index The data index of the state vector to return.
     * @param state_vector The state vector object to use to create the state vector values.
     *
     * @return MatrixXd The current state vector of the system at the specified data index.
     *
     */
    MatrixXd ReturnStateVector(mjData* d, const struct stateVectorList &state_vector);

    /**
     * Returns the current state vector of the system, where anglular dofs are represented as
     * quaternion representation.
     *
     * @param data_index The data index of the state vector to return.
     * @param state_vector The state vector object to use to create the state vector values.
     *
     * @return MatrixXd The current state vector of the system at the specified data index.
     *
     */
    MatrixXd ReturnStateVectorQuaternions(mjData *d, const struct stateVectorList &state_vector);

    /**
     * Sets the current state vector of the system in the specified data index.
     *
     * @param state_vector_values The state vector values to set.
     * @param data_index The data index of the state vector to set.
     * @param state_vector The state vector object to use to create the state vector values.
     *
     * @return bool true if there were no issues setting the state vector. I.e the size
     * of the state vector is correct.
     *
     */
    bool SetStateVectorQuat(MatrixXd state_vector_values, mjData* d, const struct stateVectorList &state_vector);

    /**
     * Sets the current state vector of the system in the specified data index, using quaternion representation
     * for state vectors.
     *
     * @param state_vector_values The state vector values to set.
     * @param data_index The data index of the state vector to set.
     * @param state_vector The state vector object to use to create the state vector values.
     *
     * @return bool true if there were no issues setting the state vector. I.e the size
     * of the state vector is correct.
     *
     */
    bool SetStateVector(MatrixXd state_vector_values, mjData* d, const struct stateVectorList &state_vector);

    /**
     * Returns the current control vector of the system in the specified data index.
     *
     * @param data_index The data index of the control vector to return.
     * @param state_vector The state vector object to use to create the state vector values.
     *
     * @return MatrixXd The current control vector of the system at the specified data index.
     *
     */
    MatrixXd ReturnControlVector(mjData* d, const struct stateVectorList &state_vector);

    /**
     * Returns the control limits
     *
     * @param state_vector The state vector object to use to create the state vector values.
     *
     * @return MatrixXd The control limits of actuators present in the state vector.
     * The return length is twice the active number of controls and is oredered,
     * low limit, high limit, low limit, high limit, ...
     *
     */
    MatrixXd ReturnControlLimits(const struct stateVectorList &state_vector);

    /**
     * Sets the current control vector of the system in the specified data index.
     *
     * @param control_vector The control vector to set.
     * @param data_index The data index of the control vector to set.
     * @param state_vector The state vector object to use to create the state vector values.
     *
     * @return bool true if there were no issues setting the control vector. I.e the size
     * of the control vector is correct.
     *
     */
    bool SetControlVector(MatrixXd control_vector, mjData* d, const struct stateVectorList &state_vector);

    /**
     * Returns the current position vector of the system in the specified data index.
     *
     * @param data_index The data index of the position vector to return.
     * @param state_vector The state vector object to use to create the state vector values.
     *
     * @return MatrixXd The current position vector of the system at the specified data index.
     *
     */
    MatrixXd ReturnPositionVector(mjData* d, const struct stateVectorList &state_vector);

    /**
     * Returns the current position vector of the system in the specified data index. But it
     * returns angular dofs as quaternion representation (meaning that if any angular dof is
     * active, 4 numbers are required at least.
     *
     * @param data_index The data index of the position vector to return.
     * @param state_vector The state vector object to use to create the state vector values.
     *
     * @return MatrixXd The current position vector of the system at the specified data index.
     *
     */
    MatrixXd ReturnPositionVectorQuat(mjData *d, const struct stateVectorList &state_vector);

    /**
     * Returns the current velocity vector of the system in the specified data index.
     *
     * @param data_index The data index of the velocity vector to return.
     * @param state_vector The state vector object to use to create the state vector values.
     *
     * @return MatrixXd The current velocity vector of the system at the specified data index.
     *
     */
    MatrixXd ReturnVelocityVector(mjData* d, const struct stateVectorList &state_vector);

    /**
     * Returns the current acceleration vector of the system in the specified data index.
     *
     * @param data_index The data index of the acceleration vector to return.
     * @param state_vector The state vector object to use to create the state vector values.
     *
     * @return MatrixXd The current acceleration vector of the system at the specified data index.
     *
     */
    MatrixXd ReturnAccelerationVector(mjData* d, const struct stateVectorList &state_vector);

    /**
     * Sets the position vector of the system at the specified data index.
     *
     * @param data_index The data index of the jerk vector to return.
     * @param position_vector The position vector to set.
     * @param state_vector The state vector object to use to create the state vector values.
     *
     * @return bool True if there no issues setting the position vector. I.e. the size of the
     * position vector is correct.
     *
     */
    bool SetPositionVector(MatrixXd position_vector, mjData* d, const struct stateVectorList &state_vector);

    /**
     * Sets the position vector of the system at the specified data index using quaternion represntation.
     *
     * @param data_index The data index of the jerk vector to return.
     * @param position_vector The position vector to set.
     * @param state_vector The state vector object to use to create the state vector values.
     *
     * @return bool True if there no issues setting the position vector. I.e. the size of the
     * position vector is correct.
     *
     */
    bool SetPositionVectorQuat(MatrixXd position_vector, mjData* d, const struct stateVectorList &state_vector);

    /**
     * Sets the velocity vector of the system at the specified data index.
     *
     * @param data_index The data index of the jerk vector to return.
     * @param velocity_vector The velocity vector to set.
     * @param state_vector The state vector object to use to create the state vector values.
     *
     * @return bool True if there no issues setting the velocity vector. I.e. the size of the
     * velocity vector is correct.
     *
     */
    bool SetVelocityVector(MatrixXd velocity_vector, mjData* d, const struct stateVectorList &state_vector);

    /**
     * Converts a state vector index to a position vector index in MuJoCo
     *
     * @param state_index The state index to convert to a position vector index.
     * @param state_vector The state vector object to use to create the state vector values.
     *
     * @return int the position vector index in MuJoCo
     *
     */
    int StateIndexToQposIndex(int state_index, const struct stateVectorList &state_vector);

    void ComputeStateDofAdrIndices(mjData* d, const struct stateVectorList &state_vector);

    void InitialiseSystemToStartState(mjData* d);

    virtual void SetGoalVisuals(mjData *d){

    }

    // Reset the state vector reduction variables
    void ResetSVR(){
        // Updates the internal number of dofs, as well as the state vector name list
        full_state_vector.Update();

        // Set current state vector to the full state vector
        current_state_vector = full_state_vector;
        UpdateSceneVisualisation();

        unused_state_vector_elements.clear();
        candidates_for_removal.clear();

        state_dof_adr_indices.clear();
        ComputeStateDofAdrIndices(MuJoCo_helper->master_reset_data, full_state_vector);
    }

    // State vector objects and names
    struct stateVectorList current_state_vector;
    struct stateVectorList full_state_vector;

    std::vector<std::string> unused_state_vector_elements;
    std::vector<std::string> candidates_for_removal;

    std::vector<std::string> iteration_readded_state_elements;

    std::vector<int> state_dof_adr_indices;

    // mujoco helper object
    std::shared_ptr<MuJoCoHelper> MuJoCo_helper;

    // file path to model xml file
    std::string model_file_path;

    // model name
    std::string model_name = "undefined";

    // Keypoint hyper parameters
    std::string keypoint_method;
    bool auto_adjust;
    int min_N;
    int max_N;
    std::vector<double> jerk_thresholds;
    std::vector<double> accel_thresholds;
    double iterative_error_threshold;
    std::vector<double> velocity_change_thresholds;

    // openloop_horizon
    int openloop_horizon;

    // MPC horizon
    int MPC_horizon;

    vector<residual> residual_list;

//    int num_residual_terms;
//    vector<double> residual_weights;
//    vector<double> residual_weights_terminal;

    // num of dofs considered between 0 and 6

    color distractor_colors[7] = {{0.4, 0.48,    0.48, 1},
                                  {0.5, 0.4, 0.4, 1},
                                  {0.6, 0.32, 0.32, 1},
                                  {0.7, 0.24, 0.24, 1},
                                  {0.8, 0.17, 0.17, 1},
                                  {0.91, 0.08, 0.08, 1},
                                  {1, 0,    0, 1}};

    color goal_colors[7] =       {{0, 0.4, 0, 1},
                                  {0, 0.5, 0, 1},
                                  {0, 0.6, 0, 1},
                                  {0, 0.7, 0, 1},
                                  {0, 0.8, 0, 1},
                                  {0, 0.9, 0, 1},
                                  {0, 1.0, 0, 1}};

protected:

private:
};