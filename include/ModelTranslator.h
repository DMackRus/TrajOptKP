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

        The data_index concept is used a lot in this file. I store the full
        system state of the simulator at indices in a buffer in the abstract
        physics simulator class. data index can be a number between 0-> horizon - 1.
        As well as some "special" situations like MAIN_DATA_STATE, VISUALISATION_DATA,
        AND MASTER_RESET_DATA. These are defined in the physics simulator class.

        Some of the functions can be overwritten to provide custom behaviour
        ,especially useful for defining custom cost functions and initial controls.
================================================================================
*/
#pragma once

#include "StdInclude.h"
#include "MuJoCoHelper.h"
#include "FileHandler.h"

enum clutterLevels{
    noClutter = 0,
    lowClutter = 1,
    heavyClutter = 2,
    constrainedClutter = 3,
    clutter_realWorld = 4
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
    void UpdateStateVector(std::vector<std::string> state_vector_names, bool add_extra_states);

    /**
     * Returns the current names of the state vector elements in order.
     *
     * @return std::vector<std::string> The names of the active state vector elements.
     *
     */
    std::vector<std::string> GetStateVectorNames();

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

    /**
     * Returns the current cost of the system at the given data index.
     *
     * @param  data_index The data index of the system to calculate the cost for.
     * @param  terminal Whether or not this is the terminal state or not.
     *
     * @return double The cost of the system at the given data index.
     */
    virtual double CostFunction(mjData* d, bool terminal);

    double CostFunctionBody(const bodyStateVec body, mjData *d, bool terminal);

    /**
     * Returns the current cost derivatives (1st and 2nd order) of the system with respect to the
     * state and control vectors at the given data index.
     *
     * @param  data_index The data index of the system to calculate the cost for.
     * @param  l_x The first order cost derivative with respect to the state vector. Passed by reference.
     * @param  l_xx The second order cost derivative with respect to the state vector. Passed by reference.
     * @param  l_u The first order cost derivative with respect to the control vector. Passed by reference.
     * @param  l_uu The second order cost derivative with respect to the control vector. Passed by reference.
     * @param  terminal Whether or not this is the terminal state or not.
     *
     */
    virtual void CostDerivatives(mjData* d, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal);

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
     * @param  data_index The data index of the state vector to return.
     *
     * @return MatrixXd The current state vector of the system at the specified data index.
     *
     */
    MatrixXd ReturnStateVector(mjData* d);

    /**
     * Sets the current state vector of the system in the specified data index.
     *
     * @param  state_vector The state vector to set.
     * @param  data_index The data index of the state vector to set.
     *
     * @return bool true if there were no issues setting the state vector. I.e the size
     * of the state vector is correct.
     *
     */
    bool SetStateVector(MatrixXd state_vector, mjData* d);

    /**
     * Returns the current control vector of the system in the specified data index.
     *
     * @param  data_index The data index of the control vector to return.
     *
     * @return MatrixXd The current control vector of the system at the specified data index.
     *
     */
    MatrixXd ReturnControlVector(mjData* d);

    /**
     * Returns the control limits
     *
     * @return MatrixXd The control limits of actuators present in the state vector.
     * The return length is twice the active number of controls and is oredered,
     * low limit, high limit, low limit, high limit, ...
     *
     */
    MatrixXd ReturnControlLimits();

    /**
     * Sets the current control vector of the system in the specified data index.
     *
     * @param  control_vector The control vector to set.
     * @param  data_index The data index of the control vector to set.
     *
     * @return bool true if there were no issues setting the control vector. I.e the size
     * of the control vector is correct.
     *
     */
    bool SetControlVector(MatrixXd control_vector, mjData* d);

    /**
     * Returns the current position vector of the system in the specified data index.
     *
     * @param  data_index The data index of the position vector to return.
     *
     * @return MatrixXd The current position vector of the system at the specified data index.
     *
     */
    MatrixXd returnPositionVector(mjData* d);

    /**
     * Returns the current velocity vector of the system in the specified data index.
     *
     * @param  data_index The data index of the velocity vector to return.
     *
     * @return MatrixXd The current velocity vector of the system at the specified data index.
     *
     */
    MatrixXd returnVelocityVector(mjData* d);

    /**
     * Returns the current acceleration vector of the system in the specified data index.
     *
     * @param  data_index The data index of the acceleration vector to return.
     *
     * @return MatrixXd The current acceleration vector of the system at the specified data index.
     *
     */
    MatrixXd returnAccelerationVector(mjData* d);

    /**
     * Sets the position vector of the system at the specified data index.
     *
     * @param  data_index The data index of the jerk vector to return.
     * @param  position_vector The position vector to set.
     *
     * @return bool True if there no issues setting the position vector. I.e. the size of the
     * position vector is correct.
     *
     */
    bool setPositionVector(MatrixXd position_vector, mjData* d);

    /**
     * Sets the velocity vector of the system at the specified data index.
     *
     * @param  data_index The data index of the jerk vector to return.
     * @param  velocity_vector The velocity vector to set.
     *
     * @return bool True if there no issues setting the velocity vector. I.e. the size of the
     * velocity vector is correct.
     *
     */
    bool setVelocityVector(MatrixXd velocity_vector, mjData* d);

    /**
     * Converts a state vector index to a position vector index in MuJoCo
     *
     * @param  state_index The state index to convert to a position vector index.
     *
     * @return int the position vector index in MuJoCo
     *
     */
    int StateIndexToQposIndex(int state_index);

    void InitialiseSystemToStartState(mjData* d);


    // Number of degrees of freedom of the system (Note, this is set by used via yaml file, it doesnt necessary
    // include all dofs for every object, if they are intentionally left out.
    int dof = 0;

    // Number of actuated joints of the system
    int num_ctrl = 0;

    // Size of the state vector (typically 2 x dof)
    int state_vector_size = 0;

    // State vector object, considers robots and bodies
    struct stateVectorList active_state_vector;
    std::vector<std::string> state_vector_names;

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

    // Cost function matrices, Q is the state cost, R is the control cost, J is the terminal cost
    DiagonalMatrix<double, Eigen::Dynamic> Q;
    DiagonalMatrix<double, Eigen::Dynamic> Q_terminal;
    DiagonalMatrix<double, Eigen::Dynamic> R;

protected:

private:
};