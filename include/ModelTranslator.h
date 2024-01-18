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

#include "stdInclude.h"
#include "MuJoCoHelper.h"
#include "fileHandler.h"

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
    ModelTranslator();

    //--------------------------------------------------------------------------------
    // Pure virtual functions that HAVE to be implemented by the child class
    //--------------------------------------------------------------------------------

    /**
     * Returns a random start state for the system. This is used mainly for generating
     * mass testing data.
     *
     * @return MatrixXd A random start state for the system.
     */
    virtual MatrixXd ReturnRandomStartState() = 0;

    /**
     * Returns a random goal state for the system. This is used mainly for generating
     * mass testing data.
     *
     * @return MatrixXd A random goal state for the system.
     */
    virtual MatrixXd ReturnRandomGoalState(MatrixXd X0) = 0;

    /**
     * Generates a random goal and start state for the system. This is used mainly for
     * generating mass testing data.
     *
     */
    virtual void GenerateRandomGoalAndStartState() = 0;

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
    virtual double CostFunction(int data_index, bool terminal);

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
    virtual void CostDerivatives(int data_index, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal);

    /**
     * Returns whether the task has been completed yet. Distance is sometimes useful depending on the task. E.g. for
     * distance between goal object and goal position.
     *
     * @param  data_index The data index of the system.
     * @param  dist The distance between the goal and the current state of the system.
     *
     * @return bool Whether the task is complete or not.
     */
    virtual bool TaskComplete(int data_index, double &dist);

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
     * Computes an initial sequence of controls for the system to optimise from. This is used to
     * setup the optimisation problem.
     *
     * @param  horizon_length The length of the horizon of the optimisation problem.
     *
     * @return std::vector<MatrixXd> The sequence of control vectors to optimise from.
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
    void InitModelTranslator(std::string file_path);

    /**
     * Returns the current state vector of the system in the specified data index.
     *
     * @param  data_index The data index of the state vector to return.
     *
     * @return MatrixXd The current state vector of the system at the specified data index.
     *
     */
    MatrixXd ReturnStateVector(int data_index);

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
    bool SetStateVector(MatrixXd state_vector, int data_index);

    /**
     * Returns the current control vector of the system in the specified data index.
     *
     * @param  data_index The data index of the control vector to return.
     *
     * @return MatrixXd The current control vector of the system at the specified data index.
     *
     */
    MatrixXd ReturnControlVector(int data_index);

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
    bool SetControlVector(MatrixXd control_vector, int data_index);

    /**
     * Returns the current position vector of the system in the specified data index.
     *
     * @param  data_index The data index of the position vector to return.
     *
     * @return MatrixXd The current position vector of the system at the specified data index.
     *
     */
    MatrixXd returnPositionVector(int data_index);

    /**
     * Returns the current velocity vector of the system in the specified data index.
     *
     * @param  data_index The data index of the velocity vector to return.
     *
     * @return MatrixXd The current velocity vector of the system at the specified data index.
     *
     */
    MatrixXd returnVelocityVector(int data_index);

    /**
     * Returns the current acceleration vector of the system in the specified data index.
     *
     * @param  data_index The data index of the acceleration vector to return.
     *
     * @return MatrixXd The current acceleration vector of the system at the specified data index.
     *
     */
    MatrixXd returnAccelerationVector(int data_index);

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
    bool setPositionVector(MatrixXd position_vector, int data_index);

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
    bool setVelocityVector(MatrixXd velocity_vector, int data_index);


    // Number of degrees of freedom of the system (Note, this is set by used via yaml file, it doesnt necessary
    // include all dofs for every object, if they are intentionally left out.
    int dof;

    // Number of actuated joints of the system
    int num_ctrl;

    // Size of the state vector (typically 2 x dof)
    int state_vector_size;

    // State vector object, considers robots and bodies
    struct stateVectorList state_vector;

    // Desired state, used for cost function
    MatrixXd X_desired;

    // Starting state of the system
    MatrixXd X_start;

    // physics simulator object
    std::shared_ptr<physicsSimulator> active_physics_simulator;

    // mujoco helper object
    std::shared_ptr<MuJoCoHelper> mujoco_helper;

    // file path to model xml file
    std::string model_file_path;

    // model name
    std::string model_name;

    // Keypoint hyper parameters
    std::string keypoint_method;
    int min_N;
    int max_N;
    std::vector<double> jerk_thresholds;
    std::vector<double> accel_thresholds;
    double iterative_error_threshold;
    std::vector<double> velocity_change_thresholds;

    // Cost function matrices, Q is the state cost, R is the control cost, J is the terminal cost
    DiagonalMatrix<double, Eigen::Dynamic> Q;
    DiagonalMatrix<double, Eigen::Dynamic> Q_terminal;
    DiagonalMatrix<double, Eigen::Dynamic> R;

protected:


private:

};