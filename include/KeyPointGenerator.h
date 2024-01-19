/*
================================================================================
    File: KeyPointGenerator.h
    Author: David Russell
    Date: January 18, 2024
    Description:
        KeypointGenerator is a class that is sued to generate key-points over a
        trajectory by a variety of methods. These key-points determine where the
        dynamics / cost derivatives will be computed via finite-differencing.

        The remainder of the dynamics derivatives will be computed via simple
        interpolation between key-points. This saves computation time and speeds
        up gradient-based optimisation with minial trade-off if the key-points
        are chosen intelligently.

        The key-point methods currently available are:
        - Set Interval (min_N) - Places key-points uniformly over the trajectory.
        - Adaptive Jerk - (min_N, max_N, jerk_thresholds) - Places key-points
            more frequently in regions of high jerk.
        - Velocity Change - (min_N, max_N, velocity_thresholds) - Places
            key-points more frequently in regions of high velocity change.
            As well as at turning points for the velocity profile.
        - Iterative Error - (min_N, max_N, error_threshold) - Works similarly
            to exact-size cell decomposition. Starts with a coarse approximation
            and iteratively refines approximation until we believe its good.
================================================================================
*/
#pragma once

#include "StdInclude.h"
#include "Differentiator.h"

struct keypoint_method{
    std::string name;
    int min_N;
    int max_N;
    std::vector<double> jerk_thresholds;
    std::vector<double> accell_thresholds;
    double iterative_error_threshold;
    std::vector<double> velocity_change_threshold;
};

struct indexTuple{
    int startIndex;
    int endIndex;
};

class KeyPointGenerator{
public:
    /**
     * Construct a new KeyPoint Generator object.
     *
     */
    KeyPointGenerator(std::shared_ptr<Differentiator> _differentiator, std::shared_ptr<PhysicsSimulator> _physics_simulator);

    /**
     * Generates a set of key-points per degree of freedom over a trajectory depending on the
     * key-point method specified. Usually takes into account the trajectory_states of the system.
     *
     * @param  horizon The length of the trajectory.
     * @param  trajectory_states A sequence of states of the system over a trajectory.
     * @param  trajectory_controls A sequence of controls of the system over a trajectory.
     * @param  keypoint_method The method and parameters used to generate key-points.
     * @param  A A vector of matrices containing the dynamics gradients per time-step with respect
     *           to the state vector. Passed by reference so they can be updated by the "Iterative Error" method.
     * @param  B A vector of matrices containing the dynamics gradients per time-step with respect
     *          to the control vector. Passed by reference so they can be updated by the "Iterative Error" method.
     *
     * @return std::vector<std::vector<int>> A set of key-points (integer indices over the trajectory) per
     * degree of freedom.
     */
    std::vector<std::vector<int>> GenerateKeyPoints(int horizon, std::vector<MatrixXd> trajectory_states, std::vector<MatrixXd> trajec_controls,
                                                    keypoint_method keypoint_method, std::vector<MatrixXd> &A, std::vector<MatrixXd> &B);

private:

    /**
     * Loops through a trajectory of states and computes a jerk profile for each degree of freedom. Jerk
     * is the time-derivative of acceleration.
     *
     * @param  horizon The length of the trajectory.
     * @param  trajectory_states A sequence of states of the system over a trajectory.
     *
     * @return std::vector<std::vector<MatrixXd>> A Jerk profile for each degree of freedom.
     */
    std::vector<MatrixXd> GenerateJerkProfile(int horizon, std::vector<MatrixXd> trajectory_states);

    /**
     * Loops through a trajectory of states and computes an acceleration profile for each degree of freedom.
     *
     * @param  horizon The length of the trajectory.
     * @param  trajectory_states A sequence of states of the system over a trajectory.
     *
     * @return std::vector<std::vector<MatrixXd>> An acceleration profile for each degree of freedom.
     */
    std::vector<MatrixXd> GenerateAccellerationProfile(int horizon, std::vector<MatrixXd> trajectory_states);

    /**
     * Loops through a trajectory of states and computes a velocity profile for each degree of freedom. Velocity
     * is already present in the state vector, this function effectively just returns that half of the state vector.
     *
     * @param  horizon The length of the trajectory.
     * @param  trajectory_states A sequence of states of the system over a trajectory.
     *
     * @return std::vector<std::vector<MatrixXd>> A velocity profile for each degree of freedom.
     */
    std::vector<MatrixXd> GenerateVelocityProfile(int horizon, std::vector<MatrixXd> trajectory_states);

    std::vector<std::vector<int>> GenerateKeyPointsIteratively(int horizon, keypoint_method keypoint_method, std::vector<MatrixXd> trajectory_states,
                                                               std::vector<MatrixXd> &A, std::vector<MatrixXd> &B);
    bool CheckDOFColumnError(indexTuple indices, int dof_index, keypoint_method keypoint_method, int num_dofs,
                                                                std::vector<MatrixXd> &A, std::vector<MatrixXd> &B);
    std::vector<std::vector<int>> GenerateKeyPointsAdaptive(int horizon, std::vector<MatrixXd> trajecProfile, keypoint_method keypoint_method);
    std::vector<std::vector<int>> GenerateKeyPointsVelocityChange(int horizon, std::vector<MatrixXd> velProfile, keypoint_method keypoint_method);



    // Generate keypoints we will calculate derivatives at

    std::shared_ptr<Differentiator> differentiator;
    std::shared_ptr<PhysicsSimulator> physics_simulator;

    std::vector<std::vector<int>> computedKeyPoints;
};
