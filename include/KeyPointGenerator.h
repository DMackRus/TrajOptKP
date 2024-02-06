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
    bool auto_adjust;
    int min_N;
    int max_N;
    std::vector<double> jerk_thresholds;
    std::vector<double> accell_thresholds;
    double iterative_error_threshold;
    std::vector<double> velocity_change_thresholds;
};

struct index_tuple{
    int start_index;
    int end_index;
};

class KeypointGenerator{
public:
    /**
     * Construct a new KeyPoint Generator object.
     *
     */
    KeypointGenerator(std::shared_ptr<Differentiator> _differentiator,
                      std::shared_ptr<PhysicsSimulator> _physics_simulator);

    /**
     * Returns the current active keypoint method
     *
     * @return keypoint_method
     */
    keypoint_method ReturnCurrentKeypointMethod();

    void SetKeypointMethod(keypoint_method method);


    /**
     * Generates a set of key-points per degree of freedom over a trajectory depending on the
     * key-point method specified. Usually takes into account the trajectory_states of the system.
     *
     * @param  horizon The length of the trajectory.
     * @param  trajectory_states A sequence of states of the system over a trajectory.
     * @param  trajectory_controls A sequence of controls of the system over a trajectory.
     * @param  A A vector of matrices containing the dynamics gradients per time-step with respect
     *           to the state vector. Passed by reference so they can be updated by the "Iterative Error" method.
     * @param  B A vector of matrices containing the dynamics gradients per time-step with respect
     *          to the control vector. Passed by reference so they can be updated by the "Iterative Error" method.
     *
     * @return std::vector<std::vector<int>> A set of key-points (integer indices over the trajectory) per
     * degree of freedom.
     */
    std::vector<std::vector<int>> GenerateKeyPoints(int horizon, std::vector<MatrixXd> trajectory_states, std::vector<MatrixXd> trajec_controls,
                                                    std::vector<MatrixXd> &A, std::vector<MatrixXd> &B);

    void AdjustKeyPointMethod(double old_cost, double new_cost, std::vector<MatrixXd> &trajectory_states);

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

    /**
     * Computes keypoints over a trajectory per degree of freedom. THis method begins with a coarse approximations
     * of the dynamics derivatives (Usually just the first and last time-step computed via finite-differencing (F.D)). All other
     * time-steps are then computed via interpolation between these two points. This method then computes the mid points
     * via interpolation and exactly via F.D and checks the error between them. If the error is above "Iterative_Error_Threshold"
     * Then we subdivide our approximation for that degree of freedom. This process is repeated until the error is below
     * "Iterative_Error_Threshold" for all degrees of freedom, over all segments of the trajectory.
     *
     * @param  horizon The length of the trajectory.
     * @param  trajectory_states A sequence of states of the system over a trajectory.
     * @param  A A vector of matrices containing the dynamics gradients per time-step with respect to the state vector.
     *           This values is passed by reference as this method actually performs some F.D computations and we
     *           might as well store them.
     * @param  B A vector of matrices containing the dynamics gradients per time-step with respect to the control vector.
     *          This values is passed by reference as this method actually performs some F.D computations and we
     *          might as well store them.
     *
     * @return std::vector<std::vector<MatrixXd>> A set of key-points (integer indices over the trajectory) per degree of freedom.
     */
    std::vector<std::vector<int>> GenerateKeyPointsIteratively(int horizon, std::vector<MatrixXd> trajectory_states,
                                                               std::vector<MatrixXd> &A, std::vector<MatrixXd> &B);

    /**
     * This method is a helper function for the "GenerateKeyPointsIteratively" method. It computes the error between an approximation and
     * actual column of the dynamics gradient matrix. If the error is above "Iterative_Error_Threshold" then we subdivide the approximation
     * for that degree of freedom.
     *
     * @param indices Start and end index of the current linear approximation.
     * @param dof_index The current degree of freedom of index that we are computing the error for.
     * @param num_dofs The number of dofs in the system, important so we update the correct column of the dynamics gradient matrix.
     * @param A A vector of all the dynamics gradients matrix with respect to the state vector. Since this method has to compute
     *         some F.D values, we cache them for later so we dont need to re-compute them.
     * @param B A vector of all the dynamics gradients matrix with respect to the control vector. Since this method has to compute
     *          some F.D values, we cache them for later so we dont need to re-compute them.
     *
     * @return true if error < "Iterative_Error_Threshold", false otherwise.
     */
    bool CheckDOFColumnError(index_tuple indices, int dof_index, int num_dofs,
                             std::vector<MatrixXd> &A, std::vector<MatrixXd> &B);

    /**
     * This method of generating keypoints considers some dynamic quality of the system (acceleration or jerk) and
     * loops through this profile. It assigns keypoints per degree of freedom more frequently when this dynamic quality
     * exceeds some threshold, as defined by keypoint_method. This method is not iterative, it is a one pass method.
     * Keypoints cannot be located closer than "min_N" steps apart, and must be located at msot "max_N" steps apart.
     *
     * @param horizon The horizon of the trajectory.
     * @param trajec_profile The dynamics quality we are currently assessing, either acceleration or jerk., for each degree of freedom.
     *
     * @return std::vector<std::vector<int>> A set of key-points (integer indices over the trajectory) per degree of freedom.
     */
    std::vector<std::vector<int>> GenerateKeyPointsAdaptive(int horizon, std::vector<MatrixXd> trajec_profile);

    /**
     * This method of generating keypoints considers the velocity profile for each degree of freedom. When the velocity has changed substantially
     * since the last keypoint, we assign a new keypoint. We also assign keypoints when we detect the velocity changes direction (turning points).
     * Keypoints cannot be located closer than "min_N" steps apart, and must be located at msot "max_N" steps apart.
     *
     * @param horizon The horizon of the trajectory.
     * @param velocity_profile A velocity profile (per degree of freedom) over the trajectory.
     *
     * @return std::vector<std::vector<int>> A set of key-points (integer indices over the trajectory) per degree of freedom.
     */
    std::vector<std::vector<int>> GenerateKeyPointsVelocityChange(int horizon, std::vector<MatrixXd> velocity_profile);

    void UpdateLastPercentageDerivatives(std::vector<std::vector<int>> keypoints, int horizon);

    // Differentiator object, computes specific columns of the A and B matrices as desired.
    std::shared_ptr<Differentiator> differentiator;

    // Physics simulator object, computes the dynamics of the system.
    std::shared_ptr<PhysicsSimulator> physics_simulator;

    // Stored keypoints for the iterative error method so we know where we have already computed keypoints. Prevents recomputation.
    std::vector<std::vector<int>> computed_keypoints;


    std::vector<double> last_percentages;
    int dof = 9;

    // Current keypoint method
    keypoint_method current_keypoint_method;

    double desired_percentage_derivative_adjustment_factor = 1.1;
    bool auto_adjust_initialisation_occured = false;
};
