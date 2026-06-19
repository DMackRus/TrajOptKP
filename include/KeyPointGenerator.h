/*
================================================================================
    File: KeyPointGenerator.h
    Author: Anon
    Date: January 18, 2024
    Description:
        KeypointGenerator is a class that is used to generate key-points over a
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
#include <algorithm>

struct keypoint_method{
    std::string name;
    bool auto_adjust;
    int min_N;
    int max_N;
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
                      std::shared_ptr<MuJoCoHelper> MuJoCo_helper,
                      int _dof, int _horizon);

    /**
     * Returns the current active keypoint method
     *
     * @return keypoint_method
     */
    keypoint_method ReturnCurrentKeypointMethod();

    void Resize(int new_num_dofs, int new_num_ctrl, int new_horizon);

    void SetKeypointMethod(keypoint_method method);


    /**
     * Generates a set of key-points per degree of freedom over a trajectory depending on the
     * key-point method specified. Usually takes into account the trajectory_states of the system.
     *
     * @param  trajectory_states A sequence of states of the system over a trajectory.
     * @param  trajectory_controls A sequence of controls applied to the system over a trajectory.
     * @param  trajectory_contacts A sequence of contacts between bodies over a trajectory.
     * @param  state_vector_list State vector object that contains relevant kinematic chain mappings
     * @param  A A vector of matrices containing the dynamics gradients per time-step with respect
     *           to the state vector. Passed by reference so they can be updated by the "Iterative Error" method.
     * @param  B A vector of matrices containing the dynamics gradients per time-step with respect
     *          to the control vector. Passed by reference so they can be updated by the "Iterative Error" method.
     *
     * @return std::vector<std::vector<int>> A set of key-points (integer indices over the trajectory) per
     * degree of freedom.
     */
    void GenerateKeyPoints(const std::vector<MatrixXd> &trajectory_states,
                           const std::vector<MatrixXd> &trajectory_controls,
                           const std::vector<std::vector<std::pair<int, int>>> &trajectory_contacts,
                           const stateVectorList &state_vector_list,
                           std::vector<MatrixXd> &A, std::vector<MatrixXd> &B);

    void ContactAwareKeyPoints(const std::vector<MatrixXd> &trajectory_states,
                               const std::vector<MatrixXd> &trajectory_controls,
                               const std::vector<std::vector<std::pair<int, int>>> &trajectory_contacts,
                               const stateVectorList &state_vector_list);

    void ContactAwareKeypointsSep(const std::vector<MatrixXd> &trajectory_states,
                               const std::vector<MatrixXd> &trajectory_controls,
                               const std::vector<std::vector<std::pair<int, int>>> &trajectory_contacts,
                               const stateVectorList &state_vector_list);

    void AdjustKeyPointMethod(double expected, double actual,
                              std::vector<MatrixXd> &trajectory_states,
                              std::vector<double> &dof_importances);

    std::vector<double> DesiredPercentageDerivs(double expected, double actual,
                                                std::vector<double> &dof_importances);

    void ContactChangeDyn(const std::vector<std::vector<std::pair<int, int>>> &trajectory_contacts,
                          const stateVectorList &state_vector_list,
                          bool dyn_mode, bool sep_kin_chains, bool enforce_intervals);

    void PrintKeypointMethod();

    void InterpolateDerivatives(const std::vector<std::vector<int>> &keyPoints, int T,
                                   std::vector<MatrixXd> &A, std::vector<MatrixXd> &B,
                                   std::vector<std::vector<MatrixXd>> &r_x, std::vector<std::vector<MatrixXd>> &r_u,
                                   bool residual_derivs, int num_ctrl);

    void ResetCache();

    void UpdateLastPercentageDerivatives(std::vector<std::vector<int>> &keypoints);

    double surprise_lower = 0.2;

    int dof;
    int horizon;

    std::vector<std::vector<int>> keypoints;
    std::vector<double> last_percentages;

private:

    /**
     * Loops through a trajectory of states and computes a jerk profile for each degree of freedom. Jerk
     * is the time-derivative of acceleration.
     *
     * @param  trajectory_states A sequence of states of the system over a trajectory.
     *
     */
    void GenerateJerkProfile(const std::vector<MatrixXd> &trajectory_states);

    /**
     * Loops through a trajectory of states and computes a velocity profile for each degree of freedom. Velocity
     * is already present in the state vector, this function effectively just returns that half of the state vector.
     *
     * @param  trajectory_states A sequence of states of the system over a trajectory.
     *
     */
    void GenerateVelocityProfile(const std::vector<MatrixXd> &trajectory_states);

    void GenerateKeyPointsSetInterval();

    std::vector<double> ComputePercentageDerivatives(std::vector<std::vector<int>> &keypoints);

    inline std::vector<int> AddKeypointsFromContactSeparate(const std::pair<int, int>& contact,
                                                                   const std::vector<std::pair<int, int>>& all_contacts,
                                                                   const stateVectorList &state_vector_list);

    void AutoAdjustKeypointParameters(const std::vector<MatrixXd> &trajectory_states,
                                      const std::vector<int> &desired_percentages, int num_iterations);

    void GenerateKeypointsOrderOfImportance(const std::vector<MatrixXd> &trajectory_states, const std::vector<int> &num_keypoints);

    std::vector<int> ConvertPercentagesToNumKeypoints(const std::vector<double> &percentages);

    std::vector<double> ConvertNumKeypointsToPercentages(const std::vector<int> &num_keypoints);

    // Differentiator object, computes specific columns of the A and B matrices as desired.
    std::shared_ptr<Differentiator> differentiator;

    // Physics simulator object, computes the dynamics of the system.
    std::shared_ptr<MuJoCoHelper> MuJoCo_helper;
    std::shared_ptr<ModelTranslator> Model_translator;

    // Stored keypoints for the iterative error method so we know where we have already computed keypoints. Prevents recomputation.
    std::vector<std::vector<bool>> computed_keypoints;

    std::vector<int> last_num_keypoints;

    // Current keypoint method
    keypoint_method current_keypoint_method;

    bool auto_adjust_initialisation_occured = false;

    std::vector<MatrixXd> jerk_profile;
    std::vector<MatrixXd> velocity_profile;

    std::vector<double> max_last_jerk;
    std::vector<double> min_last_jerk;

    std::vector<double> max_last_velocity;
    std::vector<double> min_last_velocity;

    //Cache system to prevent recomputation
    bool keypoints_computed = false;

};
