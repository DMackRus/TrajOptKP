#pragma once

/*
================================================================================
    File: iLQR.h
    Author: David Russell
    Date: January 23, 2024
    Description:
        My implementation of the iLQR algorithm. Heavily inspired by the paper
        "Synthesis and Stabilization of Complex Behaviors through
        Online Trajectory Optimization"
        (https://homes.cs.washington.edu/~todorov/papers/TassaIROS12.pdf).

        Some small differences, we still add lambda to Q_uu matrix diagonal instead
        of V_xx. I limit the line searches to a small set of values for speed. Lambda update
        law isn't currently quadratically updated, but perhaps it should be.

        The algorithm is as follows:
            1. Initialise the trajectory with some initial controls
            2. Rollout the trajectory and calculate the cost
            3. Compute the first order dynamics derivatives (f_x, f_u) and the
                second order cost derivatives (l_x, l_u, l_xx, l_uu).
            4. Backwards pass to calculate optimal feedback control law
            5. Forwards pass with line search to find new locally optimal trajectory
            6. Repeat steps 3-5 until convergence

        This class uses approximation of dynamics derivatives via the
        Keypoint Generator class to speed up dynamics derivative computation.
================================================================================
*/
#pragma once

#include "Optimiser/Optimiser.h"
#include "Differentiator.h"
#include "Visualiser.h"
#include "FileHandler.h"
#include <algorithm>
#include <future>

class iLQR_SVR: public Optimiser{
public:
    /**
     * Construct a new iLQR with state vector reduction optimiser object.
     *
     */
    iLQR_SVR(std::shared_ptr<ModelTranslator> _modelTranslator,
         std::shared_ptr<MuJoCoHelper> MuJoCo_helper,
         std::shared_ptr<Differentiator> _differentiator,
         int horizon,
         std::shared_ptr<Visualiser> _visualizer,
         std::shared_ptr<FileHandler> _yamlReader);

    /**
     * Rollout the trajectory from an initial starting state and control sequence. Return the cost of the trajectory.
     *
     *
     * @param initial_data_index - The data index of the simulation data which should be the starting state of this rollout.
     * @param save_states - Whether or not to save the states of the rollout to both X_old, and the simulator data vector.
     * @param initial_controls - The control sequence to apply from the initial state.
     *
     * @return double - The rolling cost of the trajectory.
     */
    double RolloutTrajectory(mjData *d, bool save_states, std::vector<MatrixXd> initial_controls) override;

    /**
     * Optimise the current trajectory until convergence, or max iterations has been reached. Uses the normal iLQR algorithm
     * to optimise the trajectory. Step 1 - Compute derivatives, Step 2 - backwards pass, Step 3 - forwards pass with linesearch.
     * Step 4 - check for convergence.
     *
     * @param initial_data_index - The data index of the simulation data which should be the starting state of optimisation.
     * @param initial_controls - The initial "warm start" trajectory to optimise from.
     * @param max_iterations - Maximum number of optimisation iterations.
     * @param min_iterations - Minimum number of optimisation iterations.
     * @param horizon_length - Horizon length to optimise to.
     *
     * @return std::vector<MatrixXd> - The new optimal control sequence.
     */
    std::vector<MatrixXd> Optimise(mjData *d, std::vector<MatrixXd> initial_controls, int max_iterations, int min_iterations, int horizon_length) override;

    void Iteration(int iteration_num, bool &converged, bool &lambda_exit);

    static void PrintBanner(double time_rollout);

    void PrintBannerIteration(int iteration, double _new_cost, double _old_cost, double eps,
                              double _lambda, int num_dofs, double percent_derivatives, double time_derivs, double time_bp,
                              double time_fp, double best_alpha);

    void Resize(int new_num_dofs, int new_num_ctrl, int new_horizon) override;

    /**
     * Compute the new optimal control feedback law K and k from the end of the trajectory to the beginning.
     *
     * @return bool - true if successful (all matrices were P.D), false otherwise.
     */
    bool BackwardsPassQuuRegularisation();

    double avg_surprise = 0.0;
    double avg_expected = 0.0;
    double new_cost = 0.0;
    double old_cost = 0.0;
    bool cost_reduced_last_iter = true;


private:
    // Lambda value which is added to the diagonal of the Q_uu matrix for regularisation purposes.
    double lambda = 0.1;
    double max_lambda = 10.0;
    double min_lambda = 0.0001;
    double lambda_factor = 10;

    // Last number of linesearches performed for print banner
    int last_iter_num_linesearches = 0;
    double last_alpha = 0.0f;

    // Feedback gains matrices
    // open loop feedback gains
    vector<MatrixXd> k;
    // State dependant feedback matrices
    vector<MatrixXd> K;

    int sampling_k_interval = 1;
    int num_dofs_readd = 1;
    double threshold_k_eigenvectors = 0.1; // maybe 0.001 or 0.0001
//    double threshold_k_eigenvectors = 0.1;

    // Open loop as high as 10k!! (when using sum method)
    // MPC - much smaller, maybe 0.1?
    std::vector<std::string> candidates_for_removal;

    double eps_acceptable_diff = 0.02;

    double delta_J = 0.0f;

    double expected = 0.0f;
    double surprise = 0.0f;
    std::vector<double> surprises;
    std::vector<double> expecteds;

    /**
     * Checks whether the supplied matrix is positive defeinite.
     *
     * @return bool - true if matrix is P.D, false otherwise.
     */
    bool CheckMatrixPD(Ref<MatrixXd> matrix);

    /**
     * Rollout the new feedback law from the starting state of optimisation. This function performs a line search
     * sequentially over different alpha values to try find a new optimal sequence of controls.
     *
     * @param _old_cost - Previous cost of the old trajectory.
     *
     * @return double - The cost of the new trajectory.
     */
    double ForwardsPass(double _old_cost);

    /**
     * Rollout the new feedback law from the starting state of optimisation. This function performs a line search
     * in parallel over different alpha values to try find a new optimal sequence of controls.
     *
     * @param thread_id - id of the thread
     * @param alpha - Linesearch parameter between 0 and 1 ofr openloop feedback
     *
     * @return double - The cost of the new trajectory.
     */
    double ForwardsPassParallel(int thread_id, double alpha);

    std::vector<std::string> LeastImportantDofs();

    /**
     * Perform a rollout with the previously computed k and K matrices but nullify feedback rows for the specified
     * dof indices. The cumulated cost is compared against the cost from the rollout using all the elements in the
     * K matrices. If the cost is similar we can reduce our state vector.
     *
     * @param dof_indices - Dof indices to nullify the K matrices for.
     * @param old_cost - Cost of the previous trajectory.
     * @param new_cost - Cost of the new trajectory, when all elements in the K matrices were used.
     * @param alpha - The alpha value used for the rollout.
     *
     * @return bool - true if the cost was similar, false otherwise.
     */
    bool RolloutWithKMatricesReduction(std::vector<int> dof_indices, double old_cost, double new_cost, double alpha);

    bool UpdateLambda(bool valid_backwards_pass);

    void UpdateNominal();

    void AdjustCurrentStateVector();

    // Visualiser object
    std::shared_ptr<Visualiser> active_visualiser;
};
