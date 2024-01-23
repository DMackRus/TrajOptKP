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

        This class uses approximation of dynamics derivatives via KeyPoint Generator
        class to speed up dynamics derivative computation.
================================================================================
*/
#pragma once

#include "Optimiser.h"
#include "Differentiator.h"
#include "Visualiser.h"
#include "FileHandler.h"
#include <algorithm>

class iLQR: public Optimiser{
public:
    /**
     * Construct a new iLQR optimiser object.
     *
     */
    iLQR(std::shared_ptr<ModelTranslator> _modelTranslator,
         std::shared_ptr<PhysicsSimulator> _physicsSimulator,
         std::shared_ptr<Differentiator> _differentiator,
         int _maxHorizon,
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
    double RolloutTrajectory(int initial_data_index, bool save_states, std::vector<MatrixXd> initial_controls) override;

    /**
     * Optimise the current trajectory until convergence, or max iterations has been reached. Uses the normal iLQR algorithm
     * to optimsie the trajectory. Step 1 - Compute derivatives, Step 2 - backwards pass, Step 3 - forwards pass with linesearch.
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
    std::vector<MatrixXd> Optimise(int initial_data_index, std::vector<MatrixXd> initial_controls, int max_iterations, int min_iterations, int horizon_length) override;

    /**
     * Compute the new optimal control feedback law K and k from the end of the trajectory to the beginning.
     *
     * @return bool - true if successful (all matrices were P.D), false otherwise.
     */
    bool BackwardsPassQuuRegularisation();

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
     * @param old_cost - Previous cost of the old trajectory.
     *
     * @return double - The cost of the new trajectory.
     */
    double ForwardsPass(double old_cost);

    /**
     * Rollout the new feedback law from the starting state of optimisation. This function performs a line search
     * in parallel over different alpha values to try find a new optimal sequence of controls.
     *
     * @param old_cost - Previous cost of the old trajectory.
     *
     * @return double - The cost of the new trajectory.
     */
    double ForwardsPassParallel(double old_cost);

    // Whether to save trajectory information to file
    bool save_trajec_information = false;

private:
    // Lambda value which is added to the diagonal of the Q_uu matrix for regularisation purposes.
    double lambda = 0.1;
    double max_lambda = 10.0;
    double min_lambda = 0.01;
    double lambda_factor = 10;

    // Max horizon of optimisation.
    int maxHorizon = 0;

    // Feedback gains matrices
    // open loop feedback gains
    vector<MatrixXd> k;
    // State dependant feedback matrices
    vector<MatrixXd> K;

    // Visualiser object
    std::shared_ptr<Visualiser> active_visualiser;

    // Control vector for parallel forwards pass rollout.
    vector<vector<MatrixXd>> U_alpha;
};