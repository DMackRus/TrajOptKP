/*
================================================================================
    File: PredictiveSampling.h
    Author: Anon
    Date: October 21, 2026
    Description:
        Predictive sampling optimiser, based on the algorithm described in
        "Predictive Sampling: Real-time Behaviour Synthesis with MuJoCo"
        (https://arxiv.org/abs/2212.00541).

        The control trajectory is parameterised by a set of knot points, spaced
        every "knot_spacing" time-steps over the horizon (with a final knot
        always placed at the last control index). Only the knot points are
        perturbed, intermediate controls are computed via spline interpolation
        (zero order hold, linear or cubic).

        The algorithm is as follows:
            1. Rollout the initial control trajectory and calculate its cost.
            2. Sample N noisy knot point sequences around the nominal knot points.
            3. Interpolate each knot sequence into a dense control trajectory and
                roll them all out in parallel.
            4. If the best sampled trajectory has a lower cost than the nominal,
                it becomes the new nominal.
            5. Repeat steps 2-4 until convergence or max iterations.
================================================================================
*/
#pragma once

#include "Optimiser/Optimiser.h"
#include <random>

enum class SplineType{
    ZERO_ORDER,
    LINEAR,
    CUBIC
};

class PredictiveSampling: public Optimiser{
public:
    /**
     * Construct a new Predictive Sampling optimiser object.
     *
     * @param horizon - Initial horizon length of the optimisation.
     * @param _num_rollouts - Number of noisy trajectories sampled per iteration.
     * @param _knot_spacing - Number of time-steps between knot points. If <= 0, DEFAULT_KNOT_SPACING is used.
     * @param _spline_type - Spline used to interpolate controls between knot points.
     * @param _noise_scale - Std of sampling noise, as a fraction of half of each actuator's control range.
     */
    PredictiveSampling(std::shared_ptr<ModelTranslator> _modelTranslator,
                       std::shared_ptr<MuJoCoHelper> MuJoCo_helper,
                       std::shared_ptr<FileHandler> _yamlReader,
                       std::shared_ptr<Differentiator> _differentiator,
                       int horizon,
                       int _num_rollouts = 10,
                       int _knot_spacing = DEFAULT_KNOT_SPACING,
                       SplineType _spline_type = SplineType::CUBIC,
                       double _noise_scale = 0.1);

    /**
     * Rollout the trajectory from an initial starting state and control sequence. Return the cost of the trajectory.
     *
     * @param d - The starting state of the rollout.
     * @param save_states - Whether or not to save the states of the rollout to X_old, U_old and the simulator data vector.
     * @param initial_controls - The control sequence to apply from the initial state.
     *
     * @return double - The cost of the trajectory.
     */
    double RolloutTrajectory(mjData *d, bool save_states, std::vector<MatrixXd> initial_controls) override;

    /**
     * Optimise the control trajectory via predictive sampling until convergence, or max iterations has been reached.
     *
     * @param d - The starting state of the optimisation.
     * @param initial_controls - The initial "warm start" trajectory to optimise from.
     * @param max_iterations - Maximum number of optimisation iterations.
     * @param min_iterations - Minimum number of optimisation iterations.
     * @param horizon_length - Horizon length to optimise to.
     *
     * @return std::vector<MatrixXd> - The new optimised control sequence.
     */
    std::vector<MatrixXd> Optimise(mjData *d, std::vector<MatrixXd> initial_controls, int max_iterations, int min_iterations, int horizon_length) override;

    void Resize(int new_num_dofs, int new_num_ctrl, int new_horizon) override;

    std::string ReturnName() override{
        return "PredictiveSampling";
    }

    /**
     * Set the spacing (in time-steps) between knot points. Values <= 0 reset it to DEFAULT_KNOT_SPACING.
     */
    void SetKnotSpacing(int _knot_spacing);

    void SetSplineType(SplineType _spline_type){
        spline_type = _spline_type;
    }

    /**
     * Converts a string ("zero_order", "linear" or "cubic") to a SplineType. Defaults to cubic if unrecognised.
     */
    static SplineType SplineTypeFromString(const std::string &name);

    /**
     * Interpolate a dense control trajectory (horizon_length controls) from a set of knot points, using the current
     * spline type. Interpolated controls are clamped to the control limits.
     *
     * @param knots - Control values at each knot time.
     * @param controls - Output dense control trajectory, must already be of size horizon_length.
     */
    void InterpolateControls(const std::vector<MatrixXd> &knots, std::vector<MatrixXd> &controls) const;

    static constexpr int DEFAULT_KNOT_SPACING = 10;

    int num_rollouts;
    double noise_scale;

private:
    int knot_spacing;
    SplineType spline_type;

    // Time indices of each knot point within the horizon
    std::vector<int> knot_times;

    // Nominal knot points and their sampled perturbations
    std::vector<MatrixXd> nominal_knots;
    std::vector<std::vector<MatrixXd>> sampled_knots;
    std::vector<std::vector<MatrixXd>> sampled_controls;
    std::vector<double> sampled_costs;

    // Per actuator noise standard deviation and control limits
    MatrixXd noise_std;
    MatrixXd control_limits;

    // One random generator per sampled rollout, so samples can be generated in parallel
    std::vector<std::mt19937> generators;

    // Parallel rollout bookkeeping
    std::atomic<int> next_rollout;

    void ComputeKnotTimes();

    void SampleKnots(int rollout_index);

    double RolloutSample(int rollout_index, int thread_id);

    void WorkerRolloutSamples(int thread_id);

    void ClampControl(MatrixXd &control) const;

    void PrintBanner(double time_rollout);

    void PrintBannerIteration(int iteration, double best_cost, double best_sample_cost, double time_iteration);
};
