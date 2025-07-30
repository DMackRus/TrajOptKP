#pragma once

#include "Optimiser/Optimiser.h"
#include "Differentiator.h"
#include "Visualiser.h"
#include "FileHandler.h"
#include <algorithm>
#include <future>

#include "osqp.h"
#include "Eigen/Sparse"

class SCVX: public Optimiser{
public:
    /**
     * Construct a new SCVX optimiser object.
     *
     */
    SCVX(std::shared_ptr<ModelTranslator> _modelTranslator,
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

    void Resize(int new_num_dofs, int new_num_ctrl, int new_horizon) override;

    std::string ReturnName() override{
        return "SCVX";
    }

private:

    //timing variable clock
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;

    /**
     * Rollout the new feedback law from the starting state of optimisation. This function performs a line search
     * sequentially over different alpha values to try find a new optimal sequence of controls.
     *
     * @param _old_cost - Previous cost of the old trajectory.
     *
     * @return double - The cost of the new trajectory.
     */
    double ForwardsPass(double _old_cost);

    void SolveQP(const vector<MatrixXd>& A_k,
                 const vector<MatrixXd>& B_k,
                 const vector<VectorXd>& d_k,
                 const vector<MatrixXd>& cost_hess_xx,
                 const vector<MatrixXd>& cost_hess_uu,
                 const vector<MatrixXd>& cost_hess_xu,
                 const vector<VectorXd>& cost_grad_x,
                 const vector<VectorXd>& cost_grad_u,
                 const VectorXd& terminal_grad,
                 const MatrixXd& terminal_hess,
                 int n, int m, int N,
                 double trust_box);
    void BuildEqualityConstraints();
    void BuildCostFunction();
    void BuildTrustRegion();

    void PrintBanner(double time_rollout);

    void PrintBannerIteration(int iteration, double new_cost, double old_cost, double eps,
                              double percent_derivatives, double time_derivs, double time_qp,
                              double time_fp);

    void Iteration(int iteration_num, bool &converged);

    void UpdateNominal();

    // Visualiser object
    std::shared_ptr<Visualiser> active_visualiser;

    bool cost_reduced_last_iter = false;
    double linear_cost = 0.0;
    double non_linear_cost = 0.0;
    double trust_region_radius = 0.0;
    double trust_region_max = 1.0;  //TODO - what value to use?
    double Rho = 0.0;
    double rho_lower_limit = 0.2;
    double rho_upper_limit = 0.8;

};