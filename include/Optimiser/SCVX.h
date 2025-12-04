#pragma once

#include "Optimiser/Optimiser.h"
#include "Differentiator.h"
#include "Visualiser.h"
#include "FileHandler.h"
#include <algorithm>
#include <future>

//#include "osqp.h"
#include "OsqpEigen/OsqpEigen.h"
#include "Eigen/Sparse"
#include <Eigen/SparseCholesky>
#include <Eigen/OrderingMethods>

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

    // Acrobot - 5 controls per knot, 100 trust region
    // Box sweep 100 controls per knot, 1000 trust region
    void ResetParams() override{
        trust_region_radius = 1000.0;
        controls_per_knotpoint = 100;
    }

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

    bool SolveQP();

    void EvaluateLinSolutionCost();

    void SetDynamicsConstraints(Eigen::SparseMatrix<double>& linear_matrix,
                                Eigen::VectorXd& lower_bound,
                                Eigen::VectorXd& upper_bound,
                                const Eigen::VectorXd& x0);

    void AddL1TrustRegionWithResize(Eigen::SparseMatrix<double>& A,
                                    Eigen::VectorXd& l,
                                    Eigen::VectorXd& u,
                                    Eigen::SparseMatrix<double>& hessian_matrix,
                                    Eigen::VectorXd& gradient_vector,
                                    double rho);

    void SetCostFunction(Eigen::SparseMatrix<double>& hessian_matrix, Eigen::VectorXd& gradient_vector);

    void ComputeCompressedDynamics(std::vector<Eigen::MatrixXd> &Phi,
                                   std::vector<Eigen::MatrixXd> &Gamma);

    void PrintBanner(double time_rollout);

    void PrintBannerIteration(int iteration, double new_cost, double old_cost, double eps,
                              double percent_derivatives, double time_derivs, double time_qp,
                              double time_fp);

    void Iteration(int iteration_num, bool &converged, bool &failed);

    void UpdateNominal();

    // For push no clutter - controls per knot (100) and trust region (100)

    int controls_per_knotpoint = 100; // 100 works well for box sweep and push no clutter - 5 for acrobot
    std::vector<Eigen::MatrixXd> A_compressed;
    std::vector<Eigen::MatrixXd> B_compressed;

    // Visualiser object
    std::shared_ptr<Visualiser> active_visualiser;

    bool cost_reduced_last_iter = false;
    double linear_cost = 0.0;
    double non_linear_cost = 0.0;
    double trust_region_radius = 1000.0; // Used to be 100.0 - 1000 worked well when 1 control per knotpoint
    double trust_region_max = 1.0;  //TODO - what value to use?
    double Rho = 1.0;
    double beta_upper = 1.5;
    double beta_lower = 0.8;
    double rho_lower_limit = 0.2;
    double rho_upper_limit = 0.8;

    // --- QP matrices & vectors (decision vector z = [u0..u_{N-1}, x1..xN]) ---
    // QP data (Eigen)
    Eigen::SparseMatrix<double> qp_H;        // Hessian P (size n_z x n_z)
    Eigen::VectorXd qp_h;                    // gradient q (size n_z)

    Eigen::SparseMatrix<double> qp_Aeq;      // equality A_eq (n_eq x n_z)
    Eigen::VectorXd qp_beq;                  // equality rhs (n_eq)

    // Optional inequality (trust region / box)
    Eigen::SparseMatrix<double> qp_Aineq;    // inequality rows (n_ineq x n_z)
    Eigen::VectorXd qp_lineq;                // inequality lower bounds (n_ineq)
    Eigen::VectorXd qp_uineq;                // inequality upper bounds (n_ineq)

    // Solution and candidate controls
    Eigen::VectorXd qp_dz;                   // delta z (n_z)
    std::vector<Eigen::MatrixXd> qp_candidate_controls; // candidate controls U_k
    std::vector<Eigen::MatrixXd> qp_candidate_states;  // candidate states X_k
    std::vector<Eigen::MatrixXd> X_old_no_quat;

    // solver settings
    int osqp_verbose = 0;
    int osqp_max_iter = 10000;
    double osqp_eps_abs = 1e-6;
    double osqp_eps_rel = 1e-6;

};