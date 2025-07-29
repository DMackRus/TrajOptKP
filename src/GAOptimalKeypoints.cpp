//#include "GAOptimalKeypoints.h"
//
//GAOptimalKeypoints::GAOptimalKeypoints(std::shared_ptr<ModelTranslator> _model_translator,
//                                       std::shared_ptr<MuJoCoHelper> _MuJoCo_helper,
//                                       std::shared_ptr<FileHandler> _yamlReader,
//                                       std::shared_ptr<Optimiser> _optimiser){
//    model_translator = _model_translator;
//    MuJoCo_helper = _MuJoCo_helper;
//    yamlReader = _yamlReader;
//    optimiser = _optimiser;
//
//    // Genome size = number of dofs (threshold limits) + minN + maxN
//    genome_size = model_translator->full_state_vector.dof + 2;
//}
//
//int GAOptimalKeypoints::Run(){
//
//    // TODO - add some timing mechanism and useful print statements to inform how long is left roughly
//
//    // Make sure population is even
//    if(population_size % 2 != 0){
//        population_size--;
//    }
//
//    // Initialise data logging
//    data_logging data;
//
//    // Initialise Solutions
//    vector<solution> solutions(population_size);
//    RandomlyInitPopulation(solutions);
//
//    // Start timing variable
//    auto start_time = std::chrono::high_resolution_clock::now();
//    auto current_time = std::chrono::high_resolution_clock::now();
//
//    // Loop for a number of generations
//    for(int i = 0; i < num_generations; i++){
//        // Randomly create task instantiations
//        // Random start states and goal states
//        // TODO - might need to refactor how random start state and goal generation is performed
//        vector<vector<double>> tasks(num_tasks, vector<double>(model_translator->full_state_vector.dof));
//        for(int j = 0; j < num_tasks; j++){
//            tasks[j][0] = randFloat(PI - 0.1, PI + 0.1);
//            tasks[j][1] = randFloat(-0.1, 0.1);
////            for(int k = 0; k < model_translator->full_state_vector.dof; k++){
////                tasks[j][k] = randFloat(-1, 1);
////            }
//        }
//
////        vector<double> baseline_cost_reductions = EvaluateBaselineMethodOverTasks(tasks);
//        vector<double> baseline_cost_reductions = vector<double>(num_tasks, 0.0);
//
//        // Loop through the population
//        for(int j = 0; j < population_size; j++){
//            vector<double> cost_reductions(num_tasks);
//            vector<double> percentage_derivatives(num_tasks);
//
//            // TODO - think about the naming of these two functions
//            EvaluateKeypointMethodOverTasks(solutions[j], tasks);
//            EvaluateSolutionCost(solutions[j], baseline_cost_reductions);
//        }
//
//        std::cout << "pop fitnesses: ";
//        for(int j = 0; j < population_size; j++){
//            std::cout << solutions[j].fitness << " ";
//        }
//        std::cout << endl;
//
//        // Data logging
//        UpdateDataLogging(data, solutions);
//
//        std::cout << "Average baseline CR " << std::accumulate(baseline_cost_reductions.begin(),
//                baseline_cost_reductions.end(), 0.0) / static_cast<double>(baseline_cost_reductions.size()) << std::endl;
//
//        // Select parents (winners)
//        vector<solution> parents = TournamentSelectParents(solutions, 10);
//
//        // Generate children
//        vector<solution> new_solutions;
//        for (size_t k = 0; k + 1 < parents.size(); k += 2) {
//            auto [child1, child2] = Crossover(parents[k], parents[k+1]);
//            Mutation(child1);
//            Mutation(child2);
//            new_solutions.push_back(child1);
//            new_solutions.push_back(child2);
//        }
//
//        // Elitism: copy best genome to next generation
//        auto best_it = std::min_element(solutions.begin(), solutions.end(),
//                                        [](const solution& a, const solution& b) {
//                                            return a.fitness < b.fitness;
//                                        });
//
//        int best_idx = std::distance(solutions.begin(), best_it);
//
//        // ----------- Print best keypoint methods ------------------------
//        keypoint_method genome_method;
////        genome_method.name = "velocity_change";
//        genome_method.name = "adaptive_jerk";
//        genome_method.min_N = static_cast<int>(solutions[best_idx].genome[0]);
//        genome_method.max_N = static_cast<int>(solutions[best_idx].genome[1]);
//        genome_method.velocity_change_thresholds.resize(genome_size);
//        for(int k = 2; k < genome_size; k++){
//            genome_method.velocity_change_thresholds[k-2] = solutions[best_idx].genome[k];
//        }
//        optimiser->keypoint_generator->SetKeypointMethod(genome_method);
//        optimiser->keypoint_generator->PrintKeypointMethod();
//        // -----------------------------------------------------------------
//
//
//        // TODO enable elitism of arbritary number
//        new_solutions[0] = solutions[best_idx]; // Replace first genome with elite
//
//        // Add random survivors - Better for exploration
//        for (int k = 1; k < explorer_count+1; ++k) {
//            int idx = rand() % solutions.size();
//            new_solutions[k] = solutions[idx];
//        }
//
//        solutions = std::move(new_solutions);
//
//        current_time = std::chrono::high_resolution_clock::now();
//        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
//
//        std::cout << "Generation " << i << " completed in " << elapsed_time << " seconds." << std::endl;
//
//        // Compute roughly how much time is left
//        auto remaining_time = (num_generations - i) * elapsed_time / (i + 1);
//        std::cout << "Estimated time remaining: " << remaining_time << " seconds." << std::endl;
//    }
//
//    // ----------------------- Save data to file -------------------------------------
//    // Go back two directories
//    std::string project_parent_path = __FILE__;
//    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));
//    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));
//
//    std::string task_prefix = model_translator->model_name;
//
//    std::string root_path = project_parent_path + "/TestingData";
//
//    // Check if optimiser directory exists
//    if (!filesystem::exists(root_path)) {
//        if (!filesystem::create_directories(root_path)) {
//            std::cerr << "Failed to create directory: " << root_path << std::endl;
//        }
//    }
//
////    std::string method_directory = root_path + "/" + task_prefix + "_fitness_tracking.csv";
//
//    // Check if method directory exists, if not create it
////    if (!filesystem::exists(method_directory)) {
////        if (!filesystem::create_directories(method_directory)) {
////            std::cerr << "Failed to create directory: " << method_directory << std::endl;
////            exit(1);
////        }
////    }
//    std::string filename = root_path + "/" + task_prefix + "_fitness_tracking.csv";
//
//    ofstream file_output;
//    file_output.open(filename);
//
//    // Make header
//    file_output << "Best fitness" << "," << "Average fitness" << "," << "Worst fitness" << ",";
//    file_output << "Best cost reduction" << "," << "Average cost reduction" << "," << "Worst cost reduction" << ",";
//    file_output << "Best percent derivatives" << "," << "Average percent derivatives" << "," << "Worst percent derivatives" << std::endl;
//
//    // Loop through rows
//    for(int i = 0; i < data.best_pop_fitness.size(); i++){
//        file_output << data.best_pop_fitness[i] << "," << data.average_pop_fitness[i] << "," << data.worst_pop_fitness[i] << ",";
//        file_output << data.best_pop_cost_reduction[i] << "," << data.average_pop_cost_reduction[i] << "," << data.worst_pop_cost_reduction[i] << ",";
//        file_output << data.best_pop_percent_derivs[i] << "," << data.average_pop_percent_derivs[i] << "," << data.worst_pop_percent_derivs[i] << std::endl;
//    }
//
//    file_output.close();
//
//    return EXIT_SUCCESS;
//}
//
//void GAOptimalKeypoints::UpdateDataLogging(data_logging& data, const vector<solution>& solutions){
//
//    //--------------------------- Fitness calculations --------------------------------
//
//    auto best_it = std::min_element(solutions.begin(), solutions.end(),
//                                    [](const solution& a, const solution& b) {
//                                        return a.fitness < b.fitness;
//                                    });
//
//    int best_idx = std::distance(solutions.begin(), best_it);
//
//    // Data logging
//    auto worst_it = std::min_element(solutions.begin(), solutions.end(),
//                                     [](const solution& a, const solution& b) {
//                                         return a.fitness > b.fitness;
//                                     });
//    int worst_idx = std::distance(solutions.begin(), worst_it);
//
//    double average_fitness = std::accumulate(
//            solutions.begin(), solutions.end(), 0.0,
//            [](double sum, const solution& s) {
//                return sum + s.fitness;
//            }) / static_cast<double>(solutions.size());
//
//    data.best_pop_fitness.push_back(solutions[best_idx].fitness);
//    data.worst_pop_fitness.push_back(solutions[worst_idx].fitness);
//    data.average_pop_fitness.push_back(average_fitness);
//
//    // ------------------------ Cost reduction calculations --------------------------
//    vector<double> average_cost_reductions, average_percent_derivatives;
//    // Compute average cost reduction for each solutions
//    for(auto solution : solutions){
//
//        double average_cr = std::accumulate(solution.cost_reductions.begin(),
//                                                         solution.cost_reductions.end(),
//                                                         0.0) / static_cast<double>(solution.cost_reductions.size());
//        average_cost_reductions.push_back(average_cr);
//
//        double average_pd = std::accumulate(solution.percentage_derivatives.begin(),
//                                            solution.percentage_derivatives.end(),
//                                            0.0) / static_cast<double>(solution.percentage_derivatives.size());
//        average_percent_derivatives.push_back(average_pd);
//    }
//
//    double average_cr_population = std::accumulate(average_cost_reductions.begin(),
//                                                   average_cost_reductions.end(),
//                                                   0.0) / static_cast<double>(average_cost_reductions.size());
//
//    double average_pd_population = std::accumulate(average_percent_derivatives.begin(),
//                                                   average_percent_derivatives.end(),
//                                                   0.0) / static_cast<double>(average_percent_derivatives.size());
//
//    data.average_pop_cost_reduction.push_back(average_cr_population);
//    data.best_pop_cost_reduction.push_back(average_cost_reductions[best_idx]);
//    data.worst_pop_cost_reduction.push_back(average_cost_reductions[worst_idx]);
//
//    data.average_pop_percent_derivs.push_back(average_pd_population);
//    data.best_pop_percent_derivs.push_back(average_percent_derivatives[best_idx]);
//    data.worst_pop_percent_derivs.push_back(average_percent_derivatives[worst_idx]);
//
//    std::cout << "best pop fitness: " << solutions[best_idx].fitness << " average pop fitness: " << average_fitness << " worst pop fitness: " << solutions[worst_idx].fitness << std::endl;
//
//    std::cout << "cost reduction: ";
//    for(int i = 0; i < solutions.size(); i++){
//        std::cout << average_cost_reductions[i] << " ";
//    }
//    std::cout << std::endl;
//
//    std::cout << "percent derivatives: ";
//    for(int i = 0; i < solutions.size(); i++){
//        std::cout << average_percent_derivatives[i] << " ";
//    }
//    std::cout << std::endl;
//
//}
//
//vector<double> GAOptimalKeypoints::EvaluateBaselineMethodOverTasks(vector<vector<double>> &tasks){
//    // TODO - temp, make it from yaml reader later
////    int task_horizon = 900;
//
//    vector<double> baseline_cost_reductions;
//
//    // ---------- Set Keypoint method (baseline) --------------------------
//    keypoint_method genome_method = optimiser->ReturnCurrentKeypointMethod();
//    genome_method.name = "set_interval";
//    genome_method.min_N = static_cast<int>(1);
//    genome_method.max_N = static_cast<int>(1);
//    for(int i = 0; i < genome_size - 2; i++){
//        genome_method.velocity_change_thresholds[i] = 0;
//    }
//    optimiser->keypoint_generator->SetKeypointMethod(genome_method);
//
//    for (int i = 0; i < num_tasks; i++) {
//        optimiser->verbose_output = false;
//
////        if(i == 0){
////            std::cout << "BASELINE evaluation task " << i << std::endl;
////            optimiser->verbose_output = true;
////        }
//
//
//        // Reset internal optimisation data and clear key-points cache
//        optimiser->Reset();
//        optimiser->keypoint_generator->ResetCache();
//        // Load the task - use random function to prevent over-fitting
////        model_translator->GenerateRandomGoalAndStartState();
//
//        // Load the task from CSV file
//        yamlReader->LoadTaskFromFile(model_translator->model_name,
//                                     i, model_translator->full_state_vector,
//                                     model_translator->residual_list);
//
//        // Load the task from vector
////        model_translator->full_state_vector.robots[0].start_pos[0] = tasks[i][0];
////        model_translator->full_state_vector.robots[0].start_pos[1] = tasks[i][1];
//
//        // Reset state vector (only really applicable for iLQR_SVR method)
//        model_translator->ResetSVR();
//        model_translator->InitialiseSystemToStartState(model_translator->MuJoCo_helper->master_reset_data);
//
//        // Setup mj data objects
//        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->main_data,
//                                                         model_translator->MuJoCo_helper->master_reset_data);
//        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->vis_data,
//                                                         model_translator->MuJoCo_helper->master_reset_data);
//
//
//        mj_step(model_translator->MuJoCo_helper->model, model_translator->MuJoCo_helper->master_reset_data);
//
//        if (!model_translator->MuJoCo_helper->CheckIfDataIndexExists(0)) {
//            model_translator->MuJoCo_helper->AppendSystemStateToEnd(
//                    model_translator->MuJoCo_helper->master_reset_data);
//        }
//
//        // Perform any setup controls for this task
//        std::vector<MatrixXd> initSetupControls = model_translator->CreateInitSetupControls(1000);
//        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->master_reset_data,
//                                                         model_translator->MuJoCo_helper->main_data);
//        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->main_data,
//                                                         model_translator->MuJoCo_helper->master_reset_data);
//        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->vis_data,
//                                                         model_translator->MuJoCo_helper->master_reset_data);
//
//        // Create init optimisation controls
//        std::vector<MatrixXd> init_opt_controls = model_translator->CreateInitOptimisationControls(task_horizon);
//        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->main_data,
//                                                         model_translator->MuJoCo_helper->master_reset_data);
//        model_translator->MuJoCo_helper->CopySystemState(
//                model_translator->MuJoCo_helper->saved_systems_state_list[0],
//                model_translator->MuJoCo_helper->master_reset_data);
//
//        // Do the optimisation!
//        optimiser->lambda = 0.01;
//        std::vector<MatrixXd> optimised_controls = optimiser->Optimise(
//                model_translator->MuJoCo_helper->saved_systems_state_list[0],
//                init_opt_controls, opt_iters, opt_iters,
//                task_horizon);
//
////        solution.cost_reductions[i] = optimiser->cost_reduction;
////        solution.percentage_derivatives[i] = optimiser->avg_percent_derivs;
//        baseline_cost_reductions.push_back(optimiser->cost_reduction);
//    }
//
//    return baseline_cost_reductions;
//}
//
//void GAOptimalKeypoints::EvaluateKeypointMethodOverTasks(solution &solution, vector<vector<double>> &tasks){
//
////    int opt_horizon = yamlReader.
//    // TODO - temp, make it from yaml reader later
////    int task_horizon = 900;
//
//    // ---------- Set Keypoint method (genome) --------------------------
//    keypoint_method genome_method = optimiser->ReturnCurrentKeypointMethod();
//    genome_method.name = "velocity_change";
//    genome_method.min_N = static_cast<int>(solution.genome[0]);
//    genome_method.max_N = static_cast<int>(solution.genome[1]);
////    genome_method.velocity_change_thresholds.resize(genome_size - 2);
//    for(int i = 0; i < genome_size - 2; i++){
//        genome_method.velocity_change_thresholds[i] = solution.genome[i + 2];
//    }
//    optimiser->keypoint_generator->SetKeypointMethod(genome_method);
//
//    // Suppress terminal output
//    optimiser->verbose_output = false;
//
//    for (int i = 0; i < num_tasks; i++) {
//
//        optimiser->verbose_output = false;
//
////        if(i == 0){
////            std::cout << "genome evaluation task " << i << std::endl;
////            optimiser->verbose_output = true;
////        }
//
//        // - Optimise once with set interval 1 so we have something to compare against -
//        optimiser->Reset();
//        optimiser->keypoint_generator->ResetCache();
//
//
//        // Reset internal optimisation data and clear key-points cache
//        optimiser->Reset();
//        optimiser->keypoint_generator->ResetCache();
//        // Load the task - use random function to prevent over-fitting
////        model_translator->GenerateRandomGoalAndStartState();
//
//        // Load the task from CSV file
//        yamlReader->LoadTaskFromFile(model_translator->model_name,
//                                     i, model_translator->full_state_vector,
//                                     model_translator->residual_list);
//
//        // Load the task from vector
////        model_translator->full_state_vector.robots[0].start_pos[0] = tasks[i][0];
////        model_translator->full_state_vector.robots[0].start_pos[1] = tasks[i][1];
//
//        // Reset state vector (only really applicable for iLQR_SVR method)
//        model_translator->ResetSVR();
//        model_translator->InitialiseSystemToStartState(model_translator->MuJoCo_helper->master_reset_data);
//
//        // Setup mj data objects
//        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->main_data,
//                                                              model_translator->MuJoCo_helper->master_reset_data);
//        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->vis_data,
//                                                              model_translator->MuJoCo_helper->master_reset_data);
//
//
//        mj_step(model_translator->MuJoCo_helper->model, model_translator->MuJoCo_helper->master_reset_data);
//
//        if (!model_translator->MuJoCo_helper->CheckIfDataIndexExists(0)) {
//            model_translator->MuJoCo_helper->AppendSystemStateToEnd(
//                    model_translator->MuJoCo_helper->master_reset_data);
//        }
//
//        // Perform any setup controls for this task
//        std::vector<MatrixXd> initSetupControls = model_translator->CreateInitSetupControls(1000);
//        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->master_reset_data,
//                                                         model_translator->MuJoCo_helper->main_data);
//        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->main_data,
//                                                              model_translator->MuJoCo_helper->master_reset_data);
//        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->vis_data,
//                                                              model_translator->MuJoCo_helper->master_reset_data);
//
//        // Create init optimisation controls
//        std::vector<MatrixXd> init_opt_controls = model_translator->CreateInitOptimisationControls(task_horizon);
//        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->main_data,
//                                                              model_translator->MuJoCo_helper->master_reset_data);
//        model_translator->MuJoCo_helper->CopySystemState(
//                model_translator->MuJoCo_helper->saved_systems_state_list[0],
//                model_translator->MuJoCo_helper->master_reset_data);
//
//        // Do the optimisation!
//        optimiser->lambda = 0.01;
//        std::vector<MatrixXd> optimised_controls = optimiser->Optimise(
//                model_translator->MuJoCo_helper->saved_systems_state_list[0],
//                init_opt_controls, opt_iters, opt_iters,
//                task_horizon);
//
//        solution.cost_reductions[i] = optimiser->cost_reduction;
//        solution.percentage_derivatives[i] = optimiser->avg_percent_derivs;
//    }
//}
//
//vector<solution>  GAOptimalKeypoints::TournamentSelectParents(const vector<solution>& solutions,
//                                                              int tournament_size){
//    // TODO - validate this code works
//    vector<solution> selected_parents;
//
//    for (int i = 0; i < solutions.size(); ++i) {
//        double best_fitness = std::numeric_limits<double>::infinity();
//        int best_index = -1;
//
//        for (int t = 0; t < tournament_size; ++t) {
//            int idx = rand() % solutions.size();
//            if (solutions[idx].fitness < best_fitness) {
//                best_fitness = solutions[idx].fitness;
//                best_index = idx;
//            }
//        }
//
//        selected_parents.push_back(solutions[best_index]);
//    }
//
//    return selected_parents;
//}
//
//void GAOptimalKeypoints::RandomlyInitPopulation(vector<solution> &solutions){
//    for(int i = 0; i < population_size; i++){
//        solutions[i].genome.resize(genome_size);
//        RandomGenome(solutions[i].genome);
//        solutions[i].cost_reductions.resize(num_tasks);
//        solutions[i].percentage_derivatives.resize(num_tasks);
//        solutions[i].fitness = 0.0;
//    }
//}
//
//void GAOptimalKeypoints::RandomGenome(vector<double> &genome){
////    genome[0] = randFloat(0, 50);
////    genome[1] = genome[0] * 2;
//    genome[0] = 1;
//    genome[1] = 50;
//    for(int i = 2; i < genome_size; i++){
//        // TODO - not sure about this as a method for random genome specification either.
//        genome[i] = randFloat(0, 1);
//    }
//}
//
//pair<solution, solution>  GAOptimalKeypoints::Crossover(const solution &parent1, const solution &parent2){
//    solution child1;
//    solution child2;
//
//    // Allocate memory correctly
//    child1.genome.resize(genome_size);
//    child1.cost_reductions.resize(num_tasks);
//    child1.percentage_derivatives.resize(num_tasks);
//    child2.genome.resize(genome_size);
//    child2.cost_reductions.resize(num_tasks);
//    child2.percentage_derivatives.resize(num_tasks);
//
//    int crossover_point = rand() % genome_size;
//
//    // Blend Crossover (BLX-α)
//    for(int i = 0; i < genome_size; i++){
//        float alpha = randFloat(0, 1);
//        child1.genome[i] = (alpha * parent1.genome[i]) + ((1-alpha)*parent2.genome[i]);
//        child2.genome[i] = (alpha * parent2.genome[i]) + ((1-alpha)*parent1.genome[i]);
//    }
//
//    // Enforce max_N > min_N
//    if(child1.genome[1] < child1.genome[0]){
//        child1.genome[1] = child1.genome[0] + 1;
//    } if(child2.genome[1] < child2.genome[0]){
//        child2.genome[1] = child2.genome[0] + 1;
//    }
//
//    // Temporary - keep minN and maxN fixed
//    child1.genome[0] = 1;
//    child2.genome[0] = 1;
//    child1.genome[1] = 50;
//    child2.genome[1] = 50;
//
//    // Crossover genome point - maybe bad for this domain
////    for (int i = 0; i < genome_size; ++i) {
////        if (i < crossover_point) {
////            child1.genome[i] = parent1.genome[i];
////            child2.genome[i] = parent2.genome[i];
////        } else {
////            child1.genome[i] = parent2.genome[i];
////            child2.genome[i] = parent1.genome[i];
////        }
////    }
//
//    return {child1, child2};
//}
//
//void GAOptimalKeypoints::Mutation(solution &child){
//
//    // Temp - remove mutate on min and max n
////    if(randFloat(0, 1) < mutate_chance){
////        child.genome[0] += randFloat(-10, 10);
////        child.genome[1] += randFloat(-10, 10);
////    }
////
////
////    if(child.genome[0] < 1){
////        child.genome[0] = 1;
////    }
////
////    if(child.genome[1] < child.genome[0]){
////        child.genome[1] = child.genome[0] + 1;
////    }
//
//    // Randomly mutate genome variables
//    for(int i = 2; i < genome_size; i++){
//        // Random chance check
//        if(randFloat(0, 1) < mutate_chance){
//            // TODO - Is this the best method to mutate my genomes?
//            child.genome[i] += randFloat(-3, 3);
//
//            if(child.genome[i] < 0){
//                child.genome[i] = 0;
//            }
//        }
//    }
//}
//
//void GAOptimalKeypoints::EvaluateSolutionCost(solution &solutions, vector<double> baseline_cost_reductions) {
//    double fitness = 0.0;
//
//    // Turn two optimisation variables into a single one, via weightings
//    for(int i = 0; i < solutions.cost_reductions.size(); i++){
//        // Penalise Low cost reduction
//        double cost_reduction_diff = baseline_cost_reductions[i] - solutions.cost_reductions[i];
////        double cost_term = cost_fitness_scalar * (1.0 /(solutions.cost_reductions[i] + 1));
//        double cost_term = cost_fitness_scalar * cost_reduction_diff;
//
//        // Penalise high derivative usage (log scale as closer to zero the better)
//        double deriv_term = derivatives_fitness_scalar * std::log(1.0 + (solutions.percentage_derivatives[i]/100));
////        double deriv_term = derivatives_fitness_scalar * (solutions.percentage_derivatives[i] / 100);
//
//        double task_fitness = cost_term + deriv_term;
//        fitness += task_fitness;
////        std::cout << "cost reduction: " << cost_reductions[i] << " PD: " << percentage_derivs[i] << " fitness: " << task_fitness << "\n";
//    }
//
//    // Normalise for number of tasks
//    fitness /= static_cast<double>(solutions.cost_reductions.size());
//
//    solutions.fitness = fitness;
//}