#include "GAOptimalKeypoints.h"

GAOptimalKeypoints::GAOptimalKeypoints(std::shared_ptr<ModelTranslator> _model_translator,
                                       std::shared_ptr<MuJoCoHelper> _MuJoCo_helper,
                                       std::shared_ptr<FileHandler> _yamlReader,
                                       std::shared_ptr<Optimiser> _optimiser){
    model_translator = _model_translator;
    MuJoCo_helper = _MuJoCo_helper;
    yamlReader = _yamlReader;
    optimiser = _optimiser;

    // Genome size = number of dofs (threshhold limits) + minN + maxN
    genome_size = model_translator->full_state_vector.dof + 2;

}

int GAOptimalKeypoints::Run(){
    // Initialise Genomes
    vector<vector<double>> genomes(population_size, vector<double>(genome_size));
    RandomlyInitPopulation(genomes);

    vector<double> average_pop_fitness, best_pop_fitness, worst_pop_fitness;

    // Loop for a number of generations
    for(int i = 0; i < num_generations; i++){
        // Loop through the population
        vector<double> population_fitness(population_size);
        for(int j = 0; j < population_size; j++){
            vector<double> cost_reductions(num_tasks);
            vector<double> percentage_derivatives(num_tasks);

            EvaluateKeypointMethodOverTasks(cost_reductions, percentage_derivatives, genomes[j]);

            population_fitness[j] = EvaluateFitness(cost_reductions, percentage_derivatives);
        }

        std::cout << "pop fitnesses: ";
        for(int j = 0; j < population_size; j++){
            std::cout << population_fitness[j] << " ";
        }
        std::cout << endl;

        // Select parents (winners)
        vector<vector<double>> parents = TournamentSelectParents(genomes, population_fitness, 3);

        // Generate children
        vector<vector<double>> new_genomes;
        for (size_t k = 0; k + 1 < parents.size(); k += 2) {
            auto [child1, child2] = Crossover(parents[k], parents[k+1]);
            Mutation(child1);
            Mutation(child2);
            new_genomes.push_back(child1);
            new_genomes.push_back(child2);
        }

        // Elitism: copy best genome to next generation
        int best_idx = std::min_element(population_fitness.begin(), population_fitness.end()) - population_fitness.begin();
        // TODO enable elitism of arbritary number
        new_genomes[0] = genomes[best_idx]; // Replace first genome with elite

        // Add random survivors - Better for exploration
        for (int k = 1; k < explorer_count+1; ++k) {
            int idx = rand() % genomes.size();
            new_genomes[k] = genomes[idx];
        }

        genomes = new_genomes;

        // Data logging
        int worst_idx = std::max_element(population_fitness.begin(), population_fitness.end()) - population_fitness.begin();
        double average_fitness = std::accumulate(population_fitness.begin(), population_fitness.end(), 0.0) / population_fitness.size();

        best_pop_fitness.push_back(population_fitness[best_idx]);
        worst_pop_fitness.push_back(population_fitness[worst_idx]);
        average_pop_fitness.push_back(average_fitness);

        // ----------- Print best keypoint methods ------------------------
        keypoint_method genome_method;
        genome_method.name = "velocity_change";
        genome_method.min_N = static_cast<int>(genomes[best_idx][0]);
        genome_method.max_N = static_cast<int>(genomes[best_idx][1]);
        genome_method.velocity_change_thresholds.resize(genome_size);
        for(int k = 2; k < genome_size; k++){
            genome_method.velocity_change_thresholds[k-2] = genomes[best_idx][k];
        }
        optimiser->keypoint_generator->SetKeypointMethod(genome_method);
        optimiser->keypoint_generator->PrintKeypointMethod();
        // -----------------------------------------------------------------
    }

    // ----------------------- Save data to file -------------------------------------
    // Go back two directories
    std::string project_parent_path = __FILE__;
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));

    std::string task_prefix = model_translator->model_name;

    std::string root_path = project_parent_path + "/TestingData";

    // Check if optimiser directory exists
    if (!filesystem::exists(root_path)) {
        if (!filesystem::create_directories(root_path)) {
            std::cerr << "Failed to create directory: " << root_path << std::endl;
        }
    }

//    std::string method_directory = root_path + "/" + task_prefix + "_fitness_tracking.csv";

    // Check if method directory exists, if not create it
//    if (!filesystem::exists(method_directory)) {
//        if (!filesystem::create_directories(method_directory)) {
//            std::cerr << "Failed to create directory: " << method_directory << std::endl;
//            exit(1);
//        }
//    }
    std::string filename = root_path + "/" + task_prefix + "_fitness_tracking.csv";

    ofstream file_output;
    file_output.open(filename);

    // Make header
    file_output << "Best fitness" << "," << "Average fitness" << "," << "Worst fitness" << std::endl;

    // Loop through rows
    for(int i = 0; i < best_pop_fitness.size(); i++){
        file_output << best_pop_fitness[i] << "," << average_pop_fitness[i] << "," << worst_pop_fitness[i] << std::endl;
    }

    file_output.close();

    return EXIT_SUCCESS;
}

void GAOptimalKeypoints::EvaluateKeypointMethodOverTasks(vector<double> &cost_reductions,
                                     vector<double> &percentage_derivs,
                                     const vector<double> &genome){

//    int opt_horizon = yamlReader.
    // TODO - temp, make it from yaml reader later
    int task_horizon = 100;

    // ---------- Set Keypoint method (genome) --------------------------
    keypoint_method genome_method = optimiser->ReturnCurrentKeypointMethod();
    genome_method.name = "velocity_change";
    genome_method.min_N = static_cast<int>(genome[0]);
    genome_method.max_N = static_cast<int>(genome[1]);
//    genome_method.velocity_change_thresholds.resize(genome_size - 2);
    for(int i = 0; i < genome_size - 2; i++){
        genome_method.velocity_change_thresholds[i] = genome[i + 2];
    }
    optimiser->keypoint_generator->SetKeypointMethod(genome_method);

    // Suppress terminal output
    optimiser->verbose_output = false;

    for (int i = 0; i < num_tasks; i++) {

        // Reset internal optimisation data and clear key-points cache
        optimiser->Reset();
        optimiser->keypoint_generator->ResetCache();
        // Load start and desired state from csv file

        // Load the task from CSV file
        yamlReader->LoadTaskFromFile(model_translator->model_name,
                                     i, model_translator->full_state_vector,
                                     model_translator->residual_list);

        // Reset state vector (only really applicable for iLQR_SVR method)
        model_translator->ResetSVR();
        model_translator->InitialiseSystemToStartState(model_translator->MuJoCo_helper->master_reset_data);

        // Setup mj data objects
        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->main_data,
                                                              model_translator->MuJoCo_helper->master_reset_data);
        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->vis_data,
                                                              model_translator->MuJoCo_helper->master_reset_data);


        mj_step(model_translator->MuJoCo_helper->model, model_translator->MuJoCo_helper->master_reset_data);

        if (!model_translator->MuJoCo_helper->CheckIfDataIndexExists(0)) {
            model_translator->MuJoCo_helper->AppendSystemStateToEnd(
                    model_translator->MuJoCo_helper->master_reset_data);
        }

        // Perform any setup controls for this task
        std::vector<MatrixXd> initSetupControls = model_translator->CreateInitSetupControls(1000);
        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->master_reset_data,
                                                         model_translator->MuJoCo_helper->main_data);
        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->main_data,
                                                              model_translator->MuJoCo_helper->master_reset_data);
        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->vis_data,
                                                              model_translator->MuJoCo_helper->master_reset_data);

        // Create init optimisation controls
        std::vector<MatrixXd> init_opt_controls = model_translator->CreateInitOptimisationControls(task_horizon);
        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->main_data,
                                                              model_translator->MuJoCo_helper->master_reset_data);
        model_translator->MuJoCo_helper->CopySystemState(
                model_translator->MuJoCo_helper->saved_systems_state_list[0],
                model_translator->MuJoCo_helper->master_reset_data);

        // Do the optimisation!
        optimiser->lambda = 0.01;
        std::vector<MatrixXd> optimised_controls = optimiser->Optimise(
                model_translator->MuJoCo_helper->saved_systems_state_list[0], init_opt_controls, 1, 1,
                task_horizon);

        cost_reductions[i] = optimiser->cost_reduction;
        percentage_derivs[i] = optimiser->avg_percent_derivs;
    }
}

vector<vector<double>> GAOptimalKeypoints::TournamentSelectParents(const vector<vector<double>>& genomes,
                                               const vector<double>& fitnesses,
                                               int tournament_size){
    // TODO - validate this code works
    vector<vector<double>> selected_parents;

    for (int i = 0; i < genomes.size(); ++i) {
        double best_fitness = std::numeric_limits<double>::infinity();
        int best_index = -1;

        for (int t = 0; t < tournament_size; ++t) {
            int idx = rand() % genomes.size();
            if (fitnesses[idx] < best_fitness) {
                best_fitness = fitnesses[idx];
                best_index = idx;
            }
        }

        selected_parents.push_back(genomes[best_index]);
    }

    return selected_parents;
}

void GAOptimalKeypoints::RandomlyInitPopulation(vector<vector<double>> &genomes){
    for(int i = 0; i < population_size; i++){
        RandomGenome(genomes[i]);
    }
}

void GAOptimalKeypoints::RandomGenome(vector<double> &genome){
    genome[0] = randFloat(1, 5);
    genome[1] = genome[0] * 2;
    for(int i = 2; i < genome_size; i++){
        // TODO - not sure about this as a method for random genome specification either.
        genome[i] = randFloat(0, 100);
    }
}

pair<vector<double>, vector<double>>  GAOptimalKeypoints::Crossover(const vector<double> &parent1, const vector<double> &parent2){
    vector<double> child1(genome_size);
    vector<double> child2(genome_size);
    int crossover_point = rand() % genome_size;

    for (int i = 0; i < genome_size; ++i) {
        if (i < crossover_point) {
            child1[i] = parent1[i];
            child2[i] = parent2[i];
        } else {
            child1[i] = parent2[i];
            child2[i] = parent1[i];
        }
    }

    // Simple averaging crossover method
//    for(int i = 0; i < genome_size; i++){
//        child[i] = (parent1[i] + parent2[i]) / 2;
//    }

    return {child1, child2};
}

void GAOptimalKeypoints::Mutation(vector<double> &child){

    child[0] += randFloat(-2, 2);
    child[1] += randFloat(-2, 2);

    if(child[0] < 1){
        child[0] = 1;
    }

    // Randomly mutate genome variables
    for(int i = 2; i < genome_size; i++){
        // Random chance check
        if(randFloat(0, 1) < mutate_chance){
            // TODO - Is this the best method to mutate my genomes?
            child[i] += randFloat(-3, 3);

            if(child[i] < 0){
                child[i] = 0;
            }
        }
    }
}

double GAOptimalKeypoints::EvaluateFitness(vector<double> cost_reductions, vector<double> percentage_derivs) {
    double fitness = 0.0;

    // Turn two optimisation variables into a single one, via weightings
    for(int i = 0; i < cost_reductions.size(); i++){
        // Penalise poor cost reduction (non-linear)
        double cost_term = cost_fitness_scalar * std::pow(cost_reductions[i], 2.0);

        // Penalise high derivative usage (log scale for stability)
        double deriv_term = derivatives_fitness_scalar * std::log(1.0 + percentage_derivs[i]);

//        fitness += (cost_fitness_scalar * cost_reductions[i]) + (derivatives_fitness_scalar * (1.0 / percentage_derivs[i]));
        fitness += cost_term + deriv_term;
    }

    fitness /= cost_reductions.size();

    return fitness;
}