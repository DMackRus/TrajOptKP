#include "GAOptimalKeypoints.h"

GAOptimalKeypoints::GAOptimalKeypoints(std::shared_ptr<ModelTranslator> _model_translator,
                                       std::shared_ptr<MuJoCoHelper> _MuJoCo_helper,
                                       std::shared_ptr<FileHandler> _yamlReader,
                                       std::shared_ptr<Optimiser> _optimiser){
    model_translator = _model_translator;
    MuJoCo_helper = _MuJoCo_helper;
    yamlReader = _yamlReader;
    optimiser = _optimiser;
//    keypoint_generator = _keypoint_generator;
    genome_size = model_translator->full_state_vector.dof;

}

int GAOptimalKeypoints::Run(){
    // Initialise Genomes
    vector<vector<double>> genomes(population_size, vector<double>(genome_size));
    RandomlyInitPopulation(genomes);

    // Loop for a number of generations
    for(int i = 0; i < num_generations; i++){
        // Loop through the population
        vector<double> population_fitness(population_size);
        for(int j = 0; j < population_size; j++){
            vector<double> costs(num_tasks);
            vector<double> percentage_derivatives(num_tasks);

            EvaluateKeypointMethodOverTasks(costs, percentage_derivatives, genomes[j]);

            population_fitness[j] = EvaluateFitness(costs, percentage_derivatives);
        }

        std::cout << "pop fitnesses: ";
        for(int j = 0; j < population_size; j++){
            std::cout << population_fitness[j] << " ";
        }
        std::cout << endl;



        // Select parents (winners)

        // Apply crossover operation to mutate genomes

        // Apply mutation to children (random chance)

        // Create new population (elitism plus children)
    }

    return EXIT_SUCCESS;
}

void GAOptimalKeypoints::EvaluateKeypointMethodOverTasks(vector<double> &costs,
                                     vector<double> &percentage_derivs,
                                     const vector<double> &genome){

//    int opt_horizon = yamlReader.
    // Todo - temp, make it from yaml reader later
    int task_horizon = 100;

    // ---------- Set Keypoint method (genome) --------------------------
    keypoint_method genome_method;
    genome_method.name = "velocity_change";
    genome_method.min_N = 1;
    genome_method.max_N = 100;
    genome_method.velocity_change_thresholds.resize(genome_size);
    for(int i = 0; i < genome_size; i++){
        genome_method.velocity_change_thresholds[i] = genome[i];
    }
    optimiser->keypoint_generator->SetKeypointMethod(genome_method);

    // Test the keypoint method performance over the number of tasks
//    for (int i = 0; i < 100; i++) {
////        std::cout << "trial: " << i << "\n";
//
//        // Reset internal optimisation data and clear key-points cache
//        optimiser->Reset();
//        optimiser->keypoint_generator->ResetCache();
//
//        // Load the task from CSV file
//        yamlReader->LoadTaskFromFile(model_translator->model_name,
//                                     i, model_translator->full_state_vector,
//                                     model_translator->residual_list);
//
//        // Reset state vector (only really applicable for iLQR_SVR method)
//        model_translator->ResetSVR();
//        model_translator->InitialiseSystemToStartState(model_translator->MuJoCo_helper->master_reset_data);
//
//        // Setup mj data objects
//        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->main_data,
//                                                         model_translator->MuJoCo_helper->master_reset_data);
//        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->vis_data,
//                                                              model_translator->MuJoCo_helper->master_reset_data);
//
////        MatrixXd test_state_start = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->master_reset_data,
////                                                                             activeModelTranslator->full_state_vector);
////        std::cout << "state vector after initialised: " << test_state_start.transpose() << "\n";
//
//        mj_step(model_translator->MuJoCo_helper->model, model_translator->MuJoCo_helper->master_reset_data);
////        test_state_start = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->master_reset_data,
////                                                                    activeModelTranslator->full_state_vector);
////        std::cout << "state vector after step: " << test_state_start.transpose() << "\n";
//
//        if (!model_translator->MuJoCo_helper->CheckIfDataIndexExists(0)) {
//            model_translator->MuJoCo_helper->AppendSystemStateToEnd(
//                    model_translator->MuJoCo_helper->master_reset_data);
//        }
//
//        // Perform any setup controls for this task
//        std::vector<MatrixXd> initSetupControls = model_translator->CreateInitSetupControls(1000);
//        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->master_reset_data,
//                                                              model_translator->MuJoCo_helper->main_data);
//        model_translator->MuJoCo_helper->CopySystemState(model_translator->MuJoCo_helper->main_data,
//                                                         model_translator->MuJoCo_helper->master_reset_data);
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
//        optimiser->lambda = 0.01;   // Make sure lambda is the same value is important!
//        std::vector<MatrixXd> optimised_controls = optimiser->Optimise(
//                model_translator->MuJoCo_helper->saved_systems_state_list[0], init_opt_controls, 1, 1,
//                task_horizon);
//
//
//        // ------------------------- Update the data storages -------------------------------------
//        costs[i] = optimiser->new_cost;
//        percentage_derivs[i] = optimiser->avg_percent_derivs;
//    }
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

        costs[i] = optimiser->new_cost;
        percentage_derivs[i] = optimiser->avg_percent_derivs;
    }
}

void GAOptimalKeypoints::RandomlyInitPopulation(vector<vector<double>> &genomes){
    for(int i = 0; i < population_size; i++){
        RandomGenome(genomes[i]);
    }
}

void GAOptimalKeypoints::RandomGenome(vector<double> &genome){
    for(int i = 0; i < genome_size; i++){
        // TODO - not sure about this as a method for random genome specification either.
        genome[i] = randFloat(0, 100);
    }
}

void GAOptimalKeypoints::EvaluateGenomes(vector<vector<double>> genomes){

}

vector<double> GAOptimalKeypoints::Crossover(const vector<double> &parent1, const vector<double> &parent2){
    vector<double> child(genome_size);

    // Simple averaging crossover method
    for(int i = 0; i < genome_size; i++){
        child[i] = (parent1[i] + parent2[i]) / 2;
    }

    return child;
}

void GAOptimalKeypoints::Mutation(vector<double> &child){
    // Randomly mutate genome variables
    for(int i = 0; i < genome_size; i++){
        // Random chance check
        if(randFloat(0, 1) < mutate_chance){
            // TODO - Is this the best method to mutate my genomes?
            child[i] += randFloat(-1, 1);
        }
    }
}

double GAOptimalKeypoints::EvaluateFitness(vector<double> costs, vector<double> percentage_derivs) {
    double fitness = 0.0;

    // Turn two optimisation variables into a single one, via weightings
    for(int i = 0; i < costs.size(); i++){
        fitness += (cost_fitness_scalar * costs[i]) + (derivatives_fitness_scalar * percentage_derivs[i]);
    }

    return fitness;
}