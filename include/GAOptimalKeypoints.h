#pragma once

#include "StdInclude.h"
#include "ModelTranslator/ModelTranslator.h"
#include "Differentiator.h"
#include "Optimiser/Optimiser.h"

struct solution{
    vector<double> genome;
    vector<double> cost_reductions;
    vector<double> percentage_derivatives;
    double fitness;
};

struct data_logging{
    vector<double> average_pop_fitness;
    vector<double> best_pop_fitness;
    vector<double> worst_pop_fitness;
    vector<double> average_pop_cost_reduction;
    vector<double> best_pop_cost_reduction;
    vector<double> worst_pop_cost_reduction;
    vector<double> average_pop_percent_derivs;
    vector<double> best_pop_percent_derivs;
    vector<double> worst_pop_percent_derivs;
};

class GAOptimalKeypoints{
public:
    GAOptimalKeypoints(std::shared_ptr<ModelTranslator> _modelTranslator,
                       std::shared_ptr<MuJoCoHelper> _MuJoCo_helper,
                       std::shared_ptr<FileHandler> _yamlReader,
                       std::shared_ptr<Optimiser> _optimiser);

    int Run();

    void RandomlyInitPopulation(vector<solution> &solutions);
    void RandomGenome(vector<double> &genome);

    pair<solution, solution>  Crossover(const solution &parent1, const solution &parent2);

    void Mutation(solution &child);

    void EvaluateSolutionCost(solution &solutions);

    void EvaluateKeypointMethodOverTasks(solution &solution);

    vector<solution> TournamentSelectParents(const vector<solution>& solutions,
                                                   int tournament_size);

    void UpdateDataLogging(data_logging& data, const vector<solution>& solutions);

private:

    std::shared_ptr<ModelTranslator> model_translator;
    std::shared_ptr<MuJoCoHelper> MuJoCo_helper;
    std::shared_ptr<FileHandler> yamlReader;
    std::shared_ptr<Optimiser> optimiser;
    int genome_size;

    double cost_fitness_scalar = 4;
    double derivatives_fitness_scalar = 1;
    int num_generations = 50;
    int population_size = 30;
    int num_tasks = 20;
    double mutate_chance = 5;

    int elite_count = 1;
    int explorer_count = 3;

};