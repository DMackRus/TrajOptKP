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

private:

    std::shared_ptr<ModelTranslator> model_translator;
    std::shared_ptr<MuJoCoHelper> MuJoCo_helper;
    std::shared_ptr<FileHandler> yamlReader;
    std::shared_ptr<Optimiser> optimiser;
    int genome_size;

    double cost_fitness_scalar = 1;
    double derivatives_fitness_scalar = 1;
    int num_generations = 20;
    int population_size = 20;
    int num_tasks = 10;
    double mutate_chance = 5;

    int elite_count = 1;
    int explorer_count = 3;

};