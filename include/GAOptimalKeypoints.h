#pragma once

#include "StdInclude.h"
#include "ModelTranslator/ModelTranslator.h"
#include "Differentiator.h"
#include "Optimiser/Optimiser.h"

class GAOptimalKeypoints{
public:
    GAOptimalKeypoints(std::shared_ptr<ModelTranslator> _modelTranslator,
                       std::shared_ptr<MuJoCoHelper> _MuJoCo_helper,
                       std::shared_ptr<FileHandler> _yamlReader,
                       std::shared_ptr<Optimiser> _optimiser);

    int Run();

    void RandomlyInitPopulation(vector<vector<double>> &genomes);
    void RandomGenome(vector<double> &genome);

    pair<vector<double>, vector<double>>  Crossover(const vector<double> &parent1, const vector<double> &parent2);

    void Mutation(vector<double> &child);

    double EvaluateFitness(vector<double> cost_reductions, vector<double> percentage_derivs);

    void EvaluateKeypointMethodOverTasks(vector<double> &cost_reductions,
                                         vector<double> &percentage_derivs,
                                         const vector<double> &genome);

    vector<vector<double>> TournamentSelectParents(const vector<vector<double>>& genomes,
                                                   const vector<double>& fitnesses,
                                                   int tournament_size);

private:

    std::shared_ptr<ModelTranslator> model_translator;
    std::shared_ptr<MuJoCoHelper> MuJoCo_helper;
    std::shared_ptr<FileHandler> yamlReader;
    std::shared_ptr<Optimiser> optimiser;
//    std::shared_ptr<KeypointGenerator> keypoint_generator;
    int genome_size;

    double cost_fitness_scalar = 1;
    double derivatives_fitness_scalar = 1;
    int num_generations = 100;
    int population_size = 20;
    int num_tasks = 10;
    double mutate_chance = 5;

};