#ifndef OPTIMISER_H
#define OPTIMISER_H


#include "stdInclude.h"
#include "modelTranslator.h"
#include "physicsSimulator.h"

enum interpMethod{
    linear = 0,
    quadratic = 1,
    cubic = 2,
};

enum keyPointsMethod{
    setInterval = 0,
    adaptive_jerk = 1,
    adaptive_accel = 2,
    iterative_error = 3
};

class optimiser{
public:
    optimiser(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator);

    virtual double rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls) = 0;
    virtual std::vector<MatrixXd> optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength) = 0;
    virtual bool checkForConvergence(double oldCost, double newCost);
    void setupTestingExtras(int _trajecNumber, int _interpMethod, int _keyPointsMethod, int minN);

    void returnOptimisationData(double &_optTime, double &_costReduction, int &_avgNumDerivs, double &_avgTimeGettingDerivs);

    int currentTrajecNumber = 0;
    int interpMethod = linear;
    int keyPointsMethod = setInterval;

    double optTime;

    double initialCost;
    double costReduction = 1.0f;

    std::vector<int> numDerivsPerIter;
    int avgNumDerivs;

    std::vector<double> timeDerivsPerIter;
    double avgTimePerDerivs;

    int min_interval = 1;
    int max_interval = 100;


protected:
    modelTranslator *activeModelTranslator;
    physicsSimulator *activePhysicsSimulator;

    int dof;
    int num_ctrl;
    int horizonLength;


private:
    double epsConverge = 0.02;
    

};



#endif