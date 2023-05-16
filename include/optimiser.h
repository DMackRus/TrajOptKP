#ifndef OPTIMISER_H
#define OPTIMISER_H


#include "stdInclude.h"
#include "modelTranslator.h"
#include "physicsSimulator.h"
#include "differentiator.h"

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

struct indexTuple{
    int startIndex;
    int endIndex;
};

class optimiser{
public:
    optimiser(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator, fileHandler *_yamlReader, differentiator *_differentiator);

    virtual double rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls) = 0;
    virtual std::vector<MatrixXd> optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength) = 0;
    virtual bool checkForConvergence(double oldCost, double newCost);
    void setupTestingExtras(int _trajecNumber, int _interpMethod, int _keyPointsMethod, int minN);

    void returnOptimisationData(double &_optTime, double &_costReduction, int &_avgNumDerivs, double &_avgTimeGettingDerivs);

    int currentTrajecNumber = 0;
    int interpMethod = linear;
    int keyPointsMethod = setInterval;
    std::string interpMethodsStrings[3] = {"linear", "quadratic", "cubic"};
    std::string keyPointsMethodsStrings[4] = {"setInterval", "adaptive_jerk", "adaptive_accel", "iterative_error"};

    double optTime;

    double initialCost;
    double costReduction = 1.0f;

    std::vector<int> numDerivsPerIter;
    int avgNumDerivs;

    std::vector<double> timeDerivsPerIter;
    double avgTimePerDerivs;

    int min_interval = 1;
    int max_interval = 100;

    bool filteringMatrices = true;


protected:
    modelTranslator *activeModelTranslator;
    physicsSimulator *activePhysicsSimulator;

    int dof;
    int num_ctrl;
    int horizonLength;

    // -------------- Vectors of matrices for gradient information about the trajectory -------------
    // First order dynamics
    vector<MatrixXd> f_x;
    vector<MatrixXd> f_u;
    vector<MatrixXd> A;
    vector<MatrixXd> B;

    // First and second order cost derivatives
    vector<MatrixXd> l_x;
    vector<MatrixXd> l_xx;
    vector<MatrixXd> l_u;
    vector<MatrixXd> l_uu;

    // Saved states and controls
    vector<MatrixXd> U_new;
    vector<MatrixXd> U_old;
    vector<MatrixXd> X_new;
    vector<MatrixXd> X_old;

    fileHandler *activeYamlReader;
    differentiator *activeDifferentiator;


    std::vector<int> computedKeyPoints;

    void generateDerivatives();
    std::vector<int> generateKeyPoints(std::vector<MatrixXd> trajecStates, std::vector<MatrixXd> trajecControls);
    void getDerivativesAtSpecifiedIndices(std::vector<int> indices);
    void getCostDerivs();
    void interpolateDerivatives(std::vector<int> calculatedIndices);
    std::vector<MatrixXd> generateJerkProfile();
    std::vector<MatrixXd> generateAccelProfile();
    std::vector<int> generateKeyPointsIteratively();
    bool checkOneMatrixError(indexTuple indices);
    void filterMatrices();
    std::vector<double> filterIndividualValue(std::vector<double> unfiltered);


private:
    double epsConverge = 0.02;
    

};



#endif