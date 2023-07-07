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
    void setupTestingExtras(int _trajecNumber, int _interpMethod, int _keyPointsMethod, int minN, bool approxBackwardsPass);

    void returnOptimisationData(double &_optTime, double &_costReduction, double &_avgPercentageDerivs, double &_avgTimeGettingDerivs, int &_numIterations);

    int currentTrajecNumber = 0;
    int interpMethod = linear;
    int keyPointsMethod = setInterval;
    std::string interpMethodsStrings[3] = {"linear", "quadratic", "cubic"};
    std::string keyPointsMethodsStrings[4] = {"setInterval", "adaptive_jerk", "adaptive_accel", "iterative_error"};

    double optTime;

    double initialCost;
    double costReduction = 1.0f;

    std::vector<double> percentDerivsPerIter;
    double avgPercentDerivs;
    int numberOfTotalDerivs = 0;

    std::vector<double> timeDerivsPerIter;
    double avgTimePerDerivs;

    int numIterationsForConvergence;

    int min_interval = 1;
    int max_interval = 100;

    bool filteringMatrices = true;
    bool approximate_backwardsPass = false;

    double time_getDerivs_ms = 0.0f;
    double time_backwardsPass_ms = 0.0f;
    double time_forwardsPass_ms = 0.0f;
    bool verboseOutput = false;


protected:
    modelTranslator *activeModelTranslator;
    physicsSimulator *activePhysicsSimulator;

    int dof;
    int num_ctrl;
    int horizonLength;

    // -------------- Vectors of matrices for gradient information about the trajectory -------------
    // First order dynamics
//    vector<MatrixXd> f_x;
//    vector<MatrixXd> f_u;
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

    std::vector<std::vector<int>> computedKeyPoints;

    // - Top level function - ensures all derivates are calculate over an entire trajectory by some method
    void generateDerivatives();

    // Generate keypoints we will calculate derivatives at
    std::vector<std::vector<int>> generateKeyPoints(std::vector<MatrixXd> trajecStates, std::vector<MatrixXd> trajecControls);
    std::vector<std::vector<int>> generateKeyPointsIteratively();
    bool checkOneMatrixError(indexTuple indices);
    bool checkDoFColumnError(indexTuple indices, int dof);
    std::vector<std::vector<int>> generateKeyPointsAdaptive(std::vector<MatrixXd> trajecProfile);
    std::vector<MatrixXd> generateJerkProfile();
    std::vector<MatrixXd> generateAccelProfile();

    // Calculate derivatives at key points
    void getDerivativesAtSpecifiedIndices(std::vector<std::vector<int>> keyPoints);
    void getCostDerivs();
    void interpolateDerivatives(std::vector<std::vector<int>> keyPoints);



    void filterMatrices();
    std::vector<double> filterIndividualValue(std::vector<double> unfiltered);

    bool convergeThisIteration = false;


private:
    double epsConverge = 0.02;
    

};



#endif