#ifndef OPTIMISER_H
#define OPTIMISER_H


#include "stdInclude.h"
#include "modelTranslator.h"
#include "physicsSimulator.h"
#include "differentiator.h"
#include <atomic>

enum keyPointsMethod{
    setInterval = 0,
    adaptive_jerk = 1,
    adaptive_accel = 2,
    iterative_error = 3,
    magvel_change = 4
};

struct indexTuple{
    int startIndex;
    int endIndex;
};

struct derivative_interpolator{
    std::string keypoint_method;
    int minN;
    int maxN;
    std::vector<double> jerkThresholds;
    std::vector<double> accelThresholds;
    double iterativeErrorThreshold;
    std::vector<double> magVelChangeThresholds;

};

class optimiser{
public:
    optimiser(std::shared_ptr<modelTranslator> _modelTranslator, std::shared_ptr<physicsSimulator> _physicsSimulator, std::shared_ptr<fileHandler> _yamlReader, std::shared_ptr<differentiator> _differentiator);

    virtual double rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls) = 0;
    virtual std::vector<MatrixXd> optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength) = 0;
    virtual bool checkForConvergence(double oldCost, double newCost);
    void setTrajecNumber(int _trajecNumber);

    void returnOptimisationData(double &_optTime, double &_costReduction, double &_avgPercentageDerivs, double &_avgTimeGettingDerivs, int &_numIterations);

    derivative_interpolator returnDerivativeInterpolator();
    void setDerivativeInterpolator(derivative_interpolator _derivativeInterpolator);

    void worker(int threadId);
    std::vector<void (differentiator::*)(MatrixXd &A, MatrixXd &B, std::vector<int> cols, MatrixXd &l_x, MatrixXd &l_u, MatrixXd &l_xx, MatrixXd &l_uu, bool costDerivs, int dataIndex, bool terminal, int threadId)> tasks;
    std::atomic<int> current_iteration;
    int num_threads_iterations;
    std::vector<int> timeIndicesGlobal;
    std::vector<std::vector<int>> keypointsGlobal;

    int currentTrajecNumber = 0;
    std::string keyPointsMethodsStrings[5] = {"setInterval", "adaptive_jerk", "adaptive_accel", "iterative_error", "magVel_change"};

    double optTime;

    double initialCost;
    double costReduction = 1.0f;

    std::vector<double> percentDerivsPerIter;
    double avgPercentDerivs;
    int numberOfTotalDerivs = 0;

    std::vector<double> timeDerivsPerIter;

    int numIterationsForConvergence;

    std::string filteringMethod = "none";

    std::vector<double> time_getDerivs_ms;
    double avgTime_getDerivs_ms = 0.0f;
    std::vector<double> time_backwardsPass_ms;
    double avgTime_backwardsPass_ms = 0.0f;
    std::vector<double> time_forwardsPass_ms;
    double avgTime_forwardsPass_ms = 0.0f;
    bool verboseOutput = true;

    // - Top level function - ensures all derivates are calculate over an entire trajectory by some method
    void generateDerivatives();

    // -------------- Vectors of matrices for gradient information about the trajectory -------------
    // First order dynamics
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

    int horizonLength;

    std::vector<double> costHistory;
    double lowPassACoefficient = 0.25;
    std::vector<double> FIRCoefficients = {0.1, 0.15, 0.5, 0.15, 0.1};
    void setFIRFilter(std::vector<double> _FIRCoefficients);


protected:
    std::shared_ptr<modelTranslator> activeModelTranslator;
    std::shared_ptr<physicsSimulator> activePhysicsSimulator;

    int dof;
    int num_ctrl;
    derivative_interpolator activeDerivativeInterpolator;

    std::shared_ptr<fileHandler> activeYamlReader;
    std::shared_ptr<differentiator> activeDifferentiator;

    std::vector<std::vector<int>> computedKeyPoints;

    // Generate keypoints we will calculate derivatives at
    std::vector<std::vector<int>> generateKeyPoints(std::vector<MatrixXd> trajecStates, std::vector<MatrixXd> trajecControls);
    std::vector<std::vector<int>> generateKeyPointsIteratively();
    bool checkDoFColumnError(indexTuple indices, int dof);
    std::vector<std::vector<int>> generateKeyPointsAdaptive(std::vector<MatrixXd> trajecProfile);
    std::vector<std::vector<int>> generateKeyPointsMagVelChange(std::vector<MatrixXd> velProfile);
    std::vector<MatrixXd> generateJerkProfile();
    std::vector<MatrixXd> generateAccelProfile();
    std::vector<MatrixXd> generateVelProfile();

    // Calculate derivatives at key points
    void getDerivativesAtSpecifiedIndices(std::vector<std::vector<int>> keyPoints);
    void getCostDerivs();
    void interpolateDerivatives(std::vector<std::vector<int>> keyPoints, bool costDerivs);

    // Filtering
    void filterDynamicsMatrices();
    std::vector<double> filterIndValFIRFilter(std::vector<double> unfiltered, std::vector<double> filterCoefficients);
    std::vector<double> filterIndValLowPass(std::vector<double> unfiltered);


private:
    double epsConverge = 0.02;

};



#endif