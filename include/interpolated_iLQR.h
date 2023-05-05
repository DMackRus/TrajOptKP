#ifndef INTERPOLATED_ILQR_H
#define INTERPOLATED_ILQR_H

#include "optimiser.h"
#include "differentiator.h"
#include "visualizer.h"
#include "fileHandler.h"

class interpolatediLQR: public optimiser{
public:
    interpolatediLQR(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator, differentiator *_differentiator, int _maxHorizon, visualizer *_visualizer, fileHandler *_yamlReader);

    double rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls) override;
    std::vector<MatrixXd> optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength) override;

    std::vector<int> generateKeyPoints(std::vector<MatrixXd> trajecStates, std::vector<MatrixXd> trajecControls);
    void getDerivativesAtSpecifiedIndices(std::vector<int> indices);
    void interpolateDerivatives(std::vector<int> calculatedIndices);

    bool backwardsPass_Quu_reg();
    bool isMatrixPD(Ref<MatrixXd> matrix);

    double forwardsPass(double oldCost, bool &costReduced);
    double forwardsPassParallel(double oldCost, bool &costReduced);

    void filterMatrices();
    std::vector<double> filterIndividualValue(std::vector<double> unfiltered);

    std::vector<double> costHistory;
    int numIters = 0;
    bool filteringMatrices = true;

    bool saveTrajecInfomation = false;
    bool saveCostHistory = true;

private:
    double lambda = 0.1;
    double maxLambda = 10000;
    double minLambda = 0.00001;
    double lambdaFactor = 10;
    int maxHorizon = 0;
    int min_interval = 1;
    int max_interval = 100;

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

    // Feedback gains matrices
    vector<MatrixXd> k;
    vector<MatrixXd> K;

    // Saved states and controls
    vector<MatrixXd> U_new;
    vector<MatrixXd> U_old;
    vector<MatrixXd> X_new;
    vector<MatrixXd> X_old;

    differentiator *activeDifferentiator;
    visualizer *activeVisualizer;
    fileHandler *activeYamlReader;

    vector<vector<MatrixXd>> U_alpha;

    std::vector<MatrixXd> generateJerkProfile();

    int testNumber = 0;

};


#endif