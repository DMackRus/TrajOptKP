#ifndef INTERPOLATED_ILQR_H
#define INTERPOLATED_ILQR_H

#include "optimiser.h"

class interpolatediLQR: public optimiser{
public:
    interpolatediLQR(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator);

    double rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls) override;
    std::vector<MatrixXd> optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIterations, int _horizonLength) override;

    std::vector<int> generateEvalWaypoints(std::vector<MatrixXd> trajecStates, std::vector<MatrixXd> trajecControls);
    void getDerivativesAtSpecifiedIndices();
    void interpolateDerivatives(std::vector<int> calculatedIndices);

    bool backwardsPass_Quu_reg();
    bool isMatrixPD(Ref<MatrixXd> matrix);

    double forwardsPass(double oldCost, bool &costReduced);

private:
    double lambda = 0.1;
    double maxLambda = 10000;
    double minLambda = 0.00001;
    double lambdaFactor = 10;

    // -------------- Vectors of matrices for gradient information about the trajectory -------------
    // First order dynamics
    vector<MatrixXd> f_x;
    vector<MatrixXd> f_u;

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
    vector<MatrixXd> X_final;
    vector<MatrixXd> X_old;



};


#endif