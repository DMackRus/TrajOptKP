//
// Created by dave on 03/03/23.
//

#ifndef PHYSICSSIMSWITCHING_MODELTRANSLATOR_H
#define PHYSICSSIMSWITCHING_MODELTRANSLATOR_H

#include "stdInclude.h"
#include "MuJoCoHelper.h"
#include "fileHandler.h"

enum clutterLevels{
    noClutter = 0,
    lowClutter = 1,
    heavyClutter = 2,
    constrainedClutter = 3,
    clutter_realWorld = 4
};

class modelTranslator {
public:
    modelTranslator();

    // - Functions that work for all tasks in base class
    void initModelTranslator(std::string filePath);

    MatrixXd returnStateVector(int dataIndex);
    bool setStateVector(MatrixXd _stateVector, int dataIndex);
    MatrixXd returnControlVector(int dataIndex);
    bool setControlVector(MatrixXd _controlVector, int dataIndex);

    MatrixXd returnPositionVector(int dataIndex);
    MatrixXd returnVelocityVector(int dataIndex);
    MatrixXd returnAccelerationVector(int dataIndex);
    bool setPositionVector(MatrixXd _positionVector, int dataIndex);
    bool setVelocityVector(MatrixXd _velocityVector, int dataIndex);

    // - Optional override functions, have default implementations but can be overwritten
    virtual double costFunction(int dataIndex, bool terminal);
    virtual void costDerivatives(int dataIndex, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal);
    virtual bool taskComplete(int dataIndex, double &dist);
    virtual std::vector<MatrixXd> createInitSetupControls(int horizonLength);
    virtual std::vector<MatrixXd> createInitOptimisationControls(int horizonLength);

    // - Pure virtual functions that HAVE to be overwritten
    virtual MatrixXd returnRandomStartState() = 0;
    virtual MatrixXd returnRandomGoalState(MatrixXd X0) = 0;
    virtual void generateRandomGoalAndStartState() = 0;

    int dof;
    int num_ctrl;
    int stateVectorSize;
    struct stateVectorList myStateVector;
    MatrixXd X_desired;
    MatrixXd X_start;

    std::shared_ptr<physicsSimulator> activePhysicsSimulator;
    std::shared_ptr<MuJoCoHelper> myHelper;
    std::string modelFilePath;
    std::string modelName;

    // Default derivative interpolation settings
    std::string keypointMethod;
    int minN;
    int maxN;
    std::vector<double> jerkThresholds;
    std::vector<double> accelThresholds;
    double iterativeErrorThreshold;
    std::vector<double> magVelChangeThresholds;

    DiagonalMatrix<double, Eigen::Dynamic> Q;
    DiagonalMatrix<double, Eigen::Dynamic> Q_terminal;
    DiagonalMatrix<double, Eigen::Dynamic> R;
    DiagonalMatrix<double, Eigen::Dynamic> J;

protected:


private:

};

#endif //PHYSICSSIMSWITCHING_MODELTRANSLATOR_H
