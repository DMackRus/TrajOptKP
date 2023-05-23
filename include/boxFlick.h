#ifndef BOX_FLICK_H
#define BOX_FLICK_H

#include "modelTranslator.h"

class boxFlick: public modelTranslator{
public:
    boxFlick(int _clutterLevel);

    double costFunction(MatrixXd Xt, MatrixXd Ut, MatrixXd X_last, MatrixXd U_last, bool terminal) override;
    void costDerivatives(MatrixXd Xt, MatrixXd Ut, MatrixXd X_last, MatrixXd U_last, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal) override;

    MatrixXd returnRandomStartState() override;
    MatrixXd returnRandomGoalState(MatrixXd X0) override;
    std::vector<MatrixXd> createInitOptimisationControls(int horizonLength) override;
    void initControls_mainWayPoints_optimisation(m_point desiredObjectEnd, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon);
    std::vector<MatrixXd> createInitSetupControls(int horizonLength) override;
    void initControls_mainWayPoints_setup(m_point desiredObjectEnd, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon);


    std::vector<m_point> initControls_createAllWayPoints(std::vector<m_point> mainWayPoints, std::vector<int> wayPointsTiming);
    std::vector<MatrixXd> generate_initControls_fromWayPoints(std::vector<m_point> initPath);

    bool taskComplete(int dataIndex) override;



private:
    int clutterLevel;

    double boxStartX = 0.5;
    double boxStartY = 0.1;

    double A = 1;
    double sigma = 0.005;

};


#endif