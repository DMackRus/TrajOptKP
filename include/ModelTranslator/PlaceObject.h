#pragma once

#include "StdInclude.h"
#include "MuJoCoHelper.h"
#include "ModelTranslator/ModelTranslator.h"

class PlaceObject: virtual public ModelTranslator{
public:

    PlaceObject(std::string EE_name, std::string body_name);

//    void EEWayPointsSetup(m_point desiredObjectEnd,
//                          std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon);
//
//    void EEWayPointsPush(m_point desiredObjectEnd,
//                         std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon);
//
//    std::vector<m_point> CreateAllEETransitPoints(const std::vector<m_point> &mainWayPoints, const std::vector<int> &wayPointsTiming);
//
//    std::vector<MatrixXd> JacobianEEControl(const std::vector<m_point> &EE_path, double EE_angle);

    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;
//    std::vector<MatrixXd> CreateInitSetupControls(int horizonLength) override;

    void Residuals(mjData *d, MatrixXd &residuals) override;

    bool TaskComplete(mjData *d, double &dist) override;

    void SetGoalVisuals(mjData *d) override;

protected:
    std::string EE_name;
    std::string body_name;
private:

};