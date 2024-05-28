#pragma once

#include "StdInclude.h"
#include "MuJoCoHelper.h"
#include "ModelTranslator/ModelTranslator.h"

class PushBaseClass: virtual public ModelTranslator{
public:

    PushBaseClass(std::string EE_name, std::string body_name);

    void EEWayPointsSetup(m_point desiredObjectEnd,
                          std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon);

    void EEWayPointsPush(m_point desiredObjectEnd,
                         std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon);

    std::vector<m_point> CreateAllEETransitPoints(const std::vector<m_point> &mainWayPoints, const std::vector<int> &wayPointsTiming);

    std::vector<MatrixXd> JacobianEEControl(const std::vector<m_point> &EE_path, double EE_angle);

protected:
    std::string EE_name;
    std::string body_name;
private:

};