//
// Created by dave on 02/03/23.
//

#include "stdInclude.h"

bool randInitialised = false;

float randFloat(float floor, float ceiling){

    if(!randInitialised){
        srand(time(0));
        randInitialised = true;

    }
    float random = (float)(rand()) / ((float) RAND_MAX);
    float diff = ceiling - floor;
    float r = random * diff;
    return floor + r;
}

m_quat eul2Quat(m_point eulerAngles){
    m_quat quat;

    Quaternionf q;
    q = AngleAxisf(eulerAngles(0), Vector3f::UnitX())
        * AngleAxisf(eulerAngles(1), Vector3f::UnitY())
        * AngleAxisf(eulerAngles(2), Vector3f::UnitZ());

    quat(0) = q.w();
    quat(1) = q.x();
    quat(2) = q.y();
    quat(3) = q.z();

    return quat;
}

m_point quat2Eul(m_quat quaternion){
    m_point eulAngles;

    Quaternionf q;
    q.x() = quaternion(1);
    q.y() = quaternion(2);
    q.z() = quaternion(3);
    q.w() = quaternion(0);

    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

    eulAngles(0) = euler(0);
    eulAngles(1) = euler(1);
    eulAngles(2) = euler(2);

    return eulAngles;
}
