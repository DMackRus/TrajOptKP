//
// Created by dave on 02/03/23.
//

#include "StdInclude.h"

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

m_point quat2Axis(m_quat quaternion){
    m_point axisAngles;
    double x, y, z, w;

    w = quaternion(0);
    x = quaternion(1);
    y = quaternion(2);
    z = quaternion(3);

    if(w > 1){
        w = 1;
    }

    // angles to rotate about
    double angle = 2 * acos(w);

    double s = sqrt(1-(w * w));
    if(s < 0.001){  // test to see if divide by zero???
        axisAngles(0) = x;
        axisAngles(1) = y;
        axisAngles(2) = z;
    }
    else{
        axisAngles(0) = x / s;
        axisAngles(1) = y / s;
        axisAngles(2) = z / s;
    }
    axisAngles *= angle;

    return axisAngles;
}

m_quat rotMat2Quat(Eigen::Matrix3d rotMat){
    m_quat quat;

    Eigen::Quaterniond q(rotMat);

    quat(0) = q.w();
    quat(1) = q.x();
    quat(2) = q.y();
    quat(3) = q.z();

    return quat;
}

m_quat invQuat(m_quat quat){
    m_quat invQuat;

    invQuat(0) = quat(0);
    invQuat(1) = -quat(1);
    invQuat(2) = -quat(2);
    invQuat(3) = -quat(3);
    return invQuat;
}

m_quat multQuat(m_quat quat_l, m_quat quat_r){
    m_quat result;
    double wr, xr, yr, zr;  // left side quaternion
    double ws, xs, ys, zs;  // right side quaternion

    wr = quat_l(0);
    xr = quat_l(1);
    yr = quat_l(2);
    zr = quat_l(3);

    ws = quat_r(0);
    xs = quat_r(1);
    ys = quat_r(2);
    zs = quat_r(3);

    // new W
    result(0) = (wr * ws) - (xr * xs) - (yr * ys) - (zr * zs);

    // new X
    result(1) = (ws * xr) + (wr * xs) + (yr * zs) - (zr * ys);

    // new Y
    result(2) = (ws * yr) + (wr * ys) + (zr * xs) - (xr * zs);

    // new Z
    result(3) = (ws * zr) + (wr * zs) + (xr * ys) - (yr * xs);

    return result;
}

m_point crossProduct(m_point vec1, m_point vec2){
    m_point crossProduct;

    crossProduct(0) = (vec1(1) * vec2(2)) - (vec1(2) * vec2(1));
    crossProduct(1) = (vec1(2) * vec2(0)) - (vec1(0) * vec2(2));
    crossProduct(2) = (vec1(0) * vec2(1)) - (vec1(1) * vec2(0));

    return crossProduct;
}

double GaussNoise(double mean, double stddev) {
    // Create a random engine
    std::random_device rd;
    std::mt19937 gen(rd());

    // Create a normal distribution
    std::normal_distribution<double> dist(mean, stddev);

    // Generate a random number from the distribution
    return dist(gen);
}

bool compare(const std::pair<double, int>& a, const std::pair<double, int>& b) {
    return a.first > b.first; // Sort in descending order
}

std::vector<int> sortIndices(const std::vector<double>& values) {
    std::vector<std::pair<double, int>> indexedValues;
    for (int i = 0; i < values.size(); ++i) {
        indexedValues.push_back({values[i], i});
    }
    std::sort(indexedValues.begin(), indexedValues.end(), compare);
    std::vector<int> sortedIndices;
    for (const auto& pair : indexedValues) {
        sortedIndices.push_back(pair.second);
    }
    return sortedIndices;
}
