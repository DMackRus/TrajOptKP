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

m_quat axis2Quat(m_point axisAngles){
    m_quat quat;

    // Quat order is w, x, y, z

    double angle = sqrt(axisAngles(0) * axisAngles(0) +
                        axisAngles(1) * axisAngles(1) +
                        axisAngles(2) * axisAngles(2));

    double halfAngle = angle * 0.5;

    double sinHalfAngle = sin(halfAngle);
    quat(0) = cos(halfAngle);
    quat(1) = axisAngles(0) * sinHalfAngle;
    quat(2) = axisAngles(1) * sinHalfAngle;
    quat(3) = axisAngles(2) * sinHalfAngle;

    return quat;
}

Eigen::Matrix3d eul2RotMat(m_point euler){
    Eigen::Matrix3d rot_mat;

    double cos_roll = cos(euler(0));
    double sin_roll = sin(euler(0));
    double cos_pitch = cos(euler(1));
    double sin_pitch = sin(euler(1));
    double cos_yaw = cos(euler(2));
    double sin_yaw = sin(euler(2));

    // Left column (x)
    rot_mat(0, 0) = cos_yaw * cos_pitch;
    rot_mat(1, 0) = sin_yaw * cos_pitch;
    rot_mat(2, 0) = -sin_pitch;

    // Middle column (y)
    rot_mat(0, 1) = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    rot_mat(1, 1) = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    rot_mat(2, 1) = cos_pitch * sin_roll;

    // Right column (z)
    rot_mat(0, 2) = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
    rot_mat(1, 2) = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
    rot_mat(2, 2) = cos_pitch * cos_roll;

    return rot_mat;
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

bool endsWith(const std::string& mainString, const std::string& subString) {
    if (mainString.length() < subString.length())
        return false;

    return mainString.substr(mainString.length() - subString.length()) == subString;
}
