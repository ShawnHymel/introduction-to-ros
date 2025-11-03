#ifndef TURTLE_TRAIN_CPP__UTIL_HPP_
#define TURTLE_TRAIN_CPP__UTIL_HPP_

#include <cmath>
#include <array>

namespace turtle_train_cpp
{

inline std::array<double, 4> euler_to_quaternion(double roll, double pitch, double yaw)
{
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    std::array<double, 4> q;
    q[0] = sr * cp * cy - cr * sp * sy;  // x
    q[1] = cr * sp * cy + sr * cp * sy;  // y
    q[2] = cr * cp * sy - sr * sp * cy;  // z
    q[3] = cr * cp * cy + sr * sp * sy;  // w

    return q;
}

}

#endif  // TURTLE_TRAIN_CPP__UTIL_HPP_