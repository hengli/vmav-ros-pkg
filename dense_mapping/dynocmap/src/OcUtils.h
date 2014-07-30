#ifndef OCUTILS_H
#define OCUTILS_H

#include <Eigen/Core>

namespace px
{

template<typename T>
T cube(T x)
{
    return x * x * x;
}

template<typename T>
Eigen::Vector3i
pointToGridCoords(const Eigen::Matrix<T,3,1>& point, T resolution)
{
    Eigen::Vector3i gridCoords;

    gridCoords(0) = static_cast<int>(floor(point(0) / resolution));
    gridCoords(1) = static_cast<int>(floor(point(1) / resolution));
    gridCoords(2) = static_cast<int>(floor(point(2) / resolution));

    return gridCoords;
}

template<typename T>
Eigen::Matrix<T,3,1>
gridCoordsToPoint(const Eigen::Vector3i& gridCoords, T resolution)
{
    Eigen::Matrix<T,3,1> point;

    point(0) = static_cast<T>(gridCoords(0)) * resolution;
    point(1) = static_cast<T>(gridCoords(1)) * resolution;
    point(2) = static_cast<T>(gridCoords(2)) * resolution;

    return point;
}

}

#endif
