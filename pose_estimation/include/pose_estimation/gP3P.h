#ifndef GP3P_H
#define GP3P_H

#include <Eigen/Dense>
#include <vector>

#include "cauldron/PLine.h"

namespace px
{

bool solvegP3P(const std::vector<PLine, Eigen::aligned_allocator<PLine> >& plineVectors,
               const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& worldPoints,
               std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& solutions);

}

#endif
