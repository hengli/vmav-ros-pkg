#ifndef OCCUPANCYCELL_H_
#define OCCUPANCYCELL_H_

namespace px
{

typedef struct
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3i coords;
    unsigned short width;
    float occupancyLogOdds;
    float occupancyProb;
} OccupancyCell;

}

#endif
