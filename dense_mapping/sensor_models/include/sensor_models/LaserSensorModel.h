#ifndef LASERSENSORMODEL_H_
#define LASERSENSORMODEL_H_

#include <boost/multi_array.hpp>
#include <boost/shared_ptr.hpp>

#include "SensorModel.h"

namespace px
{

class LaserSensorModel: public SensorModel
{
public:
    LaserSensorModel(double freeSpaceProbability,
                     double occSpaceProbability,
                     double maxSensorRange,
                     double sigma);

    void setObstacle(const Eigen::Vector3d& pObstacle);

    double freeSpaceLogOdds(void) const;
    double getLikelihood(double r) const;
    double getLikelihood(double r1, double r2) const;
    double getLogOddsLikelihood(double r) const;
    double getLogOddsLikelihood(double r1, double r2) const;

private:
    double m_freeSpaceProbability;
    double m_unknownSpaceProbability;
    double m_occSpaceProbability;

    double m_freeSpaceLogOdds;
    double m_unknownSpaceLogOdds;
    double m_occSpaceLogOdds;

    double m_sigma;

    double m_r_p;
};

typedef boost::shared_ptr<LaserSensorModel> LaserSensorModelPtr;

}

#endif
