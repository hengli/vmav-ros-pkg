#include "sensor_models/LaserSensorModel.h"

namespace px
{

LaserSensorModel::LaserSensorModel(double freeSpaceProbability,
                                   double occSpaceProbability,
                                   double maxSensorRange,
                                   double sigma)
 : SensorModel(SensorModel::LASER, maxSensorRange)
 , m_freeSpaceProbability(freeSpaceProbability)
 , m_unknownSpaceProbability(0.5)
 , m_occSpaceProbability(occSpaceProbability)
 , m_freeSpaceLogOdds(probToLogOdds(m_freeSpaceProbability))
 , m_unknownSpaceLogOdds(probToLogOdds(m_unknownSpaceProbability))
 , m_occSpaceLogOdds(probToLogOdds(m_occSpaceProbability))
 , m_sigma(sigma)
 , m_r_p(0.0)
{
    m_maxRange = m_maxSensorRange;
}

void
LaserSensorModel::setObstacle(const Eigen::Vector3d& pObstacle)
{
    m_r_p = pObstacle.norm();

    m_pObstacle = pObstacle;
    m_validRange = m_pObstacle.norm() + 2.0 * m_sigma;
}

double
LaserSensorModel::freeSpaceLogOdds(void) const
{
    return m_freeSpaceLogOdds;
}

double
LaserSensorModel::getLikelihood(double r) const
{
    if (r < m_r_p - m_sigma)
    {
        return m_freeSpaceProbability;
    }
    else if (r > m_r_p + m_sigma)
    {
        return m_unknownSpaceProbability;
    }
    else
    {
        return m_occSpaceProbability;
    }
}

double
LaserSensorModel::getLikelihood(double r1, double r2) const
{
    if (r1 == r2)
    {
        return getLikelihood(r1);
    }
    else if (r1 > r2)
    {
        std::swap(r1, r2);
    }

    if (r2 < m_r_p)
    {
        return getLikelihood(r2);
    }
    else if (r1 > m_r_p)
    {
        return getLikelihood(r1);
    }
    else
    {
        return m_occSpaceProbability;
    }
}

double
LaserSensorModel::getLogOddsLikelihood(double r) const
{
    if (r < m_r_p - m_sigma)
    {
        return m_freeSpaceLogOdds;
    }
    else if (r > m_r_p + m_sigma)
    {
        return m_unknownSpaceLogOdds;
    }
    else
    {
        return m_occSpaceLogOdds;
    }
}

double
LaserSensorModel::getLogOddsLikelihood(double r1, double r2) const
{
    if (r1 == r2)
    {
        return getLogOddsLikelihood(r1);
    }
    else if (r1 > r2)
    {
        std::swap(r1, r2);
    }

    if (r2 < m_r_p)
    {
        return getLogOddsLikelihood(r2);
    }
    else if (r1 > m_r_p)
    {
        return getLogOddsLikelihood(r1);
    }
    else
    {
        return m_occSpaceLogOdds;
    }
}

}
