#ifndef SENSORMODEL_H_
#define SENSORMODEL_H_

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <tf/LinearMath/Transform.h>

namespace px
{

class SensorModel
{
public:
    enum SensorType
    {
        LASER,
        STEREO
    };

    SensorModel(SensorType sensorType)
     : m_sensorType(sensorType)
     , m_maxSensorRange(0.0)
     , m_maxRange(0.0)
     , m_validRange(0.0)
     , m_pObstacle(Eigen::Vector3d::Zero())
     , m_sensorToBodyTransform(tf::Transform::getIdentity())
    {

    }

    explicit SensorModel(SensorType sensorType, double maxSensorRange)
     : m_sensorType(sensorType)
     , m_maxSensorRange(maxSensorRange)
     , m_maxRange(0.0)
     , m_validRange(0.0)
     , m_pObstacle(Eigen::Vector3d::Zero())
     , m_sensorToBodyTransform(tf::Transform::getIdentity())
    {

    }

    SensorType sensorType(void) const
    {
        return m_sensorType;
    }

    virtual double maxSensorRange(void) const
    {
        return m_maxSensorRange;
    }

    virtual double maxRange(void) const
    {
        return m_maxRange;
    }

    virtual double validRange(void) const
    {
        return m_validRange;
    }

    virtual void setObstacle(const Eigen::Vector3d& pObstacle)
    {
        m_pObstacle = pObstacle;
    }

    virtual double freeSpaceLogOdds(void) const = 0;
    virtual double getLikelihood(double r) const = 0;
    virtual double getLikelihood(double r1, double r2) const = 0;
    virtual double getLogOddsLikelihood(double r) const = 0;
    virtual double getLogOddsLikelihood(double r1, double r2) const = 0;

    const tf::Transform& sensorToBodyTransform(void) const
    {
        return m_sensorToBodyTransform;
    }

protected:
    double square(double x) const
    {
        return x * x;
    }

    double probToLogOdds(double prob) const
    {
        return log(prob / (1.0 - prob));
    }

    double logOddsToProb(double logodds) const
    {
        double e = exp(logodds);

        return e / (1.0 + e);
    }

    SensorType m_sensorType;
    double m_maxSensorRange;
    double m_maxRange;
    double m_validRange;
    Eigen::Vector3d m_pObstacle;

    tf::Transform m_sensorToBodyTransform;
};

typedef boost::shared_ptr<SensorModel> SensorModelPtr;

}

#endif
