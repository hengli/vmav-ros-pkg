#ifndef STEREOSENSORMODEL_H_
#define STEREOSENSORMODEL_H_

#include <boost/multi_array.hpp>
#include <boost/shared_ptr.hpp>

#include "SensorModel.h"

namespace px
{

class StereoSensorModel: public SensorModel
{
public:
    StereoSensorModel(double k,
                      double baseline, double focalLength, double disparityError,
                      double freeSpaceProbability, double maxSensorRange,
                      bool useLUT = true);

    double focalLength(void) const;

    void setObstacle(const Eigen::Vector3d& pObstacle);

    double freeSpaceLogOdds(void) const;
    double getLikelihood(double r) const;
    double getLikelihood(double r1, double r2) const;
    double getLogOddsLikelihood(double r) const;
    double getLogOddsLikelihood(double r1, double r2) const;

private:
    void buildLUT(void);

    double m_k;

    double m_baseline;
    double m_focalLength;
    double m_disparityError;

    double m_freeSpaceProbability;
    double m_unknownSpaceProbability;
    double m_freeSpaceLogOdds;
    double m_unknownSpaceLogOdds;

    int m_z;
    int m_r_p;

    boost::multi_array<double, 3> m_lut;
    std::vector<double> m_maxLogOddsLikelihood;
    int m_lutRange[3];
    double m_lutResolution;
    bool m_useLUT;
};

typedef boost::shared_ptr<StereoSensorModel> StereoSensorModelPtr;

}

#endif
