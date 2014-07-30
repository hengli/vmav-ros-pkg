#include "sensor_models/StereoSensorModel.h"

namespace px
{

StereoSensorModel::StereoSensorModel(double k,
                                     double baseline,
                                     double focalLength,
                                     double disparityError,
                                     double freeSpaceProbability,
                                     double maxSensorRange,
                                     bool useLUT)
 : SensorModel(SensorModel::STEREO, maxSensorRange)
 , m_k(k)
 , m_baseline(baseline)
 , m_focalLength(focalLength)
 , m_disparityError(disparityError)
 , m_freeSpaceProbability(freeSpaceProbability)
 , m_unknownSpaceProbability(0.5)
 , m_freeSpaceLogOdds(probToLogOdds(m_freeSpaceProbability))
 , m_unknownSpaceLogOdds(probToLogOdds(m_unknownSpaceProbability))
 , m_z(0)
 , m_r_p(0)
 , m_lutResolution(0.05)
 , m_useLUT(useLUT)
{
    double delta_r_p = square(m_maxSensorRange) * m_disparityError / (m_baseline * m_focalLength);
    double a = m_k * (1.0 - m_unknownSpaceProbability) * exp(-delta_r_p);
    double c = sqrt(log(1.0 + a / (a + 2.0 * (m_unknownSpaceProbability - m_freeSpaceProbability))) / log(2.0)) * delta_r_p;

    m_maxRange = m_maxSensorRange + sqrt(-2.0 * square(c) * log(0.01 / a));

    if (useLUT)
    {
        buildLUT();
    }
}

double
StereoSensorModel::focalLength(void) const
{
    return m_focalLength;
}

void
StereoSensorModel::setObstacle(const Eigen::Vector3d& pObstacle)
{
    if (m_useLUT)
    {
        m_z = pObstacle(2) / m_lutResolution;
        m_r_p = pObstacle.norm() / m_lutResolution;
    }

    m_pObstacle = pObstacle;

    double delta_r_p = square(m_pObstacle(2)) * m_disparityError / (m_baseline * m_focalLength);
    double a = m_k * (1.0 - m_unknownSpaceProbability) * exp(-delta_r_p);
    double c = sqrt(log(1.0 + a / (a + 2.0 * (m_unknownSpaceProbability - m_freeSpaceProbability))) / log(2.0)) * delta_r_p;

    m_validRange = m_pObstacle.norm() + sqrt(-2.0 * square(c) * log(0.01 / a));
}

double
StereoSensorModel::freeSpaceLogOdds(void) const
{
    return m_freeSpaceLogOdds;
}

double
StereoSensorModel::getLikelihood(double r) const
{
    if (m_useLUT)
    {
        if (m_z < 0 || m_z > m_lutRange[0])
        {
            return m_unknownSpaceProbability;
        }

        if (m_r_p < 0 || m_r_p > m_lutRange[1])
        {
            return m_unknownSpaceProbability;
        }

        int r_i = r / m_lutResolution;

        if (r_i < 0 || r_i > m_lutRange[2])
        {
            return m_unknownSpaceProbability;
        }

        return logOddsToProb(getLogOddsLikelihood(r));
    }
    else
    {
        double r_p = m_pObstacle.norm();
        double delta_r_p = square(m_pObstacle(2)) * m_disparityError / (m_baseline * m_focalLength);
        double a = m_k * (1.0 - m_unknownSpaceProbability) * exp(-delta_r_p);

        if (r < r_p)
        {
            return m_freeSpaceProbability +
                   (a + m_unknownSpaceProbability - m_freeSpaceProbability) * exp(- 0.5 * square((r - r_p) / delta_r_p));
        }
        else
        {
            double c = sqrt(log(1.0 + a / (a + 2.0 * (m_unknownSpaceProbability - m_freeSpaceProbability))) / log(2.0)) * delta_r_p;

            return m_unknownSpaceProbability + a * exp(- 0.5 * square((r - r_p) / c));
        }
    }
}

double
StereoSensorModel::getLikelihood(double r1, double r2) const
{
    if (r1 == r2)
    {
        return getLikelihood(r1);
    }
    else if (r1 > r2)
    {
        std::swap(r1, r2);
    }

    if (m_useLUT)
    {
        double r_p = m_r_p * m_lutResolution;

        if (r2 < r_p)
        {
            return getLikelihood(r2);
        }
        else if (r1 > r_p)
        {
            return getLikelihood(r1);
        }
        else
        {
            return logOddsToProb(m_maxLogOddsLikelihood[m_z]);
        }
    }
    else
    {
        double r_p = m_pObstacle.norm();

        if (r2 < r_p)
        {
            return getLikelihood(r2);
        }
        else if (r1 > r_p)
        {
            return getLikelihood(r1);
        }
        else
        {
            double delta_r_p = square(m_pObstacle(2)) * m_disparityError / (m_baseline * m_focalLength);
            double a = m_k * (1.0 - m_unknownSpaceProbability) * exp(-delta_r_p);

            double ml = a + (m_unknownSpaceProbability - m_freeSpaceProbability) / 2.0 +
                             m_freeSpaceProbability;

            return ml;
        }
    }
}

double
StereoSensorModel::getLogOddsLikelihood(double r) const
{
    if (m_useLUT)
    {
        if (m_z < 0 || m_z > m_lutRange[0])
        {
            return m_unknownSpaceLogOdds;
        }

        if (m_r_p < 0 || m_r_p > m_lutRange[1])
        {
            return m_unknownSpaceLogOdds;
        }

        int r_i = r / m_lutResolution;

        if (r_i < 0 || r_i > m_lutRange[2])
        {
            return m_unknownSpaceLogOdds;
        }

        return m_lut[m_z][m_r_p][r_i];
    }
    else
    {
        return logOddsToProb(getLikelihood(r));
    }
}

double
StereoSensorModel::getLogOddsLikelihood(double r1, double r2) const
{
    if (r1 == r2)
    {
        return getLogOddsLikelihood(r1);
    }
    else if (r1 > r2)
    {
        std::swap(r1, r2);
    }

    if (m_useLUT)
    {
        double r_p = m_r_p * m_lutResolution;

        if (r2 < r_p)
        {
            return getLogOddsLikelihood(r2);
        }
        else if (r1 > r_p)
        {
            return getLogOddsLikelihood(r1);
        }
        else
        {
            return m_maxLogOddsLikelihood[m_z];
        }
    }
    else
    {
        double r_p = m_pObstacle.norm();

        if (r2 < r_p)
        {
            return getLogOddsLikelihood(r2);
        }
        else if (r1 > r_p)
        {
            return getLogOddsLikelihood(r1);
        }
        else
        {
            return probToLogOdds(getLikelihood(r1, r2));
        }
    }
}

void
StereoSensorModel::buildLUT(void)
{
    m_lutRange[0] = m_lutRange[1] = ceil(m_maxSensorRange / m_lutResolution);
    m_lutRange[2] = ceil(m_maxRange / m_lutResolution);

    m_lut.resize(boost::extents[m_lutRange[0] + 1][m_lutRange[1] + 1][m_lutRange[2] + 1]);
    m_maxLogOddsLikelihood.resize(m_lutRange[0] + 1);

    for (int z = 0; z <= m_lutRange[0]; ++z)
    {
        for (int r_p = 0; r_p <= m_lutRange[1]; ++r_p)
        {
            for (int r = 0; r <= m_lutRange[2]; ++r)
            {
                m_lut[z][r_p][r] = m_unknownSpaceLogOdds;
            }
        }
    }

    for (int z = 0; z <= m_lutRange[0]; ++z)
    {
        double delta_r_p = square(z * m_lutResolution) * m_disparityError / (m_baseline * m_focalLength);

        // a - decreasing exponential curve with minimum equal to 0 and
        //     maximum equal to 1 - unknownProbability
        double a = m_k * (1.0 - m_unknownSpaceProbability) * exp(-delta_r_p);

        if (z == 0)
        {
            m_maxLogOddsLikelihood[z] = m_freeSpaceLogOdds;
        }
        else
        {
            m_maxLogOddsLikelihood[z] = probToLogOdds(a + (m_unknownSpaceProbability - m_freeSpaceProbability) / 2.0 +
                                                      m_freeSpaceProbability);
        }

        for (int r_p = z; r_p <= m_lutRange[1]; ++r_p)
        {
            double c = sqrt(log(1.0 + a / (a + 2.0 * (m_unknownSpaceProbability - m_freeSpaceProbability))) / log(2.0)) * delta_r_p;
            int max_r = ceil((r_p * m_lutResolution + sqrt(-2.0 * square(c) * log(0.01 / a))) / m_lutResolution);

            max_r = fmin(max_r, m_lutRange[2]);

            for (int r = 0; r <= max_r; ++r)
            {
                double p = 0.0;
                if (z == 0)
                {
                    p = m_freeSpaceProbability;
                }
                else
                {
                    if (r < r_p)
                    {
                        p =  m_freeSpaceProbability +
                             (a + m_unknownSpaceProbability - m_freeSpaceProbability) * exp(- 0.5 * square((r - r_p) * m_lutResolution / delta_r_p));
                    }
                    else
                    {
                        p = m_unknownSpaceProbability + a * exp(- 0.5 * square((r - r_p) * m_lutResolution / c));
                    }
                }

                m_lut[z][r_p][r] = probToLogOdds(p);
            }
        }
    }
}

}
