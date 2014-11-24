#include "sparse_graph/Transform.h"

namespace px
{

Transform::Transform(TransformType type)
 : m_type(type)
 , m_q(Eigen::Quaterniond::Identity())
 , m_t(Eigen::Vector3d::Zero())
 , m_s(1.0)
{

}

Transform::Transform(const Eigen::Matrix4d& H,
                     TransformType type)
 : m_type(type)
 , m_q(H.block<3,3>(0,0))
 , m_t(H.block<3,1>(0,3))
 , m_s(1.0)
{

}

void
Transform::setIdentity(void)
{
    m_q.setIdentity();
    m_t.setZero();
}

Eigen::Quaterniond&
Transform::rotation(void)
{
    return m_q;
}

const Eigen::Quaterniond&
Transform::rotation(void) const
{
    return m_q;
}

double*
Transform::rotationData(void)
{
    return m_q.coeffs().data();
}

const double* const
Transform::rotationData(void) const
{
    return m_q.coeffs().data();
}

Eigen::Vector3d&
Transform::translation(void)
{
    return m_t;
}

const Eigen::Vector3d&
Transform::translation(void) const
{
    return m_t;
}

double*
Transform::translationData(void)
{
    return m_t.data();
}

const double* const
Transform::translationData(void) const
{
    return m_t.data();
}

double&
Transform::scale(void)
{
    return m_s;
}

double
Transform::scale(void) const
{
    return m_s;
}

double*
Transform::scaleData(void)
{
    return &m_s;
}

const double* const
Transform::scaleData(void) const
{
    return &m_s;
}

Eigen::Matrix4d
Transform::toMatrix(void) const
{
    Eigen::Matrix4d H;
    H.setIdentity();
    H.block<3,3>(0,0) = m_q.toRotationMatrix();
    H.block<3,1>(0,3) = m_t;

    if (m_type == TRANSFORM_SIM3)
    {
        H.block<3,3>(0,0) *= m_s;
    }

    return H;
}

}
