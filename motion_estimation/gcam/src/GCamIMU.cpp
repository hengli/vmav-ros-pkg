#include "gcam/GCamIMU.h"

#include "camera_models/CostFunctionFactory.h"
#include "cauldron/EigenQuaternionParameterization.h"
#include "cauldron/EigenUtils.h"
#include "ceres/ceres.h"

namespace px
{

GCamIMU::GCamIMU(const CameraSystemConstPtr& cameraSystem)
 : k_sphericalErrorThresh(0.999976)
 , m_cameraSystem(cameraSystem)
{

}

bool
GCamIMU::estimateT(const std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> >& lcVec,
                   const Eigen::Matrix3d& R, Eigen::Vector3d& t) const
{
    if (lcVec.size() < 3)
    {
        // a minimum of 3 correspondences is required to find a solution.
        return false;
    }

    Eigen::MatrixXd A(lcVec.size(), 3);
    Eigen::VectorXd B(lcVec.size());
    for (size_t i = 0; i < lcVec.size(); ++i)
    {
        const PLineCorrespondence& lc = lcVec.at(i);

        Eigen::Matrix<double,6,1> l1;
        l1 << lc.l1().directionVector(), lc.l1().momentVector();

        Eigen::Matrix<double,6,1> l2;
        l2 << lc.l2().directionVector(), lc.l2().momentVector();

        A(i,0) = l1(0)*(R(1,0)*l2(2) - R(2,0)*l2(1)) + l1(1)*(R(1,1)*l2(2) - R(2,1)*l2(1)) + l1(2)*(R(1,2)*l2(2) - R(2,2)*l2(1));
        A(i,1) = - l1(0)*(R(0,0)*l2(2) - R(2,0)*l2(0)) - l1(1)*(R(0,1)*l2(2) - R(2,1)*l2(0)) - l1(2)*(R(0,2)*l2(2) - R(2,2)*l2(0));
        A(i,2) = l1(0)*(R(0,0)*l2(1) - R(1,0)*l2(0)) + l1(1)*(R(0,1)*l2(1) - R(1,1)*l2(0)) + l1(2)*(R(0,2)*l2(1) - R(1,2)*l2(0));
        B(i) = - l1(3)*(R(0,0)*l2(0) + R(1,0)*l2(1) + R(2,0)*l2(2)) - l1(4)*(R(0,1)*l2(0) + R(1,1)*l2(1) + R(2,1)*l2(2)) - l1(0)*(R(0,0)*l2(3) + R(1,0)*l2(4) + R(2,0)*l2(5)) - l1(5)*(R(0,2)*l2(0) + R(1,2)*l2(1) + R(2,2)*l2(2)) - l1(1)*(R(0,1)*l2(3) + R(1,1)*l2(4) + R(2,1)*l2(5)) - l1(2)*(R(0,2)*l2(3) + R(1,2)*l2(4) + R(2,2)*l2(5));
    }

    if (lcVec.size() == 3)
    {
        double a[3], b[3], c[3], d[3];

        for (int i = 0; i < 3; ++i)
        {
            a[i] = A(i,0);
            b[i] = A(i,1);
            c[i] = A(i,2);
            d[i] = -B(i);
        }

        t(0) = - b[0]*c[1]*d[2] + b[0]*c[2]*d[1] + b[1]*c[0]*d[2] - b[1]*c[2]*d[0] - b[2]*c[0]*d[1] + b[2]*c[1]*d[0];
        t(1) = a[0]*c[1]*d[2] - a[0]*c[2]*d[1] - a[1]*c[0]*d[2] + a[1]*c[2]*d[0] + a[2]*c[0]*d[1] - a[2]*c[1]*d[0];
        t(2) = - a[0]*b[1]*d[2] + a[0]*b[2]*d[1] + a[1]*b[0]*d[2] - a[1]*b[2]*d[0] - a[2]*b[0]*d[1] + a[2]*b[1]*d[0];

        t /= a[0]*b[1]*c[2] - a[0]*b[2]*c[1] - a[1]*b[0]*c[2] + a[1]*b[2]*c[0] + a[2]*b[0]*c[1] - a[2]*b[1]*c[0];

        return true;
    }
    else
    {
        // compute least-squares solution
        t = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

        return true;
    }
}

bool
GCamIMU::estimateH(const std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> >& lcVec,
                   const Eigen::Matrix3d& R, Eigen::Matrix4d& H) const
{
    std::vector<size_t> inliers;

    return estimateH(lcVec, R, H, inliers);
}

bool
GCamIMU::estimateH(const std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> >& lcVec,
                   const Eigen::Matrix3d& R, Eigen::Matrix4d& H,
                   std::vector<size_t>& inliers) const
{
    if (lcVec.size() < 3)
    {
        // a minimum of 3 correspondences is required to find a solution.
        return false;
    }

    // Check for degenerate cases.

    // Case 1: no inter-camera correspondences
    bool isDegenerate = true;
    for (size_t i = 0; i < lcVec.size(); ++i)
    {
        const PLineCorrespondence& lc = lcVec.at(i);

        if (lc.cameraId1() != lc.cameraId2())
        {
            isDegenerate = false;
            break;
        }
    }

    if (isDegenerate)
    {
        return false;
    }

    double p = 0.99; // probability that at least one set of random samples does not contain an outlier
    double v = 0.5; // probability of observing an outlier

    double u = 1.0 - v;
    int N = static_cast<int>(log(1.0 - p) / log(1.0 - u * u * u) + 0.5);

    std::vector<size_t> indices;
    for (size_t i = 0; i < lcVec.size(); ++i)
    {
        indices.push_back(i);
    }

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > H_sys_cam;
    for (int i = 0; i < m_cameraSystem->cameraCount(); ++i)
    {
        const Eigen::Matrix4d& H = m_cameraSystem->getGlobalCameraPose(i);

        H_sys_cam.push_back(invertHomogeneousTransform(H));
    }

    // run RANSAC to find initial t
    Eigen::Vector3d t_best;
    std::vector<size_t> inliers_best;
    for (int i = 0; i < N; ++i)
    {
        std::random_shuffle(indices.begin(), indices.end());

        std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> > samples;
        samples.push_back(lcVec.at(indices.at(0)));
        samples.push_back(lcVec.at(indices.at(1)));
        samples.push_back(lcVec.at(indices.at(2)));

        Eigen::Vector3d t;
        if (!estimateT(samples, R, t))
        {
            continue;
        }

        std::vector<size_t> inliers;
        for (size_t j = 0; j < lcVec.size(); ++j)
        {
            const PLineCorrespondence& lc = lcVec.at(j);

            Eigen::Vector3d P = triangulate3DPoint(lc, R, t);

            Eigen::Matrix4d H_sys1_sys2 = Eigen::Matrix4d::Identity();
            H_sys1_sys2.block<3,3>(0,0) = R;
            H_sys1_sys2.block<3,1>(0,3) = t;

            const Eigen::Matrix4d& H_cam1 = H_sys_cam.at(lc.cameraId1());
            Eigen::Vector3d P_cam1 = transformPoint(H_cam1, P);

            Eigen::Vector3d ray1_est = P_cam1.normalized();

            double err = fabs(ray1_est.dot(lc.ray1()));
            if (err < k_sphericalErrorThresh)
            {
                continue;
            }

            Eigen::Matrix4d H_cam2 = H_sys_cam.at(lc.cameraId2()) * H_sys1_sys2;
            Eigen::Vector3d P_cam2 = transformPoint(H_cam2, P);

            Eigen::Vector3d ray2_est = P_cam2.normalized();

            err = fabs(ray2_est.dot(lc.ray2()));
            if (err < k_sphericalErrorThresh)
            {
                continue;
            }

            inliers.push_back(j);
        }

        if (inliers.size() > inliers_best.size())
        {
            t_best = t;
            inliers_best = inliers;
        }
    }

    if (inliers_best.size() < 3)
    {
        return false;
    }

    // run non-linear refinement to find H from R and t
    ceres::Problem problem;
    ceres::Solver::Options options;

    Eigen::Quaterniond q(R);
    double t[3] = {t_best(0), t_best(1), t_best(2)};

    double q_zero[4] = {0.0, 0.0, 0.0, 1.0};
    double t_zero[3] = {0.0, 0.0, 0.0};

    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > q_sys_cam;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > t_sys_cam;

    for (int i = 0; i < m_cameraSystem->cameraCount(); ++i)
    {
        const Eigen::Matrix4d& H = H_sys_cam.at(i);

        q_sys_cam.push_back(Eigen::Quaterniond(H.block<3,3>(0,0)));
        t_sys_cam.push_back(H.block<3,1>(0,3));
    }

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > scenePoints(inliers_best.size());

    for (size_t i = 0; i < inliers_best.size(); ++i)
    {
        size_t inlierIdx = inliers_best.at(i);

        const PLineCorrespondence& lc = lcVec.at(inlierIdx);

        scenePoints.at(i) = triangulate3DPoint(lc, R, t_best);

        ceres::LossFunction* lossFunction = new ceres::CauchyLoss(0.0000055555);

        ceres::CostFunction* costFunction =
            CostFunctionFactory::instance()->generateCostFunction(q_sys_cam.at(lc.cameraId1()),
                                                                  t_sys_cam.at(lc.cameraId1()),
                                                                  lc.ray1());

        problem.AddResidualBlock(costFunction, lossFunction, q_zero, t_zero, scenePoints.at(i).data());

        lossFunction = new ceres::CauchyLoss(0.0000055555);

        costFunction =
            CostFunctionFactory::instance()->generateCostFunction(q_sys_cam.at(lc.cameraId2()),
                                                                  t_sys_cam.at(lc.cameraId2()),
                                                                  lc.ray2());

        problem.AddResidualBlock(costFunction, lossFunction, q.coeffs().data(), t, scenePoints.at(i).data());
    }

    ceres::LocalParameterization* quaternionParameterization =
        new EigenQuaternionParameterization;

    problem.SetParameterization(q.coeffs().data(), quaternionParameterization);

    problem.SetParameterBlockConstant(q_zero);
    problem.SetParameterBlockConstant(t_zero);

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    H.setIdentity();
    H.block<3,3>(0,0) = q.toRotationMatrix();
    H(0,3) = t[0];
    H(1,3) = t[1];
    H(2,3) = t[2];

    return true;
}

Eigen::Vector3d
GCamIMU::triangulate3DPoint(const PLineCorrespondence& lc,
                            const Eigen::Matrix3d& R,
                            const Eigen::Vector3d& t) const
{
    Eigen::Vector3d P;

    Eigen::Vector3d q1 = lc.l1().directionVector();
    Eigen::Vector3d q1p = lc.l1().momentVector();

    Eigen::Vector3d q2 = lc.l2().directionVector();
    Eigen::Vector3d q2p = lc.l2().momentVector();

    Eigen::Vector3d q1xq1p = q1.cross(q1p);

    Eigen::MatrixXd A(3,2);
    A.col(0) = R * q1;
    A.col(1) = -q2;

    Eigen::Vector3d b = q2.cross(q2p) - R * q1xq1p - t;

    Eigen::Vector2d gamma = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    P = q1xq1p + gamma(0) * q1;

    return P;
}

}
