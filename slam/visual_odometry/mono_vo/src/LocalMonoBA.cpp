#include "mono_vo/LocalMonoBA.h"

#include <boost/unordered_set.hpp>

#include "camera_models/CostFunctionFactory.h"
#include "cauldron/EigenQuaternionParameterization.h"
#include "cauldron/EigenUtils.h"
#include "ceres/ceres.h"

namespace px
{

LocalMonoBA::LocalMonoBA(const CameraSystemConstPtr& cameraSystem,
                         int cameraId, int N)
 : k_cameraId(cameraId)
 , k_N(N)
{
    m_H_c_s = cameraSystem->getGlobalCameraPose(cameraId);
    Eigen::Matrix4d H_s_c = invertHomogeneousTransform(m_H_c_s);
    m_q_s_c = Eigen::Quaterniond(H_s_c.block<3,3>(0,0));
    m_t_s_c = H_s_c.block<3,1>(0,3);
}

bool
LocalMonoBA::addFrameSet(FrameSetPtr& frameSet, bool replaceCurrentFrameSet)
{
    if (replaceCurrentFrameSet)
    {
        m_window.pop_back();
    }

    m_window.push_back(frameSet);
    while (m_window.size() > k_N)
    {
        m_window.pop_front();
    }

    if (m_window.size() > 1)
    {
        optimize();
    }

    return true;
}

void
LocalMonoBA::optimize(void)
{
    ceres::Problem problem;

    boost::unordered_set<FrameSet*> frameSetsActive;
    boost::unordered_set<FrameSet*> frameSetsInactive;
    boost::unordered_set<Point3DFeature*> scenePoints;

    for (std::list<FrameSetPtr>::iterator it = m_window.begin(); it != m_window.end(); ++it)
    {
        FrameSet* frameSet = it->get();

        frameSetsActive.insert(frameSet);

        std::vector<Point2DFeaturePtr>& features = frameSet->frames().at(0)->features2D();

        for (size_t i = 0; i < features.size(); ++i)
        {
            Point3DFeaturePtr& scenePoint = features.at(i)->feature3D();

            if (!scenePoint)
            {
                continue;
            }

            scenePoints.insert(scenePoint.get());
        }
    }

    for (boost::unordered_set<Point3DFeature*>::iterator it = scenePoints.begin(); it != scenePoints.end(); ++it)
    {
        Point3DFeature* scenePoint = *it;

        for (size_t i = 0; i < scenePoint->features2D().size(); ++i)
        {
            Point2DFeature* feature = scenePoint->features2D().at(i);

            Frame* frame = feature->frame();

            if (frame->cameraId() != k_cameraId)
            {
                continue;
            }

            FrameSet* frameSet = frame->frameSet();

            ceres::LossFunction* lossFunction = new ceres::HuberLoss(0.0000055555);

            ceres::CostFunction* costFunction =
                CostFunctionFactory::instance()->generateCostFunction(m_q_s_c,
                                                                      m_t_s_c,
                                                                      feature->ray());

            problem.AddResidualBlock(costFunction, lossFunction,
                                     frameSet->systemPose()->rotationData(),
                                     frameSet->systemPose()->translationData(),
                                     scenePoint->pointData());

            if (frameSetsActive.find(frameSet) == frameSetsActive.end())
            {
                frameSetsInactive.insert(frameSet);
            }
        }
    }

    for (boost::unordered_set<FrameSet*>::iterator it = frameSetsActive.begin();
             it != frameSetsActive.end(); ++it)
    {
        ceres::LocalParameterization* quaternionParameterization =
            new EigenQuaternionParameterization;

        problem.SetParameterization((*it)->systemPose()->rotationData(),
                                    quaternionParameterization);
    }

    for (boost::unordered_set<FrameSet*>::iterator it = frameSetsInactive.begin();
             it != frameSetsInactive.end(); ++it)
    {
        problem.SetParameterBlockConstant((*it)->systemPose()->rotationData());
        problem.SetParameterBlockConstant((*it)->systemPose()->translationData());
    }

    if (m_window.size() < k_N)
    {
        problem.SetParameterBlockConstant(m_window.front()->systemPose()->rotationData());
        problem.SetParameterBlockConstant(m_window.front()->systemPose()->translationData());
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 20;
    options.num_threads = 4;
    options.num_linear_solver_threads = 4;
    options.max_num_consecutive_invalid_steps = 2;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
}

}
