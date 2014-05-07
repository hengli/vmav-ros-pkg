#include "stereo_vo/LocalStereoBA.h"

#include <boost/unordered_set.hpp>

#include "camera_models/CostFunctionFactory.h"
#include "cauldron/EigenQuaternionParameterization.h"
#include "cauldron/EigenUtils.h"
#include "ceres/ceres.h"

namespace px
{

LocalStereoBA::LocalStereoBA(const CameraSystemConstPtr& cameraSystem,
                             int cameraId1, int cameraId2,
                             int N)
 : k_cameraId1(cameraId1)
 , k_cameraId2(cameraId2)
 , k_N(N)
{
    m_H_1_s = cameraSystem->getGlobalCameraPose(cameraId1);
    Eigen::Matrix4d H_s_1 = invertHomogeneousTransform(m_H_1_s);
    m_q_s_1 = Eigen::Quaterniond(H_s_1.block<3,3>(0,0));
    m_t_s_1 = H_s_1.block<3,1>(0,3);

    m_H_2_s = cameraSystem->getGlobalCameraPose(cameraId2);
    Eigen::Matrix4d H_s_2 = invertHomogeneousTransform(m_H_2_s);
    m_q_s_2 = Eigen::Quaterniond(H_s_2.block<3,3>(0,0));
    m_t_s_2 = H_s_2.block<3,1>(0,3);
}

bool
LocalStereoBA::addFrameSet(FrameSetPtr& frameSet, bool replaceCurrentFrameSet)
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
LocalStereoBA::optimize(void)
{
    ceres::Problem problem;

    boost::unordered_set<FrameSet*> frameSetsActive;
    boost::unordered_set<FrameSet*> frameSetsInactive;
    boost::unordered_set<Point3DFeature*> scenePoints;

    for (std::list<FrameSetPtr>::iterator it = m_window.begin(); it != m_window.end(); ++it)
    {
        FrameSet* frameSet = it->get();

        frameSetsActive.insert(frameSet);

        std::vector<Point2DFeaturePtr>& features1 = frameSet->frames().at(0)->features2D();

        for (size_t i = 0; i < features1.size(); ++i)
        {
            Point2DFeaturePtr& feature1 = features1.at(i);

            if (feature1->prevMatches().empty() && feature1->nextMatches().empty())
            {
                continue;
            }

            scenePoints.insert(feature1->feature3D().get());
        }
    }

    for (boost::unordered_set<Point3DFeature*>::iterator it = scenePoints.begin(); it != scenePoints.end(); ++it)
    {
        Point3DFeature* scenePoint = *it;

        for (size_t i = 0; i < scenePoint->features2D().size(); ++i)
        {
            Point2DFeature* feature = scenePoint->features2D().at(i);

            Frame* frame = feature->frame();

            if (frame->cameraId() != k_cameraId1)
            {
                continue;
            }

            FrameSet* frameSet = frame->frameSet();
            Point2DFeature* feature2 = feature->match();

            ceres::LossFunction* lossFunction = new ceres::HuberLoss(0.0000055555);

            ceres::CostFunction* costFunction =
                CostFunctionFactory::instance()->generateCostFunction(feature->ray(),
                                                                      feature2->ray(),
                                                                      m_q_s_1, m_t_s_1,
                                                                      m_q_s_2, m_t_s_2);

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

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 20;
    options.num_threads = 4;
    options.num_linear_solver_threads = 4;
    options.max_num_consecutive_invalid_steps = 2;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Update 3D coordinates of scene points that are only observed in a single frame set
    // since these points are not optimized in local bundle adjustment.
    for (std::list<FrameSetPtr>::iterator it = m_window.begin(); it != m_window.end(); ++it)
    {
        FrameSetPtr& frameSet = *it;

        Eigen::Matrix4d camPose = invertHomogeneousTransform(frameSet->systemPose()->toMatrix()) *
                                  m_H_1_s;

        std::vector<Point2DFeaturePtr>& features1 = frameSet->frames().at(0)->features2D();

        for (size_t i = 0; i < features1.size(); ++i)
        {
            Point2DFeaturePtr& feature1 = features1.at(i);

            if (feature1->prevMatches().empty() && feature1->nextMatches().empty())
            {
                Point3DFeaturePtr& scenePoint = feature1->feature3D();

                scenePoint->point() = transformPoint(camPose, scenePoint->pointFromStereo());
            }
        }
    }
}

}
