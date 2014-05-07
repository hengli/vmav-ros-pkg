#include "gcam_vo/GCamLocalBA.h"

#include <boost/unordered_set.hpp>

#include "camera_models/CostFunctionFactory.h"
#include "cauldron/EigenQuaternionParameterization.h"
#include "cauldron/EigenUtils.h"
#include "ceres/ceres.h"

namespace px
{

GCamLocalBA::GCamLocalBA(const CameraSystemConstPtr& cameraSystem,
                         int N)
 : k_N(N)
{
    for (int i = 0; i < cameraSystem->cameraCount(); ++i)
    {
        Eigen::Matrix4d H_cam_sys = cameraSystem->getGlobalCameraPose(i);

        m_H_cam_sys.push_back(H_cam_sys);

        Eigen::Matrix4d H_sys_cam = invertHomogeneousTransform(H_cam_sys);

        m_T_sys_cam.push_back(Transform(H_sys_cam));
    }
}

bool
GCamLocalBA::addFrameSet(FrameSetPtr& frameSet, bool replaceCurrentFrameSet)
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
GCamLocalBA::optimize(void)
{
    ceres::Problem problem;

    boost::unordered_set<FrameSet*> frameSetsActive;
    boost::unordered_set<FrameSet*> frameSetsInactive;
    boost::unordered_set<Point3DFeature*> scenePoints;

    for (std::list<FrameSetPtr>::iterator it = m_window.begin(); it != m_window.end(); ++it)
    {
        FrameSet* frameSet = it->get();

        frameSetsActive.insert(frameSet);

        for (size_t i = 0; i < frameSet->frames().size(); i += 2)
        {
            std::vector<Point2DFeaturePtr>& features = frameSet->frames().at(i)->features2D();

            for (size_t i = 0; i < features.size(); ++i)
            {
                Point2DFeaturePtr& feature = features.at(i);

                if (feature->prevMatches().empty() && feature->nextMatches().empty())
                {
                    continue;
                }

                scenePoints.insert(feature->feature3D().get());
            }
        }
    }

    for (boost::unordered_set<Point3DFeature*>::iterator it = scenePoints.begin(); it != scenePoints.end(); ++it)
    {
        Point3DFeature* scenePoint = *it;

        for (size_t i = 0; i < scenePoint->features2D().size(); ++i)
        {
            Point2DFeature* feature1 = scenePoint->features2D().at(i);
            Frame* frame1 = feature1->frame();
            int cameraId1 = frame1->cameraId();

            if (cameraId1 % 2 == 1)
            {
                continue;
            }

            FrameSet* frameSet = frame1->frameSet();
            Point2DFeature* feature2 = feature1->match();
            Frame* frame2 = feature2->frame();
            int cameraId2 = frame2->cameraId();

            ceres::LossFunction* lossFunction = new ceres::HuberLoss(0.0000055555);

            ceres::CostFunction* costFunction =
                CostFunctionFactory::instance()->generateCostFunction(feature1->ray(),
                                                                      feature2->ray(),
                                                                      m_T_sys_cam.at(cameraId1).rotation(),
                                                                      m_T_sys_cam.at(cameraId1).translation(),
                                                                      m_T_sys_cam.at(cameraId2).rotation(),
                                                                      m_T_sys_cam.at(cameraId2).translation());

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

    // Update 3D coordinates of scene points that are only observed in a single frame set
    // since these points are not optimized in local bundle adjustment.
    for (std::list<FrameSetPtr>::iterator it = m_window.begin(); it != m_window.end(); ++it)
    {
        FrameSetPtr& frameSet = *it;

        for (size_t i = 0; i < frameSet->frames().size(); i += 2)
        {
            Eigen::Matrix4d H_cam = invertHomogeneousTransform(frameSet->systemPose()->toMatrix()) *
                                    m_H_cam_sys.at(frameSet->frames().at(i)->cameraId());

            std::vector<Point2DFeaturePtr>& features = frameSet->frames().at(i)->features2D();

            for (size_t i = 0; i < features.size(); ++i)
            {
                Point2DFeaturePtr& feature = features.at(i);

                if (feature->prevMatches().empty() && feature->nextMatches().empty())
                {
                    Point3DFeaturePtr& scenePoint = feature->feature3D();

                    scenePoint->point() = transformPoint(H_cam, scenePoint->pointFromStereo());
                }
            }
        }
    }
}

}
