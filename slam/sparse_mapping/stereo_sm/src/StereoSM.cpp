#include "stereo_sm/StereoSM.h"

#include <boost/unordered_set.hpp>

#include "cauldron/EigenQuaternionParameterization.h"
#include "cauldron/EigenUtils.h"
#include "camera_models/CostFunctionFactory.h"
#include "ceres/ceres.h"
#include "pose_graph/PoseGraph.h"
#include "pose_graph/PoseGraphViz.h"

namespace px
{

StereoSM::StereoSM(ros::NodeHandle& nh,
                   const CameraSystemConstPtr& cameraSystem,
                   const SparseGraphPtr& sparseGraph)
 : m_nh(nh)
 , m_cameraSystem(cameraSystem)
 , m_sparseGraph(sparseGraph)
 , m_svo(cameraSystem, 0, 1, true)
 , m_sgv(nh, sparseGraph)
{

}

bool
StereoSM::init(const std::string& detectorType,
               const std::string& descriptorExtractorType,
               const std::string& descriptorMatcherType)
{
    return m_svo.init(detectorType, descriptorExtractorType, descriptorMatcherType);
}

bool
StereoSM::readFrames(const ros::Time& stamp,
                     const cv::Mat& imageL, const cv::Mat& imageR)
{
    return m_svo.readFrames(stamp, imageL, imageR);
}

bool
StereoSM::processFrames(void)
{
    FrameSetPtr frameSet;
    if (!m_svo.processFrames(frameSet))
    {
        return false;
    }

    if (m_sparseGraph->frameSetSegment(0).empty())
    {
        m_sparseGraph->frameSetSegment(0).push_back(frameSet);

        m_sgv.visualize(10);

        return true;        
    }

    if (m_svo.getCurrent2D3DCorrespondenceCount() < 40)
    {
        m_svo.keyCurrentFrameSet();

        m_sparseGraph->frameSetSegment(0).push_back(frameSet);

        m_sgv.visualize(10);
    }

    return true;
}

void
StereoSM::runPG(const std::string& vocFilename)
{
    PoseGraphPtr poseGraph = boost::make_shared<PoseGraph>(boost::cref(m_cameraSystem),
                                                           boost::ref(m_sparseGraph),
                                                           cv::Mat(), 50, 10);
    PoseGraphViz pgv(m_nh, poseGraph);

    poseGraph->setVerbose(true);

    poseGraph->buildEdges(vocFilename);

    pgv.visualize("pose_graph_before");
    m_sgv.visualize();

    double avgError, maxError;
    size_t featureCount;
    reprojErrorStats(avgError, maxError, featureCount);

    ROS_INFO("Reprojection error before pose graph optimization: avg = %.3f | max = %.3f | count = %lu",
             avgError, maxError, featureCount);

    poseGraph->optimize(true);

    pgv.visualize("pose_graph_after");

    reprojErrorStats(avgError, maxError, featureCount);

    ROS_INFO("Reprojection error after pose graph optimization: avg = %.3f | max = %.3f | count = %lu",
             avgError, maxError, featureCount);

    // merge pairs of duplicate scene points
    std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > corr2D3D = poseGraph->getCorrespondences2D3D();

    int nMergedScenePoints = 0;
    for (size_t i = 0; i < corr2D3D.size(); ++i)
    {
        Point3DFeaturePtr scenePoint1 = corr2D3D.at(i).first->feature3D();
        Point3DFeaturePtr scenePoint2 = corr2D3D.at(i).second;

        if (scenePoint1 == scenePoint2)
        {
            continue;
        }

        bool merge = false;
        for (size_t j = 0; j < scenePoint2->features2D().size(); ++j)
        {
            Point2DFeature* feature2 = scenePoint2->features2D().at(j);

            bool found = false;
            for (size_t k = 0; k < scenePoint1->features2D().size(); ++k)
            {
                Point2DFeature* feature1 = scenePoint1->features2D().at(k);

                if (feature1 == feature2)
                {
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                scenePoint1->features2D().push_back(feature2);
                merge = true;
            }
        }

        for (size_t j = 0; j < scenePoint1->features2D().size(); ++j)
        {
            Point2DFeature* feature = scenePoint1->features2D().at(j);
            feature->feature3D() = scenePoint1;
        }

        if (merge)
        {
            ++nMergedScenePoints;
        }
    }

    ROS_INFO("Merged %d pairs of duplicate scene points.", nMergedScenePoints);

    // reconstruct scene points
    boost::unordered_set<Point3DFeaturePtr> scenePoints;

    for (size_t i = 0; i < m_sparseGraph->frameSetSegments().size(); ++i)
    {
        FrameSetSegment& segment = m_sparseGraph->frameSetSegment(i);

        for (size_t j = 0; j < segment.size(); ++j)
        {
            FrameSetPtr& frameSet = segment.at(j);

            for (size_t k = 0; k < frameSet->frames().size(); ++k)
            {
                FramePtr& frame = frameSet->frames().at(k);

                std::vector<Point2DFeaturePtr>& features = frame->features2D();

                for (size_t l = 0; l < features.size(); ++l)
                {
                    Point3DFeaturePtr& scenePoint = features.at(l)->feature3D();

                    scenePoints.insert(scenePoint);
                }
            }
        }
    }

    for (boost::unordered_set<Point3DFeaturePtr>::iterator it = scenePoints.begin();
             it != scenePoints.end(); ++it)
    {
        Point3DFeaturePtr scenePoint = *it;

        // reset 3D coordinates to those originally computed from stereo
        Frame* frame = scenePoint->features2D().front()->frame();
        FrameSet* frameSet = frame->frameSet();
        Eigen::Matrix4d pose = frameSet->systemPose()->toMatrix().inverse() * m_cameraSystem->getGlobalCameraPose(frame->cameraId());

        scenePoint->point() = transformPoint(pose, scenePoint->pointFromStereo());
    }

    reprojErrorStats(avgError, maxError, featureCount);

    ROS_INFO("Reprojection error after scene point reconstruction: avg = %.3f | max = %.3f | count = %lu",
             avgError, maxError, featureCount);

    m_sgv.visualize();
}

void
StereoSM::runBA(void)
{
    // run bundle adjustment
    ceres::Problem problem;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 1000;
    options.num_threads = 8;
    options.num_linear_solver_threads = 8;

    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > q_veh_cam;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > t_veh_cam;
    for (int i = 0; i < m_cameraSystem->cameraCount(); ++i)
    {
        Eigen::Matrix4d H_inv = invertHomogeneousTransform(m_cameraSystem->getGlobalCameraPose(i));

        q_veh_cam.push_back(Eigen::Quaterniond(H_inv.block<3,3>(0,0)));
        t_veh_cam.push_back(H_inv.block<3,1>(0,3));
    }

    for (size_t i = 0; i < m_sparseGraph->frameSetSegments().size(); ++i)
    {
        FrameSetSegment& segment = m_sparseGraph->frameSetSegment(i);

        for (size_t j = 0; j < segment.size(); ++j)
        {
            FrameSetPtr& frameSet = segment.at(j);

            for (size_t k = 0; k < frameSet->frames().size(); ++k)
            {
                FramePtr& frame = frameSet->frames().at(k);
                int cameraId = frame->cameraId();

                std::vector<Point2DFeaturePtr>& features = frame->features2D();

                for (size_t l = 0; l < features.size(); ++l)
                {
                    Point2DFeaturePtr& feature = features.at(l);
                    Point3DFeaturePtr& scenePoint = feature->feature3D();

                    ceres::LossFunction* lossFunction = new ceres::HuberLoss(0.0000055555);

                    ceres::CostFunction* costFunction =
                        CostFunctionFactory::instance()->generateCostFunction(q_veh_cam.at(cameraId),
                                                                              t_veh_cam.at(cameraId),
                                                                              feature->ray(),
                                                                              SYSTEM_POSE | SCENE_POINT);

                    problem.AddResidualBlock(costFunction, lossFunction,
                                             frameSet->systemPose()->rotationData(),
                                             frameSet->systemPose()->translationData(),
                                             scenePoint->pointData());
                }
            }

            ceres::LocalParameterization* quaternionParameterization =
                new EigenQuaternionParameterization;

            problem.SetParameterization(frameSet->systemPose()->rotationData(),
                                        quaternionParameterization);
        }
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;

    double avgError, maxError;
    size_t featureCount;

    reprojErrorStats(avgError, maxError, featureCount);

    ROS_INFO("Reprojection error after bundle adjustment: avg = %.3f | max = %.3f | count = %lu",
             avgError, maxError, featureCount);

    m_sgv.visualize();
}

void
StereoSM::reconstructScenePoint(Point3DFeaturePtr& scenePoint) const
{
    size_t nFeatures = scenePoint->features2D().size();

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > H_sys_cam;
    for (int i = 0; i < m_cameraSystem->cameraCount(); ++i)
    {
        Eigen::Matrix4d H_inv = invertHomogeneousTransform(m_cameraSystem->getGlobalCameraPose(i));

        H_sys_cam.push_back(H_inv);
    }

    Eigen::MatrixXd A(nFeatures * 3, 3 + nFeatures);
    A.setZero();

    Eigen::VectorXd b(nFeatures * 3);

    for (size_t i = 0; i < scenePoint->features2D().size(); ++i)
    {
        Point2DFeature* feature = scenePoint->features2D().at(i);
        Frame* frame = feature->frame();
        FrameSet* frameSet = frame->frameSet();

        Eigen::Matrix4d pose = H_sys_cam.at(frame->cameraId()) * frameSet->systemPose()->toMatrix();

        A.block<3,3>(i * 3, 0) = - pose.block<3,3>(0,0);
        A.block<3,1>(i * 3, 3 + i) = feature->ray();

        b.block<3,1>(i * 3, 0) = pose.block<3,1>(0,3);
    }

    Eigen::VectorXd x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    scenePoint->point() = x.block<3,1>(0,0);
}

void
StereoSM::reprojErrorStats(double& avgError, double& maxError,
                           size_t& featureCount) const
{
    size_t count = 0;
    double sumError = 0.0;
    maxError = 0.0;
    for (size_t i = 0; i < m_sparseGraph->frameSetSegments().size(); ++i)
    {
        const FrameSetSegment& segment = m_sparseGraph->frameSetSegment(i);

        for (size_t j = 0; j < segment.size(); ++j)
        {
            const FrameSetPtr& frameSet = segment.at(j);

            for (size_t k = 0; k < frameSet->frames().size(); ++k)
            {
                FramePtr& frame = frameSet->frames().at(k);
                int cameraId = frame->cameraId();

                for (size_t l = 0; l < frame->features2D().size(); ++l)
                {
                    const Point2DFeaturePtr& feature = frame->features2D().at(l);
                    const Point3DFeaturePtr& scenePoint = feature->feature3D();

                    Eigen::Matrix4d pose = m_cameraSystem->getGlobalCameraPose(cameraId).inverse() * frameSet->systemPose()->toMatrix();

                    Eigen::Vector3d P = transformPoint(pose, scenePoint->point());
                    Eigen::Vector2d p;
                    m_cameraSystem->getCamera(cameraId)->spaceToPlane(P, p);

                    double err = (p - Eigen::Vector2d(feature->keypoint().pt.x,
                                                      feature->keypoint().pt.y)).norm();

                    ++count;
                    sumError += err;

                    if (maxError < err)
                    {
                        maxError = err;
                    }
                }
            }
        }
    }

    if (count == 0)
    {
        avgError = 0.0;
        featureCount = 0;
    }
    else
    {
        avgError = sumError / static_cast<double>(count);
        featureCount = count;
    }
}

}
