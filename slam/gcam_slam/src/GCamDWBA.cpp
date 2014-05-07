#include "gcam_slam/GCamDWBA.h"

#include <boost/unordered_set.hpp>
#include <visualization_msgs/Marker.h>

#include "camera_models/CostFunctionFactory.h"
#include "cauldron/EigenQuaternionParameterization.h"
#include "cauldron/EigenUtils.h"
#include "ceres/ceres.h"
#include "PoseGraphError.h"

namespace px
{

class VerticalError
{
public:
    VerticalError(double r_imu, double p_imu)
     : m_r_imu(r_imu)
     , m_p_imu(p_imu)
    {

    }

    template <typename T>
    bool operator()(const T* const q_s_coeffs, T* residuals) const
    {
        T q_s_coeffs_ceres[4] = {q_s_coeffs[3], -q_s_coeffs[0], -q_s_coeffs[1], -q_s_coeffs[2]};

        Eigen::Matrix<T,3,3> R;
        ceres::QuaternionToRotation(q_s_coeffs_ceres, ceres::ColumnMajorAdapter3x3(R.data()));

        T r, p, y;
        mat2RPY(R, r, p, y);

        residuals[0] = r - T(m_r_imu);
        residuals[1] = p - T(m_p_imu);

        return true;
    }

private:
    double m_r_imu;
    double m_p_imu;
};

GCamDWBA::GCamDWBA(ros::NodeHandle& nh,
                   const CameraSystemConstPtr& cameraSystem,
                   int M1, int M2)
 : k_M1(M1)
 , k_M2(M2)
{
    m_mapVizPub = nh.advertise<visualization_msgs::Marker>("map_marker", 1);
    m_poseVizPub = nh.advertise<visualization_msgs::Marker>("pose_marker", 1);

    for (int i = 0; i < cameraSystem->cameraCount(); ++i)
    {
        Eigen::Matrix4d H_cam_sys = cameraSystem->getGlobalCameraPose(i);

        m_H_cam_sys.push_back(H_cam_sys);

        Eigen::Matrix4d H_sys_cam = invertHomogeneousTransform(H_cam_sys);

        m_T_sys_cam.push_back(Transform(H_sys_cam));
    }
}

bool
sortFunction(std::pair<size_t, FrameSet*> x, std::pair<size_t, FrameSet*> y)
{
    return x.first > y.first;
}

void
GCamDWBA::optimize(FrameSetPtr& refFrameSet)
{
    // construct double windows
    int N = k_M1 + k_M2;

    boost::unordered_set<FrameSet*> W1;
    boost::unordered_set<FrameSet*> W2;

    std::vector<std::pair<size_t, FrameSet*> > queue;
    queue.push_back(std::make_pair(0, refFrameSet.get()));

    while (!queue.empty())
    {
        std::sort(queue.begin(), queue.end(), sortFunction);

        boost::unordered_map<FrameSet*,int> neighborMap;

        for (std::vector<std::pair<size_t, FrameSet*> >::iterator it = queue.begin();
                it != queue.end(); ++it)
        {
            FrameSet* frameSet = it->second;

            if (frameSet->prevFrameSet() &&
                W1.find(frameSet->prevFrameSet()) == W1.end() &&
                W2.find(frameSet->prevFrameSet()) == W2.end())
            {
                size_t weight = 0;
                for (size_t i = 0; i < frameSet->frames().size(); i += 2)
                {
                    const std::vector<Point2DFeaturePtr>& features = frameSet->frame(i)->features2D();
                    for (size_t j = 0; j < features.size(); ++j)
                    {
                        if (!features.at(j)->prevMatches().empty())
                        {
                            ++weight;
                        }
                    }
                }

                neighborMap.insert(std::make_pair(frameSet->prevFrameSet(), weight));
            }

            if (frameSet->nextFrameSet() &&
                W1.find(frameSet->nextFrameSet()) == W1.end() &&
                W2.find(frameSet->nextFrameSet()) == W2.end())
            {
                size_t weight = 0;
                for (size_t i = 0; i < frameSet->frames().size(); i += 2)
                {
                    const std::vector<Point2DFeaturePtr>& features = frameSet->frame(i)->features2D();
                    for (size_t j = 0; j < features.size(); ++j)
                    {
                        if (!features.at(j)->nextMatches().empty())
                        {
                            ++weight;
                        }
                    }
                }

                neighborMap.insert(std::make_pair(frameSet->nextFrameSet(), weight));
            }

            for (size_t i = 0; i < frameSet->frames().size(); i += 2)
            {
                const std::vector<LoopClosureEdge>& edges = frameSet->frame(i)->loopClosureEdges();

                for (size_t j = 0; j < edges.size(); ++j)
                {
                    const LoopClosureEdge& edge = edges.at(j);

                    if (W1.find(edge.inFrame()->frameSet()) != W1.end() ||
                        W2.find(edge.inFrame()->frameSet()) != W2.end())
                    {
                        continue;
                    }

                    size_t weight = edge.inMatchIds().size();

                    boost::unordered_map<FrameSet*,int>::iterator itNeighbor = neighborMap.find(edge.inFrame()->frameSet());
                    if (itNeighbor == neighborMap.end())
                    {
                        neighborMap.insert(std::make_pair(edge.inFrame()->frameSet(), weight));
                    }
                    else
                    {
                        itNeighbor->second += weight;
                    }
                }
            }

            if (W1.size() < k_M1)
            {
                W1.insert(frameSet);
            }
            else if (W2.size() < k_M2)
            {
                W2.insert(frameSet);
            }
            else
            {
                break;
            }
        }

        if (W2.size() < k_M2)
        {
            queue.clear();

            for (boost::unordered_map<FrameSet*,int>::iterator it = neighborMap.begin();
                 it != neighborMap.end(); ++it)
            {
                queue.push_back(std::make_pair(it->second, it->first));
            }
        }
        else
        {
            break;
        }
    }

    // build optimization problem

    ceres::Problem problem;

    // find all scene points visible from inner window
    boost::unordered_set<Point3DFeature*> scenePoints;
    for (boost::unordered_set<FrameSet*>::iterator it = W1.begin();
            it != W1.end(); ++it)
    {
        FrameSet* frameSet = *it;

        for (size_t i = 0; i < frameSet->frames().size(); i += 2)
        {
            std::vector<Point2DFeaturePtr>& features = frameSet->frames().at(i)->features2D();

            for (size_t j = 0; j < features.size(); ++j)
            {
                const Point2DFeaturePtr& feature = features.at(j);
                Point3DFeature* scenePoint = feature->feature3D().get();

                if (scenePoint->features2D().size() <= 2)
                {
                    continue;
                }

                if (scenePoints.find(scenePoint) == scenePoints.end())
                {
                    scenePoints.insert(scenePoint);
                }
            }
        }
    }

    if (scenePoints.empty())
    {
        return;
    }

    boost::unordered_set<FrameSet*> W;
    W.insert(W1.begin(), W1.end());
    W.insert(W2.begin(), W2.end());

    // create residuals corresponding to point-pose constraints
    for (boost::unordered_set<Point3DFeature*>::iterator it = scenePoints.begin();
            it != scenePoints.end(); ++it)
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

            if (W.find(frameSet) == W.end())
            {
                continue;
            }

            Point2DFeature* feature2 = feature1->match();
            int cameraId2 = feature2->frame()->cameraId();

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
        }
    }

    double wPosePose = 0.001;
    boost::unordered_map<FrameSet*, FrameSet*> posePoseMap;

    // create residuals corresponding to pose-pose constraints
    for (boost::unordered_set<FrameSet*>::iterator it = W2.begin();
             it != W2.end(); ++it)
    {
        FrameSet* frameSet = *it;

        if (frameSet->prevFrameSet() &&
            W.find(frameSet->prevFrameSet()) != W.end())
        {
            ceres::CostFunction* costFunction =
                new ceres::AutoDiffCostFunction<PoseGraphError, 6, 4, 3, 4, 3>(
                    new PoseGraphError(frameSet->prevTransformMeasurement()));

            ceres::LossFunction* lossFunction = new ceres::ScaledLoss(0, wPosePose, ceres::TAKE_OWNERSHIP);

            problem.AddResidualBlock(costFunction, lossFunction,
                                     frameSet->systemPose()->rotationData(),
                                     frameSet->systemPose()->translationData(),
                                     frameSet->prevFrameSet()->systemPose()->rotationData(),
                                     frameSet->prevFrameSet()->systemPose()->translationData());
        }

        if (frameSet->nextFrameSet() &&
            W.find(frameSet->nextFrameSet()) != W.end())
        {
            ceres::CostFunction* costFunction =
                new ceres::AutoDiffCostFunction<PoseGraphError, 6, 4, 3, 4, 3>(
                    new PoseGraphError(frameSet->nextTransformMeasurement()));

            ceres::LossFunction* lossFunction = new ceres::ScaledLoss(0, wPosePose, ceres::TAKE_OWNERSHIP);

            problem.AddResidualBlock(costFunction, lossFunction,
                                     frameSet->systemPose()->rotationData(),
                                     frameSet->systemPose()->translationData(),
                                     frameSet->nextFrameSet()->systemPose()->rotationData(),
                                     frameSet->nextFrameSet()->systemPose()->translationData());
        }

        for (size_t i = 0; i < frameSet->frames().size(); i += 2)
        {
            const std::vector<LoopClosureEdge>& edges = frameSet->frame(i)->loopClosureEdges();

           for (size_t j = 0; j < edges.size(); ++j)
           {
               const LoopClosureEdge& edge = edges.at(j);

               if (W.find(edge.inFrame()->frameSet()) == W.end())
               {
                   continue;
               }

               ceres::CostFunction* costFunction =
                   new ceres::AutoDiffCostFunction<PoseGraphError, 6, 4, 3, 4, 3>(
                       new PoseGraphError(edge.measurement()));

               ceres::LossFunction* lossFunction = new ceres::ScaledLoss(new ceres::CauchyLoss(0.01), wPosePose, ceres::TAKE_OWNERSHIP);

               problem.AddResidualBlock(costFunction, lossFunction,
                                        frameSet->systemPose()->rotationData(),
                                        frameSet->systemPose()->translationData(),
                                        edge.inFrame()->frameSet()->systemPose()->rotationData(),
                                        edge.inFrame()->frameSet()->systemPose()->translationData());
           }
        }
    }

    if (refFrameSet->prevFrameSet() &&
        W.find(refFrameSet->prevFrameSet()) != W.end())
    {
        problem.SetParameterBlockConstant(refFrameSet->prevFrameSet()->systemPose()->rotationData());
        problem.SetParameterBlockConstant(refFrameSet->prevFrameSet()->systemPose()->translationData());
    }

    // add residuals corresponding to vertical direction
    for (boost::unordered_set<FrameSet*>::iterator it = W.begin();
             it != W.end(); ++it)
    {
        FrameSet* frameSet = *it;

        const sensor_msgs::ImuConstPtr& imu = frameSet->imuMeasurement();

        Eigen::Matrix3d R_imu = Eigen::Quaterniond(imu->orientation.w,
                                                   imu->orientation.x,
                                                   imu->orientation.y,
                                                   imu->orientation.z).toRotationMatrix();

        double r_imu, p_imu, y_imu;
        mat2RPY(R_imu, r_imu, p_imu, y_imu);

        Eigen::Matrix3d R_sys = invertHomogeneousTransform(frameSet->systemPose()->toMatrix()).block<3,3>(0,0);

        double r_sys, p_sys, y_sys;
        mat2RPY(R_sys, r_sys, p_sys, y_sys);

        ceres::CostFunction* costFunction =
            new ceres::AutoDiffCostFunction<VerticalError, 2, 4>(
                new VerticalError(r_imu, p_imu));

        problem.AddResidualBlock(costFunction, 0,
                                 frameSet->systemPose()->rotationData());

        ceres::LocalParameterization* quaternionParameterization =
            new EigenQuaternionParameterization;

        problem.SetParameterization(frameSet->systemPose()->rotationData(),
                                    quaternionParameterization);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 3;
    options.num_threads = 8;
    options.num_linear_solver_threads = 8;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Update 3D coordinates of scene points that are only observed in a single frame set
    // since these points are not optimized in bundle adjustment.
    for (boost::unordered_set<FrameSet*>::iterator it = W.begin();
            it != W.end(); ++it)
    {
        FrameSet* frameSet = *it;

        for (size_t i = 0; i < frameSet->frames().size(); i += 2)
        {
            Eigen::Matrix4d H_cam = invertHomogeneousTransform(frameSet->systemPose()->toMatrix()) *
                                    m_H_cam_sys.at(frameSet->frames().at(i)->cameraId());

            const std::vector<Point2DFeaturePtr>& features = frameSet->frames().at(i)->features2D();

            for (size_t j = 0; j < features.size(); ++j)
            {
                const Point2DFeaturePtr& feature = features.at(j);

                if (feature->prevMatches().empty() && feature->nextMatches().empty())
                {
                    const Point3DFeaturePtr& scenePoint = feature->feature3D();

                    scenePoint->point() = transformPoint(H_cam, scenePoint->pointFromStereo());
                }
            }
        }
    }

    // Update transform measurements for pose-pose constraints in inner window
    for (boost::unordered_set<FrameSet*>::iterator it = W1.begin();
             it != W1.end(); ++it)
    {
        FrameSet* frameSet = *it;

        if (frameSet->prevFrameSet() &&
            W1.find(frameSet->prevFrameSet()) != W1.end())
        {
            frameSet->prevTransformMeasurement() = Transform(frameSet->prevFrameSet()->systemPose()->toMatrix() *
                                                             invertHomogeneousTransform(frameSet->systemPose()->toMatrix()));
        }

        if (frameSet->nextFrameSet() &&
            W1.find(frameSet->nextFrameSet()) != W1.end())
        {
            frameSet->nextTransformMeasurement() = Transform(frameSet->nextFrameSet()->systemPose()->toMatrix() *
                                                             invertHomogeneousTransform(frameSet->systemPose()->toMatrix()));
        }
    }

    // visualize
    // construct marker msgs
    visualization_msgs::Marker mapMarker;

    mapMarker.header.frame_id = "vmav";
    mapMarker.header.stamp = ros::Time::now();

    mapMarker.id = 1;

    mapMarker.type = visualization_msgs::Marker::SPHERE_LIST;

    mapMarker.action = visualization_msgs::Marker::ADD;

    mapMarker.pose.position.x = 0.0;
    mapMarker.pose.position.y = 0.0;
    mapMarker.pose.position.z = 0.0;
    mapMarker.pose.orientation.x = 0.0;
    mapMarker.pose.orientation.y = 0.0;
    mapMarker.pose.orientation.z = 0.0;
    mapMarker.pose.orientation.w = 1.0;

    mapMarker.scale.x = 0.05;
    mapMarker.scale.y = 0.0;
    mapMarker.scale.z = 0.0;

    mapMarker.color.r = 1.0f;
    mapMarker.color.g = 1.0f;
    mapMarker.color.b = 0.0f;
    mapMarker.color.a = 1.0f;

    mapMarker.lifetime = ros::Duration();

    visualization_msgs::Marker poseW1Marker;

    poseW1Marker.header.frame_id = "vmav";
    poseW1Marker.header.stamp = ros::Time::now();

    poseW1Marker.id = 1;

    poseW1Marker.type = visualization_msgs::Marker::SPHERE_LIST;

    poseW1Marker.action = visualization_msgs::Marker::ADD;

    poseW1Marker.pose.position.x = 0.0;
    poseW1Marker.pose.position.y = 0.0;
    poseW1Marker.pose.position.z = 0.0;
    poseW1Marker.pose.orientation.x = 0.0;
    poseW1Marker.pose.orientation.y = 0.0;
    poseW1Marker.pose.orientation.z = 0.0;
    poseW1Marker.pose.orientation.w = 1.0;

    poseW1Marker.scale.x = 0.1;
    poseW1Marker.scale.y = 0.0;
    poseW1Marker.scale.z = 0.0;

    poseW1Marker.color.r = 1.0f;
    poseW1Marker.color.g = 0.0f;
    poseW1Marker.color.b = 0.0f;
    poseW1Marker.color.a = 1.0f;

    poseW1Marker.lifetime = ros::Duration();

    visualization_msgs::Marker poseW2Marker;

    poseW2Marker.header.frame_id = "vmav";
    poseW2Marker.header.stamp = ros::Time::now();

    poseW2Marker.id = 2;

    poseW2Marker.type = visualization_msgs::Marker::SPHERE_LIST;

    poseW2Marker.action = visualization_msgs::Marker::ADD;

    poseW2Marker.pose.position.x = 0.0;
    poseW2Marker.pose.position.y = 0.0;
    poseW2Marker.pose.position.z = 0.0;
    poseW2Marker.pose.orientation.x = 0.0;
    poseW2Marker.pose.orientation.y = 0.0;
    poseW2Marker.pose.orientation.z = 0.0;
    poseW2Marker.pose.orientation.w = 1.0;

    poseW2Marker.scale.x = 0.1;
    poseW2Marker.scale.y = 0.0;
    poseW2Marker.scale.z = 0.0;

    poseW2Marker.color.r = 0.0f;
    poseW2Marker.color.g = 0.0f;
    poseW2Marker.color.b = 1.0f;
    poseW2Marker.color.a = 1.0f;

    poseW2Marker.lifetime = ros::Duration();

    for (boost::unordered_set<Point3DFeature*>::iterator it = scenePoints.begin();
                it != scenePoints.end(); ++it)
    {
        const Eigen::Vector3d& P = (*it)->point();

        geometry_msgs::Point p;
        p.x = P(0);
        p.y = P(1);
        p.z = P(2);

        mapMarker.points.push_back(p);
    }

    for (boost::unordered_set<FrameSet*>::iterator it = W1.begin();
             it != W1.end(); ++it)
    {
        FrameSet* frameSet = *it;

        Eigen::Matrix4d H = invertHomogeneousTransform(frameSet->systemPose()->toMatrix());

        geometry_msgs::Point p;
        p.x = H(0,3);
        p.y = H(1,3);
        p.z = H(2,3);

        poseW1Marker.points.push_back(p);
    }

    for (boost::unordered_set<FrameSet*>::iterator it = W2.begin();
             it != W2.end(); ++it)
    {
        FrameSet* frameSet = *it;

        Eigen::Matrix4d H = invertHomogeneousTransform(frameSet->systemPose()->toMatrix());

        geometry_msgs::Point p;
        p.x = H(0,3);
        p.y = H(1,3);
        p.z = H(2,3);

        poseW2Marker.points.push_back(p);
    }

    m_mapVizPub.publish(mapMarker);
    m_poseVizPub.publish(poseW1Marker);
    m_poseVizPub.publish(poseW2Marker);
}

}
