#include "gcam_slam/GCamSLAM.h"

#include <boost/make_shared.hpp>
#include <boost/unordered_set.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "cauldron/EigenUtils.h"
#include "gcam_slam/GCamDWBA.h"
#include "gcam_vo/GCamVO.h"
#include "location_recognition/OrbLocationRecognition.h"
#include "pose_estimation/P3P.h"
#include "sparse_graph/SparseGraphViz.h"

namespace px
{

GCamSLAM::GCamSLAM(ros::NodeHandle& nh,
                   const CameraSystemConstPtr& cameraSystem)
 : m_nh(nh)
 , m_cameraSystem(cameraSystem)
 , m_vo(boost::make_shared<GCamVO>(boost::ref(cameraSystem), false, false))
 , m_sparseGraph(boost::make_shared<SparseGraph>())
 , k_minVOCorrespondenceCount(50)
 , k_minLoopCorrespondenceCount(15)
 , k_nLocationMatches(5)
 , k_sphericalErrorThresh(0.999976)
{
    m_sgv = boost::make_shared<SparseGraphViz>(boost::ref(nh), m_sparseGraph);
}

bool
GCamSLAM::init(const std::string& detectorType,
               const std::string& descriptorExtractorType,
               const std::string& descriptorMatcherType,
               const std::string& poseTopicName,
               const std::string& vocFilename)
{
    if (!m_vo->init(detectorType,
                    descriptorExtractorType,
                    descriptorMatcherType))
    {
        return false;
    }

    m_posePub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(poseTopicName, 2);

    cv::Size imageSize;
    imageSize.width = m_cameraSystem->getCamera(0)->imageWidth();
    imageSize.height = m_cameraSystem->getCamera(0)->imageHeight();

    m_locRec = boost::make_shared<OrbLocationRecognition>();
    m_locRec->setup(vocFilename);

    m_dwba = boost::make_shared<GCamDWBA>(boost::ref(m_nh), boost::ref(m_cameraSystem), 15, 50);

    return true;
}

bool
GCamSLAM::processFrames(const ros::Time& stamp,
                        const std::vector<cv::Mat>& imageVec,
                        const sensor_msgs::ImuConstPtr& imu)
{
    if (m_vo->isRunning())
    {
        return false;
    }

    // find loop closure edges
    boost::shared_ptr<boost::thread> loopClosureThread;
    std::vector<std::pair<LoopClosureEdge,LoopClosureEdge> > edges;
    if (m_frameSetKey)
    {
        loopClosureThread = boost::make_shared<boost::thread>(boost::bind(&GCamSLAM::findLoopClosures,
                                                                          this,
                                                                          boost::ref(edges)));
    }

    boost::shared_ptr<boost::thread> vizThread;
    if (!m_sparseGraph->frameSetSegment(0).empty())
    {
        vizThread = boost::make_shared<boost::thread>(&SparseGraphViz::visualize, m_sgv.get(), 0);
    }

    px::FrameSetPtr frameSet;
    bool success = m_vo->processFrames(stamp, imageVec, imu, frameSet);

    if (loopClosureThread)
    {
        loopClosureThread->join();
    }
    if (vizThread)
    {
        vizThread->join();
    }

    if (!success)
    {
        return false;
    }

    if (m_frameSetKey)
    {
        // add loop closure edges to graph

        int nStereoCams = m_cameraSystem->cameraCount() / 2;

        for (int i = 0; i < nStereoCams; ++i)
        {
            if (edges.at(i).first.inFrame() == 0)
            {
                continue;
            }

            FramePtr& frameQuery = m_frameSetKey->frames().at(i * 2);

            frameQuery->loopClosureEdges().push_back(edges.at(i).first);

            Frame* frameMatch = edges.at(i).first.inFrame();

            frameMatch->loopClosureEdges().push_back(edges.at(i).second);

            // merge pairs of scene points
            const std::vector<size_t>& inMatchIds = edges.at(i).first.inMatchIds();
            const std::vector<size_t>& outMatchIds = edges.at(i).first.outMatchIds();

            for (size_t j = 0; j < inMatchIds.size(); ++j)
            {
                Point3DFeaturePtr& scenePoint1 = frameQuery->features2D().at(outMatchIds.at(j))->feature3D();
                Point3DFeaturePtr& scenePoint2 = frameMatch->features2D().at(inMatchIds.at(j))->feature3D();

                if (scenePoint1 == scenePoint2)
                {
                    continue;
                }

                for (size_t k = 0; k < scenePoint2->features2D().size(); ++k)
                {
                    Point2DFeature* feature2 = scenePoint2->features2D().at(k);

                    bool found = false;
                    for (size_t l = 0; l < scenePoint1->features2D().size(); ++l)
                    {
                        Point2DFeature* feature1 = scenePoint1->features2D().at(l);

                        if (feature1 == feature2)
                        {
                            found = true;
                            break;
                        }
                    }

                    if (!found)
                    {
                        scenePoint1->features2D().push_back(feature2);
                    }
                }

                for (size_t k = 0; k < scenePoint1->features2D().size(); ++k)
                {
                    Point2DFeature* feature = scenePoint1->features2D().at(k);
                    feature->feature3D() = scenePoint1;
                }
            }
        }

        m_frameSetKey.reset();
    }

    m_dwba->optimize(frameSet);

    Eigen::Quaterniond q = frameSet->systemPose()->rotation().conjugate();
    Eigen::Vector3d t = q * (- frameSet->systemPose()->translation());

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "vmav";
    pose.pose.pose.orientation.w = q.w();
    pose.pose.pose.orientation.x = q.x();
    pose.pose.pose.orientation.y = q.y();
    pose.pose.pose.orientation.z = q.z();
    pose.pose.pose.position.x = t(0);
    pose.pose.pose.position.y = t(1);
    pose.pose.pose.position.z = t(2);

    m_posePub.publish(pose);

    if (m_vo->getCurrentCorrespondenceCount() < k_minVOCorrespondenceCount)
    {
        m_vo->keyCurrentFrameSet();

        m_sparseGraph->frameSetSegment(0).push_back(frameSet);

        m_frameSetKey = frameSet;
    }
}

bool
GCamSLAM::writePosesToTextFile(const std::string& filename, bool wrtWorld) const
{
    std::ofstream ofs(filename.c_str());
    if (!ofs.is_open())
    {
        return false;
    }

    ofs << std::fixed << std::setprecision(20);

    const std::vector<FrameSetPtr>& frameSets = m_sparseGraph->frameSetSegment(0);
    for (size_t i = 0; i < frameSets.size(); ++i)
    {
        const PosePtr& pose = frameSets.at(i)->systemPose();

        Eigen::Quaterniond q;
        Eigen::Vector3d t;
        if (wrtWorld)
        {
            q = pose->rotation().conjugate();
            t = q * (- pose->translation());
        }
        else
        {
            q = pose->rotation();
            t = pose->translation();
        }

        ofs << pose->timeStamp().toSec() << " "
            << q.x() << " "
            << q.y() << " "
            << q.z() << " "
            << q.w() << " "
            << t(0) << " "
            << t(1) << " "
            << t(2) << std::endl;
    }

    ofs.close();

    return true;

    return true;
}

bool
GCamSLAM::writeScenePointsToTextFile(const std::string& filename) const
{
    std::ofstream ofs(filename.c_str());
    if (!ofs.is_open())
    {
        return false;
    }

    ofs << std::fixed << std::setprecision(20);

    boost::unordered_set<Point3DFeature*> scenePoints;
    const std::vector<FrameSetPtr>& frameSets = m_sparseGraph->frameSetSegment(0);
    for (size_t i = 0; i < frameSets.size(); ++i)
    {
        const FrameSetPtr& frameSet = frameSets.at(i);

        for (size_t j = 0; j < frameSet->frames().size(); ++j)
        {
            const std::vector<Point2DFeaturePtr>& features = frameSet->frame(j)->features2D();

            for (size_t k = 0; k < features.size(); ++k)
            {
                const Point3DFeaturePtr& scenePoint = features.at(k)->feature3D();

                if (scenePoint->features2D().size() <= 2)
                {
                    continue;
                }

                scenePoints.insert(scenePoint.get());
            }
        }
    }

    for (boost::unordered_set<Point3DFeature*>::iterator it = scenePoints.begin();
         it != scenePoints.end(); ++it)
    {
        Point3DFeature* scenePoint = *it;

        ofs << scenePoint->point()(0) << " "
            << scenePoint->point()(1) << " "
            << scenePoint->point()(2) << " " << std::endl;
    }

    ofs.close();

    return true;

    return true;
}

void
GCamSLAM::findLoopClosures(std::vector<std::pair<LoopClosureEdge,LoopClosureEdge> >& edges)
{
    int nStereoCams = m_cameraSystem->cameraCount() / 2;

    edges.clear();
    edges.resize(nStereoCams);

    std::vector<boost::shared_ptr<boost::thread> > threads(nStereoCams);
    for (int i = 0; i < nStereoCams; ++i)
    {
        FramePtr& frameQuery = m_frameSetKey->frames().at(i * 2);

        threads.at(i) = boost::make_shared<boost::thread>(boost::bind(&GCamSLAM::findLoopClosuresHelper,
                                                                      this,
                                                                      frameQuery,
                                                                      boost::ref(edges.at(i))));
    }

    for (int i = 0; i < nStereoCams; ++i)
    {
        threads.at(i)->join();
    }
}

void
GCamSLAM::findLoopClosuresHelper(const FrameConstPtr& frameQuery,
                                 std::pair<LoopClosureEdge,LoopClosureEdge>& edge)
{
    cv::BFMatcher descriptorMatcher(cv::NORM_HAMMING, true);

    std::vector<FrameConstPtr> frameMatches;
    if (!m_locRec->detectSimilarLocations(frameQuery, k_nLocationMatches,
                                          frameMatches))
    {
        return;
    }

    cv::Mat dtorsQuery;
    getDescriptorMat(frameQuery, dtorsQuery);

    std::vector<cv::DMatch> matchesBest;
    Transform transformBest;
    FrameConstPtr frameBest;

    for (size_t i = 0; i < frameMatches.size(); ++i)
    {
        FrameConstPtr& frameMatch = frameMatches.at(i);

        // find 2D-3D correspondences
        cv::Mat dtorsMatch;
        getDescriptorMat(frameMatch, dtorsMatch);

        std::vector<cv::DMatch> rawMatches;
        descriptorMatcher.match(dtorsMatch, dtorsQuery, rawMatches);

        if (rawMatches.size() < k_minLoopCorrespondenceCount)
        {
            continue;
        }

        // find camera pose from P3P RANSAC
        Eigen::Matrix4d systemPose;
        std::vector<cv::DMatch> matches;
        solveP3PRansac(frameMatch, frameQuery, rawMatches, systemPose, matches);

        int nInliers = matches.size();

        if (nInliers < k_minLoopCorrespondenceCount)
        {
            continue;
        }

        if (nInliers > matchesBest.size())
        {
            frameBest = frameMatch;
            matchesBest = matches;

            // compute loop closure constraint
            Eigen::Matrix4d H_01 = frameBest->frameSet()->systemPose()->toMatrix() *
                                   invertHomogeneousTransform(systemPose);

            transformBest.rotation() = Eigen::Quaterniond(H_01.block<3,3>(0,0));
            transformBest.translation() = H_01.block<3,1>(0,3);
        }
    }

    if (frameBest)
    {
        LoopClosureEdge& outEdge = edge.first;
        outEdge.inFrame() = const_cast<Frame*>(frameBest.get());
        outEdge.measurement() = transformBest;

        outEdge.inMatchIds().resize(matchesBest.size());
        outEdge.outMatchIds().resize(matchesBest.size());
        for (size_t i = 0; i < matchesBest.size(); ++i)
        {
            const cv::DMatch& match = matchesBest.at(i);

            outEdge.inMatchIds().at(i) = match.queryIdx;
            outEdge.outMatchIds().at(i) = match.trainIdx;
        }

        LoopClosureEdge& inEdge = edge.second;
        inEdge.inFrame() = const_cast<Frame*>(frameQuery.get());
        inEdge.measurement() = Transform(invertHomogeneousTransform(transformBest.toMatrix()));

        inEdge.inMatchIds().resize(matchesBest.size());
        inEdge.outMatchIds().resize(matchesBest.size());
        for (size_t i = 0; i < matchesBest.size(); ++i)
        {
            const cv::DMatch& match = matchesBest.at(i);

            inEdge.inMatchIds().at(i) = match.trainIdx;
            inEdge.outMatchIds().at(i) = match.queryIdx;
        }
    }
}

void
GCamSLAM::getDescriptorMat(const FrameConstPtr& frame, cv::Mat& dmat) const
{
    const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

    if (features2D.empty())
    {
        dmat = cv::Mat();

        return;
    }

    dmat = cv::Mat(features2D.size(), features2D.front()->descriptor().cols,
                   features2D.front()->descriptor().type());

    for (size_t i = 0; i < features2D.size(); ++i)
    {
        const Point2DFeatureConstPtr& feature2D = features2D.at(i);

        feature2D->descriptor().copyTo(dmat.row(i));
    }
}

void
GCamSLAM::solveP3PRansac(const FrameConstPtr& frame1,
                         const FrameConstPtr& frame2,
                         const std::vector<cv::DMatch>& matches,
                         Eigen::Matrix4d& H,
                         std::vector<cv::DMatch>& inliers) const
{
    inliers.clear();

    double p = 0.99; // probability that at least one set of random samples does not contain an outlier
    double v = 0.6; // probability of observing an outlier

    double u = 1.0 - v;
    int N = static_cast<int>(log(1.0 - p) / log(1.0 - u * u * u) + 0.5);

    std::vector<size_t> indices;
    for (size_t i = 0; i < matches.size(); ++i)
    {
        indices.push_back(i);
    }

    const std::vector<Point2DFeaturePtr>& features1 = frame1->features2D();
    const std::vector<Point2DFeaturePtr>& features2 = frame2->features2D();

    // run RANSAC to find best H
    Eigen::Matrix4d H_best;
    std::vector<size_t> inlierIds_best;
    for (int i = 0; i < N; ++i)
    {
        std::random_shuffle(indices.begin(), indices.end());

        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > rays(3);
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > worldPoints(3);
        for (int j = 0; j < 3; ++j)
        {
            const cv::DMatch& match = matches.at(indices.at(j));

            worldPoints.at(j) = features1.at(match.queryIdx)->feature3D()->point();
            rays.at(j) = features2.at(match.trainIdx)->ray();
        }

        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > solutions;
        if (!solveP3P(rays, worldPoints, solutions))
        {
            continue;
        }

        for (size_t j = 0; j < solutions.size(); ++j)
        {
            Eigen::Matrix4d H_inv = invertHomogeneousTransform(solutions.at(j));

            std::vector<size_t> inliersIds;
            for (size_t k = 0; k < matches.size(); ++k)
            {
                const cv::DMatch& match = matches.at(k);

                Eigen::Vector3d P1 = features1.at(match.queryIdx)->feature3D()->point();

                Eigen::Vector3d P2 = transformPoint(H_inv, P1);

                const Point2DFeatureConstPtr& f2 = features2.at(match.trainIdx);

                double err = fabs(P2.normalized().dot(f2->ray()));
                if (err < k_sphericalErrorThresh)
                {
                    continue;
                }

                inliersIds.push_back(k);
            }

            if (inliersIds.size() > inlierIds_best.size())
            {
                H_best = H_inv;
                inlierIds_best = inliersIds;
            }
        }
    }

    for (size_t i = 0; i < inlierIds_best.size(); ++i)
    {
        inliers.push_back(matches.at(inlierIds_best.at(i)));
    }

    H = m_cameraSystem->getGlobalCameraPose(frame2->cameraId()) * H_best;
}

}
