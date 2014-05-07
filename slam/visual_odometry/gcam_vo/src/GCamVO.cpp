#include "gcam_vo/GCamVO.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <ros/ros.h>

#include "cauldron/EigenUtils.h"
#include "gcam/GCamIMU.h"
#include "gcam_vo/GCamLocalBA.h"
#include "pose_estimation/gP3P.h"

namespace px
{

GCamVO::GCamVO(const CameraSystemConstPtr& cameraSystem,
               bool preUndistort, bool useLocalBA)
 : k_epipolarThresh(0.00005)
 , k_maxDistanceRatio(0.7f)
 , k_maxStereoRange(20.0)
 , k_preUndistort(preUndistort)
 , k_sphericalErrorThresh(0.999976)
 , m_cameraSystem(cameraSystem)
 , m_nCorrespondences(0)
 , m_debug(false)
{
    int nCameras = cameraSystem->cameraCount();

    m_metadataVec.resize(nCameras);

    for (int i = 0; i < nCameras; ++i)
    {
        CameraMetadata& metadata = m_metadataVec.at(i);

        metadata.camera = cameraSystem->getCamera(i);

        if (k_preUndistort)
        {
            metadata.camera->initUndistortMap(metadata.undistortMapX,
                                              metadata.undistortMapY);
            metadata.camera->setZeroDistortion();
        }
    }

    int nStereoCameras = nCameras / 2;
    for (int i = 0; i < nStereoCameras; ++i)
    {
        Eigen::Matrix4d H_stereo = invertHomogeneousTransform(cameraSystem->getGlobalCameraPose(i * 2 + 1)) *
                                   cameraSystem->getGlobalCameraPose(i * 2);

        m_H_stereo.push_back(H_stereo);

        Eigen::Matrix3d R = H_stereo.block<3,3>(0,0);
        Eigen::Vector3d t = H_stereo.block<3,1>(0,3);

        Eigen::Matrix3d E = skew(t) * R;

        m_E.push_back(E);
    }

    m_gcam = boost::make_shared<GCamIMU>(cameraSystem);

    if (useLocalBA)
    {
        m_lba = boost::make_shared<GCamLocalBA>(cameraSystem);
    }
}

bool
GCamVO::init(const std::string& detectorType,
             const std::string& descriptorExtractorType,
             const std::string& descriptorMatcherType)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    cv::initModule_nonfree();

    m_featureDetector = cv::FeatureDetector::create(detectorType);
    if (!m_featureDetector)
    {
        ROS_ERROR("Failed to create feature detector of type: %s",
                  detectorType.c_str());
        return false;
    }

    m_descriptorExtractor =
        cv::DescriptorExtractor::create(descriptorExtractorType);
    if (!m_descriptorExtractor)
    {
        ROS_ERROR("Failed to create descriptor extractor of type: %s",
                  descriptorExtractorType.c_str());
        return false;
    }

    m_descriptorMatcher = cv::DescriptorMatcher::create(descriptorMatcherType);
    if (!m_descriptorMatcher)
    {
        ROS_ERROR("Failed to create descriptor matcher of type: %s",
                  descriptorMatcherType.c_str());
        return false;
    }

    return true;
}

bool
GCamVO::processFrames(const ros::Time& stamp,
                      const std::vector<cv::Mat>& imageVec,
                      const sensor_msgs::ImuConstPtr& imu,
                      FrameSetPtr& frameSet)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    ros::Time tsStart = ros::Time::now();

    int nCameras = m_cameraSystem->cameraCount();
    int nStereoCameras = nCameras / 2;

    if (imageVec.size() != nCameras)
    {
        ROS_WARN("# elements in imageVec does not match # cameras.");
        return false;
    }

    for (int i = 0; i < m_cameraSystem->cameraCount(); ++i)
    {
        imageVec.at(i).copyTo(m_metadataVec.at(i).rawImage);
    }

    ros::Time tsStartProcMono = ros::Time::now();

    std::vector<boost::shared_ptr<boost::thread> > threadVec(nCameras);
    for (int i = 0; i < nCameras; ++i)
    {
        threadVec.at(i) = boost::make_shared<boost::thread>(boost::bind(&GCamVO::processFrame,
                                                                        this,
                                                                        boost::ref(m_metadataVec.at(i))));
    }
    for (int i = 0; i < nCameras; ++i)
    {
        threadVec.at(i)->join();
    }

    if (m_debug)
    {
        ROS_INFO("### Frame processing took %.3f s. ###", (ros::Time::now() - tsStartProcMono).toSec());
    }

    frameSet = boost::make_shared<FrameSet>();
    if (m_frameSetPrev)
    {
        frameSet->seq() = m_frameSetPrev->seq() + 1;
    }
    else
    {
        frameSet->seq() = 0;
    }

    frameSet->imuMeasurement() = imu;
    for (int i = 0; i < nCameras; ++i)
    {
        FramePtr frame = boost::make_shared<Frame>();
        frame->cameraId() = i;
        frame->frameSet() = frameSet.get();
        frameSet->frames().push_back(frame);

        m_metadataVec.at(i).frame = frame;
    }

    // --- Find feature correspondences between the stereo images. ---

    ros::Time tsStartProcStereo = ros::Time::now();

    std::vector<std::vector<cv::DMatch> > stereoMatches(nStereoCameras);
    for (int i = 0; i < nStereoCameras; ++i)
    {
        threadVec.at(i) = boost::make_shared<boost::thread>(boost::bind(&GCamVO::processStereoFrame,
                                                                        this,
                                                                        boost::ref(m_metadataVec.at(i * 2)),
                                                                        boost::ref(m_metadataVec.at(i * 2 + 1))));
    }
    for (int i = 0; i < nStereoCameras; ++i)
    {
        threadVec.at(i)->join();
    }

    if (m_debug)
    {
        ROS_INFO("### Stereo frame processing took %.3f s. ###", (ros::Time::now() - tsStartProcStereo).toSec());
    }

    // Avoid copying image data as image data takes up significant memory.
    if (m_debug)
    {
        for (int i = 0; i < nCameras; ++i)
        {
            m_metadataVec.at(i).procImage.copyTo(frameSet->frame(i)->image());
        }
    }

    bool replaceCurrentFrameSet = false;
    if (m_frameSetCurr)
    {
        // Current frame set is not keyed. Remove all references to
        // the current frame set.
        for (size_t i = 0; i < m_frameSetPrev->frames().size(); ++i)
        {
            std::vector<Point2DFeaturePtr>& features = m_frameSetPrev->frame(i)->features2D();

            for (size_t j = 0; j < features.size(); ++j)
            {
                Point2DFeaturePtr& feature = features.at(j);

                if (!feature->nextMatches().empty())
                {
                    feature->bestNextMatchId() = -1;
                    feature->nextMatches().clear();
                }
            }
        }
        m_frameSetPrev->nextFrameSet() = 0;

        for (size_t i = 0; i < m_frameSetCurr->frames().size(); ++i)
        {
            std::vector<Point2DFeaturePtr>& features = m_frameSetCurr->frame(i)->features2D();

            for (size_t j = 0; j < features.size(); ++j)
            {
                Point2DFeature* feature = features.at(j).get();

                if (feature->prevMatches().empty())
                {
                    continue;
                }

                Point3DFeaturePtr& scenePoint = feature->feature3D();

                std::vector<Point2DFeature*>::reverse_iterator it = scenePoint->features2D().rbegin();
                while (it != scenePoint->features2D().rend())
                {
                    if (*it == feature)
                    {
                        scenePoint->features2D().erase(--it.base());
                        break;
                    }

                    ++it;
                }
            }
        }
        m_frameSetCurr->prevFrameSet() = 0;

        replaceCurrentFrameSet = true;
    }

    if (!m_frameSetPrev)
    {
        Eigen::Matrix3d R = Eigen::Quaterniond(imu->orientation.w,
                                               imu->orientation.x,
                                               imu->orientation.y,
                                               imu->orientation.z).toRotationMatrix();

        double r, p, y;
        mat2RPY(R, r, p, y);

        R = RPY2mat(r, p, 0.0);

        Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
        H.block<3,3>(0,0) = R.transpose();

        PosePtr pose = boost::make_shared<Pose>(H);
        pose->timeStamp() = stamp;

        frameSet->systemPose() = pose;
    }
    else
    {
        ros::Time tsStartPoseEst = ros::Time::now();

        // Find feature correspondences between previous and current frames.
        std::vector<std::vector<cv::DMatch> > rawMatches;
        matchCorrespondences(m_frameSetPrev, frameSet, rawMatches);

        size_t nRawMatches = 0;
        for (size_t i = 0; i < rawMatches.size(); ++i)
        {
            nRawMatches += rawMatches.at(i).size();
        }
        if (nRawMatches <= 4)
        {
            ROS_WARN("# raw matches <= 4");
            return false;
        }

        Eigen::Matrix4d systemPose;
        std::vector<std::vector<cv::DMatch> > matches;

        if ((frameSet->systemPose()->timeStamp() -
             m_frameSetPrev->systemPose()->timeStamp()).toSec() > 0.3)
        {
            if (m_debug)
            {
                ROS_INFO("Estimating pose with 3-pt RANSAC...");
            }

            // Estimate pose from 3-pt RANSAC
            solve3PRansac(m_frameSetPrev, frameSet, rawMatches, systemPose, matches);
        }
        else
        {
            if (m_debug)
            {
                ROS_INFO("Estimating pose with P3P RANSAC...");
            }

            // Estimate pose from P3P RANSAC.
            solveP3PRansac(m_frameSetPrev, frameSet, rawMatches, systemPose, matches);
        }

        m_nCorrespondences = 0;
        for (size_t i = 0; i < matches.size(); ++i)
        {
            m_nCorrespondences += matches.at(i).size();
        }

        if (m_nCorrespondences < 10)
        {
            ROS_WARN("# correspondences (%lu) is too low for reliable motion estimation.",
                     m_nCorrespondences);
        }

        PosePtr pose = boost::make_shared<Pose>(systemPose);
        pose->timeStamp() = stamp;

        frameSet->systemPose() = pose;

        for (int i = 0; i < nStereoCameras; ++i)
        {
            int cameraId1 = i * 2;
            int cameraId2 = i * 2 + 1;

            const std::vector<Point2DFeaturePtr>& features1Prev = m_frameSetPrev->frame(cameraId1)->features2D();
            const std::vector<Point2DFeaturePtr>& features2Prev = m_frameSetPrev->frame(cameraId2)->features2D();
            const std::vector<Point2DFeaturePtr>& features1Curr = frameSet->frame(cameraId1)->features2D();
            const std::vector<Point2DFeaturePtr>& features2Curr = frameSet->frame(cameraId2)->features2D();

            for (size_t j = 0; j < matches.at(i).size(); ++j)
            {
                const cv::DMatch& match = matches.at(i).at(j);

                const Point2DFeaturePtr& feature1Prev = features1Prev.at(match.queryIdx);
                const Point2DFeaturePtr& feature2Prev = features2Prev.at(match.queryIdx);
                const Point2DFeaturePtr& feature1Curr = features1Curr.at(match.trainIdx);
                const Point2DFeaturePtr& feature2Curr = features2Curr.at(match.trainIdx);

                feature1Prev->nextMatches().push_back(feature1Curr.get());
                feature1Prev->bestNextMatchId() = 0;

                feature1Curr->prevMatches().push_back(feature1Prev.get());
                feature1Curr->bestPrevMatchId() = 0;

                feature2Prev->nextMatches().push_back(feature2Curr.get());
                feature2Prev->bestNextMatchId() = 0;

                feature2Curr->prevMatches().push_back(feature2Prev.get());
                feature2Curr->bestPrevMatchId() = 0;
            }

            // Update scene point information for current frame set.
            Eigen::Matrix4d systemPose_inv = invertHomogeneousTransform(systemPose);

            for (size_t j = 0; j < features1Curr.size(); ++j)
            {
                const Point2DFeaturePtr& feature1Curr = features1Curr.at(j);
                const Point2DFeaturePtr& feature2Curr = features2Curr.at(j);

                if (feature1Curr->prevMatches().empty())
                {
                    Point3DFeaturePtr& scenePoint = feature1Curr->feature3D();

                    scenePoint->point() = transformPoint(systemPose_inv, scenePoint->point());
                }
                else
                {
                    Point2DFeature* feature1Prev = feature1Curr->prevMatch();

                    Point3DFeaturePtr& scenePoint = feature1Prev->feature3D();

                    feature1Curr->feature3D() = scenePoint;
                    scenePoint->features2D().push_back(feature1Curr.get());

                    feature2Curr->feature3D() = scenePoint;
                    scenePoint->features2D().push_back(feature2Curr.get());
                }
            }
        }

        if (m_debug)
        {
            ROS_INFO("### Pose estimation took %.3f s. ###", (ros::Time::now() - tsStartPoseEst).toSec());
        }
    }

    if (m_debug)
    {
        for (int i = 0; i < nStereoCameras; ++i)
        {
            FramePtr& frame1 = frameSet->frame(i * 2);

            ROS_INFO("# stereo correspondences:   %lu", frame1->features2D().size());

            int nTempCorr = 0;
            const std::vector<Point2DFeaturePtr>& features1Curr = frame1->features2D();
            for (size_t j = 0; j < features1Curr.size(); ++j)
            {
                if (!features1Curr.at(j)->prevMatches().empty())
                {
                    ++nTempCorr;
                }
            }

            ROS_INFO("[Stereo Cam %d] # temporal correspondences: %d", i, nTempCorr);
        }

        double avgError, maxError;
        size_t featureCount;
        reprojErrorStats(frameSet, avgError, maxError, featureCount);

        ROS_INFO("Reprojection error before local BA: avg = %.3f | max = %.3f | count = %lu",
                 avgError, maxError, featureCount);
    }

    ros::Time tsStartLBA = ros::Time::now();

    if (m_lba)
    {
        m_lba->addFrameSet(frameSet, replaceCurrentFrameSet);
    }

    // remove stereo correspondences that have high reprojection errors
    for (int i = 0; i < nStereoCameras; ++i)
    {
        int cameraId1 = i * 2;
        int cameraId2 = i * 2 + 1;

        const FramePtr& frame1 = frameSet->frame(cameraId1);
        const FramePtr& frame2 = frameSet->frame(cameraId2);

        std::vector<Point2DFeaturePtr>::iterator it1 = frame1->features2D().begin();
        std::vector<Point2DFeaturePtr>::iterator it2 = frame2->features2D().begin();

        Eigen::Matrix4d H_sys_cam1 = invertHomogeneousTransform(m_cameraSystem->getGlobalCameraPose(cameraId1));
        Eigen::Matrix4d H_sys_cam2 = invertHomogeneousTransform(m_cameraSystem->getGlobalCameraPose(cameraId2));

        while (it1 != frame1->features2D().end())
        {
            Point2DFeature* feature1 = it1->get();
            Point2DFeature* feature2 = it2->get();

            if (feature1->prevMatches().empty())
            {
                ++it1;
                ++it2;
                continue;
            }

            const Point3DFeaturePtr& scenePoint = feature1->feature3D();
            Eigen::Vector3d P = scenePoint->point();

            bool remove = false;

            Eigen::Matrix4d H_1 = H_sys_cam1 * frameSet->systemPose()->toMatrix();
            Eigen::Vector3d P_1 = transformPoint(H_1, P);
            Eigen::Vector3d ray1_est = P_1.normalized();

            double err1 = fabs(ray1_est.dot(feature1->ray()));
            if (err1 < k_sphericalErrorThresh)
            {
                remove = true;
            }

            Eigen::Matrix4d H_2 = H_sys_cam2 * frameSet->systemPose()->toMatrix();
            Eigen::Vector3d P_2 = transformPoint(H_2, P);
            Eigen::Vector3d ray2_est = P_2.normalized();

            double err2 = fabs(ray2_est.dot(feature2->ray()));
            if (err2 < k_sphericalErrorThresh)
            {
                remove = true;
            }

            if (remove)
            {
                Point2DFeature* feature1Prev = feature1->prevMatch();
                Point2DFeature* feature2Prev = feature2->prevMatch();

                feature1Prev->nextMatches().clear();
                feature1Prev->bestNextMatchId() = -1;

                feature2Prev->nextMatches().clear();
                feature2Prev->bestNextMatchId() = -1;

                bool flag1 = false, flag2 = false;
                std::vector<Point2DFeature*>::reverse_iterator it = scenePoint->features2D().rbegin();
                while (it != scenePoint->features2D().rend())
                {
                    if (*it == feature1)
                    {
                        scenePoint->features2D().erase(--it.base());
                        flag1 = true;
                    }
                    if (*it == feature2)
                    {
                        scenePoint->features2D().erase(--it.base());
                        flag2 = true;
                    }
                    if (flag1 && flag2)
                    {
                        break;
                    }

                    ++it;
                }

                frame1->features2D().erase(it1);
                frame2->features2D().erase(it2);
            }
            else
            {
                ++it1;
                ++it2;
            }
        }
    }

    if (m_debug)
    {
        ROS_INFO("### Local BA took %.3f s. ###", (ros::Time::now() - tsStartLBA).toSec());
        ROS_INFO("VO took %.3f s.", (ros::Time::now() - tsStart).toSec());

        double avgError, maxError;
        size_t featureCount;
        reprojErrorStats(frameSet, avgError, maxError, featureCount);

        ROS_INFO("Reprojection error after local BA: avg = %.3f | max = %.3f | count = %lu",
                 avgError, maxError, featureCount);

        visualizeCorrespondences(m_frameSetPrev, frameSet);
    }

    if (!m_frameSetPrev)
    {
        m_frameSetPrev = frameSet;
    }
    else
    {
        frameSet->prevFrameSet() = m_frameSetPrev.get();
        frameSet->prevTransformMeasurement() =
            Transform(m_frameSetPrev->systemPose()->toMatrix() *
                      invertHomogeneousTransform(frameSet->systemPose()->toMatrix()));

        m_frameSetPrev->nextFrameSet() = frameSet.get();
        m_frameSetPrev->nextTransformMeasurement() =
            Transform(frameSet->systemPose()->toMatrix() *
                      invertHomogeneousTransform(m_frameSetPrev->systemPose()->toMatrix()));

        m_frameSetCurr = frameSet;
    }

    return true;
}

size_t
GCamVO::getCurrentCorrespondenceCount(void) const
{
    return m_nCorrespondences;
}

void
GCamVO::keyCurrentFrameSet(void)
{
    if (!m_frameSetCurr)
    {
        return;
    }

    m_frameSetPrev = m_frameSetCurr;
    m_frameSetCurr.reset();

    if (m_debug)
    {
        ROS_INFO("Keyed frameset.");
    }
}

bool
GCamVO::isRunning(void)
{
    if (!m_globalMutex.try_lock())
    {
        return true;
    }

    m_globalMutex.unlock();

    return false;
}

void
GCamVO::getDescriptorMat(const FrameConstPtr& frame, cv::Mat& dmat) const
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
GCamVO::getDescriptorMatVec(const FrameSetConstPtr& frameSet,
                            std::vector<cv::Mat>& dmatVec) const
{
    for (size_t i = 0; i < frameSet->frames().size(); ++i)
    {
        cv::Mat dmat;
        getDescriptorMat(frameSet->frame(i), dmat);

        dmatVec.push_back(dmat);
    }
}

void
GCamVO::matchDescriptors(const cv::Mat& queryDescriptors,
                         const cv::Mat& trainDescriptors,
                         std::vector<cv::DMatch>& matches,
                         const cv::Mat& mask,
                         DescriptorMatchMethod matchMethod,
                         float matchParam) const
{
    matches.clear();

    switch (matchMethod)
    {
    case RATIO_MATCH:
    {
        std::vector<std::vector<cv::DMatch> > rawMatches;
        m_descriptorMatcher->knnMatch(queryDescriptors, trainDescriptors,
                                      rawMatches, 2, mask, true);

        matches.reserve(rawMatches.size());
        for (size_t i = 0; i < rawMatches.size(); ++i)
        {
            const std::vector<cv::DMatch>& rawMatch = rawMatches.at(i);

            if (rawMatch.size() < 2)
            {
                continue;
            }

            float distanceRatio = rawMatch.at(0).distance / rawMatch.at(1).distance;

            if (distanceRatio > matchParam)
            {
                continue;
            }

            matches.push_back(rawMatch.at(0));
        }
        break;
    }
    case BEST_MATCH:
    default:
    {
        m_descriptorMatcher->match(queryDescriptors, trainDescriptors,
                                   matches, mask);
    }
    }
}

void
GCamVO::matchCorrespondences(const FrameSetConstPtr& frameSet1,
                             const FrameSetConstPtr& frameSet2,
                             std::vector<std::vector<cv::DMatch> >& matches) const
{
    std::vector<cv::Mat> dtors1;
    getDescriptorMatVec(frameSet1, dtors1);

    std::vector<cv::Mat> dtors2;
    getDescriptorMatVec(frameSet2, dtors2);

    int nStereoCameras = m_cameraSystem->cameraCount() / 2;
    std::vector<boost::shared_ptr<boost::thread> > threads(nStereoCameras);

    matches.resize(nStereoCameras);

    for (int i = 0; i < nStereoCameras; ++i)
    {
        int cameraId1 = i * 2;
        int cameraId2 = i * 2 + 1;

        threads.at(i) = boost::make_shared<boost::thread>(boost::bind(&GCamVO::matchStereoCorrespondences,
                                                                      this,
                                                                      boost::cref(dtors1.at(cameraId1)),
                                                                      boost::cref(dtors1.at(cameraId2)),
                                                                      boost::cref(dtors2.at(cameraId1)),
                                                                      boost::cref(dtors2.at(cameraId2)),
                                                                      boost::ref(matches.at(i))));
    }
    for (int i = 0; i < nStereoCameras; ++i)
    {
        threads.at(i)->join();
    }
}

void
GCamVO::matchStereoCorrespondences(const cv::Mat& dtors11,
                                   const cv::Mat& dtors12,
                                   const cv::Mat& dtors21,
                                   const cv::Mat& dtors22,
                                   std::vector<cv::DMatch>& matches) const
{
    matches.clear();

    std::vector<cv::DMatch> rawMatches1, rawMatches2;
    boost::shared_ptr<boost::thread> threads[2];

    threads[0] = boost::make_shared<boost::thread>(boost::bind(&GCamVO::matchDescriptors, this,
                                                               boost::cref(dtors11), boost::cref(dtors21), boost::ref(rawMatches1),
                                                               cv::Mat(), BEST_MATCH, k_maxDistanceRatio));

    threads[1] = boost::make_shared<boost::thread>(boost::bind(&GCamVO::matchDescriptors, this,
                                                               boost::cref(dtors12), boost::cref(dtors22), boost::ref(rawMatches2),
                                                               cv::Mat(), BEST_MATCH, k_maxDistanceRatio));

    threads[0]->join();
    threads[1]->join();

    std::vector<int> matchLUT(dtors11.rows, -1);
    for (size_t i = 0; i < rawMatches1.size(); ++i)
    {
        const cv::DMatch& rawMatch = rawMatches1.at(i);

        matchLUT.at(rawMatch.queryIdx) = rawMatch.trainIdx;
    }

    std::vector<cv::DMatch> candidateMatches;
    for (size_t i = 0; i < rawMatches2.size(); ++i)
    {
        const cv::DMatch& rawMatch = rawMatches2.at(i);

        int trainIdx = matchLUT.at(rawMatch.queryIdx);
        if (trainIdx != -1 && trainIdx == rawMatch.trainIdx)
        {
            candidateMatches.push_back(cv::DMatch(rawMatch.queryIdx, trainIdx, 0.0f));
        }
    }

    // Remove matches which have the same training descriptor index.
    std::vector<std::vector<cv::DMatch> > revMatches(dtors21.rows);
    for (size_t i = 0; i < candidateMatches.size(); ++i)
    {
        const cv::DMatch& match = candidateMatches.at(i);

        revMatches.at(match.trainIdx).push_back(match);
    }

    for (size_t i = 0; i < revMatches.size(); ++i)
    {
        const std::vector<cv::DMatch>& subRevMatches = revMatches.at(i);

        if (subRevMatches.size() == 1)
        {
            matches.push_back(subRevMatches.front());
        }
    }
}

void
GCamVO::processFrame(CameraMetadata& metadata) const
{
    if (k_preUndistort)
    {
        // Undistort images so that we avoid the computationally expensive step of
        // applying distortion and undistortion in projection and backprojection
        // respectively.
        cv::remap(metadata.rawImage, metadata.procImage,
                  metadata.undistortMapX, metadata.undistortMapY,
                  cv::INTER_LINEAR);
    }
    else
    {
        metadata.rawImage.copyTo(metadata.procImage);
    }

    // Detect features.
    m_featureDetector->detect(metadata.procImage, metadata.kpts);

    // Backproject feature coordinates to rays with spherical coordinates.
    metadata.spts.resize(metadata.kpts.size());
    for (size_t i = 0; i < metadata.kpts.size(); ++i)
    {
        const cv::KeyPoint& kpt = metadata.kpts.at(i);

        metadata.camera->liftSphere(Eigen::Vector2d(kpt.pt.x, kpt.pt.y),
                                    metadata.spts.at(i));
    }

    m_descriptorExtractor->compute(metadata.procImage, metadata.kpts,
                                   metadata.dtors);
}

void
GCamVO::processStereoFrame(CameraMetadata& metadata1,
                           CameraMetadata& metadata2) const
{
    // Match descriptors between stereo images.
    std::vector<cv::DMatch> rawMatches;
    matchDescriptors(metadata1.dtors, metadata2.dtors, rawMatches,
                     cv::Mat(), BEST_MATCH);

    // Use the epipolar constraint to filter out outlier matches.
    Eigen::Matrix3d E = m_E.at(metadata1.frame->cameraId() / 2);

    std::vector<cv::DMatch> matches;
    matches.reserve(rawMatches.size());
    for (size_t i = 0; i < rawMatches.size(); ++i)
    {
        const cv::DMatch& rawMatch = rawMatches.at(i);

        const Eigen::Vector3d& spt1 = metadata1.spts.at(rawMatch.queryIdx);
        const Eigen::Vector3d& spt2 = metadata2.spts.at(rawMatch.trainIdx);

        double err = sampsonError(E, spt1, spt2);
        if (err < k_epipolarThresh)
        {
            matches.push_back(rawMatch);
        }
    }

    // For each match, reconstruct the scene point.
    // If the reprojection error of the scene point in either camera exceeds
    // a threshold, mark the match as an outlier.
    Eigen::Matrix4d H_stereo = m_H_stereo.at(metadata1.frame->cameraId() / 2);
    Eigen::Matrix4d H_cam1_sys = m_cameraSystem->getGlobalCameraPose(metadata1.frame->cameraId());

    metadata1.frame->features2D().reserve(matches.size());
    metadata2.frame->features2D().reserve(matches.size());
    for (size_t i = 0; i < matches.size(); ++i)
    {
        const cv::DMatch& match = matches.at(i);

        Point2DFeaturePtr feature1 = boost::make_shared<Point2DFeature>();
        feature1->frame() = metadata1.frame.get();
        feature1->keypoint() = metadata1.kpts.at(match.queryIdx);
        metadata1.dtors.row(match.queryIdx).copyTo(feature1->descriptor());
        feature1->ray() = metadata1.spts.at(match.queryIdx);

        Point2DFeaturePtr feature2 = boost::make_shared<Point2DFeature>();
        feature2->frame() = metadata2.frame.get();
        feature2->keypoint() = metadata2.kpts.at(match.trainIdx);
        metadata2.dtors.row(match.trainIdx).copyTo(feature2->descriptor());
        feature2->ray() = metadata2.spts.at(match.trainIdx);

        if (!reconstructScenePoint(feature1, feature2, H_stereo))
        {
            continue;
        }

        Eigen::Vector3d& P = feature1->feature3D()->point();
        P = transformPoint(H_cam1_sys, P);

        metadata1.frame->features2D().push_back(feature1);
        metadata2.frame->features2D().push_back(feature2);

        feature1->bestMatchId() = 0;
        feature1->matches().push_back(feature2.get());

        feature2->bestMatchId() = 0;
        feature2->matches().push_back(feature1.get());
    }
}

bool
GCamVO::reconstructScenePoint(Point2DFeaturePtr& f1,
                              Point2DFeaturePtr& f2,
                              const Eigen::Matrix4d& H) const
{
    Eigen::MatrixXd A(3,2);
    A.col(0) = H.block<3,3>(0,0) * f1->ray();
    A.col(1) = - f2->ray();

    Eigen::Vector3d b = - H.block<3,1>(0,3);

    Eigen::Vector2d gamma = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    // check if scene point is behind camera
    if (gamma(0) < 0.0 || gamma(1) < 0.0)
    {
        return false;
    }

    // check if scene point is outside allowable stereo range
    if (gamma(0) > k_maxStereoRange)
    {
        return false;
    }

    Eigen::Vector3d P1 = gamma(0) * f1->ray();
    Eigen::Vector3d P2 = transformPoint(H, P1);

    if (P2(2) < 0.0)
    {
        return false;
    }

    Eigen::Vector3d ray2_est = P2.normalized();

    double err = fabs(ray2_est.dot(f2->ray()));
    if (err < k_sphericalErrorThresh)
    {
        return false;
    }

    Point3DFeaturePtr p3D = boost::make_shared<Point3DFeature>();
    p3D->point() = P1;
    p3D->pointFromStereo() = P1;
    p3D->features2D().push_back(f1.get());
    p3D->features2D().push_back(f2.get());

    f1->feature3D() = p3D;
    f2->feature3D() = p3D;

    return true;
}

void
GCamVO::reprojErrorStats(const FrameSetConstPtr& frameSet,
                         double& avgError, double& maxError,
                         size_t& featureCount) const
{
    size_t count = 0;
    double sumError = 0.0;
    maxError = 0.0;

    for (size_t i = 0; i < frameSet->frames().size(); ++i)
    {
        const FrameConstPtr& frame = frameSet->frame(i);
        const CameraConstPtr& camera = m_cameraSystem->getCamera(i);

        Eigen::Matrix4d pose = invertHomogeneousTransform(m_cameraSystem->getGlobalCameraPose(i)) *
                               frameSet->systemPose()->toMatrix();

        for (size_t j = 0; j < frame->features2D().size(); ++j)
        {
            const Point2DFeaturePtr& feature = frame->features2D().at(j);
            const Point3DFeaturePtr& scenePoint = feature->feature3D();

            Eigen::Vector3d P = transformPoint(pose, scenePoint->point());
            Eigen::Vector2d p;
            camera->spaceToPlane(P, p);

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

void
GCamVO::solve3PRansac(const FrameSetConstPtr& frameSet1,
                      const FrameSetConstPtr& frameSet2,
                      const std::vector<std::vector<cv::DMatch> >& matches,
                      Eigen::Matrix4d& systemPose,
                      std::vector<std::vector<cv::DMatch> >& inliers) const
{
    inliers.clear();

    double p = 0.99; // probability that at least one set of random samples does not contain an outlier
    double v = 0.6; // probability of observing an outlier

    double u = 1.0 - v;
    int N = static_cast<int>(log(1.0 - p) / log(1.0 - u * u * u) + 0.5);

    Eigen::Quaterniond q1(frameSet1->imuMeasurement()->orientation.w,
                          frameSet1->imuMeasurement()->orientation.x,
                          frameSet1->imuMeasurement()->orientation.y,
                          frameSet1->imuMeasurement()->orientation.z);
    Eigen::Quaterniond q2(frameSet2->imuMeasurement()->orientation.w,
                          frameSet2->imuMeasurement()->orientation.x,
                          frameSet2->imuMeasurement()->orientation.y,
                          frameSet2->imuMeasurement()->orientation.z);
    Eigen::Matrix3d R = (q2.conjugate() * q1).toRotationMatrix();

    std::vector<std::pair<size_t,size_t> > indices;
    int nMatches = 0;
    for (size_t i = 0; i < matches.size(); ++i)
    {
        for (size_t j = 0; j < matches.at(i).size(); ++j)
        {
            indices.push_back(std::make_pair(i,j));
        }
    }

    // run RANSAC to find best H
    Eigen::Matrix4d H_best;
    size_t nInliers_best = 0;
    std::vector<std::vector<size_t> > inlierIds_best;
    for (int i = 0; i < N; ++i)
    {
        std::random_shuffle(indices.begin(), indices.end());

        std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> > lcVec;
        for (int j = 0; j < 3; ++j)
        {
            int cameraId = indices.at(j).first * 2;
            int matchId = indices.at(j).second;

            const cv::DMatch& match = matches.at(indices.at(j).first).at(matchId);

            lcVec.push_back(PLineCorrespondence(cameraId,
                                                frameSet1->frame(cameraId)->features2D().at(match.queryIdx)->ray(),
                                                m_cameraSystem->getGlobalCameraPose(cameraId),
                                                cameraId + 1,
                                                frameSet2->frame(cameraId + 1)->features2D().at(match.trainIdx)->ray(),
                                                m_cameraSystem->getGlobalCameraPose(cameraId + 1)));
        }

        Eigen::Vector3d t;
        if (!m_gcam->estimateT(lcVec, R, t))
        {
            continue;
        }

        Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
        H.block<3,3>(0,0) = R;
        H.block<3,1>(0,3) = t;

        size_t nInliers = 0;
        std::vector<std::vector<size_t> > inlierIds(matches.size());
        for (size_t j = 0; j < matches.size(); ++j)
        {
            const std::vector<cv::DMatch>& subMatches = matches.at(j);
            std::vector<size_t>& subInlierIds = inlierIds.at(j);

            int cameraId = j * 2;

            Eigen::Matrix4d H_cam1_cam2 = invertHomogeneousTransform(m_cameraSystem->getGlobalCameraPose(cameraId + 1)) * H * m_cameraSystem->getGlobalCameraPose(cameraId);

            R = H_cam1_cam2.block<3,3>(0,0);
            t = H_cam1_cam2.block<3,1>(0,3);
            Eigen::Matrix3d E = skew(t) * R;

            subInlierIds.reserve(subMatches.size());
            for (size_t k = 0; k < subMatches.size(); ++k)
            {
                const cv::DMatch& match = subMatches.at(k);

                const Point2DFeatureConstPtr& f1 = frameSet1->frame(cameraId)->features2D().at(match.queryIdx);
                const Point2DFeatureConstPtr& f2 = frameSet2->frame(cameraId + 1)->features2D().at(match.trainIdx);

                double err = sampsonError(E, f1->ray(), f2->ray());
                if (err > k_epipolarThresh)
                {
                    continue;
                }

                subInlierIds.push_back(k);
                ++nInliers;
            }
        }

        if (nInliers > nInliers_best)
        {
            H_best = H;
            nInliers_best = nInliers;
            inlierIds_best = inlierIds;
        }
    }

    inliers.resize(inlierIds_best.size());
    for (size_t i = 0; i < inlierIds_best.size(); ++i)
    {
        for (size_t j = 0; j < inlierIds_best.at(i).size(); ++j)
        {
            inliers.at(i).push_back(matches.at(i).at(inlierIds_best.at(i).at(j)));
        }
    }

    systemPose = H_best * frameSet1->systemPose()->toMatrix();
}

void
GCamVO::solveP3PRansac(const FrameSetConstPtr& frameSet1,
                       const FrameSetConstPtr& frameSet2,
                       const std::vector<std::vector<cv::DMatch> >& matches,
                       Eigen::Matrix4d& systemPose,
                       std::vector<std::vector<cv::DMatch> >& inliers) const
{
    inliers.clear();

    double p = 0.99; // probability that at least one set of random samples does not contain an outlier
    double v = 0.6; // probability of observing an outlier

    double u = 1.0 - v;
    int N = static_cast<int>(log(1.0 - p) / log(1.0 - u * u * u) + 0.5);

    std::vector<std::pair<size_t,size_t> > indices;
    int nMatches = 0;
    for (size_t i = 0; i < matches.size(); ++i)
    {
        size_t nSubMatches = matches.at(i).size();

        indices.reserve(indices.size() + nSubMatches);
        for (size_t j = 0; j < nSubMatches; ++j)
        {
            indices.push_back(std::make_pair(i,j));
        }
    }

    // run RANSAC to find best H
    Eigen::Matrix4d H_best;
    size_t nInliers_best = 0;
    std::vector<std::vector<size_t> > inlierIds_best;
    for (int i = 0; i < N; ++i)
    {
        std::random_shuffle(indices.begin(), indices.end());

        std::vector<PLine, Eigen::aligned_allocator<PLine> > plines(3);
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > worldPoints(3);
        for (int j = 0; j < 3; ++j)
        {
            int stereoId = indices.at(j).first;
            int cameraId = stereoId * 2;
            int matchId = indices.at(j).second;

            const cv::DMatch& match = matches.at(stereoId).at(matchId);

            worldPoints.at(j) = frameSet1->frame(cameraId)->features2D().at(match.queryIdx)->feature3D()->point();
            plines.at(j) = PLine(frameSet2->frame(cameraId)->features2D().at(match.trainIdx)->ray(),
                                 m_cameraSystem->getGlobalCameraPose(cameraId));
        }

        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > H;
        if (!solvegP3P(plines, worldPoints, H))
        {
            continue;
        }

        for (size_t j = 0; j < H.size(); ++j)
        {
            Eigen::Matrix4d H_inv = invertHomogeneousTransform(H.at(j));

            size_t nInliers = 0;
            std::vector<std::vector<size_t> > inlierIds(matches.size());
            for (size_t k = 0; k < matches.size(); ++k)
            {
                const std::vector<cv::DMatch>& subMatches = matches.at(k);
                std::vector<size_t>& subInlierIds = inlierIds.at(k);

                int cameraId = k * 2;
                const std::vector<Point2DFeaturePtr>& features1 = frameSet1->frame(cameraId)->features2D();
                const std::vector<Point2DFeaturePtr>& features2 = frameSet2->frame(cameraId)->features2D();
                Eigen::Matrix4d H_cam = invertHomogeneousTransform(m_cameraSystem->getGlobalCameraPose(cameraId)) * H_inv;

                subInlierIds.reserve(subMatches.size());
                for (size_t l = 0; l < subMatches.size(); ++l)
                {
                    const cv::DMatch& match = subMatches.at(l);

                    Eigen::Vector3d P = features1.at(match.queryIdx)->feature3D()->point();

                    Eigen::Vector3d P_cam_pred = transformPoint(H_cam, P);

                    const Point2DFeatureConstPtr& f2 = features2.at(match.trainIdx);

                    double err = fabs(P_cam_pred.normalized().dot(f2->ray()));
                    if (err < k_sphericalErrorThresh)
                    {
                        continue;
                    }

                    subInlierIds.push_back(l);
                    ++nInliers;
                }
            }

            if (nInliers > nInliers_best)
            {
                H_best = H_inv;
                nInliers_best = nInliers;
                inlierIds_best = inlierIds;
            }
        }
    }

    inliers.resize(inlierIds_best.size());
    for (size_t i = 0; i < inlierIds_best.size(); ++i)
    {
        std::vector<cv::DMatch>& subInliers = inliers.at(i);
        std::vector<size_t>& subInlierIds_best = inlierIds_best.at(i);
        const std::vector<cv::DMatch>& subMatches = matches.at(i);

        for (size_t j = 0; j < subInlierIds_best.size(); ++j)
        {
            subInliers.push_back(subMatches.at(subInlierIds_best.at(j)));
        }
    }

    systemPose = H_best;
}

void
GCamVO::visualizeCorrespondences(const FrameSetConstPtr& frameSetPrev,
                                 const FrameSetConstPtr& frameSetCurr) const
{
    int nStereoCameras = m_cameraSystem->cameraCount() / 2;

    for (int i = 0; i < nStereoCameras; ++i)
    {
        int cameraId1 = i * 2;
        int cameraId2 = i * 2 + 1;

        const CameraMetadata& metadata1 = m_metadataVec.at(cameraId1);
        const CameraMetadata& metadata2 = m_metadataVec.at(cameraId2);

        cv::Mat sketch(metadata1.procImage.rows,
                       metadata1.procImage.cols + metadata2.procImage.cols,
                       CV_8UC3);

        cv::Mat image1(sketch, cv::Rect(0, 0, metadata1.procImage.cols, metadata1.procImage.rows));
        cv::cvtColor(metadata1.procImage, image1, CV_GRAY2BGR);

        cv::Mat image2(sketch, cv::Rect(metadata1.procImage.cols, 0,
                                        metadata2.procImage.cols, metadata2.procImage.rows));
        cv::cvtColor(metadata2.procImage, image2, CV_GRAY2BGR);

        const std::vector<Point2DFeaturePtr>& features1Curr = frameSetCurr->frame(cameraId1)->features2D();
        const std::vector<Point2DFeaturePtr>& features2Curr = frameSetCurr->frame(cameraId2)->features2D();

        const int draw_shift_bits = 4;
        const int draw_multiplier = 1 << draw_shift_bits;
        const int radius = 3 * draw_multiplier;

        Eigen::Matrix4d systemPose = frameSetCurr->systemPose()->toMatrix();

        for (size_t j = 0; j < features1Curr.size(); ++j)
        {
            Point2DFeature* feature1 = features1Curr.at(j).get();
            Point2DFeature* feature2 = features2Curr.at(j).get();

            Eigen::Vector3d P = transformPoint(systemPose, feature1->feature3D()->point());
            double range = P.norm();
            float r, g, b;
            colormap("jet", fmin(range, 10.0) / 10.0 * 128.0, r, g, b);

            cv::Scalar color(r * 256.0f, g * 256.0f, b * 256.0f);

            std::vector<cv::Point2f> pts1;
            while (feature1)
            {
                pts1.push_back(feature1->keypoint().pt);

                if (feature1->prevMatches().empty())
                {
                    feature1 = 0;
                }
                else
                {
                    feature1 = feature1->prevMatch();
                }
            }

            if (pts1.size() > 1)
            {
                cv::Point p(cvRound(pts1.at(0).x * draw_multiplier),
                            cvRound(pts1.at(0).y * draw_multiplier));
                cv::circle(image1, p, radius, color, 1, CV_AA, draw_shift_bits);

                for (size_t k = 0; k < pts1.size() - 1; ++k)
                {
                    cv::Point p1(cvRound(pts1.at(k).x * draw_multiplier),
                                 cvRound(pts1.at(k).y * draw_multiplier));

                    cv::Point p2(cvRound(pts1.at(k + 1).x * draw_multiplier),
                                 cvRound(pts1.at(k + 1).y * draw_multiplier));

                    cv::line(image1, p1, p2, color, 1, CV_AA, draw_shift_bits);
                }
            }

            std::vector<cv::Point2f> pts2;
            while (feature2)
            {
                pts2.push_back(feature2->keypoint().pt);

                if (feature2->prevMatches().empty())
                {
                    feature2 = 0;
                }
                else
                {
                    feature2 = feature2->prevMatch();
                }
            }

            if (pts2.size() > 1)
            {
                cv::Point p(cvRound(pts2.at(0).x * draw_multiplier),
                            cvRound(pts2.at(0).y * draw_multiplier));
                cv::circle(image2, p, radius, color, 1, CV_AA, draw_shift_bits);

                for (size_t k = 0; k < pts2.size() - 1; ++k)
                {
                    cv::Point p1(cvRound(pts2.at(k).x * draw_multiplier),
                                 cvRound(pts2.at(k).y * draw_multiplier));

                    cv::Point p2(cvRound(pts2.at(k + 1).x * draw_multiplier),
                                 cvRound(pts2.at(k + 1).y * draw_multiplier));

                    cv::line(image2, p1, p2, color, 1, CV_AA, draw_shift_bits);
                }
            }

            if (pts1.size() > 1 && pts2.size() > 1)
            {
                cv::Point p1(cvRound(pts1.at(0).x * draw_multiplier),
                             cvRound(pts1.at(0).y * draw_multiplier));

                cv::Point dp2(cvRound(std::min(pts2.at(0).x + image1.cols,
                                               static_cast<float>(sketch.cols - 1)) * draw_multiplier),
                              cvRound(pts2.at(0).y * draw_multiplier));

                cv::line(sketch, p1, dp2, color, 1, CV_AA, draw_shift_bits);
            }
        }

        std::ostringstream oss;
        oss << "Correspondences [Stereo Cam " << i << "]";
        cv::imshow(oss.str(), sketch);
    }

    cv::waitKey(2);
}

}
