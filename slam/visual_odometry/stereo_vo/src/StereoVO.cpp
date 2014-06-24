#include "stereo_vo/StereoVO.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

#include "cauldron/EigenUtils.h"
#include "pose_estimation/P3P.h"

namespace px
{

StereoVO::StereoVO(const CameraSystemConstPtr& cameraSystem,
                   int cameraId1, int cameraId2, bool preUndistort)
 : k_epipolarThresh(0.00005)
 , k_maxDistanceRatio(0.7f)
 , k_maxStereoRange(10.0)
 , k_preUndistort(preUndistort)
 , k_sphericalErrorThresh(0.999976)
 , m_cameraSystem(cameraSystem)
 , m_cameraId1(cameraId1)
 , m_cameraId2(cameraId2)
 , m_n2D3DCorrespondences(0)
 , m_debug(false)
{
    if (k_preUndistort)
    {
        cameraSystem->getCamera(cameraId1)->initUndistortMap(m_undistortMapX1, m_undistortMapY1);
        cameraSystem->getCamera(cameraId2)->initUndistortMap(m_undistortMapX2, m_undistortMapY2);

        cameraSystem->getCamera(cameraId1)->setZeroDistortion();
        cameraSystem->getCamera(cameraId2)->setZeroDistortion();
    }

    m_H_1_2 = cameraSystem->getGlobalCameraPose(cameraId2).inverse() * cameraSystem->getGlobalCameraPose(cameraId1);

    Eigen::Matrix3d R = m_H_1_2.block<3,3>(0,0);
    Eigen::Vector3d t = m_H_1_2.block<3,1>(0,3);

    m_E = skew(t) * R;

    m_lba = boost::make_shared<LocalStereoBA>(cameraSystem, m_cameraId1, m_cameraId2);
}

bool
StereoVO::init(const std::string& detectorType,
               const std::string& descriptorExtractorType,
               const std::string& descriptorMatcherType)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

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
StereoVO::readFrames(const ros::Time& stamp,
                     const cv::Mat& image1, const cv::Mat& image2)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    m_imageStamp = stamp;
    image1.copyTo(m_image1);
    image2.copyTo(m_image2);

    return true;
}

bool
StereoVO::processFrames(FrameSetPtr& frameSet)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    ros::Time tsStart = ros::Time::now();

    std::vector<cv::KeyPoint> kpts1, kpts2;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > spts1, spts2;
    cv::Mat dtors1, dtors2;

    boost::shared_ptr<boost::thread> threads[2];

    ImageMetadata metadata1(m_cameraSystem->getCamera(m_cameraId1),
                            m_image1, m_undistortMapX1, m_undistortMapY1);
    threads[0] = boost::make_shared<boost::thread>(boost::bind(&StereoVO::processFrame, this,
                                                               boost::cref(metadata1),
                                                               boost::ref(m_imageProc1),
                                                               boost::ref(kpts1),
                                                               boost::ref(spts1),
                                                               boost::ref(dtors1)));

    ImageMetadata metadata2(m_cameraSystem->getCamera(m_cameraId2),
                            m_image2, m_undistortMapX2, m_undistortMapY2);
    threads[1] = boost::make_shared<boost::thread>(boost::bind(&StereoVO::processFrame, this,
                                                               boost::cref(metadata2),
                                                               boost::ref(m_imageProc2),
                                                               boost::ref(kpts2),
                                                               boost::ref(spts2),
                                                               boost::ref(dtors2)));

    threads[0]->join();
    threads[1]->join();

    // --- Find feature correspondences between the stereo images. ---

    // Match descriptors between stereo images.
    std::vector<cv::DMatch> rawMatches;
    matchDescriptors(dtors1, dtors2, rawMatches, cv::Mat(), BEST_MATCH);

    // Use the epipolar constraint to filter out outlier matches.
    std::vector<cv::DMatch> matches;
    for (size_t i = 0; i < rawMatches.size(); ++i)
    {
        const cv::DMatch& rawMatch = rawMatches.at(i);

        const Eigen::Vector3d& spt1 = spts1.at(rawMatch.queryIdx);
        const Eigen::Vector3d& spt2 = spts2.at(rawMatch.trainIdx);

        double err = sampsonError(m_E, spt1, spt2);
        if (err < k_epipolarThresh)
        {
            matches.push_back(rawMatch);
        }
    }

    FramePtr frame1 = boost::make_shared<Frame>();
    frame1->cameraId() = m_cameraId1;

    FramePtr frame2 = boost::make_shared<Frame>();
    frame2->cameraId() = m_cameraId2;

    frameSet = boost::make_shared<FrameSet>();
    frame1->frameSet() = frameSet.get();
    frame2->frameSet() = frameSet.get();
    frameSet->frames().push_back(frame1);
    frameSet->frames().push_back(frame2);

    // For each match, reconstruct the scene point.
    // If the reprojection error of the scene point in either camera exceeds
    // a threshold, mark the match as an outlier.
    for (size_t i = 0; i < matches.size(); ++i)
    {
        const cv::DMatch& match = matches.at(i);

        Point2DFeaturePtr feature1 = boost::make_shared<Point2DFeature>();
        feature1->frame() = frame1.get();
        feature1->keypoint() = kpts1.at(match.queryIdx);
        dtors1.row(match.queryIdx).copyTo(feature1->descriptor());
        feature1->ray() = spts1.at(match.queryIdx);

        Point2DFeaturePtr feature2 = boost::make_shared<Point2DFeature>();
        feature2->frame() = frame2.get();
        feature2->keypoint() = kpts2.at(match.trainIdx);
        dtors2.row(match.trainIdx).copyTo(feature2->descriptor());
        feature2->ray() = spts2.at(match.trainIdx);

        if (!reconstructScenePoint(feature1, feature2, m_H_1_2))
        {
            continue;
        }

        frame1->features2D().push_back(feature1);
        frame2->features2D().push_back(feature2);

        feature1->bestMatchId() = 0;
        feature1->matches().push_back(feature2.get());

        feature2->bestMatchId() = 0;
        feature2->matches().push_back(feature1.get());
    }

    // Avoid copying image data as image data takes up significant memory.
    if (m_debug)
    {
        m_imageProc1.copyTo(frame1->image());
        m_imageProc2.copyTo(frame2->image());
    }

    bool replaceCurrentFrameSet = false;
    if (m_frameSetCurr)
    {
        // Current frame set is not keyed. Remove all references to
        // the current frame set.
        for (size_t i = 0; i < m_frameSetPrev->frames().size(); ++i)
        {
            std::vector<Point2DFeaturePtr>& features = m_frameSetPrev->frames().at(i)->features2D();

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

        for (size_t i = 0; i < m_frameSetCurr->frames().size(); ++i)
        {
            std::vector<Point2DFeaturePtr>& features = m_frameSetCurr->frames().at(i)->features2D();

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

        replaceCurrentFrameSet = true;
    }

    if (!m_frameSetPrev)
    {
        PosePtr pose = boost::make_shared<Pose>(Eigen::Matrix4d::Identity());
        pose->timeStamp() = m_imageStamp;

        frameSet->systemPose() = pose;
    }
    else
    {
        // Find 2D-3D correspondences between previous and current frames.
        match2D3DCorrespondences(m_frameSetPrev, frameSet, rawMatches);

        // Estimate relative pose from P3P RANSAC.
        Eigen::Matrix4d cameraPose;
        solveP3PRansac(m_frameSetPrev->frames().at(0), frameSet->frames().at(0),
                       rawMatches, cameraPose, matches);

        Eigen::Matrix4d systemPose = m_cameraSystem->getGlobalCameraPose(m_cameraId1) * cameraPose;

        m_n2D3DCorrespondences = matches.size();
        if (matches.size() < 10)
        {
            ROS_WARN("# 2D-3D correspondences (%lu) is too low for reliable motion estimation.",
                     matches.size());
        }

        PosePtr pose = boost::make_shared<Pose>(systemPose);
        pose->timeStamp() = m_imageStamp;

        frameSet->systemPose() = pose;

        std::vector<Point2DFeaturePtr>& features1Prev = m_frameSetPrev->frames().at(0)->features2D();
        std::vector<Point2DFeaturePtr>& features2Prev = m_frameSetPrev->frames().at(1)->features2D();
        std::vector<Point2DFeaturePtr>& features1Curr = frameSet->frames().at(0)->features2D();
        std::vector<Point2DFeaturePtr>& features2Curr = frameSet->frames().at(1)->features2D();

        for (size_t i = 0; i < matches.size(); ++i)
        {
            const cv::DMatch& match = matches.at(i);

            Point2DFeaturePtr& feature1Prev = features1Prev.at(match.queryIdx);
            Point2DFeaturePtr& feature2Prev = features2Prev.at(match.queryIdx);
            Point2DFeaturePtr& feature1Curr = features1Curr.at(match.trainIdx);
            Point2DFeaturePtr& feature2Curr = features2Curr.at(match.trainIdx);

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

        for (size_t i = 0; i < features1Curr.size(); ++i)
        {
            Point2DFeature* feature1Curr = features1Curr.at(i).get();
            Point2DFeature* feature2Curr = features2Curr.at(i).get();

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
                scenePoint->features2D().push_back(feature1Curr);

                feature2Curr->feature3D() = scenePoint;
                scenePoint->features2D().push_back(feature2Curr);
            }
        }
    }

    if (m_debug)
    {
        ROS_INFO("Motion estimation took %.3f s.", (ros::Time::now() - tsStart).toSec());

        ROS_INFO("# stereo correspondences:   %lu", frame1->features2D().size());

        int nTempCorr = 0;
        const std::vector<Point2DFeaturePtr>& features1Curr = frameSet->frames().at(0)->features2D();
        for (size_t i = 0; i < features1Curr.size(); ++i)
        {
            if (!features1Curr.at(i)->prevMatches().empty())
            {
                ++nTempCorr;
            }
        }

        ROS_INFO("# temporal correspondences: %d", nTempCorr);

        double avgError, maxError;
        size_t featureCount;
        reprojErrorStats(frameSet, avgError, maxError, featureCount);

        ROS_INFO("Reprojection error before local BA: avg = %.3f | max = %.3f | count = %lu",
                 avgError, maxError, featureCount);
    }

    tsStart = ros::Time::now();

    m_lba->addFrameSet(frameSet, replaceCurrentFrameSet);

    // remove stereo correspondences that have high reprojection errors
    std::vector<Point2DFeaturePtr>::iterator it1 = frame1->features2D().begin();
    std::vector<Point2DFeaturePtr>::iterator it2 = frame2->features2D().begin();

    Eigen::Matrix4d H_sys_cam1 = invertHomogeneousTransform(m_cameraSystem->getGlobalCameraPose(frame1->cameraId()));
    Eigen::Matrix4d H_sys_cam2 = invertHomogeneousTransform(m_cameraSystem->getGlobalCameraPose(frame2->cameraId()));

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

        Point3DFeaturePtr& scenePoint = feature1->feature3D();
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

    if (m_debug)
    {
        ROS_INFO("Local BA took %.3f s.", (ros::Time::now() - tsStart).toSec());

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
        m_frameSetCurr = frameSet;
    }

    return true;
}

bool
StereoVO::getPreviousPose(Eigen::Matrix4d& pose) const
{
    if (m_frameSetPrev)
    {
        pose = m_frameSetPrev->systemPose()->toMatrix().inverse();

        return true;
    }
    else
    {
        return false;
    }
}

bool
StereoVO::getPreviousPose(geometry_msgs::PoseStampedPtr& pose) const
{
    if (m_frameSetPrev)
    {
        pose = boost::make_shared<geometry_msgs::PoseStamped>();

        pose->header.stamp = m_frameSetPrev->systemPose()->timeStamp();

        Eigen::Matrix4d systemPose_inv = m_frameSetPrev->systemPose()->toMatrix().inverse();

        pose->pose.position.x = systemPose_inv(0,3);
        pose->pose.position.y = systemPose_inv(1,3);
        pose->pose.position.z = systemPose_inv(2,3);

        Eigen::Quaterniond q(systemPose_inv.block<3,3>(0,0));
        pose->pose.orientation.w = q.w();
        pose->pose.orientation.x = q.x();
        pose->pose.orientation.y = q.y();
        pose->pose.orientation.z = q.z();

        return true;
    }
    else
    {
        return false;
    }
}

bool
StereoVO::getCurrentPose(Eigen::Matrix4d& pose) const
{
    if (m_frameSetCurr)
    {
        pose = m_frameSetCurr->systemPose()->toMatrix().inverse();

        return true;
    }
    else
    {
        return false;
    }
}

bool
StereoVO::getCurrentPose(geometry_msgs::PoseStampedPtr& pose) const
{
    if (m_frameSetCurr)
    {
        pose = boost::make_shared<geometry_msgs::PoseStamped>();

        pose->header.stamp = m_frameSetCurr->systemPose()->timeStamp();

        Eigen::Matrix4d systemPose_inv = m_frameSetCurr->systemPose()->toMatrix().inverse();

        pose->pose.position.x = systemPose_inv(0,3);
        pose->pose.position.y = systemPose_inv(1,3);
        pose->pose.position.z = systemPose_inv(2,3);

        Eigen::Quaterniond q(systemPose_inv.block<3,3>(0,0));
        pose->pose.orientation.w = q.w();
        pose->pose.orientation.x = q.x();
        pose->pose.orientation.y = q.y();
        pose->pose.orientation.z = q.z();

        return true;
    }
    else
    {
        return false;
    }
}

size_t
StereoVO::getCurrent2D3DCorrespondenceCount(void) const
{
    return m_n2D3DCorrespondences;
}

void
StereoVO::keyCurrentFrameSet(void)
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

void
StereoVO::getDescriptorMat(const FrameConstPtr& frame, cv::Mat& dmat) const
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
StereoVO::getDescriptorMatVec(const FrameSetConstPtr& frameSet,
                              std::vector<cv::Mat>& dmatVec) const
{
    for (size_t i = 0; i < frameSet->frames().size(); ++i)
    {
        cv::Mat dmat;
        getDescriptorMat(frameSet->frames().at(i), dmat);

        dmatVec.push_back(dmat);
    }
}

void
StereoVO::matchDescriptors(const cv::Mat& queryDescriptors,
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
StereoVO::match2D3DCorrespondences(const FrameSetConstPtr& frameSet1,
                                   const FrameSetConstPtr& frameSet2,
                                   std::vector<cv::DMatch>& matches) const
{
    matches.clear();

    std::vector<cv::Mat> dtors1;
    getDescriptorMatVec(frameSet1, dtors1);

    std::vector<cv::Mat> dtors2;
    getDescriptorMatVec(frameSet2, dtors2);

    // match between left image in frame set 1 and left image in frame set 2
    std::vector<cv::DMatch> rawMatches1, rawMatches2;
    boost::shared_ptr<boost::thread> threads[2];

    threads[0] = boost::make_shared<boost::thread>(boost::bind(&StereoVO::matchDescriptors, this,
                                                               boost::cref(dtors1.at(0)), boost::cref(dtors2.at(0)), boost::ref(rawMatches1),
                                                               cv::Mat(), RATIO_MATCH, k_maxDistanceRatio));

    threads[1] = boost::make_shared<boost::thread>(boost::bind(&StereoVO::matchDescriptors, this,
                                                               boost::cref(dtors1.at(1)), boost::cref(dtors2.at(1)), boost::ref(rawMatches2),
                                                               cv::Mat(), RATIO_MATCH, k_maxDistanceRatio));

    threads[0]->join();
    threads[1]->join();

    std::vector<int> matchLUT(dtors1.at(0).rows, -1);
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
    std::vector<std::vector<cv::DMatch> > revMatches(dtors2.at(0).rows);
    for (size_t i = 0; i < candidateMatches.size(); ++i)
    {
        const cv::DMatch& match = candidateMatches.at(i);

        revMatches.at(match.trainIdx).push_back(match);
    }

    for (size_t i = 0; i < revMatches.size(); ++i)
    {
        if (revMatches.at(i).size() == 1)
        {
            matches.push_back(revMatches.at(i).front());
        }
    }
}

void
StereoVO::processFrame(const ImageMetadata& metadata,
                       cv::Mat& imageProc,
                       std::vector<cv::KeyPoint>& kpts,
                       std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& spts,
                       cv::Mat& dtors) const
{
    if (k_preUndistort)
    {
        // Undistort images so that we avoid the computationally expensive step of
        // applying distortion and undistortion in projection and backprojection
        // respectively.
        cv::remap(metadata.image, imageProc,
                  metadata.undistortMapX, metadata.undistortMapY,
                  cv::INTER_LINEAR);
    }
    else
    {
        metadata.image.copyTo(imageProc);
    }

    // Detect features.
    m_featureDetector->detect(imageProc, kpts, cv::Mat());

    // Backproject feature coordinates to rays with spherical coordinates.
    spts.resize(kpts.size());
    for (size_t i = 0; i < kpts.size(); ++i)
    {
        const cv::KeyPoint& kpt = kpts.at(i);

        metadata.cam->liftSphere(Eigen::Vector2d(kpt.pt.x, kpt.pt.y), spts.at(i));
    }

    m_descriptorExtractor->compute(imageProc, kpts, dtors);
}

bool
StereoVO::reconstructScenePoint(Point2DFeaturePtr& f1,
                                Point2DFeaturePtr& f2,
                                const Eigen::Matrix4d& H) const
{
    Frame* frame1 = f1->frame();

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
    Eigen::Vector3d P2 = H.block<3,3>(0,0) * P1 + H.block<3,1>(0,3);

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
    p3D->point() = transformPoint(m_cameraSystem->getGlobalCameraPose(frame1->cameraId()), P1);
    p3D->pointFromStereo() = P1;
    p3D->features2D().push_back(f1.get());
    p3D->features2D().push_back(f2.get());

    f1->feature3D() = p3D;
    f2->feature3D() = p3D;

    return true;
}

int
StereoVO::reconstructScenePoints(FrameSetPtr& frameSet) const
{
    FramePtr& frame1 = frameSet->frames().at(0);
    FramePtr& frame2 = frameSet->frames().at(1);

    std::vector<Point2DFeaturePtr>& features1 = frame1->features2D();
    std::vector<Point2DFeaturePtr>& features2 = frame2->features2D();

    int nScenePoints = 0;
    for (size_t i = 0; i < features1.size(); ++i)
    {
        if (reconstructScenePoint(features1.at(i), features2.at(i), m_H_1_2))
        {
            ++nScenePoints;
        }
    }

    return nScenePoints;
}

void
StereoVO::reprojErrorStats(const FrameSetConstPtr& frameSet,
                           double& avgError, double& maxError,
                           size_t& featureCount) const
{
    const FrameConstPtr& frame1 = frameSet->frames().at(0);
    const FrameConstPtr& frame2 = frameSet->frames().at(1);

    const CameraConstPtr& cam1 = m_cameraSystem->getCamera(m_cameraId1);
    const CameraConstPtr& cam2 = m_cameraSystem->getCamera(m_cameraId2);

    Eigen::Matrix4d pose1 = invertHomogeneousTransform(m_cameraSystem->getGlobalCameraPose(m_cameraId1)) *
                            frameSet->systemPose()->toMatrix();

    size_t count = 0;
    double sumError = 0.0;
    maxError = 0.0;
    for (size_t j = 0; j < frame1->features2D().size(); ++j)
    {
        const Point2DFeaturePtr& feature = frame1->features2D().at(j);

        if (feature->prevMatches().empty())
        {
            continue;
        }

        const Point3DFeaturePtr& scenePoint = feature->feature3D();

        Eigen::Vector3d P = transformPoint(pose1, scenePoint->point());
        Eigen::Vector2d p;
        cam1->spaceToPlane(P, p);

        double err = (p - Eigen::Vector2d(feature->keypoint().pt.x,
                                          feature->keypoint().pt.y)).norm();

        ++count;
        sumError += err;

        if (maxError < err)
        {
            maxError = err;
        }
    }

    Eigen::Matrix4d pose2 = invertHomogeneousTransform(m_cameraSystem->getGlobalCameraPose(m_cameraId2)) *
                            frameSet->systemPose()->toMatrix();

    for (size_t j = 0; j < frame2->features2D().size(); ++j)
    {
        const Point2DFeaturePtr& feature = frame2->features2D().at(j);

        if (feature->prevMatches().empty())
        {
            continue;
        }

        const Point3DFeaturePtr& scenePoint = feature->feature3D();

        Eigen::Vector3d P = transformPoint(pose2, scenePoint->point());
        Eigen::Vector2d p;
        cam2->spaceToPlane(P, p);

        double err = (p - Eigen::Vector2d(feature->keypoint().pt.x,
                                          feature->keypoint().pt.y)).norm();

        ++count;
        sumError += err;

        if (maxError < err)
        {
            maxError = err;
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
StereoVO::solveP3PRansac(const FrameConstPtr& frame1,
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

        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > H;
        if (!solveP3P(rays, worldPoints, H))
        {
            continue;
        }

        for (size_t j = 0; j < H.size(); ++j)
        {
            Eigen::Matrix4d H_inv = invertHomogeneousTransform(H.at(j));

            std::vector<size_t> inliersIds;
            for (size_t k = 0; k < matches.size(); ++k)
            {
                const cv::DMatch& match = matches.at(k);

                Eigen::Vector3d P_1 = features1.at(match.queryIdx)->feature3D()->point();

                Eigen::Vector3d P_2_pred = transformPoint(H_inv, P_1);

                const Point2DFeatureConstPtr& f2 = features2.at(match.trainIdx);

                double err = fabs(P_2_pred.normalized().dot(f2->ray()));
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

    H = H_best;
}

void
StereoVO::visualizeCorrespondences(const FrameSetConstPtr& frameSetPrev,
                                   const FrameSetConstPtr& frameSetCurr) const
{
    cv::Mat sketch(m_imageProc1.rows, m_imageProc1.cols + m_imageProc2.cols, CV_8UC3);

    cv::Mat image1(sketch, cv::Rect(0, 0, m_imageProc1.cols, m_imageProc1.rows));
    cv::cvtColor(m_imageProc1, image1, CV_GRAY2BGR);

    cv::Mat image2(sketch, cv::Rect(m_imageProc1.cols, 0, m_imageProc2.cols, m_imageProc2.rows));
    cv::cvtColor(m_imageProc2, image2, CV_GRAY2BGR);

    const std::vector<Point2DFeaturePtr>& features1Curr = frameSetCurr->frames().at(0)->features2D();
    const std::vector<Point2DFeaturePtr>& features2Curr = frameSetCurr->frames().at(1)->features2D();

    const int draw_shift_bits = 4;
    const int draw_multiplier = 1 << draw_shift_bits;
    const int radius = 3 * draw_multiplier;

    Eigen::Matrix4d systemPose = frameSetCurr->systemPose()->toMatrix();

    for (size_t i = 0; i < features1Curr.size(); ++i)
    {
        Point2DFeature* feature1 = features1Curr.at(i).get();
        Point2DFeature* feature2 = features2Curr.at(i).get();

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

            for (size_t j = 0; j < pts1.size() - 1; ++j)
            {
                cv::Point p1(cvRound(pts1.at(j).x * draw_multiplier),
                             cvRound(pts1.at(j).y * draw_multiplier));

                cv::Point p2(cvRound(pts1.at(j + 1).x * draw_multiplier),
                             cvRound(pts1.at(j + 1).y * draw_multiplier));

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

            for (size_t j = 0; j < pts2.size() - 1; ++j)
            {
                cv::Point p1(cvRound(pts2.at(j).x * draw_multiplier),
                             cvRound(pts2.at(j).y * draw_multiplier));

                cv::Point p2(cvRound(pts2.at(j + 1).x * draw_multiplier),
                             cvRound(pts2.at(j + 1).y * draw_multiplier));

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

    cv::imshow("Correspondences", sketch);
    cv::waitKey(2);
}

}
