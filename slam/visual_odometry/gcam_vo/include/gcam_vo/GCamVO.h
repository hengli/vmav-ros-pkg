#ifndef GCAMVO_H
#define GCAMVO_H

#include <boost/thread.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "camera_systems/CameraSystem.h"
#include "sparse_graph/SparseGraph.h"

namespace px
{

class GCamIMU;
class GCamLocalBA;

class GCamVO
{
public:
    GCamVO(const CameraSystemConstPtr& cameraSystem,
           bool preUndistort, bool useLocalBA);

    bool init(const std::string& detectorType,
              const std::string& descriptorExtractorType,
              const std::string& descriptorMatcherType);

    bool processFrames(const ros::Time& stamp,
                       const std::vector<cv::Mat>& imageVec,
                       const sensor_msgs::ImuConstPtr& imu,
                       FrameSetPtr& frameSet);

    size_t getCurrentCorrespondenceCount(void) const;

    void keyCurrentFrameSet(void);

    bool isRunning(void);

private:
    enum DescriptorMatchMethod
    {
        BEST_MATCH,
        RATIO_MATCH
    };

    struct CameraMetadata
    {
        CameraPtr camera;
        cv::Mat rawImage;      // raw image
        cv::Mat procImage;     // processed image
        cv::Mat undistortMapX; // maps for undistorting image
        cv::Mat undistortMapY;
        std::vector<cv::KeyPoint> kpts;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > spts;
        cv::Mat dtors;
        FramePtr frame;
    };

    void getDescriptorMat(const FrameConstPtr& frame, cv::Mat& dmat) const;
    void getDescriptorMatVec(const FrameSetConstPtr& frameSet,
                             std::vector<cv::Mat>& dmatVec) const;

    void matchDescriptors(const cv::Mat& queryDescriptors,
                          const cv::Mat& trainDescriptors,
                          std::vector<cv::DMatch>& matches,
                          const cv::Mat& mask = cv::Mat(),
                          DescriptorMatchMethod matchMethod = BEST_MATCH,
                          float matchParam = 0.0f) const;

    void matchCorrespondences(const FrameSetConstPtr& frameSet1,
                              const FrameSetConstPtr& frameSet2,
                              std::vector<std::vector<cv::DMatch> >& matches) const;
    void matchStereoCorrespondences(const cv::Mat& dtors11,
                                    const cv::Mat& dtors12,
                                    const cv::Mat& dtors21,
                                    const cv::Mat& dtors22,
                                    std::vector<cv::DMatch>& matches) const;

    void processFrame(CameraMetadata& metadata) const;

    void processStereoFrame(CameraMetadata& metadata1,
                            CameraMetadata& metadata2) const;

    bool reconstructScenePoint(Point2DFeaturePtr& f1,
                               Point2DFeaturePtr& f2,
                               const Eigen::Matrix4d& H) const;

    void reprojErrorStats(const FrameSetConstPtr& frameSet,
                          double& avgError, double& maxError,
                          size_t& featureCount) const;

    void solve3PRansac(const FrameSetConstPtr& frameSet1,
                       const FrameSetConstPtr& frameSet2,
                       const std::vector<std::vector<cv::DMatch> >& matches,
                       Eigen::Matrix4d& systemPose,
                       std::vector<std::vector<cv::DMatch> >& inliers) const;
    void solveP3PRansac(const FrameSetConstPtr& frameSet1,
                        const FrameSetConstPtr& frameSet2,
                        const std::vector<std::vector<cv::DMatch> >& matches,
                        Eigen::Matrix4d& systemPose,
                        std::vector<std::vector<cv::DMatch> >& inliers) const;

    void visualizeCorrespondences(const FrameSetConstPtr& frameSetPrev,
                                  const FrameSetConstPtr& frameSetCurr) const;

    const double k_epipolarThresh;
    const float k_maxDistanceRatio;
    const double k_maxStereoRange;
    const bool k_preUndistort;
    const double k_sphericalErrorThresh;

    // input
    CameraSystemConstPtr m_cameraSystem;

    // camera metadata
    std::vector<CameraMetadata> m_metadataVec;

    // essential matrix between cameras i and i+1
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > m_E;

    // transform between cameras i and i+1
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > m_H_stereo;

    // previous frame set
    FrameSetPtr m_frameSetPrev;
    FrameSetPtr m_frameSetCurr;

    cv::Ptr<cv::FeatureDetector> m_featureDetector;
    cv::Ptr<cv::DescriptorExtractor> m_descriptorExtractor;
    cv::Ptr<cv::DescriptorMatcher> m_descriptorMatcher;

    boost::shared_ptr<GCamIMU> m_gcam;
    boost::shared_ptr<GCamLocalBA> m_lba;

    boost::mutex m_globalMutex;
    size_t m_nCorrespondences;
    bool m_debug;
};

}

#endif
