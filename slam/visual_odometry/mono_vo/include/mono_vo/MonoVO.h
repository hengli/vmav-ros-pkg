#ifndef MONOVO_H
#define MONOVO_H

#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/features2d/features2d.hpp>

#include "camera_systems/CameraSystem.h"
#include "sparse_graph/SparseGraph.h"
#include "mono_vo/LocalMonoBA.h"

namespace px
{

class MonoVO
{
public:
    MonoVO(const CameraSystemConstPtr& cameraSystem,
           int cameraId, bool preUndistort);

    bool init(const std::string& detectorType,
              const std::string& descriptorExtractorType,
              const std::string& descriptorMatcherType);

    bool readFrame(const ros::Time& stamp,
                   const cv::Mat& image);

    bool processFrames(FrameSetPtr& frameSet);

    // poses are with respect to the world frame
    bool getPreviousPose(Eigen::Matrix4d& pose) const;
    bool getPreviousPose(geometry_msgs::PoseStampedPtr& pose) const;

    bool getCurrentPose(Eigen::Matrix4d& pose) const;
    bool getCurrentPose(geometry_msgs::PoseStampedPtr& pose) const;

    size_t getCurrentInlierCorrespondenceCount(void) const;

    void keyCurrentFrameSet(void);

private:
    enum DescriptorMatchMethod
    {
        BEST_MATCH,
        RATIO_MATCH
    };

    struct ImageMetadata
    {
        ImageMetadata(const CameraConstPtr& _cam,
                      const cv::Mat& _image,
                      const cv::Mat& _undistortMapX,
                      const cv::Mat& _undistortMapY)
         : cam(_cam)
         , image(_image)
         , undistortMapX(_undistortMapX)
         , undistortMapY(_undistortMapY)
        {

        }

        const CameraConstPtr& cam;
        const cv::Mat& image;
        const cv::Mat& undistortMapX;
        const cv::Mat& undistortMapY;
    };

    void removeSingletonFeatures(FrameSetPtr& frameSet) const;

    void removeCorrespondence(Point2DFeature* featurePrev,
                              Point2DFeature* featureCurr,
                              bool removeScenePointFor2CorrespondenceCase) const;

    void getDescriptorMat(const FrameConstPtr& frame, cv::Mat& dmat) const;
    void getDescriptorMatVec(const FrameSetConstPtr& frameSet,
                             std::vector<cv::Mat>& dmatVec) const;

    void matchDescriptors(const cv::Mat& queryDescriptors,
                          const cv::Mat& trainDescriptors,
                          std::vector<cv::DMatch>& matches,
                          const cv::Mat& mask = cv::Mat(),
                          DescriptorMatchMethod matchMethod = BEST_MATCH,
                          float matchParam = 0.0f) const;

    void match2D2DCorrespondences(const FrameSetConstPtr& frameSet1,
                                  const FrameSetConstPtr& frameSet2,
                                  std::vector<cv::DMatch>& matches) const;

    void processFrame(const ImageMetadata& metadata,
                      cv::Mat& imageProc,
                      std::vector<cv::KeyPoint>& kpts,
                      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& spts,
                      cv::Mat& dtors) const;

    bool reconstructScenePoint(Point2DFeaturePtr& f1,
                               Point2DFeaturePtr& f2,
                               const Eigen::Matrix4d& H) const;

    void reprojErrorStats(const FrameSetConstPtr& frameSet,
                          double& avgError, double& maxError,
                          size_t& featureCount) const;

    void solve5PointRansac(const FrameConstPtr& frame1,
                           const FrameConstPtr& frame2,
                           const std::vector<cv::DMatch>& matches,
                           Eigen::Matrix4d& relativeMotion,
                           std::vector<bool>& inliers) const;

    void solveP3PRansac(const FrameConstPtr& frame1,
                        const FrameConstPtr& frame2,
                        const std::vector<cv::DMatch>& matches,
                        Eigen::Matrix4d& cameraPose,
                        std::vector<bool>& inliers) const;

    void visualizeCorrespondences(const FrameSetConstPtr& frameSet) const;

    const double k_epipolarThresh;
    const float k_imageMotionThresh;
    const float k_maxDistanceRatio;
    const double k_maxStereoRange;
    const double k_nominalFocalLength;
    const bool k_preUndistort;
    const double k_reprojErrorThresh;
    const double k_sphericalErrorThresh;

    // input
    CameraSystemConstPtr m_cameraSystem;
    int m_cameraId;

    // raw images
    cv::Mat m_image;
    ros::Time m_imageStamp;

    // maps for undistorting images from the camera
    cv::Mat m_undistortMapX, m_undistortMapY;

    // processed image
    cv::Mat m_imageProc;

    // previous frame set
    FrameSetPtr m_frameSetPrev;
    FrameSetPtr m_frameSetCurr;

    cv::Ptr<cv::FeatureDetector> m_featureDetector;
    cv::Ptr<cv::DescriptorExtractor> m_descriptorExtractor;
    cv::Ptr<cv::DescriptorMatcher> m_descriptorMatcher;

    boost::shared_ptr<LocalMonoBA> m_lba;

    boost::mutex m_globalMutex;
    bool m_init;
    size_t m_nInlierCorrespondences;
    bool m_debug;
};

}

#endif
