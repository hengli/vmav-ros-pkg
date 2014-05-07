#ifndef STEREOVO_H
#define STEREOVO_H

#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/features2d/features2d.hpp>

#include "camera_systems/CameraSystem.h"
#include "sparse_graph/SparseGraph.h"
#include "stereo_vo/LocalStereoBA.h"

namespace px
{

class StereoVO
{
public:
    StereoVO(const CameraSystemConstPtr& cameraSystem,
             int cameraId1, int cameraId2, bool preUndistort);

    bool init(const std::string& detectorType,
              const std::string& descriptorExtractorType,
              const std::string& descriptorMatcherType);

    bool readFrames(const ros::Time& stamp,
                    const cv::Mat& image1, const cv::Mat& image2);

    bool processFrames(FrameSetPtr& frameSet);

    // poses are with respect to the world frame
    bool getPreviousPose(Eigen::Matrix4d& pose) const;
    bool getPreviousPose(geometry_msgs::PoseStampedPtr& pose) const;

    bool getCurrentPose(Eigen::Matrix4d& pose) const;
    bool getCurrentPose(geometry_msgs::PoseStampedPtr& pose) const;

    size_t getCurrent2D3DCorrespondenceCount(void) const;

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

    void getDescriptorMat(const FrameConstPtr& frame, cv::Mat& dmat) const;
    void getDescriptorMatVec(const FrameSetConstPtr& frameSet,
                             std::vector<cv::Mat>& dmatVec) const;

    void matchDescriptors(const cv::Mat& queryDescriptors,
                          const cv::Mat& trainDescriptors,
                          std::vector<cv::DMatch>& matches,
                          const cv::Mat& mask = cv::Mat(),
                          DescriptorMatchMethod matchMethod = BEST_MATCH,
                          float matchParam = 0.0f) const;

    void match2D3DCorrespondences(const FrameSetConstPtr& frameSet1,
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

    int reconstructScenePoints(FrameSetPtr& frameSet) const;

    void reprojErrorStats(const FrameSetConstPtr& frameSet,
                          double& avgError, double& maxError,
                          size_t& featureCount) const;

    void solveP3PRansac(const FrameConstPtr& frame1,
                        const FrameConstPtr& frame2,
                        const std::vector<cv::DMatch>& matches,
                        Eigen::Matrix4d& cameraPose,
                        std::vector<cv::DMatch>& inliers) const;

    void visualizeCorrespondences(const FrameSetConstPtr& frameSetPrev,
                                  const FrameSetConstPtr& frameSetCurr) const;

    const double k_epipolarThresh;
    const float k_maxDistanceRatio;
    const double k_maxStereoRange;
    const bool k_preUndistort;
    const double k_sphericalErrorThresh;

    // input
    CameraSystemConstPtr m_cameraSystem;
    int m_cameraId1;
    int m_cameraId2;

    // raw images
    cv::Mat m_image1, m_image2;
    ros::Time m_imageStamp;

    // maps for undistorting images from both cameras
    cv::Mat m_undistortMapX1, m_undistortMapY1;
    cv::Mat m_undistortMapX2, m_undistortMapY2;

    // processed images
    cv::Mat m_imageProc1, m_imageProc2;

    // essential matrix between cameras 1 and 2
    Eigen::Matrix3d m_E;

    // transform between cameras 1 and 2
    Eigen::Matrix4d m_H_1_2;

    // previous frame set
    FrameSetPtr m_frameSetPrev;
    FrameSetPtr m_frameSetCurr;

    cv::Ptr<cv::FeatureDetector> m_featureDetector;
    cv::Ptr<cv::DescriptorExtractor> m_descriptorExtractor;
    cv::Ptr<cv::DescriptorMatcher> m_descriptorMatcher;

    boost::shared_ptr<LocalStereoBA> m_lba;

    boost::mutex m_globalMutex;
    size_t m_n2D3DCorrespondences;
    bool m_debug;
};

}

#endif
