#ifndef GCAMSLAM_H
#define GCAMSLAM_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "camera_systems/CameraSystem.h"
#include "sparse_graph/SparseGraph.h"

namespace px
{

class GCamDWBA;
class GCamVO;
class OrbLocationRecognition;
class SparseGraphViz;

class GCamSLAM
{
public:
    GCamSLAM(ros::NodeHandle& nh,
             const CameraSystemConstPtr& cameraSystem);

    bool init(const std::string& detectorType,
              const std::string& descriptorExtractorType,
              const std::string& descriptorMatcherType,
              const std::string& poseTopicName,
              const std::string& vocFilename);

    bool processFrames(const ros::Time& stamp,
                       const std::vector<cv::Mat>& imageVec,
                       const sensor_msgs::ImuConstPtr& imu);

    bool writePosesToTextFile(const std::string& filename, bool wrtWorld = false) const;
    bool writeScenePointsToTextFile(const std::string& filename) const;

private:
    void findLoopClosures(std::vector<std::pair<LoopClosureEdge,LoopClosureEdge> >& edges);
    void findLoopClosuresHelper(const FrameConstPtr& frameQuery,
                                std::pair<LoopClosureEdge,LoopClosureEdge>& edge);

    void getDescriptorMat(const FrameConstPtr& frame, cv::Mat& dmat) const;

    void solveP3PRansac(const FrameConstPtr& frame1,
                        const FrameConstPtr& frame2,
                        const std::vector<cv::DMatch>& matches,
                        Eigen::Matrix4d& H,
                        std::vector<cv::DMatch>& inliers) const;

    ros::NodeHandle m_nh;
    CameraSystemConstPtr m_cameraSystem;

    boost::shared_ptr<GCamVO> m_vo;
    SparseGraphPtr m_sparseGraph;
    boost::shared_ptr<SparseGraphViz> m_sgv;
    ros::Publisher m_posePub;
    boost::shared_ptr<OrbLocationRecognition> m_locRec;
    boost::shared_ptr<GCamDWBA> m_dwba;

    FrameSetPtr m_frameSetKey;

    size_t k_minVOCorrespondenceCount;
    size_t k_minLoopCorrespondenceCount;
    int k_nLocationMatches;
    double k_sphericalErrorThresh;
};

}

#endif
