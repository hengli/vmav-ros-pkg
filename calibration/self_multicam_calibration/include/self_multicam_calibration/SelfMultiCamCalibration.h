#ifndef SELFMULTICAMCALIBRATION_H
#define SELFMULTICAMCALIBRATION_H

#include "mono_vo/MonoVO.h"
#include "sparse_graph/SparseGraph.h"
#include "sparse_graph/SparseGraphViz.h"
#include "stereo_vo/StereoVO.h"

namespace px
{

class SelfMultiCamCalibration
{
public:
    SelfMultiCamCalibration(ros::NodeHandle& nh,
                            const CameraSystemPtr& cameraSystem,
                            const SparseGraphPtr& sparseGraph);

    bool init(const std::string& detectorType,
              const std::string& descriptorExtractorType,
              const std::string& descriptorMatcherType);

    bool processFrames(const ros::Time& stamp,
                       const std::vector<cv::Mat>& images,
                       const sensor_msgs::ImuConstPtr& imuMsg);

    bool run(const std::string& vocFilename,
             const std::string& chessboardDataDir,
             bool readIntermediateData = false);

    bool writePosesToTextFile(const std::string& filename) const;
    bool writeMapToVRMLFile(const std::string& filename) const;

private:
    void processSubGraph(const SparseGraphPtr& graph,
                         const boost::shared_ptr<SparseGraphViz>& graphViz,
                         const std::string& vocFilename,
                         const cv::Mat& matchingMask);
    bool runHandEyeCalibration(void);
    void runPG(const SparseGraphPtr& graph, const std::string& vocFilename,
               int minLoopCorrespondences2D3D,
               int nImageMatches,
               const cv::Mat& matchingMask = cv::Mat());
    void runBA(const SparseGraphPtr& graph,
               const boost::shared_ptr<SparseGraphViz>& graphViz) const;
    void runJointOptimization(const std::vector<std::string>& chessboardDataFilenames);
    bool runPoseIMUCalibration(void);

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > computeRelativeSystemPoses(const std::vector<FrameSetPtr>& frameSets) const;

    void reconstructScenePoint(Point3DFeaturePtr& scenePoint) const;

    void mergeMaps(void);

    void reprojErrorStats(const SparseGraphConstPtr& graph,
                          double& avgError, double& maxError,
                          double& avgScenePointDepth,
                          size_t& featureCount) const;

    enum
    {
        MONO_VO = 0,
        STEREO_VO = 1
    };

    ros::NodeHandle m_nh;
    CameraSystemPtr m_cameraSystem;
    std::vector<boost::shared_ptr<MonoVO> > m_mvo;
    std::vector<std::pair<int,int> > m_voMap;
    std::vector<SparseGraphPtr> m_subSparseGraphs;
    SparseGraphPtr m_sparseGraph;
    std::vector<boost::shared_ptr<StereoVO> > m_svo;
    std::vector<boost::shared_ptr<SparseGraphViz> > m_subsgv;
    SparseGraphViz m_sgv;
};

}

#endif
