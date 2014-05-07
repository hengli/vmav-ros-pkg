#ifndef STEREOSM_H
#define STEREOSM_H

#include "sparse_graph/SparseGraph.h"
#include "sparse_graph/SparseGraphViz.h"
#include "stereo_vo/StereoVO.h"

namespace px
{

class StereoSM
{
public:
    StereoSM(ros::NodeHandle& nh,
             const CameraSystemConstPtr& cameraSystem,
             const SparseGraphPtr& sparseGraph);

    bool init(const std::string& detectorType,
              const std::string& descriptorExtractorType,
              const std::string& descriptorMatcherType);

    bool readFrames(const ros::Time& stamp,
                    const cv::Mat& imageL, const cv::Mat& imageR);

    bool processFrames(void);

    void runPG(const std::string& vocFilename);
    void runBA(void);

private:
    void reconstructScenePoint(Point3DFeaturePtr& scenePoint) const;

    void reprojErrorStats(double& avgError, double& maxError,
                          size_t& featureCount) const;

    ros::NodeHandle m_nh;
    CameraSystemConstPtr m_cameraSystem;
    SparseGraphPtr m_sparseGraph;
    StereoVO m_svo;
    SparseGraphViz m_sgv;
};

}

#endif
