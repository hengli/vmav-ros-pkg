#ifndef POSEGRAPH_H
#define POSEGRAPH_H

#include <vector>

#include "camera_systems/CameraSystem.h"
#include "pose_graph/DirectedEdge.h"
#include "sparse_graph/SparseGraph.h"

namespace px
{

// forward declaration
class OrbLocationRecognition;

class PoseGraph
{
public:
    PoseGraph(const CameraSystemConstPtr& cameraSystem,
              const SparseGraphPtr& sparseGraph,
              const cv::Mat& matchingMask,
              int minLoopCorrespondences2D3D,
              int nImageMatches);

    void setVerbose(bool onoff);

    void buildEdges(const std::string& vocFilename);

    void optimize(bool useRobustOptimization);

    std::vector<std::pair<PoseConstWPtr, PoseConstWPtr> > getLoopClosureEdges(bool correct) const;
    std::vector<std::pair<PoseConstWPtr, PoseConstWPtr> > getVOEdges(void) const;
    std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > getCorrespondences2D3D(void) const;

private:
    enum EdgeSwitchState
    {
        DISABLED,
        OFF,
        ON,
    };

    typedef DirectedEdge<Transform, Pose> Edge;

    void getDescriptorMat(const FrameConstPtr& frame, cv::Mat& dmat) const;

    std::vector<Edge, Eigen::aligned_allocator<Edge> > findVOEdges(void) const;
    void findLoopClosures(const std::string& vocFilename,
                          std::vector<Edge, Eigen::aligned_allocator<Edge> >& loopClosureEdges,
                          std::vector<std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > >& correspondences2D3D) const;

    void findLoopClosuresHelper(FrameTag frameTagQuery,
                                const boost::shared_ptr<const OrbLocationRecognition>& locRec,
                                const std::vector<FrameTag>& validMatchingFrameTags,
                                PoseGraph::Edge& edge,
                                std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> >& correspondences2D3D) const;

    std::vector<FrameTag> computeValidMatchingFrameTags(FrameTag queryTag) const;

    bool iterateEM(bool useRobustOptimization);
    void classifySwitches(void);

    void solveP3PRansac(const FrameConstPtr& frame1,
                        const FrameConstPtr& frame2,
                        const std::vector<cv::DMatch>& matches,
                        Eigen::Matrix4d& H,
                        std::vector<cv::DMatch>& inliers) const;

    CameraSystemConstPtr m_cameraSystem;
    SparseGraphPtr m_sparseGraph;
    std::vector<Edge, Eigen::aligned_allocator<Edge> > m_voEdges;
    std::vector<Edge, Eigen::aligned_allocator<Edge> > m_loopClosureEdges;
    std::vector<EdgeSwitchState> m_loopClosureEdgeSwitches;
    std::vector<std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > > m_correspondences2D3D;

    cv::Ptr<cv::DescriptorMatcher> m_descriptorMatcher;

    const double k_lossWidth;
    const cv::Mat k_matchingMask;
    const int k_minLoopCorrespondences2D3D;
    const int k_nImageMatches;
    const double k_sphericalErrorThresh;

    bool m_verbose;
};

typedef boost::shared_ptr<PoseGraph> PoseGraphPtr;
typedef boost::shared_ptr<const PoseGraph> PoseGraphConstPtr;

}

#endif
