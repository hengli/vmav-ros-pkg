#include "pose_graph/PoseGraph.h"

#include <boost/thread.hpp>
#include <boost/unordered_set.hpp>
#include <ceres/ceres.h>
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>

#include "cauldron/EigenQuaternionParameterization.h"
#include "location_recognition/OrbLocationRecognition.h"
#include "pose_estimation/P3P.h"
#include "PoseGraphError.h"

namespace px
{

PoseGraph::PoseGraph(const CameraSystemConstPtr& cameraSystem,
                     const SparseGraphPtr& sparseGraph,
                     const cv::Mat& matchingMask,
                     int minLoopCorrespondences2D3D,
                     int nImageMatches)
 : m_cameraSystem(cameraSystem)
 , m_sparseGraph(sparseGraph)
 , k_lossWidth(0.01)
 , k_matchingMask(matchingMask)
 , k_minLoopCorrespondences2D3D(minLoopCorrespondences2D3D)
 , k_nImageMatches(nImageMatches)
 , k_sphericalErrorThresh(0.999976)
 , m_verbose(false)
{
    m_descriptorMatcher = cv::Ptr<cv::DescriptorMatcher>(new cv::BFMatcher(cv::NORM_HAMMING, true));
}

void
PoseGraph::setVerbose(bool onoff)
{
    m_verbose = onoff;
}

void
PoseGraph::buildEdges(const std::string& vocFilename)
{
    if (m_verbose)
    {
        ROS_INFO("Building VO edges...");
    }

    m_voEdges = findVOEdges();

    if (m_verbose)
    {
        ROS_INFO("Built %lu VO edges.", m_voEdges.size());
    }

    if (m_verbose)
    {
        ROS_INFO("Building loop closure edges...");
    }

    findLoopClosures(vocFilename, m_loopClosureEdges, m_correspondences2D3D);

    if (m_verbose)
    {
        ROS_INFO("Built %lu loop closure edges.", m_loopClosureEdges.size());
    }

    m_loopClosureEdgeSwitches.assign(m_loopClosureEdges.size(), ON);
}

void
PoseGraph::optimize(bool useRobustOptimization)
{
    // This implementation is based on the following paper:
    // G.H. Lee, F. Fraundorfer, and M. Pollefeys,
    // Robust Pose-Graph Loop-Closures with Expectation-Maximization,
    // In International Conference on Intelligent Robots and Systems, 2013.
    if (useRobustOptimization)
    {
        for (int i = 0; i < 20; ++i)
        {
            if (!iterateEM(true))
            {
                return;
            }
        }
    }
    else
    {
        iterateEM(false);
    }
}

std::vector<std::pair<PoseConstWPtr, PoseConstWPtr> >
PoseGraph::getLoopClosureEdges(bool correct) const
{
    std::vector<std::pair<PoseConstWPtr, PoseConstWPtr> > loopClosureEdges;

    for (size_t i = 0; i < m_loopClosureEdges.size(); ++i)
    {
        const Edge& edge = m_loopClosureEdges.at(i);

        if (m_loopClosureEdgeSwitches.at(i) == ON && !correct)
        {
            continue;
        }
        if (m_loopClosureEdgeSwitches.at(i) != ON && correct)
        {
            continue;
        }
        
        loopClosureEdges.push_back(std::make_pair(edge.inVertex(), edge.outVertex()));
    }

    return loopClosureEdges;
}

std::vector<std::pair<PoseConstWPtr, PoseConstWPtr> >
PoseGraph::getVOEdges(void) const
{
    std::vector<std::pair<PoseConstWPtr, PoseConstWPtr> > voEdges;

    for (size_t i = 0; i < m_voEdges.size(); ++i)
    {
        const Edge& edge = m_voEdges.at(i);

        voEdges.push_back(std::make_pair(edge.inVertex(), edge.outVertex()));
    }

    return voEdges;
}

std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> >
PoseGraph::getCorrespondences2D3D(void) const
{
    // return 2D-3D correspondences from switched-on loop closure edges
    std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > correspondences2D3D;

    for (size_t i = 0; i < m_correspondences2D3D.size(); ++i)
    {
        if (m_loopClosureEdgeSwitches.at(i) != ON)
        {
            continue;
        }

        correspondences2D3D.insert(correspondences2D3D.end(),
                                   m_correspondences2D3D.at(i).begin(),
                                   m_correspondences2D3D.at(i).end());
    }

    return correspondences2D3D;
}

void
PoseGraph::getDescriptorMat(const FrameConstPtr& frame, cv::Mat& dmat) const
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

std::vector<PoseGraph::Edge, Eigen::aligned_allocator<PoseGraph::Edge> >
PoseGraph::findVOEdges(void) const
{
    std::vector<PoseGraph::Edge, Eigen::aligned_allocator<PoseGraph::Edge> > edges;

    for (size_t i = 0; i < m_sparseGraph->frameSetSegments().size(); ++i)
    {
        const FrameSetSegment& segment = m_sparseGraph->frameSetSegment(i);

        if (segment.size() <= 1)
        {
            continue;
        }

        for (size_t j = 0; j < segment.size() - 1; ++j)
        {
            edges.push_back(Edge(segment.at(j)->systemPose(),
                                 segment.at(j + 1)->systemPose()));

            edges.back().type() = EDGE_VO;

            Eigen::Matrix4d H_01 = segment.at(j + 1)->systemPose()->toMatrix() *
                                   segment.at(j)->systemPose()->toMatrix().inverse();

            edges.back().property().rotation() = Eigen::Quaterniond(H_01.block<3,3>(0,0));
            edges.back().property().translation() = H_01.block<3,1>(0,3);
            edges.back().weight().assign(6, 1.0);
        }
    }

    return edges;
}

void
PoseGraph::findLoopClosures(const std::string& vocFilename,
                            std::vector<PoseGraph::Edge, Eigen::aligned_allocator<PoseGraph::Edge> >& loopClosureEdges,
                            std::vector<std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > >& correspondences2D3D) const
{
    std::vector<bool> cameraFlags(k_matchingMask.rows);
    for (int i = 0; i < k_matchingMask.rows; ++i)
    {
        cameraFlags.at(i) = (cv::countNonZero(k_matchingMask.row(i)) > 0);
    }

    boost::shared_ptr<OrbLocationRecognition> locRec = boost::make_shared<OrbLocationRecognition>();
    locRec->setup(vocFilename, m_sparseGraph, k_matchingMask);

    for (int i = 0; i < m_sparseGraph->frameSetSegments().size(); ++i)
    {
        const FrameSetSegment& segment = m_sparseGraph->frameSetSegment(i);

        for (int j = 0; j < segment.size(); ++j)
        {
            const FrameSetPtr& frameSet = segment.at(j);

            std::vector<boost::shared_ptr<boost::thread> > threads(frameSet->frames().size());
            std::vector<PoseGraph::Edge> edges(frameSet->frames().size());
            std::vector<std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > > corr2D3D(frameSet->frames().size());

            for (size_t k = 0; k < frameSet->frames().size(); ++k)
            {
                const FramePtr& frame = frameSet->frames().at(k);

                if (!frame)
                {
                    continue;
                }

                if (!cameraFlags.at(frame->cameraId()))
                {
                    continue;
                }

                FrameTag frameTag;
                frameTag.frameSetSegmentId = i;
                frameTag.frameSetId = j;
                frameTag.frameId = k;

                std::vector<FrameTag> validMatchingFrameTags = computeValidMatchingFrameTags(frameTag);

                threads.at(k) = boost::make_shared<boost::thread>(boost::bind(&PoseGraph::findLoopClosuresHelper, this,
                                                                              frameTag,
                                                                              boost::cref(locRec),
                                                                              validMatchingFrameTags,
                                                                              boost::ref(edges.at(k)),
                                                                              boost::ref(corr2D3D.at(k))));
            }

            for (int k = 0; k < frameSet->frames().size(); ++k)
            {
                if (!threads.at(k))
                {
                    continue;
                }
                threads.at(k)->join();

                if (!corr2D3D.at(k).empty())
                {
                    loopClosureEdges.push_back(edges.at(k));
                    correspondences2D3D.push_back(corr2D3D.at(k));
                }
            }
        }
    }
}

void
PoseGraph::findLoopClosuresHelper(FrameTag frameTagQuery,
                                  const boost::shared_ptr<const OrbLocationRecognition>& locRec,
                                  const std::vector<FrameTag>& validMatchingFrameTags,
                                  PoseGraph::Edge& edge,
                                  std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> >& correspondences2D3D) const
{
    FramePtr& frameQuery = m_sparseGraph->frameSetSegment(frameTagQuery.frameSetSegmentId).at(frameTagQuery.frameSetId)->frames().at(frameTagQuery.frameId);

    if (!frameQuery)
    {
        return;
    }

    // find closest matching images
    std::vector<FrameTag> frameTags;
    locRec->knnMatch(frameQuery, k_nImageMatches, validMatchingFrameTags, frameTags);

    std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > corr2D3DBest;
    Transform transformBest;
    FramePtr frameBest;
    FrameTag frameTagBest;

    for (size_t i = 0; i < frameTags.size(); ++i)
    {
        FrameTag frameTag = frameTags.at(i);

        const FramePtr& frame = m_sparseGraph->frameSetSegment(frameTag.frameSetSegmentId).at(frameTag.frameSetId)->frames().at(frameTag.frameId);

        // find 2D-3D correspondences
        cv::Mat dtors, dtorsQuery;
        getDescriptorMat(frame, dtors);
        getDescriptorMat(frameQuery, dtorsQuery);

        std::vector<cv::DMatch> rawMatches;
        m_descriptorMatcher->match(dtors, dtorsQuery, rawMatches);

        if (rawMatches.size() < k_minLoopCorrespondences2D3D)
        {
            continue;
        }

        // find camera pose from P3P RANSAC
        Eigen::Matrix4d systemPose;
        std::vector<cv::DMatch> matches;
        solveP3PRansac(frame, frameQuery, rawMatches, systemPose, matches);

        int nInliers = matches.size();

        if (nInliers < k_minLoopCorrespondences2D3D)
        {
            continue;
        }

        if (nInliers > corr2D3DBest.size())
        {
            frameBest = frame;
            frameTagBest = frameTag;

            // compute loop closure constraint;
            Eigen::Matrix4d H_01 = frameBest->frameSet()->systemPose()->toMatrix() * systemPose.inverse();

            transformBest.rotation() = Eigen::Quaterniond(H_01.block<3,3>(0,0));
            transformBest.translation() = H_01.block<3,1>(0,3);

            // find inlier 2D-3D correspondences
            corr2D3DBest.clear();

            for (size_t j = 0; j < matches.size(); ++j)
            {
                const cv::DMatch& match = matches.at(j);

                corr2D3DBest.push_back(std::make_pair(frameQuery->features2D().at(match.trainIdx),
                                                      frame->features2D().at(match.queryIdx)->feature3D()));
            }
        }
    }

    if (!corr2D3DBest.empty())
    {
        edge.inVertex() = frameQuery->frameSet()->systemPose();
        edge.outVertex() = frameBest->frameSet()->systemPose();
        edge.type() = EDGE_LOOP_CLOSURE;
        edge.property() = transformBest;
        edge.weight().assign(6, 1.0);

        correspondences2D3D = corr2D3DBest;

        if (m_verbose)
        {
            ROS_INFO("Image match: %d,%d,%d -> %d,%d,%d with %lu 2D-3D correspondences.",
                     frameTagQuery.frameSetSegmentId,
                     frameTagQuery.frameSetId,
                     frameTagQuery.frameId,
                     frameTagBest.frameSetSegmentId,
                     frameTagBest.frameSetId,
                     frameTagBest.frameId,
                     correspondences2D3D.size());
        }
    }
}

std::vector<FrameTag>
PoseGraph::computeValidMatchingFrameTags(FrameTag queryTag) const
{
    std::vector<FrameTag> matchingTags;

    for (int i = 0; i < m_sparseGraph->frameSetSegments().size(); ++i)
    {
        const FrameSetSegment& segment = m_sparseGraph->frameSetSegment(i);

        for (int j = 0; j < segment.size(); ++j)
        {
            const FrameSetPtr& frameSet = segment.at(j);

            for (size_t k = 0; k < frameSet->frames().size(); ++k)
            {
                const FramePtr& frame = frameSet->frames().at(k);

                if (!frame)
                {
                    continue;
                }

                FrameTag trainTag;
                trainTag.frameSetSegmentId = i;
                trainTag.frameSetId = j;
                trainTag.frameId = k;

                if (k_matchingMask.at<unsigned char>(queryTag.frameId, trainTag.frameId) == 0)
                {
                    continue;
                }

                if (queryTag.frameSetSegmentId == trainTag.frameSetSegmentId &&
                    std::abs(queryTag.frameSetId - trainTag.frameSetId) < 20)
                {
                    continue;
                }

                matchingTags.push_back(trainTag);
            }
        }
    }

    return matchingTags;
}

bool
PoseGraph::iterateEM(bool useRobustOptimization)
{
    ceres::Problem problem;

    // VO edges
    boost::unordered_set<PosePtr> poses;
    for (size_t i = 0; i < m_voEdges.size(); ++i)
    {
        Edge& edge = m_voEdges.at(i);

        ceres::CostFunction* costFunction =
            new ceres::AutoDiffCostFunction<PoseGraphError, 6, 4, 3, 4, 3>(
                new PoseGraphError(edge.property(), edge.weight()));

        PosePtr pose1, pose2;
        pose1 = edge.inVertex().lock();
        pose2 = edge.outVertex().lock();

        problem.AddResidualBlock(costFunction, NULL,
                                 pose1->rotationData(), pose1->translationData(),
                                 pose2->rotationData(), pose2->translationData());

        poses.insert(pose1);
        poses.insert(pose2);

        if (i == 0)
        {
            problem.SetParameterBlockConstant(pose1->rotationData());
            problem.SetParameterBlockConstant(pose1->translationData());
        }
    }

    for (boost::unordered_set<PosePtr>::iterator it = poses.begin();
         it != poses.end(); ++it)
    {
        ceres::LocalParameterization* quaternionParameterization =
            new EigenQuaternionParameterization;

        problem.SetParameterization((*it)->rotationData(),
                                    quaternionParameterization);
    }

    // loop closure edges
    for (size_t i = 0; i < m_loopClosureEdges.size(); ++i)
    {
        if (m_loopClosureEdgeSwitches.at(i) != ON)
        {
            continue;
        }

        Edge& edge = m_loopClosureEdges.at(i);

        ceres::CostFunction* costFunction =
            new ceres::AutoDiffCostFunction<PoseGraphError, 6, 4, 3, 4, 3>(
                new PoseGraphError(edge.property(), edge.weight()));

        PosePtr pose1, pose2;
        pose1 = edge.inVertex().lock();
        pose2 = edge.outVertex().lock();

        ceres::LossFunction* lossFunction = 0;
        if (useRobustOptimization)
        {
            lossFunction = new ceres::CauchyLoss(k_lossWidth);
        }

        problem.AddResidualBlock(costFunction, lossFunction,
                                 pose1->rotationData(), pose1->translationData(),
                                 pose2->rotationData(), pose2->translationData());
    }

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (m_verbose)
    {
        std::cout << summary.BriefReport() << std::endl;
    }

    int nIterations = summary.num_successful_steps + summary.num_unsuccessful_steps;

    if (nIterations != 0 && useRobustOptimization)
    {
        classifySwitches();
    }

    return (nIterations != 0);
}

void
PoseGraph::classifySwitches(void)
{
    int nSwitchesOn = 0;
    std::map<double, EdgeSwitchState*> edgeSwitchMap;

    for (size_t i = 0; i < m_loopClosureEdges.size(); ++i)
    {
        if (m_loopClosureEdgeSwitches.at(i) == DISABLED)
        {
            continue;
        }

        Edge& edge = m_loopClosureEdges.at(i);

        PosePtr pose1, pose2;
        pose1 = edge.inVertex().lock();
        pose2 = edge.outVertex().lock();

        Eigen::Matrix4d H_01 = pose2->toMatrix() * pose1->toMatrix().inverse();
        Eigen::Matrix4d H_01_meas = edge.property().toMatrix();

        // compute error
        Eigen::Matrix4d H_err = H_01_meas.inverse() * H_01;

        Eigen::Matrix3d R_err = H_err.block<3,3>(0,0);
        double r, p, y;
        mat2RPY(R_err, r, p, y);

        Eigen::Matrix<double,6,1> vec_err;
        vec_err.topRows(3) << r, p, y;
        vec_err.bottomRows(3) = H_err.block<3,1>(0,3);

        // compute weight
        double l2 = k_lossWidth * k_lossWidth;
        double w = l2 / (l2 + vec_err.transpose() * vec_err);

        if (w < 0.001 * (k_lossWidth / 0.01))
        {
            m_loopClosureEdgeSwitches.at(i) = OFF;
        }
        else
        {
            m_loopClosureEdgeSwitches.at(i) = ON;

            ++nSwitchesOn;
        }

        edgeSwitchMap.insert(std::make_pair(w, &(m_loopClosureEdgeSwitches.at(i))));
    }

    // disable edge switches corresponding to the first 10% of edges
    // sorted by increasing weight
    size_t nMaxSwitchesDisabled = edgeSwitchMap.size() / 10;
    int nSwitchesDisabled = 0;
    for (std::map<double, EdgeSwitchState*>::iterator it = edgeSwitchMap.begin();
         it != edgeSwitchMap.end(); ++it)
    {
        if (*(it->second) == ON)
        {
            break;
        }

        *(it->second) = DISABLED;

        ++nSwitchesDisabled;
        if (nSwitchesDisabled >= nMaxSwitchesDisabled)
        {
            break;
        }
    }

    if (m_verbose)
    {
        ROS_INFO("# switches turned on: %d/%lu", nSwitchesOn, edgeSwitchMap.size());
        ROS_INFO("# switches disabled: %d", nSwitchesDisabled);
    }
}

void
PoseGraph::solveP3PRansac(const FrameConstPtr& frame1,
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
            Eigen::Matrix4d H_inv = solutions.at(j).inverse();

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
