#ifndef SPARSEGRAPH_H
#define SPARSEGRAPH_H

#include <boost/unordered_map.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sensor_msgs/Imu.h>

#include "sparse_graph/Pose.h"

namespace px
{

struct FrameTag
{
    int frameSetSegmentId;
    int frameSetId;
    int frameId;
};

bool operator==(const FrameTag& t1, const FrameTag& t2);
std::size_t hash_value(const FrameTag& t);

class Frame;
class FrameSet;
class Point2DFeature;
class Point3DFeature;

typedef boost::shared_ptr<FrameSet> FrameSetPtr;
typedef boost::shared_ptr<const FrameSet> FrameSetConstPtr;

typedef boost::shared_ptr<Point2DFeature> Point2DFeaturePtr;
typedef boost::shared_ptr<const Point2DFeature> Point2DFeatureConstPtr;

typedef boost::shared_ptr<Point3DFeature> Point3DFeaturePtr;
typedef boost::shared_ptr<const Point3DFeature> Point3DFeatureConstPtr;

class LoopClosureEdge
{
public:
    LoopClosureEdge();

    Frame*& inFrame(void);
    Frame* inFrame(void) const;

    std::vector<size_t>& inMatchIds(void);
    const std::vector<size_t>& inMatchIds(void) const;

    std::vector<size_t>& outMatchIds(void);
    const std::vector<size_t>& outMatchIds(void) const;

    Transform& measurement(void);
    const Transform& measurement(void) const;

private:
    Frame* m_inFrame;

    std::vector<size_t> m_inMatchIds;
    std::vector<size_t> m_outMatchIds;

    Transform m_measurement;
};

class Frame
{
public:
    Frame();

    PosePtr& cameraPose(void);
    PoseConstPtr cameraPose(void) const;

    int& cameraId(void);
    int cameraId(void) const;

    FrameSet*& frameSet(void);
    FrameSet* frameSet(void) const;

    std::vector<LoopClosureEdge>& loopClosureEdges(void);
    const std::vector<LoopClosureEdge>& loopClosureEdges(void) const;

    std::vector<Point2DFeaturePtr>& features2D(void);
    const std::vector<Point2DFeaturePtr>& features2D(void) const;

    cv::Mat& image(void);
    const cv::Mat& image(void) const;

private:
    PosePtr m_cameraPose;
    int m_cameraId;

    FrameSet* m_frameSet;
    std::vector<LoopClosureEdge> m_loopClosureEdges;

    std::vector<Point2DFeaturePtr> m_features2D;

    cv::Mat m_image;
};

typedef boost::shared_ptr<Frame> FramePtr;
typedef boost::shared_ptr<const Frame> FrameConstPtr;

class Point2DFeature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Point2DFeature();

    cv::Mat& descriptor(void);
    const cv::Mat& descriptor(void) const;

    cv::KeyPoint& keypoint(void);
    const cv::KeyPoint& keypoint(void) const;

    Eigen::Vector3d& ray(void);
    const Eigen::Vector3d& ray(void) const;

    unsigned int& index(void);
    unsigned int index(void) const;

    Point2DFeature*& prevMatch(void);
    Point2DFeature* prevMatch(void) const;

    std::vector<Point2DFeature*>& prevMatches(void);
    const std::vector<Point2DFeature*>& prevMatches(void) const;

    int& bestPrevMatchId(void);
    int bestPrevMatchId(void) const;

    Point2DFeature*& match(void);
    Point2DFeature* match(void) const;

    std::vector<Point2DFeature*>& matches(void);
    const std::vector<Point2DFeature*>& matches(void) const;

    int& bestMatchId(void);
    int bestMatchId(void) const;

    Point2DFeature*& nextMatch(void);
    Point2DFeature* nextMatch(void) const;

    std::vector<Point2DFeature*>& nextMatches(void);
    const std::vector<Point2DFeature*>& nextMatches(void) const;

    int& bestNextMatchId(void);
    int bestNextMatchId(void) const;

    Point3DFeaturePtr& feature3D(void);
    Point3DFeatureConstPtr feature3D(void) const;

    Frame*& frame(void);
    Frame* frame(void) const;

protected:
    cv::Mat m_dtor;
    cv::KeyPoint m_keypoint;
    Eigen::Vector3d m_ray;

    unsigned int m_index;

private:
    std::vector<Point2DFeature*> m_prevMatches;
    std::vector<Point2DFeature*> m_matches;
    std::vector<Point2DFeature*> m_nextMatches;
    int m_bestPrevMatchId;
    int m_bestMatchId;
    int m_bestNextMatchId;
    Point3DFeaturePtr m_feature3D;
    Frame* m_frame;
};

class Point3DFeature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum
    {
        OBSERVED_BY_CAMERA_MULTIPLE_TIMES = 0x1,
        OBSERVED_BY_MULTIPLE_CAMERAS = 0x2,
        OBSERVED_IN_CURRENT_FRAME = 0x4,
        TRIANGULATED_IN_CURRENT_FRAME = 0x8
    };

    Point3DFeature();

    Eigen::Vector3d& point(void);
    const Eigen::Vector3d& point(void) const;

    double* pointData(void);
    const double* const pointData(void) const;

    Eigen::Matrix3d& pointCovariance(void);
    const Eigen::Matrix3d& pointCovariance(void) const;

    double* pointCovarianceData(void);
    const double* const pointCovarianceData(void) const;

    Eigen::Vector3d& pointFromStereo(void);
    const Eigen::Vector3d& pointFromStereo(void) const;

    int& attributes(void);
    int attributes(void) const;

    double& weight(void);
    double weight(void) const;

    std::vector<Point2DFeature*>& features2D(void);
    const std::vector<Point2DFeature*>& features2D(void) const;

private:
    Eigen::Vector3d m_point;
    Eigen::Matrix3d m_pointCovariance;
    Eigen::Vector3d m_pointFromStereo;
    int m_attributes;
    double m_weight;
    std::vector<Point2DFeature*> m_features2D;
};

class FrameSet
{
public:
    FrameSet();
    ~FrameSet();

    size_t& seq(void);
    size_t seq(void) const;

    FramePtr& frame(int cameraId);
    FrameConstPtr frame(int cameraId) const;

    std::vector<FramePtr>& frames(void);
    const std::vector<FramePtr>& frames(void) const;

    FrameSet*& prevFrameSet(void);
    FrameSet* prevFrameSet(void) const;

    FrameSet*& nextFrameSet(void);
    FrameSet* nextFrameSet(void) const;

    Transform& prevTransformMeasurement(void);
    const Transform& prevTransformMeasurement(void) const;

    Transform& nextTransformMeasurement(void);
    const Transform& nextTransformMeasurement(void) const;

    PosePtr& systemPose(void);
    PoseConstPtr systemPose(void) const;

    sensor_msgs::ImuConstPtr& imuMeasurement(void);
    const sensor_msgs::ImuConstPtr& imuMeasurement(void) const;

    PosePtr& groundTruthMeasurement(void);
    PoseConstPtr groundTruthMeasurement(void) const;

private:
    size_t m_seq;

    std::vector<FramePtr> m_frames;

    FrameSet* m_prevFrameSet;
    FrameSet* m_nextFrameSet;

    Transform m_prevTransformMeasurement;
    Transform m_nextTransformMeasurement;

    PosePtr m_systemPose;
    sensor_msgs::ImuConstPtr m_imuMeasurement;
    PosePtr m_groundTruthMeasurement;
};

typedef std::vector<FrameSetPtr> FrameSetSegment;

class SparseGraph
{
public:
    SparseGraph();

    FrameSetSegment& frameSetSegment(int segmentId);
    const FrameSetSegment& frameSetSegment(int segmentId) const;

    std::vector<FrameSetSegment>& frameSetSegments(void);
    const std::vector<FrameSetSegment>& frameSetSegments(void) const;

    size_t scenePointCount(void) const;

    bool readFromBinaryFile(const std::string& filename);
    void writeToBinaryFile(const std::string& filename) const;

private:
    template<typename T>
    void readData(std::ifstream& ifs, T& data) const;

    template<typename T>
    void writeData(std::ofstream& ofs, T data) const;

    std::vector<FrameSetSegment> m_frameSetSegments;
};

typedef boost::shared_ptr<SparseGraph> SparseGraphPtr;
typedef boost::shared_ptr<const SparseGraph> SparseGraphConstPtr;

}

#endif
