#ifndef ORBLOCATIONRECOGNITION_H
#define ORBLOCATIONRECOGNITION_H

#include <boost/thread/mutex.hpp>
#include <boost/unordered_map.hpp>

#include "dbow2/DBoW2.h"
#include "dloopdetector/DLoopDetector.h"
#include "sparse_graph/SparseGraph.h"

namespace px
{

class OrbLocationRecognition
{
public:
    OrbLocationRecognition();

    bool createVocabulary(const std::vector<std::string>& imageFilenames,
                          const std::string& detectorType,
                          std::string& vocFilename) const;
    bool createVocabulary(const std::vector<std::vector<DVision::ORB::bitset> >& features,
                          std::string& vocFilename) const;

    void setup(const std::string& vocFilename);
    void setup(const std::string& vocFilename, const SparseGraphConstPtr& graph,
               const cv::Mat& matchingMask = cv::Mat());

    bool detectSimilarLocations(const FrameConstPtr& frame, int k,
                                std::vector<FrameConstPtr>& matches);

    void knnMatch(const FrameConstPtr& frame, int k, std::vector<FrameTag>& matches) const;
    void knnMatch(const FrameConstPtr& frame, int k, const std::vector<FrameTag>& validMatches,
                  std::vector<FrameTag>& matches) const;
    void knnMatch(const FrameConstPtr& frame, int k, std::vector<FrameConstPtr>& matches) const;

private:
    DVision::ORB::bitset dtorToFeature(const cv::Mat& dtor) const;
    std::vector<DVision::ORB::bitset> dtorsToFeatures(const cv::Mat& dtors) const;
    std::vector<DVision::ORB::bitset> frameToFeatures(const FrameConstPtr& frame) const;

    OrbDatabase m_db;
    boost::mutex m_dbMutex;

    boost::unordered_map<FrameTag, size_t> m_frameTagMap;
    std::vector<FrameTag> m_frameTags;
    std::vector<FrameConstPtr> m_frames;
};

}

#endif
