#include "location_recognition/OrbLocationRecognition.h"

#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

namespace px
{

OrbLocationRecognition::OrbLocationRecognition()
{

}

bool
OrbLocationRecognition::createVocabulary(const std::vector<std::string>& imageFilenames,
                                         const std::string& detectorType,
                                         std::string& vocFilename) const
{
    cv::Ptr<cv::FeatureDetector> featureDetector =
        cv::FeatureDetector::create(detectorType);
    if (!featureDetector)
    {
        ROS_ERROR("Failed to create feature detector of type: %s",
                  detectorType.c_str());
        return false;
    }

    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor =
        cv::DescriptorExtractor::create("ORB");
    if (!descriptorExtractor)
    {
        ROS_ERROR("Failed to create ORB descriptor extractor.");
        return false;
    }

    std::vector<std::vector<DVision::ORB::bitset> > features;

    for (size_t i = 0; i < imageFilenames.size(); ++i)
    {
        cv::Mat image = cv::imread(imageFilenames.at(i), 0);
        if (image.empty())
        {
            continue;
        }

        std::vector<cv::KeyPoint> kpts;
        featureDetector->detect(image, kpts, cv::Mat());

        cv::Mat dtors;
        descriptorExtractor->compute(image, kpts, dtors);

        features.push_back(dtorsToFeatures(dtors));

        ROS_INFO("Extracted %lu features from image %lu: %s.",
                 kpts.size(), i + 1, imageFilenames.at(i).c_str());
    }

    return createVocabulary(features, vocFilename);
}

bool
OrbLocationRecognition::createVocabulary(const std::vector<std::vector<DVision::ORB::bitset> >& features,
                                         std::string& vocFilename) const
{
    OrbVocabulary voc;

    ROS_INFO("Generating vocabulary...");

    voc.create(features);

    ROS_INFO("Vocabulary information:");
    std::cout << voc << std::endl;

    voc.save(vocFilename);

    return true;
}

void
OrbLocationRecognition::setup(const std::string& vocFilename)
{
    m_frameTags.clear();
    m_frames.clear();

    OrbVocabulary voc(vocFilename);
    m_db.setVocabulary(voc);
}

void
OrbLocationRecognition::setup(const std::string& vocFilename,
                              const SparseGraphConstPtr& graph,
                              const cv::Mat& matchingMask)
{
    m_frameTags.clear();
    m_frames.clear();

    std::vector<bool> cameraFlags(matchingMask.rows);
    for (int i = 0; i < matchingMask.rows; ++i)
    {
        cameraFlags.at(i) = (cv::countNonZero(matchingMask.row(i)) > 0);
    }

    OrbVocabulary voc(vocFilename);
    m_db.setVocabulary(voc);

    // build vocabulary tree
    std::vector<std::vector<DVision::ORB::bitset> > features;

    for (size_t segmentId = 0; segmentId < graph->frameSetSegments().size(); ++segmentId)
    {
        const FrameSetSegment& segment = graph->frameSetSegment(segmentId);

        for (size_t frameSetId = 0; frameSetId < segment.size(); ++frameSetId)
        {
            const FrameSetPtr& frameSet = segment.at(frameSetId);

            for (size_t frameId = 0; frameId < frameSet->frames().size(); ++frameId)
            {
                const FramePtr& frame = frameSet->frames().at(frameId);

                if (frame.get() == 0)
                {
                    continue;
                }

                if (!cameraFlags.at(frame->cameraId()))
                {
                    continue;
                }

                FrameTag tag;
                tag.frameSetSegmentId = segmentId;
                tag.frameSetId = frameSetId;
                tag.frameId = frameId;

                m_frameTagMap.insert(std::make_pair(tag, m_frameTagMap.size()));
                m_frameTags.push_back(tag);
                m_frames.push_back(frame);

                features.push_back(frameToFeatures(frame));
            }
        }
    }

    for (size_t i = 0; i < features.size(); ++i)
    {
        m_db.add(features.at(i));
    }
}

bool
OrbLocationRecognition::detectSimilarLocations(const FrameConstPtr& frame, int k,
                                               std::vector<FrameConstPtr>& matches)
{
    std::vector<DVision::ORB::bitset> features = frameToFeatures(frame);

    FrameTag tag;
    tag.frameSetSegmentId = 0;
    tag.frameSetId = frame->frameSet()->seq();
    tag.frameId = frame->cameraId();

    m_dbMutex.lock();

    m_frameTagMap.insert(std::make_pair(tag, m_frameTagMap.size()));
    m_frameTags.push_back(tag);
    m_frames.push_back(frame);

    m_dbMutex.unlock();

    std::vector<FrameConstPtr> rawMatches;
    knnMatch(frame, k, rawMatches);

    matches.reserve(rawMatches.size());
    for (size_t i = 0; i < rawMatches.size(); ++i)
    {
        FrameConstPtr& match = rawMatches.at(i);

        if (std::abs(tag.frameSetId - match->frameSet()->seq()) < 10)
        {
            continue;
        }

        matches.push_back(match);
    }

    m_dbMutex.lock();

    m_db.add(features);

    m_dbMutex.unlock();

    return !matches.empty();
}

void
OrbLocationRecognition::knnMatch(const FrameConstPtr& frame, int k,
                                 std::vector<FrameTag>& matches) const
{
    DBoW2::QueryResults ret;
    m_db.query(frameToFeatures(frame), ret, k);

    matches.clear();
    for (size_t i = 0; i < ret.size(); ++i)
    {
        FrameTag tag = m_frameTags.at(ret.at(i).Id);

        matches.push_back(tag);
    }
}

void
OrbLocationRecognition::knnMatch(const FrameConstPtr& frame, int k,
                                 const std::vector<FrameTag>& validMatches,
                                 std::vector<FrameTag>& matches) const
{
    std::vector<bool> mask(m_frameTagMap.size(), false);
    for (size_t i = 0; i < validMatches.size(); ++i)
    {
        boost::unordered_map<FrameTag, size_t>::const_iterator it = m_frameTagMap.find(validMatches.at(i));
        if (it == m_frameTagMap.end())
        {
            continue;
        }

        mask.at(it->second) = true;
    }

    DBoW2::QueryResults ret;
    m_db.query(frameToFeatures(frame), ret, 0);

    matches.clear();
    for (size_t i = 0; i < ret.size(); ++i)
    {
        int id = ret.at(i).Id;

        if (!mask.at(id))
        {
            continue;
        }

        FrameTag tag = m_frameTags.at(id);

        matches.push_back(tag);

        if (matches.size() == k)
        {
            return;
        }
    }
}

void
OrbLocationRecognition::knnMatch(const FrameConstPtr& frame, int k,
                                 std::vector<FrameConstPtr>& matches) const
{
    DBoW2::QueryResults ret;
    m_db.query(frameToFeatures(frame), ret, k);

    matches.clear();
    for (size_t i = 0; i < ret.size(); ++i)
    {
        const FrameConstPtr& frame = m_frames.at(ret.at(i).Id);

        matches.push_back(frame);
    }
}

DVision::ORB::bitset
OrbLocationRecognition::dtorToFeature(const cv::Mat& dtor) const
{
    int nBits = dtor.cols * 8;

    DVision::ORB::bitset word(nBits);
    int shift = (dtor.cols - 1) * 8;
    for (int j = 0; j < dtor.cols; ++j)
    {
        DVision::ORB::bitset mask(nBits, dtor.at<unsigned char>(0,j));

        word |= (mask << shift);
        shift -= 8;
    }

    return word;
}

std::vector<DVision::ORB::bitset>
OrbLocationRecognition::dtorsToFeatures(const cv::Mat& dtors) const
{
    std::vector<DVision::ORB::bitset> bow;

    for (int i = 0; i < dtors.rows; ++i)
    {
        bow.push_back(dtorToFeature(dtors.row(i)));
    }

    return bow;
}

std::vector<DVision::ORB::bitset>
OrbLocationRecognition::frameToFeatures(const FrameConstPtr& frame) const
{
    std::vector<DVision::ORB::bitset> bow;

    const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();
    for (std::vector<Point2DFeaturePtr>::const_iterator it = features2D.begin();
            it != features2D.end(); ++it)
    {
        const Point2DFeatureConstPtr& feature2D = (*it);
        const cv::Mat& dtor = feature2D->descriptor();

        bow.push_back(dtorToFeature((*it)->descriptor()));
    }

    return bow;
}

}
