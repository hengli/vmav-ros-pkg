#ifndef STEREOCAMERACALIBRATION_H
#define STEREOCAMERACALIBRATION_H

#include "camera_calibration/CameraCalibration.h"

namespace px
{

class StereoCameraCalibration
{
public:
    StereoCameraCalibration();

    StereoCameraCalibration(Camera::ModelType modelType,
                            const std::string& cameraLeftName,
                            const std::string& cameraRightName,
                            const cv::Size& imageSize,
                            const cv::Size& boardSize,
                            float squareSize);

    void clear(void);

    void addChessboardData(const std::vector<cv::Point2f>& cornersLeft,
                           const std::vector<cv::Point2f>& cornersRight);

    bool calibrate(void);

    int sampleCount(void) const;
    const std::vector<std::vector<cv::Point2f> >& imagePointsLeft(void) const;
    const std::vector<std::vector<cv::Point2f> >& imagePointsRight(void) const;
    std::vector<std::vector<cv::Point3f> >& scenePoints(void);
    const std::vector<std::vector<cv::Point3f> >& scenePoints(void) const;

    CameraPtr& cameraLeft(void);
    const CameraConstPtr cameraLeft(void) const;

    CameraPtr& cameraRight(void);
    const CameraConstPtr cameraRight(void) const;

    cv::Mat& cameraPosesLeft(void);
    const cv::Mat& cameraPosesLeft(void) const;
    cv::Mat& cameraPosesRight(void);
    const cv::Mat& cameraPosesRight(void) const;

    void drawResults(std::vector<cv::Mat>& imagesLeft,
                     std::vector<cv::Mat>& imagesRight) const;

    void writeParameters(const std::string& directory) const;
    void writeParameters(px_comm::CameraInfoPtr& cameraInfoL,
                         px_comm::CameraInfoPtr& cameraInfoR) const;
    void writeExtrinsicParameters(const std::string& filename) const;

    bool writeChessboardData(const std::string& filename) const;
    bool writeChessboardData(std::ofstream& ofs) const;
    bool readChessboardData(const std::string& filename);
    bool readChessboardData(std::ifstream& ifs);

    bool readChessboardData(const std::string& filenameIntL,
                            const std::string& filenameIntR,
                            const std::string& filenameExt);

    void setVerbose(bool verbose);

private:
    CameraCalibration m_calibLeft;
    CameraCalibration m_calibRight;

    Eigen::Quaterniond m_q;
    Eigen::Vector3d m_t;

    bool m_verbose;
};

}

#endif
