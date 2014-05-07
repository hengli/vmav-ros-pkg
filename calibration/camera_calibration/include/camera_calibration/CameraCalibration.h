#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

#include <fstream>
#include <opencv2/core/core.hpp>

#include "camera_models/Camera.h"

namespace px
{

class CameraCalibration
{
public:
    CameraCalibration();

    CameraCalibration(Camera::ModelType modelType,
                      const std::string& cameraName,
                      const cv::Size& imageSize,
                      const cv::Size& boardSize,
                      float squareSize);

    void clear(void);

    void addChessboardData(const std::vector<cv::Point2f>& corners);

    bool calibrate(void);

    int sampleCount(void) const;
    std::vector<std::vector<cv::Point2f> >& imagePoints(void);
    const std::vector<std::vector<cv::Point2f> >& imagePoints(void) const;
    std::vector<std::vector<cv::Point3f> >& scenePoints(void);
    const std::vector<std::vector<cv::Point3f> >& scenePoints(void) const;
    CameraPtr& camera(void);
    const CameraConstPtr camera(void) const;

    Eigen::Matrix2d& measurementCovariance(void);
    const Eigen::Matrix2d& measurementCovariance(void) const;

    cv::Mat& cameraPoses(void);
    const cv::Mat& cameraPoses(void) const;

    void drawResults(std::vector<cv::Mat>& images) const;

    void writeParameters(const std::string& filename) const;
    void writeParameters(px_comm::CameraInfoPtr& cameraInfo) const;

    bool writeChessboardData(const std::string& filename) const;
    bool writeChessboardData(std::ofstream& ofs) const;
    bool readChessboardData(const std::string& filename);
    bool readChessboardData(std::ifstream& ifs);

    void setVerbose(bool verbose);

private:
    bool calibrateHelper(CameraPtr& camera,
                         std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const;

    void optimize(CameraPtr& camera,
                  std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const;

    template<typename T>
    void readData(std::ifstream& ifs, T& data) const;

    template<typename T>
    void writeData(std::ofstream& ofs, T data) const;

    cv::Size m_boardSize;
    float m_squareSize;

    CameraPtr m_camera;
    cv::Mat m_cameraPoses;

    std::vector<std::vector<cv::Point2f> > m_imagePoints;
    std::vector<std::vector<cv::Point3f> > m_scenePoints;

    Eigen::Matrix2d m_measurementCovariance;

    bool m_verbose;
};

}

#endif
