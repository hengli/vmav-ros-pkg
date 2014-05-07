#ifndef VICONMULTICAMCALIBRATION_H
#define VICONMULTICAMCALIBRATION_H

#include "camera_systems/CameraSystem.h"
#include "sparse_graph/Pose.h"

namespace px
{

class CameraCalibration;

class ViconMultiCamCalibration
{
public:
    ViconMultiCamCalibration(Camera::ModelType modelType,
                             const std::vector<px_comm::CameraInfoPtr>& cameraInfoVec,
                             const cv::Size& boardSize,
                             float squareSize);

    void addChessboardData(int cameraIdx,
                           const std::vector<cv::Point2f>& corners,
                           const Pose& poseChessboard,
                           const Pose& poseSystem);

    bool calibrate(const Eigen::Matrix4d& H_cb_cbv);

    std::vector<int> sampleCount(void);

    px::CameraSystemConstPtr getCameraSystem(void) const;

    bool readData(const std::string& directory);
    void writeData(const std::string& directory) const;

    void setVerbose(bool verbose);

private:
    void reprojectionError(int cameraIdx, const Eigen::Matrix4d& H_sys_cam,
                           double& errorAvg, double& errorMax) const;

    std::vector<boost::shared_ptr<CameraCalibration> > m_calibVec;
    std::vector<std::vector<std::pair<Pose, Pose> > > m_poseVec;
    CameraSystemPtr m_cameraSystem;

    Transform m_T_cb_cbv;

    bool m_verbose;
};

}

#endif

