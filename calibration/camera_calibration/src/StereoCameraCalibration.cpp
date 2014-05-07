#include "camera_calibration/StereoCameraCalibration.h"

#include <boost/filesystem.hpp>
#include <opencv2/core/eigen.hpp>

#include "camera_models/CameraFactory.h"
#include "camera_models/CostFunctionFactory.h"
#include "camera_systems/CameraSystem.h"
#include "ceres/ceres.h"
#include "cauldron/EigenQuaternionParameterization.h"
#include "cauldron/EigenUtils.h"

namespace px
{

StereoCameraCalibration::StereoCameraCalibration()
 : m_verbose(false)
{

}

StereoCameraCalibration::StereoCameraCalibration(Camera::ModelType modelType,
                                                 const std::string& cameraLeftName,
                                                 const std::string& cameraRightName,
                                                 const cv::Size& imageSize,
                                                 const cv::Size& boardSize,
                                                 float squareSize)
 : m_calibLeft(modelType, cameraLeftName, imageSize, boardSize, squareSize)
 , m_calibRight(modelType, cameraRightName, imageSize, boardSize, squareSize)
 , m_verbose(false)
{

}

void
StereoCameraCalibration::clear(void)
{
    m_calibLeft.clear();
    m_calibRight.clear();
}

void
StereoCameraCalibration::addChessboardData(const std::vector<cv::Point2f>& cornersLeft,
                                           const std::vector<cv::Point2f>& cornersRight)
{
    m_calibLeft.addChessboardData(cornersLeft);
    m_calibRight.addChessboardData(cornersRight);
}

bool
StereoCameraCalibration::calibrate(void)
{
    // calibrate cameras individually
    if (!m_calibLeft.calibrate())
    {
        return false;
    }
    if (!m_calibRight.calibrate())
    {
        return false;
    }

    // perform stereo calibration
    int imageCount = imagePointsLeft().size();

    // find best estimate for initial transform from left camera frame to right camera frame
    double minReprojErr = std::numeric_limits<double>::max();
    for (int i = 0; i < imageCount; ++i)
    {
        Eigen::Vector3d rvec;
        rvec << m_calibLeft.cameraPoses().at<double>(i,0),
                m_calibLeft.cameraPoses().at<double>(i,1),
                m_calibLeft.cameraPoses().at<double>(i,2);

        Eigen::Quaterniond q_l = AngleAxisToQuaternion(rvec);

        Eigen::Vector3d t_l;
        t_l << m_calibLeft.cameraPoses().at<double>(i,3),
               m_calibLeft.cameraPoses().at<double>(i,4),
               m_calibLeft.cameraPoses().at<double>(i,5);

        rvec << m_calibRight.cameraPoses().at<double>(i,0),
                m_calibRight.cameraPoses().at<double>(i,1),
                m_calibRight.cameraPoses().at<double>(i,2);

        Eigen::Quaterniond q_r = AngleAxisToQuaternion(rvec);

        Eigen::Vector3d t_r;
        t_r << m_calibRight.cameraPoses().at<double>(i,3),
               m_calibRight.cameraPoses().at<double>(i,4),
               m_calibRight.cameraPoses().at<double>(i,5);

        Eigen::Quaterniond q_l_r = q_r * q_l.conjugate();
        Eigen::Vector3d t_l_r = -q_l_r.toRotationMatrix() * t_l + t_r;

        std::vector<cv::Mat> rvecs(imageCount);
        std::vector<cv::Mat> tvecs(imageCount);

        for (int j = 0; j < imageCount; ++j)
        {
            rvec << m_calibLeft.cameraPoses().at<double>(j,0),
                    m_calibLeft.cameraPoses().at<double>(j,1),
                    m_calibLeft.cameraPoses().at<double>(j,2);

            q_l = AngleAxisToQuaternion(rvec);

            t_l << m_calibLeft.cameraPoses().at<double>(j,3),
                   m_calibLeft.cameraPoses().at<double>(j,4),
                   m_calibLeft.cameraPoses().at<double>(j,5);

            Eigen::Quaterniond q_r = q_l_r * q_l;
            Eigen::Vector3d t_r = q_l_r.toRotationMatrix() * t_l + t_l_r;

            QuaternionToAngleAxis(q_r.coeffs().data(), rvec);
            cv::eigen2cv(rvec, rvecs.at(j));

            cv::eigen2cv(t_r, tvecs.at(j));
        }

        double reprojErr = cameraRight()->reprojectionError(scenePoints(),
                                                            imagePointsRight(),
                                                            rvecs, tvecs);

        if (reprojErr < minReprojErr)
        {
            minReprojErr = reprojErr;
            m_q = q_l_r;
            m_t = t_l_r;
        }
    }

    std::vector<cv::Mat> rvecsL(imageCount);
    std::vector<cv::Mat> tvecsL(imageCount);
    std::vector<cv::Mat> rvecsR(imageCount);
    std::vector<cv::Mat> tvecsR(imageCount);

    double* extrinsicCameraLParams[scenePoints().size()];
    for (int i = 0; i < imageCount; ++i)
    {
        extrinsicCameraLParams[i] = new double[7];

        Eigen::Vector3d rvecL(m_calibLeft.cameraPoses().at<double>(i,0),
                              m_calibLeft.cameraPoses().at<double>(i,1),
                              m_calibLeft.cameraPoses().at<double>(i,2));

        AngleAxisToQuaternion(rvecL, extrinsicCameraLParams[i]);

        extrinsicCameraLParams[i][4] = m_calibLeft.cameraPoses().at<double>(i,3);
        extrinsicCameraLParams[i][5] = m_calibLeft.cameraPoses().at<double>(i,4);
        extrinsicCameraLParams[i][6] = m_calibLeft.cameraPoses().at<double>(i,5);

        cv::eigen2cv(rvecL, rvecsL.at(i));

        Eigen::Vector3d tvecL;
        tvecL << m_calibLeft.cameraPoses().at<double>(i,3),
                 m_calibLeft.cameraPoses().at<double>(i,4),
                 m_calibLeft.cameraPoses().at<double>(i,5);

        cv::eigen2cv(tvecL, tvecsL.at(i));

        Eigen::Quaterniond q_r = m_q * AngleAxisToQuaternion(rvecL);
        Eigen::Vector3d t_r = m_q.toRotationMatrix() * tvecL + m_t;

        Eigen::Vector3d rvecR;
        QuaternionToAngleAxis(q_r.coeffs().data(), rvecR);

        cv::eigen2cv(rvecR, rvecsR.at(i));
        cv::eigen2cv(t_r, tvecsR.at(i));
    }

    if (m_verbose)
    {
        double roll, pitch, yaw;
        mat2RPY(m_q.toRotationMatrix(), roll, pitch, yaw);

        std::cout << "[stereo]" << "# INFO: Initial extrinsics: " << std::endl
                  << "r: " << roll << "  p: " << pitch << "  yaw: " << yaw << std::endl
                  << "x: " << m_t(0) << "  y: " << m_t(1) << "  z: " << m_t(2) << std::endl;

        double err = cameraLeft()->reprojectionError(scenePoints(), imagePointsLeft(), rvecsL, tvecsL);
        std::cout << "[" << cameraLeft()->cameraName() << "] " << "# INFO: Initial reprojection error: "
                  << err << " pixels" << std::endl;

        err = cameraRight()->reprojectionError(scenePoints(), imagePointsRight(), rvecsR, tvecsR);
        std::cout << "[" << cameraRight()->cameraName() << "] " << "# INFO: Initial reprojection error: "
                  << err << " pixels" << std::endl;
    }

    std::vector<double> intrinsicCameraLParams;
    cameraLeft()->writeParameters(intrinsicCameraLParams);

    std::vector<double> intrinsicCameraRParams;
    cameraRight()->writeParameters(intrinsicCameraRParams);

    ceres::Problem problem;

    for (int i = 0; i < imageCount; ++i)
    {
        for (size_t j = 0; j < scenePoints().at(i).size(); ++j)
        {
            const cv::Point3f& spt = scenePoints().at(i).at(j);
            const cv::Point2f& iptL = imagePointsLeft().at(i).at(j);
            const cv::Point2f& iptR = imagePointsRight().at(i).at(j);

            ceres::CostFunction* costFunction =
                CostFunctionFactory::instance()->generateCostFunction(cameraLeft(),
                                                                      Eigen::Vector2d(iptL.x, iptL.y),
                                                                      cameraRight(),
                                                                      Eigen::Vector2d(iptR.x, iptR.y),
                                                                      Eigen::Vector3d(spt.x, spt.y, spt.z),
                                                                      CAMERA_INTRINSICS | STEREO_TRANSFORM | CAMERA_POSE);

            ceres::LossFunction* lossFunction = new ceres::CauchyLoss(1.0);
            problem.AddResidualBlock(costFunction, lossFunction,
                                     intrinsicCameraLParams.data(),
                                     intrinsicCameraRParams.data(),
                                     m_q.coeffs().data(),
                                     m_t.data(),
                                     extrinsicCameraLParams[i],
                                     extrinsicCameraLParams[i] + 4);
        }
    }

    for (int i = 0; i < imageCount; ++i)
    {
        ceres::LocalParameterization* quaternionParameterization =
            new EigenQuaternionParameterization;

        problem.SetParameterization(extrinsicCameraLParams[i],
                                    quaternionParameterization);
    }

    ceres::LocalParameterization* quaternionParameterization =
        new EigenQuaternionParameterization;

    problem.SetParameterization(m_q.coeffs().data(),
                                quaternionParameterization);

    ceres::Solver::Options options;
    options.max_num_iterations = 1000;
    options.num_threads = 8;

    if (m_verbose)
    {
        options.minimizer_progress_to_stdout = true;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (m_verbose)
    {
        std::cout << summary.FullReport() << "\n";
    }

    cameraLeft()->readParameters(intrinsicCameraLParams);
    cameraRight()->readParameters(intrinsicCameraRParams);

    for (int i = 0; i < imageCount; ++i)
    {
        Eigen::Vector3d rvecL;
        QuaternionToAngleAxis(extrinsicCameraLParams[i], rvecL);

        m_calibLeft.cameraPoses().at<double>(i,0) = rvecL(0);
        m_calibLeft.cameraPoses().at<double>(i,1) = rvecL(1);
        m_calibLeft.cameraPoses().at<double>(i,2) = rvecL(2);
        m_calibLeft.cameraPoses().at<double>(i,3) = extrinsicCameraLParams[i][4];
        m_calibLeft.cameraPoses().at<double>(i,4) = extrinsicCameraLParams[i][5];
        m_calibLeft.cameraPoses().at<double>(i,5) = extrinsicCameraLParams[i][6];

        cv::eigen2cv(rvecL, rvecsL.at(i));

        Eigen::Vector3d tvecL;
        tvecL << extrinsicCameraLParams[i][4],
                 extrinsicCameraLParams[i][5],
                 extrinsicCameraLParams[i][6];

        cv::eigen2cv(tvecL, tvecsL.at(i));

        Eigen::Quaterniond q_r = m_q * AngleAxisToQuaternion(rvecL);
        Eigen::Vector3d t_r = m_q.toRotationMatrix() * tvecL + m_t;

        Eigen::Vector3d rvecR;
        QuaternionToAngleAxis(q_r.coeffs().data(), rvecR);

        m_calibRight.cameraPoses().at<double>(i,0) = rvecR(0);
        m_calibRight.cameraPoses().at<double>(i,1) = rvecR(1);
        m_calibRight.cameraPoses().at<double>(i,2) = rvecR(2);
        m_calibRight.cameraPoses().at<double>(i,3) = t_r(0);
        m_calibRight.cameraPoses().at<double>(i,4) = t_r(1);
        m_calibRight.cameraPoses().at<double>(i,5) = t_r(2);

        cv::eigen2cv(rvecR, rvecsR.at(i));
        cv::eigen2cv(t_r, tvecsR.at(i));
    }

    if (m_verbose)
    {
        double roll, pitch, yaw;
        mat2RPY(m_q.toRotationMatrix(), roll, pitch, yaw);

        std::cout << "[stereo]" << "# INFO: Final extrinsics: " << std::endl
                  << "r: " << roll << "  p: " << pitch << "  yaw: " << yaw << std::endl
                  << "x: " << m_t(0) << "  y: " << m_t(1) << "  z: " << m_t(2) << std::endl;

        double err = cameraLeft()->reprojectionError(scenePoints(), imagePointsLeft(), rvecsL, tvecsL);
        std::cout << "[" << cameraLeft()->cameraName() << "] " << "# INFO: Final reprojection error: "
                  << err << " pixels" << std::endl;
        std::cout << "[" << cameraLeft()->cameraName() << "] " << "# INFO: "
                  << cameraLeft()->parametersToString() << std::endl;

        err = cameraRight()->reprojectionError(scenePoints(), imagePointsRight(), rvecsR, tvecsR);
        std::cout << "[" << cameraRight()->cameraName() << "] " << "# INFO: Final reprojection error: "
                  << err << " pixels" << std::endl;
        std::cout << "[" << cameraRight()->cameraName() << "] " << "# INFO: "
                  << cameraRight()->parametersToString() << std::endl;
    }

    return true;
}

int
StereoCameraCalibration::sampleCount(void) const
{
    return m_calibLeft.sampleCount();
}

const std::vector<std::vector<cv::Point2f> >&
StereoCameraCalibration::imagePointsLeft(void) const
{
    return m_calibLeft.imagePoints();
}

const std::vector<std::vector<cv::Point2f> >&
StereoCameraCalibration::imagePointsRight(void) const
{
    return m_calibRight.imagePoints();
}

std::vector<std::vector<cv::Point3f> >&
StereoCameraCalibration::scenePoints(void)
{
    return m_calibLeft.scenePoints();
}

const std::vector<std::vector<cv::Point3f> >&
StereoCameraCalibration::scenePoints(void) const
{
    return m_calibLeft.scenePoints();
}

CameraPtr&
StereoCameraCalibration::cameraLeft(void)
{
    return m_calibLeft.camera();
}

const CameraConstPtr
StereoCameraCalibration::cameraLeft(void) const
{
    return m_calibLeft.camera();
}

CameraPtr&
StereoCameraCalibration::cameraRight(void)
{
    return m_calibRight.camera();
}

const CameraConstPtr
StereoCameraCalibration::cameraRight(void) const
{
    return m_calibRight.camera();
}

cv::Mat&
StereoCameraCalibration::cameraPosesLeft(void)
{
    return m_calibLeft.cameraPoses();
}

const cv::Mat&
StereoCameraCalibration::cameraPosesLeft(void) const
{
    return m_calibLeft.cameraPoses();
}

cv::Mat&
StereoCameraCalibration::cameraPosesRight(void)
{
    return m_calibRight.cameraPoses();
}

const cv::Mat&
StereoCameraCalibration::cameraPosesRight(void) const
{
    return m_calibRight.cameraPoses();
}

void
StereoCameraCalibration::drawResults(std::vector<cv::Mat>& imagesLeft,
                                     std::vector<cv::Mat>& imagesRight) const
{
    m_calibLeft.drawResults(imagesLeft);
    m_calibRight.drawResults(imagesRight);
}

void
StereoCameraCalibration::writeParameters(const std::string& directory) const
{
    if (!boost::filesystem::exists(directory))
    {
        boost::filesystem::create_directory(directory);
    }

    cameraLeft()->writeParametersToYamlFile(directory + "/camera_left.yaml");
    cameraRight()->writeParametersToYamlFile(directory + "/camera_right.yaml");
    writeExtrinsicParameters(directory + "/extrinsics.txt");
}

void
StereoCameraCalibration::writeParameters(px_comm::CameraInfoPtr& cameraInfoL,
                                         px_comm::CameraInfoPtr& cameraInfoR) const
{
    m_calibLeft.writeParameters(cameraInfoL);
    m_calibRight.writeParameters(cameraInfoR);

    Eigen::Quaterniond q = m_q.conjugate();
    Eigen::Vector3d t = q * (- m_t);

    cameraInfoL->pose.orientation.w = 1.0;
    cameraInfoL->pose.orientation.x = 0.0;
    cameraInfoL->pose.orientation.y = 0.0;
    cameraInfoL->pose.orientation.z = 0.0;

    cameraInfoL->pose.position.x = 0.0;
    cameraInfoL->pose.position.y = 0.0;
    cameraInfoL->pose.position.z = 0.0;

    cameraInfoR->pose.orientation.w = q.w();
    cameraInfoR->pose.orientation.x = q.x();
    cameraInfoR->pose.orientation.y = q.y();
    cameraInfoR->pose.orientation.z = q.z();

    cameraInfoR->pose.position.x = t(0);
    cameraInfoR->pose.position.y = t(1);
    cameraInfoR->pose.position.z = t(2);
}

void
StereoCameraCalibration::writeExtrinsicParameters(const std::string& filename) const
{
    CameraSystem cs(2);
    CameraPtr cameraL = boost::const_pointer_cast<Camera>(cameraLeft());
    CameraPtr cameraR = boost::const_pointer_cast<Camera>(cameraRight());
    cs.setCamera(0, cameraL);
    cs.setGlobalCameraPose(0, Eigen::Matrix4d::Identity());
    cs.setCamera(1, cameraR);

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3,3>(0,0) = m_q.toRotationMatrix();
    pose.block<3,1>(0,3) = m_t;

    cs.setGlobalCameraPose(1, invertHomogeneousTransform(pose));

    cs.writePosesToTextFile(filename);
}

bool
StereoCameraCalibration::writeChessboardData(const std::string& filename) const
{
    std::ofstream ofs(filename.c_str(), std::ios::out | std::ios::binary);
    if (!ofs.is_open())
    {
        return false;
    }

    return writeChessboardData(ofs);
}

bool
StereoCameraCalibration::writeChessboardData(std::ofstream& ofs) const
{
    if (!m_calibLeft.writeChessboardData(ofs))
    {
        return false;
    }
    if (!m_calibRight.writeChessboardData(ofs))
    {
        return false;
    }

    ofs.write(reinterpret_cast<const char*>(m_q.coeffs().data()), sizeof(double) * 4);
    ofs.write(reinterpret_cast<const char*>(m_t.data()), sizeof(double) * 3);

    return true;
}

bool
StereoCameraCalibration::readChessboardData(const std::string& filename)
{
    std::ifstream ifs(filename.c_str(), std::ios::in | std::ios::binary);
    if (!ifs.is_open())
    {
        return false;
    }

    return readChessboardData(ifs);
}

bool
StereoCameraCalibration::readChessboardData(std::ifstream& ifs)
{
    if (!m_calibLeft.readChessboardData(ifs))
    {
        return false;
    }
    if (!m_calibRight.readChessboardData(ifs))
    {
        return false;
    }

    char* buffer = new char[sizeof(double) * 4];
    
    ifs.read(buffer, sizeof(double) * 4);
    memcpy(m_q.coeffs().data(), buffer, sizeof(double) * 4);

    ifs.read(buffer, sizeof(double) * 3);
    memcpy(m_t.data(), buffer, sizeof(double) * 3);

    return true;
}

bool
StereoCameraCalibration::readChessboardData(const std::string& filenameIntL,
                                            const std::string& filenameIntR,
                                            const std::string& filenameExt)
{
    if (!m_calibLeft.readChessboardData(filenameIntL))
    {
        return false;
    }
    if (!m_calibRight.readChessboardData(filenameIntR))
    {
        return false;
    }

    CameraSystem cameraSystem(2);
    if (!cameraSystem.readPosesFromTextFile(filenameExt))
    {
        return false;
    }

    Eigen::Matrix4d H = cameraSystem.getGlobalCameraPose(1).inverse();
    m_q = Eigen::Quaterniond(H.block<3,3>(0,0));
    m_t = H.block<3,1>(0,3);

    return true;
}

void
StereoCameraCalibration::setVerbose(bool verbose)
{
    m_verbose = verbose;
    m_calibLeft.setVerbose(verbose);
    m_calibRight.setVerbose(verbose);
}

}
