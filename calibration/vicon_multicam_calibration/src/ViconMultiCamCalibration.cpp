#include "vicon_multicam_calibration/ViconMultiCamCalibration.h"

#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>

#include "camera_calibration/CameraCalibration.h"
#include "camera_models/CataCamera.h"
#include "camera_models/EquidistantCamera.h"
#include "camera_models/PinholeCamera.h"
#include "cauldron/EigenQuaternionParameterization.h"
#include "cauldron/EigenUtils.h"
#include "ceres/ceres.h"

namespace px
{

template<class CameraT>
class CalibrationCostFunctor
{
public:
    CalibrationCostFunctor(const cv::Point2f& observed_p,
                           const cv::Point3f& observed_P,
                           const Eigen::Matrix4d& H_cbv_sys)
     : m_observed_p(observed_p.x, observed_p.y)
     , m_observed_P(observed_P.x, observed_P.y, observed_P.z)
     , m_q_cbv_sys(H_cbv_sys.block<3,3>(0,0))
     , m_t_cbv_sys(H_cbv_sys.block<3,1>(0,3))
    {

    }

    template<typename T>
    bool operator()(const T* const intrinsic_params,
                    const T* const q_sys_cam_coeffs,
                    const T* const t_sys_cam_coeffs,
                    const T* const q_cb_cbv_coeffs,
                    const T* const t_cb_cbv_coeffs,
                    T* residuals) const
    {
        Eigen::Quaternion<T> q_cb_sys = m_q_cbv_sys.cast<T>() * Eigen::Quaternion<T>(q_cb_cbv_coeffs);
        Eigen::Matrix<T,3,1> t_cb_sys = m_q_cbv_sys.cast<T>() * Eigen::Matrix<T,3,1>(t_cb_cbv_coeffs) + m_t_cbv_sys.cast<T>();

        Eigen::Quaternion<T> q_cb_cam = Eigen::Quaternion<T>(q_sys_cam_coeffs) * q_cb_sys;
        Eigen::Matrix<T,3,1> t_cb_cam = Eigen::Quaternion<T>(q_sys_cam_coeffs) * t_cb_sys + Eigen::Matrix<T,3,1>(t_sys_cam_coeffs);

        Eigen::Matrix<T,3,1> P = m_observed_P.cast<T>();

        Eigen::Matrix<T,2,1> predicted_p;
        CameraT::spaceToPlane(intrinsic_params, q_cb_cam.coeffs().data(), t_cb_cam.data(), P, predicted_p);

        residuals[0] = predicted_p(0) - T(m_observed_p(0));
        residuals[1] = predicted_p(1) - T(m_observed_p(1));

        return true;
    }

private:
    Eigen::Vector2d m_observed_p;
    Eigen::Vector3d m_observed_P;

    Eigen::Quaterniond m_q_cbv_sys;
    Eigen::Vector3d m_t_cbv_sys;
};

ViconMultiCamCalibration::ViconMultiCamCalibration(Camera::ModelType modelType,
                                                   const std::vector<px_comm::CameraInfoPtr>& cameraInfoVec,
                                                   const cv::Size& boardSize,
                                                   float squareSize)
 : m_verbose(false)
{
    m_cameraSystem = boost::make_shared<CameraSystem>(cameraInfoVec.size());

    m_calibVec.resize(cameraInfoVec.size());
    for (size_t i = 0; i < m_calibVec.size(); ++i)
    {
        const px_comm::CameraInfoPtr& cameraInfo = cameraInfoVec.at(i);

        m_calibVec.at(i) = boost::make_shared<CameraCalibration>(modelType,
                                                                 cameraInfo->camera_name,
                                                                 cv::Size(cameraInfo->image_width,
                                                                          cameraInfo->image_height),
                                                                 boardSize,
                                                                 squareSize);

        m_cameraSystem->setCamera(i, m_calibVec.at(i)->camera());
    }

    m_poseVec.resize(cameraInfoVec.size());
}

void
ViconMultiCamCalibration::addChessboardData(int cameraIdx,
                                            const std::vector<cv::Point2f>& corners,
                                            const Pose& poseChessboard,
                                            const Pose& poseSystem)
{
    m_calibVec.at(cameraIdx)->addChessboardData(corners);
    m_poseVec.at(cameraIdx).push_back(std::make_pair(poseChessboard, poseSystem));
}

bool
ViconMultiCamCalibration::calibrate(const Eigen::Matrix4d& H_cb_cbv)
{
    for (size_t i = 0; i < m_calibVec.size(); ++i)
    {
        if (!m_calibVec.at(i)->calibrate())
        {
            return false;
        }
    }

    m_T_cb_cbv = Transform(H_cb_cbv);

    std::vector<Transform, Eigen::aligned_allocator<Transform> > T_sys_cam(m_calibVec.size());

    // Find initial camera-system transforms.
    for (size_t i = 0; i < m_calibVec.size(); ++i)
    {
        boost::shared_ptr<CameraCalibration>& calib = m_calibVec.at(i);
        CameraPtr& camera = calib->camera();

        // We assume that the first transform between chessboard and camera as
        // computed by intrinsic camera calibration is accurate.
        Eigen::Vector3d rvec;
        rvec << calib->cameraPoses().at<double>(0,0),
                calib->cameraPoses().at<double>(0,1),
                calib->cameraPoses().at<double>(0,2);

        Eigen::Quaterniond q_cb_cam = AngleAxisToQuaternion(rvec);

        Eigen::Vector3d t_cb_cam;
        t_cb_cam << calib->cameraPoses().at<double>(0,3),
                    calib->cameraPoses().at<double>(0,4),
                    calib->cameraPoses().at<double>(0,5);

        Eigen::Matrix4d H_cb_cam = Eigen::Matrix4d::Identity();
        H_cb_cam.block<3,3>(0,0) = q_cb_cam.toRotationMatrix();
        H_cb_cam.block<3,1>(0,3) = t_cb_cam;

        Eigen::Matrix4d H_cbv_sys;
        H_cbv_sys = invertHomogeneousTransform(m_poseVec.at(i).at(0).second.toMatrix()) *
                    m_poseVec.at(i).at(0).first.toMatrix();

        Eigen::Matrix4d H_sys_cam = H_cb_cam *
                                    invertHomogeneousTransform(m_T_cb_cbv.toMatrix()) *
                                    invertHomogeneousTransform(H_cbv_sys);

        T_sys_cam.at(i) = Transform(H_sys_cam);
    }

    if (m_verbose)
    {
        for (size_t i = 0; i < m_calibVec.size(); ++i)
        {
            double errAvg, errMax;
            reprojectionError(i, T_sys_cam.at(i).toMatrix(), errAvg, errMax);

            std::cout << "[" << m_calibVec.at(i)->camera()->cameraName()
                      << "] " << "# INFO: Initial reprojection error: "
                      << "avg = " << errAvg << " px | "
                      << "max = " << errMax << " px" << std::endl;
        }
    }

    std::vector<std::vector<double> > intrinsicParams(m_calibVec.size());
    for (size_t i = 0; i < intrinsicParams.size(); ++i)
    {
        m_calibVec.at(i)->camera()->writeParameters(intrinsicParams.at(i));
    }

    ceres::Problem problem;

    for (size_t i = 0; i < m_calibVec.size(); ++i)
    {
        boost::shared_ptr<CameraCalibration>& calib = m_calibVec.at(i);
        CameraPtr& camera = calib->camera();

        for (size_t j = 0; j < calib->imagePoints().size(); ++j)
        {
            Eigen::Matrix4d H_cbv_sys;
            H_cbv_sys = invertHomogeneousTransform(m_poseVec.at(i).at(j).second.toMatrix()) *
                        m_poseVec.at(i).at(j).first.toMatrix();

            for (size_t k = 0; k < calib->imagePoints().at(j).size(); ++k)
            {
                const cv::Point2f& ipt = calib->imagePoints().at(j).at(k);
                const cv::Point3f& spt = calib->scenePoints().at(j).at(k);

                ceres::CostFunction* costFunction = 0;

                switch (camera->modelType())
                {
                case Camera::KANNALA_BRANDT:
                    costFunction =
                        new ceres::AutoDiffCostFunction<CalibrationCostFunctor<EquidistantCamera>, 2, 8, 4, 3, 4, 3>(
                            new CalibrationCostFunctor<EquidistantCamera>(ipt, spt, H_cbv_sys));
                    break;
                case Camera::PINHOLE:
                    costFunction =
                        new ceres::AutoDiffCostFunction<CalibrationCostFunctor<PinholeCamera>, 2, 8, 4, 3, 4, 3>(
                            new CalibrationCostFunctor<PinholeCamera>(ipt, spt, H_cbv_sys));
                    break;
                case Camera::MEI:
                    costFunction =
                        new ceres::AutoDiffCostFunction<CalibrationCostFunctor<CataCamera>, 2, 9, 4, 3, 4, 3>(
                            new CalibrationCostFunctor<CataCamera>(ipt, spt, H_cbv_sys));
                    break;
                }

                ceres::LossFunction* lossFunction = new ceres::CauchyLoss(1.0);
                problem.AddResidualBlock(costFunction, lossFunction,
                                         intrinsicParams.at(i).data(),
                                         T_sys_cam.at(i).rotationData(),
                                         T_sys_cam.at(i).translationData(),
                                         m_T_cb_cbv.rotationData(),
                                         m_T_cb_cbv.translationData());
            }
        }
    }

    for (size_t i = 0; i < m_calibVec.size(); ++i)
    {
        ceres::LocalParameterization* quaternionParameterization =
            new EigenQuaternionParameterization;

        problem.SetParameterization(T_sys_cam.at(i).rotationData(),
                                    quaternionParameterization);
    }

    ceres::LocalParameterization* quaternionParameterization =
        new EigenQuaternionParameterization;

    problem.SetParameterization(m_T_cb_cbv.rotationData(),
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

    for (size_t i = 0; i < m_calibVec.size(); ++i)
    {
        m_calibVec.at(i)->camera()->readParameters(intrinsicParams.at(i));

        Eigen::Matrix4d H_cam_sys = invertHomogeneousTransform(T_sys_cam.at(i).toMatrix());

        m_cameraSystem->setGlobalCameraPose(i, H_cam_sys);
    }

    if (m_verbose)
    {
        for (size_t i = 0; i < m_calibVec.size(); ++i)
        {
            double errAvg, errMax;
            reprojectionError(i, T_sys_cam.at(i).toMatrix(), errAvg, errMax);

            std::cout << "[" << m_calibVec.at(i)->camera()->cameraName()
                      << "] " << "# INFO: Final reprojection error: "
                      << "avg = " << errAvg << " px | "
                      << "max = " << errMax << " px" << std::endl;
        }
    }

    return true;
}

std::vector<int>
ViconMultiCamCalibration::sampleCount(void)
{
    std::vector<int> count;
    for (size_t i = 0; i < m_calibVec.size(); ++i)
    {
        count.push_back(m_calibVec.at(i)->sampleCount());
    }

    return count;
}

px::CameraSystemConstPtr
ViconMultiCamCalibration::getCameraSystem(void) const
{
    return m_cameraSystem;
}

bool
ViconMultiCamCalibration::readData(const std::string& directory)
{
    for (size_t i = 0; i < m_calibVec.size(); ++i)
    {
        if (!m_calibVec.at(i)->readChessboardData(directory + "/" + m_calibVec.at(i)->camera()->cameraName() + "_chessboard_data.dat"))
        {
            return false;
        }
    }

    std::ifstream ifs((directory + "/poses.txt").c_str());
    if (!ifs.is_open())
    {
        return false;
    }

    m_poseVec.resize(m_calibVec.size());
    for (size_t i = 0; i < m_calibVec.size(); ++i)
    {
        const boost::shared_ptr<CameraCalibration>& calib = m_calibVec.at(i);

        m_poseVec.at(i).resize(calib->imagePoints().size());
        for (size_t j = 0; j < calib->imagePoints().size(); ++j)
        {
            std::string line;
            std::getline(ifs, line);

            std::istringstream iss(line);

            Pose& poseChessboard = m_poseVec.at(i).at(j).first;
            Pose& poseSystem = m_poseVec.at(i).at(j).second;

            uint64_t stamp;
            iss >> stamp;
            poseChessboard.timeStamp() = ros::Time().fromNSec(stamp);
            iss >> poseChessboard.rotation().x();
            iss >> poseChessboard.rotation().y();
            iss >> poseChessboard.rotation().z();
            iss >> poseChessboard.rotation().w();
            iss >> poseChessboard.translation().x();
            iss >> poseChessboard.translation().y();
            iss >> poseChessboard.translation().z();
            iss >> stamp;
            poseSystem.timeStamp() = ros::Time().fromNSec(stamp);
            iss >> poseSystem.rotation().x();
            iss >> poseSystem.rotation().y();
            iss >> poseSystem.rotation().z();
            iss >> poseSystem.rotation().w();
            iss >> poseSystem.translation().x();
            iss >> poseSystem.translation().y();
            iss >> poseSystem.translation().z();
        }
    }

    ifs.close();

    return true;
}

void
ViconMultiCamCalibration::writeData(const std::string& directory) const
{
    if (!boost::filesystem::exists(directory))
    {
        boost::filesystem::create_directory(directory);
    }

    for (size_t i = 0; i < m_calibVec.size(); ++i)
    {
        m_calibVec.at(i)->writeChessboardData(directory + "/" + m_calibVec.at(i)->camera()->cameraName() + "_chessboard_data.dat");
    }

    std::ofstream ofs((directory + "/poses.txt").c_str());
    ofs << std::fixed << std::setprecision(10);
    for (size_t i = 0; i < m_calibVec.size(); ++i)
    {
        const boost::shared_ptr<CameraCalibration>& calib = m_calibVec.at(i);

        for (size_t j = 0; j < calib->imagePoints().size(); ++j)
        {
            const Pose& poseChessboard = m_poseVec.at(i).at(j).first;
            const Pose& poseSystem = m_poseVec.at(i).at(j).second;

            ofs << poseChessboard.timeStamp().toNSec() << " "
                << poseChessboard.rotation().x() << " "
                << poseChessboard.rotation().y() << " "
                << poseChessboard.rotation().z() << " "
                << poseChessboard.rotation().w() << " "
                << poseChessboard.translation().x() << " "
                << poseChessboard.translation().y() << " "
                << poseChessboard.translation().z() << " "
                << poseSystem.timeStamp().toNSec() << " "
                << poseSystem.rotation().x() << " "
                << poseSystem.rotation().y() << " "
                << poseSystem.rotation().z() << " "
                << poseSystem.rotation().w() << " "
                << poseSystem.translation().x() << " "
                << poseSystem.translation().y() << " "
                << poseSystem.translation().z() << std::endl;
        }
    }

    ofs.close();
}

void
ViconMultiCamCalibration::setVerbose(bool verbose)
{
    m_verbose = verbose;

    for (size_t i = 0; i < m_calibVec.size(); ++i)
    {
        m_calibVec.at(i)->setVerbose(verbose);
    }
}

void
ViconMultiCamCalibration::reprojectionError(int cameraIdx,
                                            const Eigen::Matrix4d& H_sys_cam,
                                            double& errorAvg,
                                            double& errorMax) const
{
    const boost::shared_ptr<CameraCalibration>& calib = m_calibVec.at(cameraIdx);
    const CameraPtr& camera = calib->camera();

    double errorSum = 0.0;
    size_t count = 0;
    errorMax = 0.0;
    for (size_t i = 0; i < calib->imagePoints().size(); ++i)
    {
        Eigen::Matrix4d H_cbv_sys;
        H_cbv_sys = invertHomogeneousTransform(m_poseVec.at(cameraIdx).at(i).second.toMatrix()) *
                    m_poseVec.at(cameraIdx).at(i).first.toMatrix();

        Eigen::Matrix4d H_cb_cam = H_sys_cam * H_cbv_sys * m_T_cb_cbv.toMatrix();
        Eigen::Quaterniond q_cb_cam(H_cb_cam.block<3,3>(0,0));
        Eigen::Vector3d t_cb_cam = H_cb_cam.block<3,1>(0,3);

        const std::vector<cv::Point3f>& scenePoints = calib->scenePoints().at(i);
        const std::vector<cv::Point2f>& imagePoints = calib->imagePoints().at(i);
        for (size_t j = 0; j < scenePoints.size(); ++j)
        {
            Eigen::Vector3d P;
            P << scenePoints.at(j).x,
                 scenePoints.at(j).y,
                 scenePoints.at(j).z;

            Eigen::Vector2d p;
            p << imagePoints.at(j).x, imagePoints.at(j).y;

            double error = camera->reprojectionError(P, q_cb_cam, t_cb_cam, p);

            errorSum += error;
            if (error > errorMax)
            {
                errorMax = error;
            }
        }

        count += scenePoints.size();
    }

    errorAvg = errorSum / static_cast<double>(count);
}

}
