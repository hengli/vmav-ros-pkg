#include <boost/make_shared.hpp>
#include <gtest/gtest.h>
#include <iostream>

#include "camera_models/EquidistantCamera.h"
#include "camera_systems/CameraSystem.h"
#include "cauldron/cauldron.h"
#include "cauldron/EigenUtils.h"
#include "pose_estimation/gP3P.h"

namespace px
{

void
generateExample(Eigen::Matrix4d& H_sys_expected,
                std::vector<PLine, Eigen::aligned_allocator<PLine> >& plineVec,
                std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& scenePoints)
{
    srand(time(0));

    CameraPtr camera1(new EquidistantCamera("camera", 1280, 800,
                                            -0.01648, -0.00203, 0.00069, -0.00048,
                                            419.22826, 420.42160, 655.45487, 389.66377));
    camera1->cameraId() = 0;

    CameraPtr camera2(new EquidistantCamera("camera", 1280, 800,
                                            -0.01648, -0.00203, 0.00069, -0.00048,
                                            419.22826, 420.42160, 655.45487, 389.66377));
    camera2->cameraId() = 1;

    // camera1 is forward-looking
    Eigen::Matrix4d camPose1;
    camPose1 << 0.0, 0.0, 1.0, 1.0,
                -1.0, 0.0, 0.0, 0.0,
                0.0, -1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0;

    // camera 2 is left-looking
    Eigen::Matrix4d camPose2;
    camPose2 << 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 1.0,
                0.0, -1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0;

    CameraSystemPtr cameraSystem = boost::make_shared<CameraSystem>(2);
    cameraSystem->setCamera(0, camera1);
    cameraSystem->setCamera(1, camera2);
    cameraSystem->setGlobalCameraPose(0, camPose1);
    cameraSystem->setGlobalCameraPose(1, camPose2);

    double r = 3.0;

    scenePoints.resize(3);
    scenePoints[0] << r * cos(M_PI_4 / 2.0), r * sin(M_PI_4 / 2.0), 2.0;
    scenePoints[1] << r * cos(M_PI_4), r * sin(M_PI_4), 0.0;
    scenePoints[2] << r * sin(M_PI_4 / 2.0), r * cos(M_PI_4 / 2.0), -2.0;

    // add Plucker lines from camera 1
    for (int i = 0; i < 1; ++i)
    {
        Eigen::Matrix4d H_sys_cam1 = cameraSystem->getGlobalCameraPose(0).inverse();

        Eigen::Vector3d ray1 = transformPoint(H_sys_cam1, scenePoints[i]).normalized();

        plineVec.push_back(PLine(ray1, cameraSystem->getGlobalCameraPose(0)));
    }

    // add Plucker lines from camera 2
    for (int i = 1; i < 3; ++i)
    {
        Eigen::Matrix4d H_sys_cam2 = cameraSystem->getGlobalCameraPose(1).inverse();

        Eigen::Vector3d ray2 = transformPoint(H_sys_cam2, scenePoints[i]).normalized();

        plineVec.push_back(PLine(ray2, cameraSystem->getGlobalCameraPose(1)));
    }

    // system pose
    H_sys_expected = Eigen::Matrix4d::Identity();
    H_sys_expected.block<3,3>(0,0) = Eigen::AngleAxisd(random(-M_PI, M_PI), Eigen::Vector3d::Random().normalized()).toRotationMatrix();
    H_sys_expected.block<3,1>(0,3) = Eigen::Vector3d::Random() * 10.0;

    for (int i = 0; i < 3; ++i)
    {
        scenePoints.at(i) = transformPoint(H_sys_expected, scenePoints.at(i));
    }
}

TEST(MotionEstimation, gP3P)
{
    Eigen::Matrix4d H_sys_expected;
    std::vector<PLine, Eigen::aligned_allocator<PLine> > plineVec;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > scenePoints;

    generateExample(H_sys_expected, plineVec, scenePoints);

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > solutions;
    ASSERT_TRUE(solvegP3P(plineVec, scenePoints, solutions));

    double normMin = std::numeric_limits<double>::max();
    size_t idxBest = 0;
    for (size_t i = 0; i < solutions.size(); ++i)
    {
        double norm = (solutions.at(i) - H_sys_expected).norm();

        if (norm < normMin)
        {
            idxBest = i;
            normMin = norm;
        }
    }

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            EXPECT_NEAR(H_sys_expected(i,j), solutions.at(idxBest)(i,j), 1e-8);
        }
    }
}

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
