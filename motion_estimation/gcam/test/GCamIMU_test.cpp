#include <boost/make_shared.hpp>
#include <gtest/gtest.h>
#include <iostream>

#include "camera_models/EquidistantCamera.h"
#include "cauldron/EigenUtils.h"
#include "gcam/GCamIMU.h"

namespace px
{

enum
{
    ANGULAR_MOTION = 0x1,
    LINEAR_MOTION = 0x2
};

void
generateExample(int motionType, int nIntraCorrespondences, int nInterCorrespondences,
                GCamIMUPtr& gcam,
                Eigen::Matrix4d& H_sys_expected,
                std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& scenePoints,
                std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> >& lcVec)
{
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

    gcam = boost::make_shared<GCamIMU>(cameraSystem);

    // relative transform: x = -0.1, y = -0.2, yaw = 0.1 rad
    H_sys_expected = Eigen::Matrix4d::Identity();

    if ((motionType & ANGULAR_MOTION) == ANGULAR_MOTION)
    {
        H_sys_expected.block<3,3>(0,0) = Eigen::AngleAxisd(0.1, Eigen::Vector3d(0.0, 0.0, 1.0)).toRotationMatrix();
    }
    if ((motionType & LINEAR_MOTION) == LINEAR_MOTION)
    {
        H_sys_expected.block<3,1>(0,3) << -0.1, -0.2, 0.0;
    }

    double r = 3.0;

    scenePoints.resize(3);
    scenePoints[0] << r * cos(M_PI_4 / 2.0), r * sin(M_PI_4 / 2.0), 2.0;
    scenePoints[1] << r * cos(M_PI_4), r * sin(M_PI_4), 0.0;
    scenePoints[2] << r * sin(M_PI_4 / 2.0), r * cos(M_PI_4 / 2.0), -2.0;

    // add intra-camera correspondences from camera 1
    for (int i = 0; i < nIntraCorrespondences; ++i)
    {
        Eigen::Matrix4d H_sys_cam1 = cameraSystem->getGlobalCameraPose(0).inverse();

        Eigen::Vector3d ray1 = transformPoint(H_sys_cam1, scenePoints[i]).normalized();

        Eigen::Matrix4d H_cam1 = H_sys_cam1 * H_sys_expected;

        Eigen::Vector3d ray2 = transformPoint(H_cam1, scenePoints[i]).normalized();

        lcVec.push_back(PLineCorrespondence(0, ray1, cameraSystem->getGlobalCameraPose(0),
                                            0, ray2, cameraSystem->getGlobalCameraPose(0)));
    }

    // add inter-camera correspondences
    for (int i = nIntraCorrespondences; i < nIntraCorrespondences + nInterCorrespondences; ++i)
    {
        Eigen::Matrix4d H_sys_cam1 = cameraSystem->getGlobalCameraPose(0).inverse();
        Eigen::Matrix4d H_sys_cam2 = cameraSystem->getGlobalCameraPose(1).inverse();

        Eigen::Vector3d ray1 = transformPoint(H_sys_cam1, scenePoints[i]).normalized();

        Eigen::Matrix4d H_cam2 = H_sys_cam2 * H_sys_expected;

        Eigen::Vector3d ray2 = transformPoint(H_cam2, scenePoints[i]).normalized();

        lcVec.push_back(PLineCorrespondence(0, ray1, cameraSystem->getGlobalCameraPose(0),
                                            1, ray2, cameraSystem->getGlobalCameraPose(1)));
    }
}

TEST(GCamIMU, Triangulate3DPointWithAngularMotion)
{
    GCamIMUPtr gcam;
    Eigen::Matrix4d H_sys_expected;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > scenePoints;
    std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> > lcVec;

    generateExample(ANGULAR_MOTION, 2, 1, gcam, H_sys_expected, scenePoints, lcVec);

    for (size_t i = 0; i < lcVec.size(); ++i)
    {
        Eigen::Vector3d P = gcam->triangulate3DPoint(lcVec.at(i),
                                                     H_sys_expected.block<3,3>(0,0),
                                                     H_sys_expected.block<3,1>(0,3));

        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(scenePoints.at(i)(j), P(j), 1e-5);
        }
    }
}

TEST(GCamIMU, Triangulate3DPointWithLinearMotion)
{
    GCamIMUPtr gcam;
    Eigen::Matrix4d H_sys_expected;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > scenePoints;
    std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> > lcVec;

    generateExample(LINEAR_MOTION, 2, 1, gcam, H_sys_expected, scenePoints, lcVec);

    for (size_t i = 0; i < lcVec.size(); ++i)
    {
        Eigen::Vector3d P = gcam->triangulate3DPoint(lcVec.at(i),
                                                     H_sys_expected.block<3,3>(0,0),
                                                     H_sys_expected.block<3,1>(0,3));

        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(scenePoints.at(i)(j), P(j), 1e-5);
        }
    }
}

TEST(GCamIMU, Triangulate3DPointWith3DOFMotion)
{
    GCamIMUPtr gcam;
    Eigen::Matrix4d H_sys_expected;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > scenePoints;
    std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> > lcVec;

    generateExample(ANGULAR_MOTION | LINEAR_MOTION, 2, 1, gcam, H_sys_expected, scenePoints, lcVec);

    for (size_t i = 0; i < lcVec.size(); ++i)
    {
        Eigen::Vector3d P = gcam->triangulate3DPoint(lcVec.at(i),
                                                     H_sys_expected.block<3,3>(0,0),
                                                     H_sys_expected.block<3,1>(0,3));

        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(scenePoints.at(i)(j), P(j), 1e-5);
        }
    }
}

TEST(GCamIMU, estimateTWithAngularMotion)
{
    GCamIMUPtr gcam;
    Eigen::Matrix4d H_sys_expected;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > scenePoints;
    std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> > lcVec;

    generateExample(ANGULAR_MOTION, 2, 1, gcam, H_sys_expected, scenePoints, lcVec);

    Eigen::Vector3d t;
    ASSERT_TRUE(gcam->estimateT(lcVec, H_sys_expected.block<3,3>(0,0), t));

    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(H_sys_expected(i,3), t(i), 1e-5);
    }
}

TEST(GCamIMU, estimateTWithLinearMotion)
{
    GCamIMUPtr gcam;
    Eigen::Matrix4d H_sys_expected;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > scenePoints;
    std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> > lcVec;

    generateExample(LINEAR_MOTION, 2, 1, gcam, H_sys_expected, scenePoints, lcVec);

    Eigen::Vector3d t;
    ASSERT_TRUE(gcam->estimateT(lcVec, H_sys_expected.block<3,3>(0,0), t));

    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(H_sys_expected(i,3), t(i), 1e-5);
    }
}

TEST(GCamIMU, estimateTWith3DOFMotion)
{
    GCamIMUPtr gcam;
    Eigen::Matrix4d H_sys_expected;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > scenePoints;
    std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> > lcVec;

    generateExample(ANGULAR_MOTION | LINEAR_MOTION, 2, 1, gcam, H_sys_expected, scenePoints, lcVec);

    Eigen::Vector3d t;
    ASSERT_TRUE(gcam->estimateT(lcVec, H_sys_expected.block<3,3>(0,0), t));

    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(H_sys_expected(i,3), t(i), 1e-5);
    }
}

TEST(GCamIMU, estimateHWithAngularMotion)
{
    GCamIMUPtr gcam;
    Eigen::Matrix4d H_sys_expected;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > scenePoints;
    std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> > lcVec;

    generateExample(ANGULAR_MOTION, 2, 1, gcam, H_sys_expected, scenePoints, lcVec);

    Eigen::Matrix4d H_sys;
    ASSERT_TRUE(gcam->estimateH(lcVec, H_sys_expected.block<3,3>(0,0), H_sys));

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            EXPECT_NEAR(H_sys_expected(i,j), H_sys(i,j), 1e-5);
        }
    }
}

TEST(GCamIMU, estimateHWithLinearMotion)
{
    GCamIMUPtr gcam;
    Eigen::Matrix4d H_sys_expected;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > scenePoints;
    std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> > lcVec;

    generateExample(LINEAR_MOTION, 2, 1, gcam, H_sys_expected, scenePoints, lcVec);

    Eigen::Matrix4d H_sys;
    ASSERT_TRUE(gcam->estimateH(lcVec, H_sys_expected.block<3,3>(0,0), H_sys));

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            EXPECT_NEAR(H_sys_expected(i,j), H_sys(i,j), 1e-5);
        }
    }
}

TEST(GCamIMU, estimateHWith3DOFMotion)
{
    GCamIMUPtr gcam;
    Eigen::Matrix4d H_sys_expected;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > scenePoints;
    std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> > lcVec;

    generateExample(ANGULAR_MOTION | LINEAR_MOTION, 2, 1, gcam, H_sys_expected, scenePoints, lcVec);

    Eigen::Matrix4d H_sys;
    ASSERT_TRUE(gcam->estimateH(lcVec, H_sys_expected.block<3,3>(0,0), H_sys));

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            EXPECT_NEAR(H_sys_expected(i,j), H_sys(i,j), 1e-5);
        }
    }
}

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
