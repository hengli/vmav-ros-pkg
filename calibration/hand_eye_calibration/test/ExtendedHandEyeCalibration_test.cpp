#include <gtest/gtest.h>
#include <iostream>

#include "cauldron/cauldron.h"
#include "cauldron/EigenUtils.h"
#include "hand_eye_calibration/ExtendedHandEyeCalibration.h"

namespace px
{

TEST(ExtendedHandEyeCalibration, FullMotion)
{
    double s_expected = 0.5;
    Eigen::Matrix4d H_12_expected = Eigen::Matrix4d::Identity();
    H_12_expected.block<3,3>(0,0) = Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.1, 0.2, 0.3).normalized()).toRotationMatrix();
    H_12_expected.block<3,1>(0,3) << 0.5, 0.6, 0.7;

    Eigen::Matrix4d s_H_12_expected = H_12_expected;
    s_H_12_expected.block<3,3>(0,0) *= s_expected;

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > H1, H2;

    int motionCount = 2;
    for (int i = 0; i < motionCount; ++i)
    {
        double droll = d2r(random(-10.0, 10.0));
        double dpitch =  d2r(random(-10.0, 10.0));
        double dyaw =  d2r(random(-10.0, 10.0));
        double dx = random(-1.0, 1.0);
        double dy = random(-1.0, 1.0);
        double dz = random(-1.0, 1.0);

        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(dpitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(droll, Eigen::Vector3d::UnitX());

        Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
        H.block<3,3>(0,0) = R;
        H.block<3,1>(0,3) << dx, dy, dz;

        H1.push_back(H);
        H2.push_back(s_H_12_expected * H * s_H_12_expected.inverse());
    }

    Eigen::Matrix4d H_12;
    double s;
    ExtendedHandEyeCalibration hec;
    hec.solve(H1, H2, H_12, s);

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            EXPECT_NEAR(H_12_expected(i,j), H_12(i,j), 1e-5) << "Elements differ at H(" << i << "," << j << ")";
        }
    }

    EXPECT_NEAR(s_expected, s, 1e-5) << "Scale differs.";
}

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
