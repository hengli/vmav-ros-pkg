#include <gtest/gtest.h>
#include <iostream>

#include "dynocmap/OcTree.h"
#include "sensor_models/LaserSensorModel.h"

// Test #1: insert node outside range and check that returned node is NULL
TEST(OcTree, NodeOutsideRange)
{
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    px::OcTree octree(0.25, 8, center);

    px::OcNodePtr node1 = octree.insertNode(100.0, 100.0, 100.0);
    EXPECT_EQ(0, node1.get());
}

// Test #2: insert node and check if query for that node returns a pointer
// to the node
TEST(OcTree, RefInsertedNode)
{
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    px::OcTree octree(0.25, 8, center);

    double nodePos[3] = {5.0, 5.0, 5.24};
    px::OcNodePtr node1 = octree.insertNode(nodePos[0], nodePos[1], nodePos[2]);
    ASSERT_TRUE(node1);

    px::OcNodePtr node2 = octree.findNode(5.0, 5.1, 5.1);
    EXPECT_EQ(node1.get(), node2.get());
}

// Test #3: check that number of obstacles is 1
TEST(OcTree, ObstacleCount)
{
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    px::OcTree octree(0.25, 8, center);

    double nodePos[3] = {5.0, 5.0, -5.24};
    px::OcNodePtr node1 = octree.insertNode(nodePos[0], nodePos[1], nodePos[2]);
    node1->setLogOdds(4.0);

    std::vector<px::OccupancyCell> obstacles = octree.obstacles();
    EXPECT_EQ(1, obstacles.size());
}

// Test #4: check that obstacles resulting from castRay operation are consistent.
TEST(OcTree, CastRay)
{
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    px::OcTree octree(0.1, 10, center);

    Eigen::Vector3d obstaclePos(1.02109, -0.765036, -0.0790663);

    px::SensorModelPtr sensorModel(new px::LaserSensorModel(0.0,
                                                            1.0,
                                                            10.0,
                                                            0.02));

    Eigen::Matrix4d sensorPose;
    sensorPose << 0.0567304,  -5.60002e-05,  0.99839,     0.277366,
                  0.998389,   -0.00115524,  -0.0567305,   0.0676122,
                  0.00115655,  0.999999,    -9.62725e-06, 0.514195,
                  0.0,         0.0,          0.0,         1.0;

    octree.castRay(sensorPose, obstaclePos, px::OcTree::SENSOR_FRAME, sensorModel);

    Eigen::Vector3d obstaclePos_w = sensorPose.block<3,3>(0,0) * obstaclePos + sensorPose.block<3,1>(0,3);

    std::vector<px::OccupancyCell> obstacles = octree.obstacles();
    for (size_t i = 0; i < obstacles.size(); ++i)
    {
        const px::OccupancyCell& obstacle = obstacles.at(i);

        Eigen::Vector3d p = (obstacle.coords.cast<double>() + Eigen::Vector3d::Constant(obstacle.width / 2.0)) * octree.resolution();

        EXPECT_LE((p - obstaclePos_w).cwiseAbs().maxCoeff(), octree.resolution());
    }
}

// Test #5: disassemble and assemble tree, ensuring that reconstructed tree
// is the same as the original tree
TEST(OcTree, TreeIntegrity)
{
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    px::OcTree octree(0.25, 8, center);

    double nodePos[3] = {5.0, 5.0, 5.24};
    px::OcNodePtr node1 = octree.insertNode(nodePos[0], nodePos[1], nodePos[2]);
    node1->setLogOdds(4.0);

    boost::multi_array<char, 1> treeData;
    octree.write(treeData);
    octree.read(treeData);

    std::vector<px::OccupancyCell> obstacles = octree.obstacles();
    ASSERT_EQ(1, obstacles.size());

    Eigen::Vector3d p = (obstacles[0].coords.cast<double>() + Eigen::Vector3d::Constant(obstacles[0].width / 2.0)) * octree.resolution();

    EXPECT_LE(fabsf(p(0) - nodePos[0]), octree.resolution() / 2.0);
    EXPECT_LE(fabsf(p(1) - nodePos[1]), octree.resolution() / 2.0);
    EXPECT_LE(fabsf(p(2) - nodePos[2]), octree.resolution() / 2.0);
}

// Test #6: write tree to file and read tree from file, ensuring that
// both trees are identical
TEST(OcTree, IO)
{
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    px::OcTree octree(0.25, 8, center);

    double nodePos[3] = {5.0, 5.0, 5.24};
    px::OcNodePtr node1 = octree.insertNode(nodePos[0], nodePos[1], nodePos[2]);
    node1->setLogOdds(4.0);

    std::string filename("tree.dat");
    octree.write(filename);
    octree.read(filename);

    std::vector<px::OccupancyCell> obstacles = octree.obstacles();
    ASSERT_EQ(1, obstacles.size());

    Eigen::Vector3d p = (obstacles[0].coords.cast<double>() + Eigen::Vector3d::Constant(obstacles[0].width / 2.0)) * octree.resolution();

    EXPECT_LE(fabsf(p(0) - nodePos[0]), octree.resolution() / 2.0);
    EXPECT_LE(fabsf(p(1) - nodePos[1]), octree.resolution() / 2.0);
    EXPECT_LE(fabsf(p(2) - nodePos[2]), octree.resolution() / 2.0);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
