#include <eigen_conversions/eigen_msg.h>
#include <gtest/gtest.h>
#include <iostream>

#include "dynocmap/DynocMap.h"
#include "sensor_models/LaserSensorModel.h"

double resolution = 0.25;
double obstacleRange = 1.9;
double freeSpaceProbability = 0.0;
double occSpaceProbability = 1.0;
double maxSensorRange = 20.0;
double sensorSigma = 0.02;

bool
expectObstacle(const Eigen::Vector3d& pExpected,
               const Eigen::Vector3i& p, int width)
{
    for (int i = 0; i < 3; ++i)
    {
        if (pExpected(i) < p(i) * resolution)
        {
            return false;
        }
        if (pExpected(i) > (p(i) + width) * resolution)
        {
            return false;
        }
    }

    return true;
}

TEST(DynocMap, CastRay1)
{
    px::SensorModelPtr sensorModel(new px::LaserSensorModel(freeSpaceProbability,
                                                            occSpaceProbability,
                                                            maxSensorRange,
                                                            sensorSigma));

    px::DynocMap map(resolution, sensorModel, "mapcache");

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();

    geometry_msgs::Pose cameraPose;
    tf::quaternionEigenToMsg(q, cameraPose.orientation);
    tf::pointEigenToMsg(t, cameraPose.position);

    Eigen::Vector3d pObstacleLocal(Eigen::Vector3d(obstacleRange, 0.0, 0.0));

    map.castRay(cameraPose, pObstacleLocal, px::DynocMap::SENSOR_FRAME);

    std::vector<px::OccupancyTile> tiles = map.tiles();

    std::vector<px::OccupancyCell> obstacles;
    for (size_t i = 0; i < tiles.size(); ++i)
    {
        std::vector<px::OccupancyCell> tileObstacles = tiles.at(i).obstacles();

        obstacles.insert(obstacles.end(), tileObstacles.begin(), tileObstacles.end());
    }

    ASSERT_EQ(1, obstacles.size());

    Eigen::Vector3i center = obstacles.front().coords;
    int width = obstacles.front().width;

    Eigen::Vector3d pObstacleGlobal = q.toRotationMatrix() * pObstacleLocal + t;

    EXPECT_TRUE(expectObstacle(pObstacleGlobal, center, width));
}

TEST(DynocMap, CastRay2)
{
    px::SensorModelPtr sensorModel(new px::LaserSensorModel(freeSpaceProbability,
                                                            occSpaceProbability,
                                                            maxSensorRange,
                                                            sensorSigma));

    px::DynocMap map(resolution, sensorModel, "mapcache");

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t(-obstacleRange / 2.0, 0.0, 0.0);

    geometry_msgs::Pose cameraPose;
    tf::quaternionEigenToMsg(q, cameraPose.orientation);
    tf::pointEigenToMsg(t, cameraPose.position);

    Eigen::Vector3d pObstacleLocal(Eigen::Vector3d(obstacleRange, 0.0, 0.0));

    map.castRay(cameraPose, pObstacleLocal, px::DynocMap::SENSOR_FRAME);

    std::vector<px::OccupancyTile> tiles = map.tiles();

    std::vector<px::OccupancyCell> obstacles;
    for (size_t i = 0; i < tiles.size(); ++i)
    {
        std::vector<px::OccupancyCell> tileObstacles = tiles.at(i).obstacles();

        obstacles.insert(obstacles.end(), tileObstacles.begin(), tileObstacles.end());
    }

    ASSERT_EQ(1, obstacles.size());

    Eigen::Vector3i center = obstacles.front().coords;
    int width = obstacles.front().width;

    Eigen::Vector3d pObstacleGlobal = q.toRotationMatrix() * pObstacleLocal + t;

    EXPECT_TRUE(expectObstacle(pObstacleGlobal, center, width));
}

// Create a map centered at the origin, and insert an obstacle within
// the map boundary. Recenter the map such that the obstacle lies outside
// the map boundary, and check for a zero obstacle count. Recenter the map
// back at the origin, and check that the obstacle exists.
TEST(DynocMap, Recenter1)
{
    px::SensorModelPtr sensorModel(new px::LaserSensorModel(freeSpaceProbability,
                                                            occSpaceProbability,
                                                            maxSensorRange,
                                                            sensorSigma));

    px::DynocMap map(resolution, sensorModel, "mapcache");

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();

    geometry_msgs::Pose cameraPose;
    tf::quaternionEigenToMsg(q, cameraPose.orientation);
    tf::pointEigenToMsg(t, cameraPose.position);

    Eigen::Vector3d pObstacleLocal(Eigen::Vector3d(obstacleRange, 0.0, 0.0));

    map.castRay(cameraPose, pObstacleLocal, px::DynocMap::SENSOR_FRAME);

    map.recenter(Eigen::Vector3d::Constant(1000.0));

    std::vector<px::OccupancyTile> tiles = map.tiles();

    int nObstacles = 0;
    for (size_t i = 0; i < tiles.size(); ++i)
    {
        nObstacles += tiles.at(i).obstacles().size();
    }

    ASSERT_EQ(0, nObstacles);

    map.recenter(Eigen::Vector3d::Zero());

    tiles = map.tiles();

    std::vector<px::OccupancyCell> obstacles;
    for (size_t i = 0; i < tiles.size(); ++i)
    {
        std::vector<px::OccupancyCell> tileObstacles = tiles.at(i).obstacles();

        obstacles.insert(obstacles.end(), tileObstacles.begin(), tileObstacles.end());
    }

    ASSERT_EQ(1, obstacles.size());

    Eigen::Vector3i center = obstacles.front().coords;
    int width = obstacles.front().width;

    Eigen::Vector3d pObstacleGlobal = q.toRotationMatrix() * pObstacleLocal + t;

    EXPECT_TRUE(expectObstacle(pObstacleGlobal, center, width));
}

// Create a map centered at the origin, and insert an obstacle within the
// map boundary. Recenter the map such that the map origin moves 1 tile in
// any direction, and insert an obstacle such that the ray intersects 2 tiles.
// Check that the two obstacles exist.
TEST(DynocMap, Recenter2)
{
    for (int i = -1; i <= 1; ++i)
    {
        for (int j = -1; j <= 1; ++j)
        {
            for (int k = -1; k <= 1; ++k)
            {
                if (std::abs(i) + std::abs(j) + std::abs(k) != 1)
                {
                    continue;
                }

                px::SensorModelPtr sensorModel(new px::LaserSensorModel(freeSpaceProbability,
                                                                        occSpaceProbability,
                                                                        maxSensorRange,
                                                                        sensorSigma));

                px::DynocMap map(resolution, sensorModel, "mapcache");

                Eigen::Quaterniond q[2];
                Eigen::Vector3d t[2];

                q[0] = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(i, j, k));
                t[0] = Eigen::Vector3d::Zero();

                q[1] = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(i, j, k));
                t[1] = q[1].toRotationMatrix() * Eigen::Vector3d(map.tileWidth() - obstacleRange / 2.0, 0.0, 0.0);

                Eigen::Vector3d pObstacleLocal;
                pObstacleLocal = Eigen::Vector3d(obstacleRange, 0.0, 0.0);

                geometry_msgs::Pose cameraPose;
                tf::quaternionEigenToMsg(q[0], cameraPose.orientation);
                tf::pointEigenToMsg(t[0], cameraPose.position);

                map.castRay(cameraPose, pObstacleLocal, px::DynocMap::SENSOR_FRAME);

                map.recenter(t[1]);

                tf::quaternionEigenToMsg(q[1], cameraPose.orientation);
                tf::pointEigenToMsg(t[1], cameraPose.position);

                map.castRay(cameraPose, pObstacleLocal, px::DynocMap::SENSOR_FRAME);

                std::vector<px::OccupancyTile> tiles = map.tiles();

                std::vector<px::OccupancyCell> obstacles;
                for (size_t l = 0; l < tiles.size(); ++l)
                {
                    std::vector<px::OccupancyCell> tileObstacles = tiles.at(l).obstacles();

                    obstacles.insert(obstacles.end(), tileObstacles.begin(), tileObstacles.end());
                }

                ASSERT_EQ(2, obstacles.size());

//                Eigen::Vector3d pObstacleGlobal[2];
//                for (int l = 0; l < 2; ++l)
//                {
//                    pObstacleGlobal[l] = (cameraPose[l].q() *
//                                         sensorModel->sensorToWorldTransform()).toRotationMatrix() *
//                                         pObstacleLocal + cameraPose[l].t();
//                }
//
//                for (int l = 0; l < 2; ++l)
//                {
//                    Eigen::Vector3d center = obstacles.at(l).coords;
//                    double width = obstacles.at(l).width;
//
//                    EXPECT_TRUE(expectObstacle(pObstacleGlobal[l], center, width) ||
//                                expectObstacle(pObstacleGlobal[1 - l], center, width));
//                }
            }
        }
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
