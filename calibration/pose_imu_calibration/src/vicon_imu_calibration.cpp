#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <iomanip>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "cauldron/EigenUtils.h"
#include "pose_imu_calibration/PoseIMUCalibration.h"
#include "SensorDataBuffer.h"

px::PosePtr
getInterpData(px::SensorDataBuffer<geometry_msgs::PoseWithCovarianceStampedConstPtr>& buffer,
              ros::Time& timestamp)
{
    geometry_msgs::PoseWithCovarianceStampedConstPtr pose1, pose2;
    if (!buffer.nearest(timestamp, pose1, pose2))
    {
        return px::PosePtr();
    }

    geometry_msgs::Quaternion quat1 = pose1->pose.pose.orientation;
    geometry_msgs::Quaternion quat2 = pose2->pose.pose.orientation;

    Eigen::Quaterniond q1(quat1.w, quat1.x, quat1.y, quat1.z);
    Eigen::Quaterniond q2(quat2.w, quat2.x, quat2.y, quat2.z);

    Eigen::Vector3d t1(pose1->pose.pose.position.x,
                       pose1->pose.pose.position.y,
                       pose1->pose.pose.position.z);
    Eigen::Vector3d t2(pose2->pose.pose.position.x,
                       pose2->pose.pose.position.y,
                       pose2->pose.pose.position.z);

    double x = (timestamp - pose1->header.stamp).toSec() /
               (pose2->header.stamp - pose1->header.stamp).toSec();

    Eigen::Quaterniond q = q1.slerp(x, q2);
    Eigen::Vector3d t = x * (t2 - t1) + t1;

    px::PosePtr pose = boost::make_shared<px::Pose>();
    pose->timeStamp() = timestamp;
    pose->rotation() = q.conjugate();
    pose->translation() = pose->rotation() * (-t);

    return pose;
}

int
main(int argc, char** argv)
{
    std::string bagPath;
    std::string machine_ns;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("input-file", boost::program_options::value<std::string>(&bagPath), "Path to bag file.")
        ("machine-ns", boost::program_options::value<std::string>(&machine_ns)->default_value("/delta"), "Machine namespace.")
        ;

    boost::program_options::positional_options_description pdesc;
    pdesc.add("input-file", 1);

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(pdesc).run(), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    rosbag::Bag bag;

    try
    {
        bag.open(bagPath, rosbag::bagmode::Read);
    }
    catch (rosbag::BagIOException&)
    {
        ROS_ERROR("Unable to open bag file: %s", bagPath.c_str());
        return -1;
    }

    std::vector<std::string> topics;
    topics.push_back(ros::names::append(machine_ns, "fcu/imu"));
    topics.push_back(ros::names::append(machine_ns, "vicon/pose"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    size_t viconBufferSize = 100;
    size_t imuBufferSize = 5;
    px::SensorDataBuffer<geometry_msgs::PoseWithCovarianceStampedConstPtr> viconBuffer(viconBufferSize);
    px::SensorDataBuffer<sensor_msgs::ImuConstPtr> imuBuffer(imuBufferSize);

    unsigned int viconMsgCount = 0;
    unsigned int imuMsgCount = 0;
    sensor_msgs::ImuConstPtr imuLast;
    std::vector<sensor_msgs::ImuConstPtr> imuData;
    std::vector<px::PoseConstPtr> poseData;
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        geometry_msgs::PoseWithCovarianceStampedConstPtr p = m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
        if (p)
        {
            ++viconMsgCount;
            viconBuffer.push(p->header.stamp, p);
        }

        sensor_msgs::ImuConstPtr i = m.instantiate<sensor_msgs::Imu>();
        if (i)
        {
            ++imuMsgCount;

            if (!imuLast)
            {
                imuLast = i;
                continue;
            }

            if ((i->header.stamp - imuLast->header.stamp).toSec() < 0.1)
            {
                continue;
            }

            imuBuffer.push(imuLast->header.stamp, imuLast);

            imuLast = i;

            if (imuBuffer.size() == imuBufferSize)
            {
                ros::Time imuTime;
                sensor_msgs::ImuConstPtr imuMsg;
                imuBuffer.at(0, imuTime, imuMsg);

                px::PosePtr pose = getInterpData(viconBuffer, imuTime);
                if (!pose)
                {
                    ROS_WARN("Unable to find requested data in buffer.");
                    continue;
                }

                imuData.push_back(imuMsg);
                poseData.push_back(pose);
            }
        }
    }

    bag.close();

    ROS_INFO("Read %u Vicon messages and %u IMU messages.", viconMsgCount, imuMsgCount);
    ROS_INFO("Running hand-eye calibration with %lu samples.", poseData.size());

    px::PoseIMUCalibration calib;
    Eigen::Quaterniond q_vicon_imu;
    if (!calib.calibrate(poseData, imuData, q_vicon_imu))
    {
        ROS_ERROR("Calibration failed.");
        return 1;
    }

    double avgError, maxError;
    calib.errorStats(q_vicon_imu, avgError, maxError);

    std::cout << "Avg error: " << px::r2d(avgError) << std::endl;
    std::cout << "Max error: " << px::r2d(maxError) << std::endl;

    std::cout << q_vicon_imu.toRotationMatrix() << std::endl;

    return 0;
}
