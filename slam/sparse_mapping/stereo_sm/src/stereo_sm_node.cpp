#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <px_comm/CameraInfo.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "camera_models/CameraFactory.h"
#include "camera_systems/CameraSystem.h"
#include "stereo_sm/StereoSM.h"

void
voCallback(const sensor_msgs::ImageConstPtr& imageMsg1,
           const sensor_msgs::ImageConstPtr& imageMsg2,
           boost::shared_ptr<px::StereoSM> ssm)
{
    cv_bridge::CvImageConstPtr cv_ptr;

    cv::Mat image1, image2;

    try
    {
        cv_ptr = cv_bridge::toCvShare(imageMsg1);
        cv_ptr->image.copyTo(image1);

        cv_ptr = cv_bridge::toCvShare(imageMsg2);
        cv_ptr->image.copyTo(image2);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ssm->readFrames(imageMsg1->header.stamp, image1, image2);

    ssm->processFrames();

    ROS_INFO("Processed images with timestamp %f.", imageMsg1->header.stamp.toSec());
}

template<class M>
class BagSubscriber: public message_filters::SimpleFilter<M>
{
public:
    void newMessage(const boost::shared_ptr<const M>& msg)
    {
        this->signalMessage(msg);
    }
};

int
main(int argc, char** argv)
{
    std::string bagFilename;
    std::string vocFilename;
    std::string cameraNs1, cameraNs2;
    std::string extrinsicFilename;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("input,i", boost::program_options::value<std::string>(&bagFilename), "ROS bag filename.")
        ("voc", boost::program_options::value<std::string>(&vocFilename)->default_value("orb.yml.gz"), "Vocabulary filename.")
        ("camera_ns_1", boost::program_options::value<std::string>(&cameraNs1), "Namespace of camera 1.")
        ("camera_ns_2", boost::program_options::value<std::string>(&cameraNs2), "Namespace of camera 2.")
        ("extrinsics", boost::program_options::value<std::string>(&extrinsicFilename), "Extrinsic filename.")
        ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    ros::init(argc, argv, "stereo_sm");

    // get extrinsics
    px::CameraSystemPtr cameraSystem = boost::make_shared<px::CameraSystem>(2);
    if (!cameraSystem->readPosesFromTextFile(extrinsicFilename))
    {
        ROS_ERROR("Failed to read extrinsic file %s.", extrinsicFilename.c_str());
        return 1;
    }

    rosbag::Bag bag;
    try
    {
        bag.open(bagFilename, rosbag::bagmode::Read);
    }
    catch (rosbag::BagIOException&)
    {
        ROS_ERROR("Unable to open bag file: %s", bagFilename.c_str());
        return 1;
    }

    std::string camInfo1TopicName = cameraNs1 + "/camera_info";
    std::string camImage1TopicName = cameraNs1 + "/image_raw";
    std::string camInfo2TopicName = cameraNs2 + "/camera_info";
    std::string camImage2TopicName = cameraNs2 + "/image_raw";

    std::vector<std::string> topics;
    topics.push_back(camInfo1TopicName);
    topics.push_back(camImage1TopicName);
    topics.push_back(camInfo2TopicName);
    topics.push_back(camImage2TopicName);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    ros::NodeHandle nh;

    boost::shared_ptr<px::StereoSM> ssm;
    px::SparseGraphPtr sparseGraph = boost::make_shared<px::SparseGraph>();

    px::CameraPtr camera1, camera2;
    BagSubscriber<sensor_msgs::Image> camImage1Sub, camImage2Sub;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(camImage1Sub, camImage2Sub, 10);

    bool init = false;
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        if (!ros::ok())
        {
            break;
        }

        if (m.getTopic() == camInfo1TopicName && !camera1)
        {
            px_comm::CameraInfo::ConstPtr cameraInfo = m.instantiate<px_comm::CameraInfo>();

            if (cameraInfo)
            {
                camera1 = px::CameraFactory::instance()->generateCamera(cameraInfo);

                if (camera2)
                {
                    init = true;
                }
            }
        }

        if (m.getTopic() == camInfo2TopicName && !camera2)
        {
            px_comm::CameraInfo::ConstPtr cameraInfo = m.instantiate<px_comm::CameraInfo>();

            if (cameraInfo)
            {
                camera2 = px::CameraFactory::instance()->generateCamera(cameraInfo);

                if (camera1)
                {
                    init = true;
                }
            }
        }

        if (init)
        {
            cameraSystem->setCamera(0, camera1);
            cameraSystem->setCamera(1, camera2);

            ssm = boost::make_shared<px::StereoSM>(boost::ref(nh),
                                                   cameraSystem,
                                                   boost::ref(sparseGraph));
            if (!ssm->init("STAR", "ORB", "BruteForce-Hamming"))
            {
                ROS_ERROR("Failed to initialize stereo sparse mapping.");
                return 1;
            }

            sync.registerCallback(boost::bind(&voCallback, _1, _2, ssm));

            ROS_INFO("Initialized stereo sparse mapping.");

            init = false;
        }

        if (camera1 && camera2)
        {
            if (m.getTopic() == camImage1TopicName)
            {
                sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();

                if (img)
                {
                    camImage1Sub.newMessage(img);
                }
            }

            if (m.getTopic() == camImage2TopicName)
            {
                sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();

                if (img)
                {
                    camImage2Sub.newMessage(img);
                }
            }
        }
    }

    bag.close();

    if (!ros::ok())
    {
        ROS_ERROR("Aborted.");
        return 1;
    }

    ROS_INFO("Detecting loop closures and running pose graph optimization...");

    ssm->runPG(vocFilename);

    ROS_INFO("Running full bundle adjustment...");

    ssm->runBA();

    return 0;
}
