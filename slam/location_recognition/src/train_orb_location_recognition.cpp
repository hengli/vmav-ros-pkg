#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <ros/ros.h>

#include "location_recognition/OrbLocationRecognition.h"

int main(int argc, char** argv)
{
    std::string inputDir;
    std::string vocFilename;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("input,i", boost::program_options::value<std::string>(&inputDir), "Directory containing training images.")
        ("output,o", boost::program_options::value<std::string>(&vocFilename)->default_value("orb.yml.gz"), "Vocabulary file name.")
        ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    // look for images in input directory
    std::vector<std::string> imageFilenames;
    boost::filesystem::directory_iterator itr;
    for (boost::filesystem::directory_iterator itr(inputDir); itr != boost::filesystem::directory_iterator(); ++itr)
    {
        if (!boost::filesystem::is_regular_file(itr->status()))
        {
            continue;
        }

        imageFilenames.push_back(itr->path().string());
    }

    if (imageFilenames.empty())
    {
        ROS_ERROR("No training images found.");
        return 1;
    }

    ROS_INFO("# training images: %lu", imageFilenames.size());

    ros::Time::init();
    ros::Time tsStart = ros::Time::now();

    px::OrbLocationRecognition lr;
    if (!lr.createVocabulary(imageFilenames, "STAR", vocFilename))
    {
        ROS_ERROR("Failed to create vocabulary.");
        return 1;
    }

    ROS_INFO("Vocabulary creation took %.1f seconds.",
             (ros::Time::now() - tsStart).toSec());

    return 0;
}
