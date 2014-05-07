#include <boost/program_options.hpp>
#include <fstream>
#include <ros/ros.h>

#include "camera_calibration/StereoCameraCalibration.h"

int main(int argc, char** argv)
{
    std::string filenameIntL;
    std::string filenameIntR;
    std::string filenameExt;
    std::string cameraNameL;
    std::string cameraNameR;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("int-l", boost::program_options::value<std::string>(&filenameIntL)->default_value("camL_chessboard_data.dat"), "Path to file containing chessboard data for left camera")
        ("int-r", boost::program_options::value<std::string>(&filenameIntR)->default_value("camR_chessboard_data.dat"), "Path to file containing chessboard data for right camera")
        ("ext", boost::program_options::value<std::string>(&filenameExt)->default_value("camL_camR_extrinsics.txt"), "Path to file containing extrinsic data for both cameras")
        ("name-l", boost::program_options::value<std::string>(&cameraNameL)->default_value("camL"), "Name of left camera")
        ("name-r", boost::program_options::value<std::string>(&cameraNameR)->default_value("camR"), "Name of right camera")
        ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    px::StereoCameraCalibration scCalib;
    if (!scCalib.readChessboardData(filenameIntL, filenameIntR, filenameExt))
    {
        ROS_ERROR("Failed to read chessboard data.");
        return 1;
    }

    std::string outputFilename = cameraNameL + "_" + cameraNameR + "_chessboard_data.dat";
    std::ofstream ofs(outputFilename.c_str());

    if (!scCalib.writeChessboardData(outputFilename))
    {
        ROS_ERROR("Failed to write chessboard data.");
        return 1;
    }

    ROS_INFO("Wrote chessboard data to %s", outputFilename.c_str());

    return 0;
}
