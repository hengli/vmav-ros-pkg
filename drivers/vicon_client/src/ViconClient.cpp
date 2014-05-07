#include "ViconClient.h"

#include <iostream>
#include <ros/ros.h>

using namespace ViconDataStreamSDK::CPP;

namespace px
{

ViconClient::ViconClient()
 : m_client(boost::make_shared<Client>())
{

}

bool
ViconClient::connect(const std::string& viconHostName)
{
    // Connect to a server
    ROS_INFO("Connecting to VICON at %s...", viconHostName.c_str());

    bool connected = false;
    do
    {
        boost::lock_guard<boost::mutex> lock(m_clientMutex);

        // Direct connection
        m_client->Connect(viconHostName);

        connected = m_client->IsConnected().Connected;

        sleep(1);
    }
    while (!connected);

    boost::lock_guard<boost::mutex> lock(m_clientMutex);

    // Enable some different data types
    m_client->EnableSegmentData();
    m_client->EnableMarkerData();
    m_client->EnableUnlabeledMarkerData();
    m_client->EnableDeviceData();

    ROS_INFO("Segment Data Enabled: %s",
             toString(m_client->IsSegmentDataEnabled().Enabled).c_str());
    ROS_INFO("Marker Data Enabled: %s",
             toString(m_client->IsMarkerDataEnabled().Enabled).c_str());
    ROS_INFO("Unlabeled Marker Data Enabled: %s",
             toString(m_client->IsUnlabeledMarkerDataEnabled().Enabled).c_str());
    ROS_INFO("Device Data Enabled: %s",
             toString(m_client->IsDeviceDataEnabled().Enabled).c_str());

    // Set the streaming mode
//    m_client->SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull);
//    m_client->SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch);
    m_client->SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);

    // Set the global up axis
    m_client->SetAxisMapping(Direction::Forward,
                             Direction::Left,
                             Direction::Up);

    Output_GetAxisMapping axisMapping = m_client->GetAxisMapping();
    ROS_INFO("Axis Mapping: X-%s Y-%s Z-%s",
             toString(axisMapping.XAxis).c_str(),
             toString(axisMapping.YAxis).c_str(),
             toString(axisMapping.ZAxis).c_str());

    // Discover the version number
    Output_GetVersion version = m_client->GetVersion();
    ROS_INFO("Version: %u.%u.%u",
             version.Major, version.Minor, version.Point);

    return true;
}

bool
ViconClient::disconnect(void)
{
    boost::lock_guard<boost::mutex> lock(m_clientMutex);

    // Disconnect and dispose
    m_client->Disconnect();

    return true;
}

bool
ViconClient::isConnected(void)
{
    boost::lock_guard<boost::mutex> lock(m_clientMutex);

    Output_IsConnected output = m_client->IsConnected();

    return output.Connected;
}

void
ViconClient::setSubject(const std::string& subjectName)
{
    boost::lock_guard<boost::mutex> lock(m_clientMutex);

    m_subjectName = subjectName;
}

void
ViconClient::waitForFrame(void)
{
    // Wait for a frame
    bool getFrame = false;
    do
    {
        boost::lock_guard<boost::mutex> lock(m_clientMutex);

        if (!ros::ok())
        {
            return;
        }

        if (m_client->GetFrame().Result == Result::Success)
        {
            getFrame = true;
        }
        else
        {
            usleep(1);
        }
    }
    while (!getFrame);

    boost::lock_guard<boost::mutex> lock(m_clientMutex);

    m_frameStamp = ros::Time::now() - ros::Duration(m_client->GetLatencyTotal().Total);
}

bool
ViconClient::getPose(geometry_msgs::PoseStamped& pose)
{
    boost::lock_guard<boost::mutex> lock(m_clientMutex);

    if (m_client->GetSubjectCount().SubjectCount == 0)
    {
        return false;
    }

    int subjectId = -1;
    for (int i = 0; i < m_client->GetSubjectCount().SubjectCount; ++i)
    {
        if (!m_subjectName.compare(std::string(m_client->GetSubjectName(i).SubjectName)))
        {
            subjectId = i;
            break;
        }
    }

    if (subjectId == -1)
    {
        return false;
    }

    if (m_client->GetSegmentCount(m_subjectName).SegmentCount == 0)
    {
        return false;
    }

    std::string segmentName = m_client->GetSegmentName(m_subjectName, 0).SegmentName;

    // Get the global segment rotation
    Output_GetSegmentGlobalRotationQuaternion rotation = m_client->GetSegmentGlobalRotationQuaternion(m_subjectName, segmentName);

    // Get the global segment translation
    Output_GetSegmentGlobalTranslation translation = m_client->GetSegmentGlobalTranslation(m_subjectName, segmentName);

    if (rotation.Occluded || translation.Occluded)
    {
        return false;
    }

    pose.header.stamp = m_frameStamp;
    pose.pose.position.x = translation.Translation[0] / 1000.0;
    pose.pose.position.y = translation.Translation[1] / 1000.0;
    pose.pose.position.z = translation.Translation[2] / 1000.0;
    pose.pose.orientation.x = rotation.Rotation[0];
    pose.pose.orientation.y = rotation.Rotation[1];
    pose.pose.orientation.z = rotation.Rotation[2];
    pose.pose.orientation.w = rotation.Rotation[3];

    return true;
}

void
ViconClient::printFrame(void)
{
    boost::lock_guard<boost::mutex> lock(m_clientMutex);

    // Get the frame number
    std::cout << "Frame Number: " << m_client->GetFrameNumber().FrameNumber << std::endl;

    // Get the timecode
    Output_GetTimecode timecode = m_client->GetTimecode();

    std::cout << "Timecode: "
              << timecode.Hours               << "h "
              << timecode.Minutes             << "m "
              << timecode.Seconds             << "s "
              << timecode.Frames              << "f "
              << timecode.SubFrame            << "sf "
              << toString(timecode.FieldFlag) << " "
              << timecode.Standard            << " "
              << timecode.SubFramesPerFrame   << " "
              << timecode.UserBits            << std::endl << std::endl;

    // Get the latency
    std::cout << "Latency: " << m_client->GetLatencyTotal().Total << "s" << std::endl;

    for (unsigned int latencySampleIndex = 0;
         latencySampleIndex < m_client->GetLatencySampleCount().Count;
         ++latencySampleIndex)
    {
        std::string sampleName  = m_client->GetLatencySampleName(latencySampleIndex).Name;
        double      sampleValue = m_client->GetLatencySampleValue(sampleName).Value;

        std::cout << "  " << sampleName << " " << sampleValue << "s" << std::endl;
    }
    std::cout << std::endl;

    // Count the number of subjects
    unsigned int subjectCount = m_client->GetSubjectCount().SubjectCount;
    std::cout << "Subjects (" << subjectCount << "):" << std::endl;
    for (unsigned int subjectIndex = 0; subjectIndex < subjectCount;
         ++subjectIndex)
    {
        std::cout << "  Subject #" << subjectIndex << std::endl;

        // Get the subject name
        std::string SubjectName = m_client->GetSubjectName(subjectIndex).SubjectName;
        std::cout << "            Name: " << SubjectName << std::endl;

        // Get the root segment
        std::string RootSegment = m_client->GetSubjectRootSegmentName(SubjectName).SegmentName;
        std::cout << "    Root Segment: " << RootSegment << std::endl;

        // Count the number of segments
        unsigned int SegmentCount = m_client->GetSegmentCount(SubjectName).SegmentCount;
        std::cout << "    Segments (" << SegmentCount << "):" << std::endl;
        for (unsigned int segmentIndex = 0; segmentIndex < SegmentCount;
             ++segmentIndex)
        {
            std::cout << "      Segment #" << segmentIndex << std::endl;

            // Get the segment name
            std::string SegmentName = m_client->GetSegmentName(SubjectName, segmentIndex).SegmentName;
            std::cout << "          Name: " << SegmentName << std::endl;

            // Get the segment parent
            std::string SegmentParentName = m_client->GetSegmentParentName(SubjectName, SegmentName).SegmentName;
            std::cout << "        Parent: " << SegmentParentName << std::endl;

            // Get the segment's children
            unsigned int ChildCount = m_client->GetSegmentChildCount(SubjectName, SegmentName).SegmentCount;
            std::cout << "     Children (" << ChildCount << "):" << std::endl;
            for (unsigned int ChildIndex = 0; ChildIndex < ChildCount; ++ChildIndex)
            {
                std::string ChildName = m_client->GetSegmentChildName(SubjectName, SegmentName, ChildIndex).SegmentName;
                std::cout << "       " << ChildName << std::endl;
            }

            // Get the static segment translation
            Output_GetSegmentStaticTranslation _Output_GetSegmentStaticTranslation =
                m_client->GetSegmentStaticTranslation(SubjectName, SegmentName);
            std::cout << "        Static Translation: (" << _Output_GetSegmentStaticTranslation.Translation[0]  << ", "
                                                         << _Output_GetSegmentStaticTranslation.Translation[1]  << ", "
                                                         << _Output_GetSegmentStaticTranslation.Translation[2]  << ") " << std::endl;

            // Get the static segment rotation in helical co-ordinates
            Output_GetSegmentStaticRotationHelical _Output_GetSegmentStaticRotationHelical =
                m_client->GetSegmentStaticRotationHelical(SubjectName, SegmentName);
            std::cout << "        Static Rotation Helical: (" << _Output_GetSegmentStaticRotationHelical.Rotation[0]     << ", "
                                                              << _Output_GetSegmentStaticRotationHelical.Rotation[1]     << ", "
                                                              << _Output_GetSegmentStaticRotationHelical.Rotation[2]     << ") " << std::endl;

            // Get the static segment rotation as a matrix
            Output_GetSegmentStaticRotationMatrix _Output_GetSegmentStaticRotationMatrix =
                m_client->GetSegmentStaticRotationMatrix(SubjectName, SegmentName);
            std::cout << "        Static Rotation Matrix: (" << _Output_GetSegmentStaticRotationMatrix.Rotation[0]     << ", "
                                                             << _Output_GetSegmentStaticRotationMatrix.Rotation[1]     << ", "
                                                             << _Output_GetSegmentStaticRotationMatrix.Rotation[2]     << ", "
                                                             << _Output_GetSegmentStaticRotationMatrix.Rotation[3]     << ", "
                                                             << _Output_GetSegmentStaticRotationMatrix.Rotation[4]     << ", "
                                                             << _Output_GetSegmentStaticRotationMatrix.Rotation[5]     << ", "
                                                             << _Output_GetSegmentStaticRotationMatrix.Rotation[6]     << ", "
                                                             << _Output_GetSegmentStaticRotationMatrix.Rotation[7]     << ", "
                                                             << _Output_GetSegmentStaticRotationMatrix.Rotation[8]     << ") " << std::endl;

            // Get the static segment rotation in quaternion co-ordinates
            Output_GetSegmentStaticRotationQuaternion _Output_GetSegmentStaticRotationQuaternion =
                m_client->GetSegmentStaticRotationQuaternion(SubjectName, SegmentName);
            std::cout << "        Static Rotation Quaternion: (" << _Output_GetSegmentStaticRotationQuaternion.Rotation[0]     << ", "
                                                                 << _Output_GetSegmentStaticRotationQuaternion.Rotation[1]     << ", "
                                                                 << _Output_GetSegmentStaticRotationQuaternion.Rotation[2]     << ", "
                                                                 << _Output_GetSegmentStaticRotationQuaternion.Rotation[3]     << ") " << std::endl;

            // Get the static segment rotation in EulerXYZ co-ordinates
            Output_GetSegmentStaticRotationEulerXYZ _Output_GetSegmentStaticRotationEulerXYZ =
                m_client->GetSegmentStaticRotationEulerXYZ(SubjectName, SegmentName);
            std::cout << "        Static Rotation EulerXYZ: (" << _Output_GetSegmentStaticRotationEulerXYZ.Rotation[0]     << ", "
                                                               << _Output_GetSegmentStaticRotationEulerXYZ.Rotation[1]     << ", "
                                                               << _Output_GetSegmentStaticRotationEulerXYZ.Rotation[2]     << ") " << std::endl;

            // Get the global segment translation
            Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation =
                m_client->GetSegmentGlobalTranslation(SubjectName, SegmentName);
            std::cout << "        Global Translation: (" << _Output_GetSegmentGlobalTranslation.Translation[0]  << ", "
                                                         << _Output_GetSegmentGlobalTranslation.Translation[1]  << ", "
                                                         << _Output_GetSegmentGlobalTranslation.Translation[2]  << ") "
                                                         << toString(_Output_GetSegmentGlobalTranslation.Occluded) << std::endl;

            // Get the global segment rotation in helical co-ordinates
            Output_GetSegmentGlobalRotationHelical _Output_GetSegmentGlobalRotationHelical =
                m_client->GetSegmentGlobalRotationHelical(SubjectName, SegmentName);
            std::cout << "        Global Rotation Helical: (" << _Output_GetSegmentGlobalRotationHelical.Rotation[0]     << ", "
                                                              << _Output_GetSegmentGlobalRotationHelical.Rotation[1]     << ", "
                                                              << _Output_GetSegmentGlobalRotationHelical.Rotation[2]     << ") "
                                                              << toString(_Output_GetSegmentGlobalRotationHelical.Occluded) << std::endl;

            // Get the global segment rotation as a matrix
            Output_GetSegmentGlobalRotationMatrix _Output_GetSegmentGlobalRotationMatrix =
                m_client->GetSegmentGlobalRotationMatrix(SubjectName, SegmentName);
            std::cout << "        Global Rotation Matrix: (" << _Output_GetSegmentGlobalRotationMatrix.Rotation[0]     << ", "
                                                             << _Output_GetSegmentGlobalRotationMatrix.Rotation[1]     << ", "
                                                             << _Output_GetSegmentGlobalRotationMatrix.Rotation[2]     << ", "
                                                             << _Output_GetSegmentGlobalRotationMatrix.Rotation[3]     << ", "
                                                             << _Output_GetSegmentGlobalRotationMatrix.Rotation[4]     << ", "
                                                             << _Output_GetSegmentGlobalRotationMatrix.Rotation[5]     << ", "
                                                             << _Output_GetSegmentGlobalRotationMatrix.Rotation[6]     << ", "
                                                             << _Output_GetSegmentGlobalRotationMatrix.Rotation[7]     << ", "
                                                             << _Output_GetSegmentGlobalRotationMatrix.Rotation[8]     << ") "
                                                             << toString(_Output_GetSegmentGlobalRotationMatrix.Occluded) << std::endl;

            // Get the global segment rotation in quaternion co-ordinates
            Output_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion =
                m_client->GetSegmentGlobalRotationQuaternion(SubjectName, SegmentName);
            std::cout << "        Global Rotation Quaternion: (" << _Output_GetSegmentGlobalRotationQuaternion.Rotation[0]     << ", "
                                                                 << _Output_GetSegmentGlobalRotationQuaternion.Rotation[1]     << ", "
                                                                 << _Output_GetSegmentGlobalRotationQuaternion.Rotation[2]     << ", "
                                                                 << _Output_GetSegmentGlobalRotationQuaternion.Rotation[3]     << ") "
                                                                 << toString(_Output_GetSegmentGlobalRotationQuaternion.Occluded) << std::endl;

            // Get the global segment rotation in EulerXYZ co-ordinates
            Output_GetSegmentGlobalRotationEulerXYZ _Output_GetSegmentGlobalRotationEulerXYZ =
                m_client->GetSegmentGlobalRotationEulerXYZ(SubjectName, SegmentName);
            std::cout << "        Global Rotation EulerXYZ: (" << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[0]     << ", "
                                                               << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[1]     << ", "
                                                               << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[2]     << ") "
                                                               << toString(_Output_GetSegmentGlobalRotationEulerXYZ.Occluded) << std::endl;

            // Get the local segment translation
            Output_GetSegmentLocalTranslation _Output_GetSegmentLocalTranslation =
                m_client->GetSegmentLocalTranslation(SubjectName, SegmentName);
            std::cout << "        Local Translation: (" << _Output_GetSegmentLocalTranslation.Translation[0]  << ", "
                                                        << _Output_GetSegmentLocalTranslation.Translation[1]  << ", "
                                                        << _Output_GetSegmentLocalTranslation.Translation[2]  << ") "
                                                        << toString(_Output_GetSegmentLocalTranslation.Occluded) << std::endl;

            // Get the local segment rotation in helical co-ordinates
            Output_GetSegmentLocalRotationHelical _Output_GetSegmentLocalRotationHelical =
                m_client->GetSegmentLocalRotationHelical(SubjectName, SegmentName);
            std::cout << "        Local Rotation Helical: (" << _Output_GetSegmentLocalRotationHelical.Rotation[0]     << ", "
                                                             << _Output_GetSegmentLocalRotationHelical.Rotation[1]     << ", "
                                                             << _Output_GetSegmentLocalRotationHelical.Rotation[2]     << ") "
                                                             << toString(_Output_GetSegmentLocalRotationHelical.Occluded) << std::endl;

            // Get the local segment rotation as a matrix
            Output_GetSegmentLocalRotationMatrix _Output_GetSegmentLocalRotationMatrix =
                m_client->GetSegmentLocalRotationMatrix(SubjectName, SegmentName);
            std::cout << "        Local Rotation Matrix: (" << _Output_GetSegmentLocalRotationMatrix.Rotation[0]     << ", "
                                                            << _Output_GetSegmentLocalRotationMatrix.Rotation[1]     << ", "
                                                            << _Output_GetSegmentLocalRotationMatrix.Rotation[2]     << ", "
                                                            << _Output_GetSegmentLocalRotationMatrix.Rotation[3]     << ", "
                                                            << _Output_GetSegmentLocalRotationMatrix.Rotation[4]     << ", "
                                                            << _Output_GetSegmentLocalRotationMatrix.Rotation[5]     << ", "
                                                            << _Output_GetSegmentLocalRotationMatrix.Rotation[6]     << ", "
                                                            << _Output_GetSegmentLocalRotationMatrix.Rotation[7]     << ", "
                                                            << _Output_GetSegmentLocalRotationMatrix.Rotation[8]     << ") "
                                                            << toString(_Output_GetSegmentLocalRotationMatrix.Occluded) << std::endl;

            // Get the local segment rotation in quaternion co-ordinates
            Output_GetSegmentLocalRotationQuaternion _Output_GetSegmentLocalRotationQuaternion =
                m_client->GetSegmentLocalRotationQuaternion(SubjectName, SegmentName);
            std::cout << "        Local Rotation Quaternion: (" << _Output_GetSegmentLocalRotationQuaternion.Rotation[0]     << ", "
                                                                << _Output_GetSegmentLocalRotationQuaternion.Rotation[1]     << ", "
                                                                << _Output_GetSegmentLocalRotationQuaternion.Rotation[2]     << ", "
                                                                << _Output_GetSegmentLocalRotationQuaternion.Rotation[3]     << ") "
                                                                << toString(_Output_GetSegmentLocalRotationQuaternion.Occluded) << std::endl;

            // Get the local segment rotation in EulerXYZ co-ordinates
            Output_GetSegmentLocalRotationEulerXYZ _Output_GetSegmentLocalRotationEulerXYZ =
                m_client->GetSegmentLocalRotationEulerXYZ(SubjectName, SegmentName);
            std::cout << "        Local Rotation EulerXYZ: (" << _Output_GetSegmentLocalRotationEulerXYZ.Rotation[0]     << ", "
                                                              << _Output_GetSegmentLocalRotationEulerXYZ.Rotation[1]     << ", "
                                                              << _Output_GetSegmentLocalRotationEulerXYZ.Rotation[2]     << ") "
                                                              << toString(_Output_GetSegmentLocalRotationEulerXYZ.Occluded) << std::endl;
        }

        // Count the number of markers
        unsigned int MarkerCount = m_client->GetMarkerCount(SubjectName).MarkerCount;
        std::cout << "    Markers (" << MarkerCount << "):" << std::endl;
        for (unsigned int MarkerIndex = 0; MarkerIndex < MarkerCount; ++MarkerIndex)
        {
            // Get the marker name
            std::string MarkerName = m_client->GetMarkerName(SubjectName, MarkerIndex).MarkerName;

            // Get the marker parent
            std::string MarkerParentName = m_client->GetMarkerParentName(SubjectName, MarkerName).SegmentName;

            // Get the global marker translation
            Output_GetMarkerGlobalTranslation _Output_GetMarkerGlobalTranslation =
                m_client->GetMarkerGlobalTranslation(SubjectName, MarkerName);

            std::cout << "      Marker #" << MarkerIndex            << ": "
                                          << MarkerName             << " ("
                                          << _Output_GetMarkerGlobalTranslation.Translation[0]  << ", "
                                          << _Output_GetMarkerGlobalTranslation.Translation[1]  << ", "
                                          << _Output_GetMarkerGlobalTranslation.Translation[2]  << ") "
                                          << toString(_Output_GetMarkerGlobalTranslation.Occluded) << std::endl;
        }
    }

    // Get the unlabeled markers
    unsigned int unlabeledMarkerCount = m_client->GetUnlabeledMarkerCount().MarkerCount;
    std::cout << "  Unlabeled Markers (" << unlabeledMarkerCount << "):" << std::endl;
    for (unsigned int unlabeledMarkerIndex = 0;
         unlabeledMarkerIndex < unlabeledMarkerCount;
         ++unlabeledMarkerIndex)
    {
        // Get the global marker translation
        Output_GetUnlabeledMarkerGlobalTranslation unlabeledMarkerGlobalTranslation =
            m_client->GetUnlabeledMarkerGlobalTranslation(unlabeledMarkerIndex);

        std::cout << "      Marker #" << unlabeledMarkerIndex << ": ("
                                      << unlabeledMarkerGlobalTranslation.Translation[0] << ", "
                                      << unlabeledMarkerGlobalTranslation.Translation[1] << ", "
                                      << unlabeledMarkerGlobalTranslation.Translation[2] << ") " << std::endl;
    }

    // Count the number of devices
    unsigned int deviceCount = m_client->GetDeviceCount().DeviceCount;
    std::cout << "  Devices (" << deviceCount << "):" << std::endl;
    for (unsigned int deviceIndex = 0; deviceIndex < deviceCount;
         ++deviceIndex)
    {
        std::cout << "    Device #" << deviceIndex << ":" << std::endl;

        // Get the device name and type
        Output_GetDeviceName deviceName = m_client->GetDeviceName(deviceIndex);
        std::cout << "      Name: " << std::string(deviceName.DeviceName) << std::endl;
        std::cout << "      Type: " << toString(deviceName.DeviceType) << std::endl;

        // Count the number of device outputs
        unsigned int deviceOutputCount = m_client->GetDeviceOutputCount(deviceName.DeviceName).DeviceOutputCount;
        std::cout << "      Device Outputs (" << deviceOutputCount << "):" << std::endl;
        for (unsigned int deviceOutputIndex = 0;
             deviceOutputIndex < deviceOutputCount;
             ++deviceOutputIndex)
        {
            // Get the device output name and unit
            Output_GetDeviceOutputName deviceOutputName =
                m_client->GetDeviceOutputName(deviceName.DeviceName, deviceOutputIndex);

            // Get the device output value
            Output_GetDeviceOutputValue deviceOutputValue =
                m_client->GetDeviceOutputValue(deviceName.DeviceName, deviceOutputName.DeviceOutputName);

            std::cout << "        Device Output #" << deviceOutputIndex << ": "
                                                   << std::string(deviceOutputName.DeviceOutputName) << " "
                                                   << deviceOutputValue.Value << " "
                                                   << toString(deviceOutputName.DeviceOutputUnit) << " "
                                                   << toString(deviceOutputValue.Occluded) << std::endl;
        }
    }
}

std::string
ViconClient::toString(const bool value) const
{
    return value ? "True" : "False";
}

std::string
ViconClient::toString(const Direction::Enum direction) const
{
    switch (direction)
    {
    case Direction::Forward:
        return "Forward";
    case Direction::Backward:
        return "Backward";
    case Direction::Left:
        return "Left";
    case Direction::Right:
        return "Right";
    case Direction::Up:
        return "Up";
    case Direction::Down:
        return "Down";
    default:
        return "Unknown";
    }
}

std::string
ViconClient::toString(const DeviceType::Enum deviceType) const
{
    switch (deviceType)
    {
    case DeviceType::ForcePlate:
        return "ForcePlate";
    case DeviceType::Unknown:
    default:
        return "Unknown";
    }
}

std::string
ViconClient::toString(const Unit::Enum unit) const
{
    switch (unit)
    {
    case Unit::Meter:
        return "Meter";
    case Unit::Volt:
        return "Volt";
    case Unit::NewtonMeter:
        return "NewtonMeter";
    case Unit::Newton:
        return "Newton";
    case Unit::Unknown:
    default:
        return "Unknown";
    }
}

}
