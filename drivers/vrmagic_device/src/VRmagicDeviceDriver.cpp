#include "VRmagicDeviceDriver.h"

#include <asctec_hl_comm/CamTriggerSrv.h>
#include <driver_base/SensorLevels.h>
#include <tf/transform_listener.h>

using namespace VRmUsbCamCPP;

namespace px
{

VRmagicDeviceDriver::VRmagicDeviceDriver(ros::NodeHandle nh)
 : m_nh(nh, "vrmagic")
 , k_triggerBufferSize(20)
 , m_state(driver_base::Driver::CLOSED)
 , m_reconfiguring(false)
 , m_server(m_nh)
 , m_cycle(1000.0)
{
    m_triggerSub = nh.subscribe<asctec_hl_comm::CamTrigger>("fcu/cam_trigger", 5, boost::bind(&VRmagicDeviceDriver::cbTrigger, this, _1));

    m_config = vrmagic_device::VRmagicDeviceConfig::__getDefault__();
}

VRmagicDeviceDriver::~VRmagicDeviceDriver()
{
    stop();
}

bool
VRmagicDeviceDriver::start(int deviceId)
{
    m_cameraInfoFound = false;

    m_server.setCallback(boost::bind(&VRmagicDeviceDriver::reconfigure, this, _1, _2));

    return true;
}

bool
VRmagicDeviceDriver::stop(void)
{
    if (m_state != driver_base::Driver::CLOSED)
    {
        stopDevice();

        ROS_INFO("Stopped VRmagic device.");
    }

    return true;
}

bool
VRmagicDeviceDriver::poll(void)
{
    bool doSleep = true;

    if (!m_reconfiguring)
    {
        if (m_state == driver_base::Driver::RUNNING)
        {
            if (grabFrames())
            {
                publishFrames();
                doSleep = false;
            }
            else
            {
                doSleep = true;
            }
        }

    }

    if (doSleep)
    {
        m_cycle.sleep();
    }

    return true;
}

void
VRmagicDeviceDriver::cbTrigger(const asctec_hl_comm::CamTriggerConstPtr& trigger)
{
    boost::unique_lock<boost::mutex> lock(m_triggerBufferMutex);

    m_triggerBuffer.push_back(std::make_pair(trigger->frame_counter, trigger->header.stamp));

    while (m_triggerBuffer.size() > k_triggerBufferSize)
    {
        m_triggerBuffer.pop_front();
    }
}

void
VRmagicDeviceDriver::reconfigure(const vrmagic_device::VRmagicDeviceConfig& config,
                                 uint32_t level)
{
    m_reconfiguring = true;

    boost::unique_lock<boost::mutex> lock(m_deviceMutex);

    std::string frame_id = config.frame_id;

    // resolve frame ID using tf_prefix parameter
    if (frame_id == "")
    {
        frame_id = "vrmagic";
    }

    std::string tf_prefix = tf::getPrefixParam(m_nh);
    m_config.frame_id = tf::resolve(tf_prefix, frame_id);

    if (m_state != driver_base::Driver::CLOSED &&
        (level & driver_base::SensorLevels::RECONFIGURE_CLOSE))
    {
        stopDevice();
    }

    if (m_state == driver_base::Driver::CLOSED)
    {
        if (!startDevice(0, config))
        {
            ROS_WARN("Failed to start VRmagic device.");
            return;
        }
        readCameraInfo();
    }
    else if (level == driver_base::SensorLevels::RECONFIGURE_RUNNING)
    {
        reconfigureDevice(config);
    }

    m_reconfiguring = false;
}

bool
VRmagicDeviceDriver::startDevice(int deviceId,
                                 const vrmagic_device::VRmagicDeviceConfig& config)
{
     int version = VRmUsbCam::get_Version();
     int v[4];
     v[0] = version / 1000;
     v[1] = (version - v[0] * 1000) / 100;
     v[2] = (version - v[0] * 1000 - v[1] * 100) / 10;
     v[3] = version - v[0] * 1000 - v[1] * 100 - v[2] * 10;

     ROS_INFO("VRmagic API version: %d.%d.%d.%d", v[0], v[1], v[2], v[3]);

     std::vector<DeviceKeyPtr> deviceKeyList = VRmUsbCam::get_DeviceKeyList();
     if (deviceKeyList.empty())
     {
         ROS_FATAL("No VRmagic devices are found.");
         return false;
     }

     if (deviceId >= deviceKeyList.size())
     {
         ROS_FATAL("VRmagic device with id %d is not found.", deviceId);
         return false;
     }

     DeviceKeyPtr deviceKey = deviceKeyList.at(deviceId);

     if (deviceKey->get_Busy())
     {
         ROS_FATAL("VRmagic device is busy.");
         return false;
     }

     std::string manufacturer = deviceKey->get_Manufacturer();
     std::string product = deviceKey->get_Product();
     int serial = deviceKey->get_Serial();

     ROS_INFO("Manufacturer: %s", manufacturer.c_str());
     ROS_INFO("Product:      %s", product.c_str());
     ROS_INFO("Serial:       %d", serial);

     m_device = VRmUsbCam::OpenDevice(deviceKey);

     m_state = driver_base::Driver::OPENED;

     std::vector<int> sensorPortList = m_device->get_SensorPortList();
     m_cameras.resize(sensorPortList.size());

     // It is safe to assume that all sensors have the same image format,
     // as VRmagic only supports multiple sensors of the same type.
     ImageFormat imageFormat = m_device->get_SourceFormat(sensorPortList.at(0));
     SizeI imageSize = imageFormat.get_Size();

     if (imageSize.m_width == 754 && imageSize.m_height == 482)
     {
         imageSize.m_height = 480;
     }

     // use factory defaults
     m_device->set_PropertyValue(VRM_PROPID_GRAB_CONFIG_E,
                                 VRM_PROPID_GRAB_CONFIG_FACTORY_DEFAULTS);

     // set source format
//     if (config.source_format == "mono8")
//     {
         // grab 8-bit raw images
         m_device->set_PropertyValue(VRM_PROPID_GRAB_SOURCE_FORMAT_E,
                                     VRM_PROPID_GRAB_SOURCE_FORMAT_8BIT_RAW);

         m_config.source_format = "mono8";
//     }
//     else if (config.source_format == "mono16")
//     {
//         // grab 16-bit raw images
//         m_device->set_PropertyValue(VRM_PROPID_GRAB_SOURCE_FORMAT_E,
//                                     VRM_PROPID_GRAB_SOURCE_FORMAT_16BIT_RAW);
//     }
//     else if (config.source_format == "mono8rle")
//     {
//         // grab 8-bit RLE images
//         m_device->set_PropertyValue(VRM_PROPID_GRAB_SOURCE_FORMAT_E,
//                                     VRM_PROPID_GRAB_SOURCE_FORMAT_8BIT_RLE);
//     }

     // set ringbuffer size
     m_device->set_PropertyValue(VRM_PROPID_GRAB_HOST_RINGBUFFER_SIZE_I,
                                 static_cast<int>(m_cameras.size()) * 2);

     for (size_t i = 0; i < sensorPortList.size(); ++i)
     {
         int sensorPort = sensorPortList.at(i);
         int cameraId = sensorPort - 1;

         VRmagicCameraPtr& camera = m_cameras.at(cameraId);

         std::ostringstream oss;

         oss << product << "_" << serial << "_cam" << cameraId;
         std::string cameraName = oss.str();
         oss.str("");
         oss.clear();

         if (!m_nh.getNamespace().empty())
         {
             oss << m_nh.getNamespace() << "/";
         }
         oss << "cam" << cameraId;
         camera = boost::make_shared<VRmagicCamera>(oss.str(),
                                                    VRmagicCamera::DDS,
                                                    config.frame_rate,
                                                    cameraName);

         camera->cameraInfo()->header.frame_id = m_config.frame_id;
         camera->cameraInfo()->camera_name = cameraName;
         camera->cameraInfo()->image_width = imageSize.m_width;
         camera->cameraInfo()->image_height = imageSize.m_height;

         camera->image()->width = imageSize.m_width;
         camera->image()->height = imageSize.m_height;
         camera->image()->step = imageSize.m_width;
         camera->image()->encoding = sensor_msgs::image_encodings::MONO8;
         camera->image()->data.resize(camera->image()->step * camera->image()->height);
         camera->image()->header.frame_id = m_config.frame_id;

         camera->sensorPort() = sensorPort;

         if (camera->sensorPort() >= m_cameraMap.size())
         {
             m_cameraMap.resize(camera->sensorPort() + 1);
             m_cameraMap.at(camera->sensorPort()) = camera;
         }

         if (!camera->setup())
         {
             ROS_FATAL("Cannot set up camera %d.", cameraId);
             return false;
         }
     }

     // Start the device before configuring the device.
     // If done the other way round, the device may miss some trigger signals
     // as the trigger is started during configuration, leading to
     // mis-synchronization and incorrect image timestamps.
     m_device->Start();

     ROS_INFO("Started VRmagic device.");

     reconfigureDevice(config);

     // set up ROS topics
     for (size_t i = 0; i < m_cameras.size(); ++i)
     {
         m_cameras.at(i)->serviceServer() =
             m_cameras.at(i)->nodeHandle().advertiseService<px_comm::SetCameraInfo::Request, px_comm::SetCameraInfo::Response>("set_camera_info", boost::bind(&VRmagicDeviceDriver::updateCameraInfo, this, _1, _2, m_cameras.at(i)));
     }

     m_state = driver_base::Driver::RUNNING;

     return true;
}

void
VRmagicDeviceDriver::reconfigureDevice(const vrmagic_device::VRmagicDeviceConfig& config)
{
    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        VRmagicCameraPtr& camera = m_cameras.at(i);

        m_device->set_PropertyValue(VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_E,
                                    static_cast<PropId>(VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_1 + camera->sensorPort() - 1));

        // set frame rate
        try
        {
            m_device->set_PropertyValue(VRM_PROPID_CAM_INTERNAL_TRIGGER_RATE_F,
                                        static_cast<float>(config.frame_rate));

            m_config.frame_rate = config.frame_rate;
        }
        catch (std::exception& e)
        {
            ROS_ERROR("Failed to set frame rate: %f", config.frame_rate);
        }

        // set auto exposure
        if (m_device->get_PropertySupported(VRM_PROPID_CAM_AUTO_EXPOSURE_B))
        {
            try
            {
                m_device->set_PropertyValue(VRM_PROPID_CAM_AUTO_EXPOSURE_B,
                                            config.auto_exposure);

                m_config.auto_exposure = config.auto_exposure;
            }
            catch (std::exception& e)
            {
                ROS_ERROR("Failed to set auto exposure: %d", config.auto_exposure);
            }
        }

        // set exposure time
        try
        {
            m_device->set_PropertyValue(VRM_PROPID_CAM_EXPOSURE_TIME_F,
                                        static_cast<float>(config.exposure_time));

            m_config.exposure_time = config.exposure_time;
        }
        catch (std::exception& e)
        {
            ROS_ERROR("Failed to set exposure time: %f", config.exposure_time);
        }

        // set maximum auto exposure time
        if (m_device->get_PropertySupported(VRM_PROPID_CAM_AUTO_EXPOSURE_MAX_F))
        {
            try
            {
                m_device->set_PropertyValue(VRM_PROPID_CAM_AUTO_EXPOSURE_MAX_F,
                                            static_cast<float>(config.auto_exposure_time_max));

                m_config.auto_exposure_time_max = config.auto_exposure_time_max;
            }
            catch (std::exception& e)
            {
                ROS_ERROR("Failed to set auto exposure: %f", config.auto_exposure_time_max);
            }
        }

        // set auto gain
        if (m_device->get_PropertySupported(VRM_PROPID_CAM_AUTO_GAIN_B))
        {
            try
            {
                m_device->set_PropertyValue(VRM_PROPID_CAM_AUTO_GAIN_B,
                                            config.auto_gain);

                m_config.auto_gain = config.auto_gain;
            }
            catch (std::exception& e)
            {
                ROS_ERROR("Failed to set auto gain: %d", config.auto_gain);
            }
        }

        // set gain
        try
        {
            m_device->set_PropertyValue(VRM_PROPID_CAM_GAIN_MONOCHROME_I,
                                        config.gain);

            m_config.gain = config.gain;
        }
        catch (std::exception& e)
        {
            ROS_ERROR("Failed to set gain: %d", config.gain);
        }

        // set maximum auto gain
        if (m_device->get_PropertySupported(VRM_PROPID_CAM_AUTO_GAIN_MAX_I))
        {
            try
            {
                m_device->set_PropertyValue(VRM_PROPID_CAM_AUTO_GAIN_MAX_I,
                                            config.auto_gain_max);

                m_config.auto_gain_max = config.auto_gain_max;
            }
            catch (std::exception& e)
            {
                ROS_ERROR("Failed to set maximum auto gain: %d", config.auto_gain_max);
            }
        }

        // set anti blooming
        if (m_device->get_PropertySupported(VRM_PROPID_CAM_ANTI_BLOOMING_B))
        {
            try
            {
                m_device->set_PropertyValue(VRM_PROPID_CAM_ANTI_BLOOMING_B,
                                            config.anti_blooming);

                m_config.anti_blooming = config.anti_blooming;
            }
            catch (std::exception& e)
            {
                ROS_ERROR("Failed to set anti blooming: %d", config.anti_blooming);
            }
        }

        // set anti blooming voltage
        if (m_device->get_PropertySupported(VRM_PROPID_CAM_ANTI_BLOOMING_VOLTAGE_F))
        {
            try
            {
                m_device->set_PropertyValue(VRM_PROPID_CAM_ANTI_BLOOMING_VOLTAGE_F,
                                            static_cast<float>(config.anti_blooming_voltage));

                m_config.anti_blooming_voltage = config.anti_blooming_voltage;
            }
            catch (std::exception& e)
            {
                ROS_ERROR("Failed to set anti blooming: %f", config.anti_blooming_voltage);
            }
        }

        // set high dynamic mode
        if (m_device->get_PropertySupported(VRM_PROPID_CAM_HIGH_DYNAMIC_MODE_B))
        {
            try
            {
                m_device->set_PropertyValue(VRM_PROPID_CAM_HIGH_DYNAMIC_MODE_B,
                                            config.high_dynamic_mode);

                m_config.high_dynamic_mode = config.high_dynamic_mode;
            }
            catch (std::exception& e)
            {
                ROS_ERROR("Failed to set high dynamic mode: %d", config.high_dynamic_mode);
            }
        }

        // set bright threshold for AEC/AGC
        if (m_device->get_PropertySupported(VRM_PROPID_CAM_AUTO_EXPOSURE_BRIGHT_THRESHOLD_I))
        {
            try
            {
                m_device->set_PropertyValue(VRM_PROPID_CAM_AUTO_EXPOSURE_BRIGHT_THRESHOLD_I,
                                            config.auto_exposure_bright_threshold);

                m_config.auto_exposure_bright_threshold = config.auto_exposure_bright_threshold;
            }
            catch (std::exception& e)
            {
                ROS_ERROR("Failed to set bright threshold: %d", config.auto_exposure_bright_threshold);
            }
        }

        // set bright percentage for AEC/AGC
        if (m_device->get_PropertySupported(VRM_PROPID_CAM_AUTO_EXPOSURE_BRIGHT_PERCENTAGE_F))
        {
            try
            {
                m_device->set_PropertyValue(VRM_PROPID_CAM_AUTO_EXPOSURE_BRIGHT_PERCENTAGE_F,
                                            static_cast<float>(config.auto_exposure_bright_percentage));

                m_config.auto_exposure_bright_percentage = config.auto_exposure_bright_percentage;
            }
            catch (std::exception& e)
            {
                ROS_ERROR("Failed to set bright percentage: %f", config.auto_exposure_bright_percentage);
            }
        }
    }

    if (m_config.external_trigger != config.external_trigger)
    {
        if (!config.external_trigger)
        {
            asctec_hl_comm::CamTriggerSrv::Request req;
            asctec_hl_comm::CamTriggerSrv::Response res;

            req.startCamTrigger = false;

            if (ros::service::call("fcu/cam_trigger", req, res))
            {
                m_config.external_trigger = res.camTriggerRunning;

                ROS_INFO("External trigger: %s", m_config.external_trigger ? "on" : "off");
            }
            else
            {
                m_config.external_trigger = true;

                ROS_ERROR("Failed to switch off external trigger.");
            }
        }
        else
        {
            // external trigger
            m_device->set_PropertyValue(VRM_PROPID_GRAB_MODE_E, VRM_PROPID_GRAB_MODE_TRIGGERED_EXT);
            m_device->set_PropertyValue(VRM_PROPID_CAM_TRIGGER_POLARITY_E, VRM_PROPID_CAM_TRIGGER_POLARITY_NEG_EDGE);

            boost::unique_lock<boost::mutex> lock(m_triggerBufferMutex);
            m_triggerBuffer.clear();

            asctec_hl_comm::CamTriggerSrv::Request req;
            asctec_hl_comm::CamTriggerSrv::Response res;

            req.startCamTrigger = true;

            if (ros::service::call("fcu/cam_trigger", req, res))
            {
                m_config.external_trigger = res.camTriggerRunning;

                if (m_config.external_trigger)
                {
                    ROS_INFO("Switched on external trigger.");
                }
                else
                {
                    ROS_INFO("Failed to switch on external trigger.");
                }
            }
            else
            {
                m_config.external_trigger = false;

                ROS_ERROR("Failed to switch on external trigger.");
            }
        }

        if (!m_config.external_trigger)
        {
            // internal trigger
            m_device->set_PropertyValue(VRM_PROPID_GRAB_MODE_E, VRM_PROPID_GRAB_MODE_TRIGGERED_INTERNAL);
        }
    }
}

bool
VRmagicDeviceDriver::stopDevice(void)
{
    if (m_state != driver_base::Driver::CLOSED)
    {
        m_device->Stop();

        ROS_INFO("Stopped VRmagic device.");

        m_device->Close();
        m_device.reset();

        m_state = driver_base::Driver::CLOSED;
    }

    return true;
}

bool
VRmagicDeviceDriver::grabFrames(void)
{
    boost::unique_lock<boost::mutex> lock(m_deviceMutex);

    int nGrabs = 0;

    try
    {
        // grab all queued images
        while (m_device->IsNextImageReady(0))
        {
            int framesDropped = 0;
            ImagePtr imageRaw = m_device->LockNextImage(0, &framesDropped);

            VRmagicCameraPtr& camera = m_cameraMap.at(imageRaw->get_SensorPort());
            ros::Time hw_stamp;

            bool foundStamp = false;
            unsigned int frameCounter = imageRaw->get_FrameCounter();

            if (m_config.external_trigger)
            {
                boost::unique_lock<boost::mutex> lock(m_triggerBufferMutex);
                for (std::list<std::pair<unsigned int, ros::Time> >::reverse_iterator rit = m_triggerBuffer.rbegin();
                     rit != m_triggerBuffer.rend(); ++rit)
                {
                    if (frameCounter > rit->first)
                    {
                        break;
                    }
                    else if (frameCounter == rit->first)
                    {
                        hw_stamp = rit->second;
                        foundStamp = true;

                        break;
                    }
                }
            }
            else
            {
                hw_stamp = ros::Time(imageRaw->get_TimeStamp() / 1000.0);
                foundStamp = true;
            }

            if (framesDropped > 0)
            {
                ROS_WARN("[%s] Dropped %d frames.", camera->cameraName().c_str(), framesDropped);
            }

            if (foundStamp)
            {
                char* buffer = reinterpret_cast<char*>(imageRaw->get_Buffer());

                if (camera->image()->data.size() != imageRaw->get_BufferSize())
                {
                    ROS_WARN("Sizes of source and destination images do not match.");

                    m_device->UnlockNextImage(imageRaw);
                    continue;
                }

                camera->grabFrame(hw_stamp, buffer);
            }
            else
            {
                ROS_WARN("[%s] Did not find trigger info for frame %u", camera->cameraName().c_str(), frameCounter);
            }

            m_device->UnlockNextImage(imageRaw);

            ++nGrabs;
        }
    }
    catch (const Exception& err)
    {
        // in case of an error, check for trigger timeouts and trigger stalls.
        // both can be recovered, so continue. otherwise exit the app
        switch(err.get_Number())
        {
        case VRM_ERROR_CODE_FUNCTION_CALL_TIMEOUT:
        case VRM_ERROR_CODE_TRIGGER_TIMEOUT:
        case VRM_ERROR_CODE_TRIGGER_STALL:
            ROS_INFO("LockNextImage() failed with %s.",
                     err.get_Description().c_str());
            break;
        case VRM_ERROR_CODE_GENERIC_ERROR:
        default:
            throw;
        }
    }

    return nGrabs > 0;
}

void
VRmagicDeviceDriver::publishFrames(void)
{
    boost::unique_lock<boost::mutex> lock(m_deviceMutex);

    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        m_cameras.at(i)->publishFrame();
    }
}

bool
VRmagicDeviceDriver::updateCameraInfo(px_comm::SetCameraInfo::Request& req,
                                      px_comm::SetCameraInfo::Response& res,
                                      VRmagicCameraPtr& camera)
{
    boost::unique_lock<boost::mutex> lock(m_deviceMutex);

    *(camera->cameraInfo()) = req.camera_info;

    writeCameraInfo();

    res.success = true;
    res.status_message = "Updated camera info for " + camera->cameraName();

    ROS_INFO("%s", res.status_message.c_str());

    return true;
}

void
VRmagicDeviceDriver::readCameraInfo(void)
{
    std::vector<unsigned char> data = m_device->LoadUserData();

    if (data.empty())
    {
        ROS_WARN("No data is stored in the EEPROM.");
        return;
    }

    try
    {
        ros::serialization::IStream stream(&data[0], data.size());

        for (size_t i = 0; i < m_cameras.size(); ++i)
        {
            ros::serialization::deserialize(stream, *(m_cameras.at(i)->cameraInfo()));
        }
    }
    catch (std::exception& e)
    {
        for (size_t i = 0; i < m_cameras.size(); ++i)
        {
            *(m_cameras.at(i)->cameraInfo()) = px_comm::CameraInfo();
        }

        m_device->SaveUserData(std::vector<unsigned char>());
        ROS_WARN("EEPROM data is incomplete. Erased data.");
        return;
    }

    m_cameraInfoFound = true;
}

void
VRmagicDeviceDriver::writeCameraInfo(void)
{
    size_t bytes = 0;
    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        bytes += ros::serialization::serializationLength(*(m_cameras.at(i)->cameraInfo()));
    }

    std::vector<unsigned char> data(bytes);
    ros::serialization::OStream stream(&data[0], bytes);

    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        ros::serialization::serialize(stream, *(m_cameras.at(i)->cameraInfo()));
    }

    m_device->SaveUserData(data);
}

}
