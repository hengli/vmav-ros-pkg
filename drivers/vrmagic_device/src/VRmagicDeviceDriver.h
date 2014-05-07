#ifndef VRMAGICDEVICEDRIVER_H
#define VRMAGICDEVICEDRIVER_H

#include <ros/ros.h>
#include <driver_base/driver.h>
#include <dynamic_reconfigure/server.h>
#include <asctec_hl_comm/CamTrigger.h>
#include <px_comm/SetCameraInfo.h>

#include "vrmagic_device/VRmagicDeviceConfig.h"
#include "vrmusbcamcpp.h"
#include "VRmagicCamera.h"

namespace px
{

class VRmagicDeviceDriver
{
public:
    VRmagicDeviceDriver(ros::NodeHandle nh);
    ~VRmagicDeviceDriver();

    bool start(int deviceId = 0);
    bool stop(void);

    bool poll(void);

private:
    void cbTrigger(const asctec_hl_comm::CamTriggerConstPtr& trigger);

    void reconfigure(const vrmagic_device::VRmagicDeviceConfig& config,
                     uint32_t level);

    bool startDevice(int deviceId,
                     const vrmagic_device::VRmagicDeviceConfig& config);

    void reconfigureDevice(const vrmagic_device::VRmagicDeviceConfig& config);

    bool stopDevice(void);

    bool grabFrames(void);
    void publishFrames(void);

    bool updateCameraInfo(px_comm::SetCameraInfo::Request& req,
                          px_comm::SetCameraInfo::Response& res,
                          VRmagicCameraPtr& camera);

    // read camera info data from EEPROM
    void readCameraInfo(void);

    // write device info data to EEPROM
    void writeCameraInfo(void);

    ros::NodeHandle m_nh;
    ros::Subscriber m_triggerSub;

    boost::mutex m_triggerBufferMutex;
    const size_t k_triggerBufferSize;
    std::list<std::pair<unsigned int, ros::Time> > m_triggerBuffer;

    VRmUsbCamCPP::DevicePtr m_device;
    boost::mutex m_deviceMutex;
    bool m_cameraInfoFound;
    std::vector<VRmagicCameraPtr> m_cameras;
    std::vector<VRmagicCameraPtr> m_cameraMap;

    driver_base::Driver::state_t m_state;
    bool m_reconfiguring;

    vrmagic_device::VRmagicDeviceConfig m_config;
    dynamic_reconfigure::Server<vrmagic_device::VRmagicDeviceConfig> m_server;
    ros::Rate m_cycle;
};

}

#endif

