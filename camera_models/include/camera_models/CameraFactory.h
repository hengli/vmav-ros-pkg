#ifndef CAMERAFACTORY_H
#define CAMERAFACTORY_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <px_comm/CameraInfo.h>

#include "camera_models/Camera.h"

namespace px
{

class CameraFactory
{
public:
    CameraFactory();

    static boost::shared_ptr<CameraFactory> instance(void);

    CameraPtr generateCamera(Camera::ModelType modelType,
                             const std::string& cameraName,
                             cv::Size imageSize) const;
    CameraPtr generateCamera(const px_comm::CameraInfoConstPtr& cameraInfo) const;

    CameraPtr generateCameraFromYamlFile(const std::string& filename);

private:
    static boost::shared_ptr<CameraFactory> m_instance;
};

}

#endif
