#include "camera_models/CameraFactory.h"

#include <boost/algorithm/string.hpp>
#include <boost/make_shared.hpp>

#include "ceres/ceres.h"
#include "camera_models/CataCamera.h"
#include "camera_models/EquidistantCamera.h"
#include "camera_models/PinholeCamera.h"

namespace px
{

boost::shared_ptr<CameraFactory> CameraFactory::m_instance;

CameraFactory::CameraFactory()
{

}

boost::shared_ptr<CameraFactory>
CameraFactory::instance(void)
{
    if (!m_instance)
    {
        m_instance = boost::make_shared<CameraFactory>();
    }

    return m_instance;
}

CameraPtr
CameraFactory::generateCamera(Camera::ModelType modelType,
                              const std::string& cameraName,
                              const std::string& cameraType,
                              cv::Size imageSize) const
{
    switch (modelType)
    {
    case Camera::KANNALA_BRANDT:
    {
        EquidistantCameraPtr camera = boost::make_shared<EquidistantCamera>();

        EquidistantCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.cameraType() = cameraType;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    case Camera::PINHOLE:
    {
        PinholeCameraPtr camera = boost::make_shared<PinholeCamera>();

        PinholeCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.cameraType() = cameraType;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    case Camera::MEI:
    default:
    {
        CataCameraPtr camera = boost::make_shared<CataCamera>();

        CataCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.cameraType() = cameraType;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    }
}

CameraPtr
CameraFactory::generateCamera(const px_comm::CameraInfoConstPtr& cameraInfo) const
{
    if (cameraInfo->camera_model == "KANNALA_BRANDT")
    {
        EquidistantCameraPtr camera = boost::make_shared<EquidistantCamera>();
        camera->readParameters(cameraInfo);

        return camera;
    }
    else if (cameraInfo->camera_model == "PINHOLE")
    {
        PinholeCameraPtr camera = boost::make_shared<PinholeCamera>();
        camera->readParameters(cameraInfo);

        return camera;
    }
    else if (cameraInfo->camera_model == "MEI")
    {
        CataCameraPtr camera = boost::make_shared<CataCamera>();
        camera->readParameters(cameraInfo);

        return camera;
    }
    else
    {
        return CameraPtr();
    }
}

CameraPtr
CameraFactory::generateCameraFromYamlFile(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        return CameraPtr();
    }

    Camera::ModelType modelType = Camera::MEI;
    if (!fs["model_type"].isNone())
    {
        std::string sModelType;
        fs["model_type"] >> sModelType;

        if (boost::iequals(sModelType, "kannala_brandt"))
        {
            modelType = Camera::KANNALA_BRANDT;
        }
        else if (boost::iequals(sModelType, "mei"))
        {
            modelType = Camera::MEI;
        }
        else if (boost::iequals(sModelType, "pinhole"))
        {
            modelType = Camera::PINHOLE;
        }
        else
        {
            std::cerr << "# ERROR: Unknown camera model: " << sModelType << std::endl;
            return CameraPtr();
        }
    }

    switch (modelType)
    {
    case Camera::KANNALA_BRANDT:
    {
        EquidistantCameraPtr camera = boost::make_shared<EquidistantCamera>();

        EquidistantCamera::Parameters params = camera->getParameters();
        params.readFromYamlFile(filename);
        camera->setParameters(params);
        return camera;
    }
    case Camera::PINHOLE:
    {
        PinholeCameraPtr camera = boost::make_shared<PinholeCamera>();

        PinholeCamera::Parameters params = camera->getParameters();
        params.readFromYamlFile(filename);
        camera->setParameters(params);
        return camera;
    }
    case Camera::MEI:
    default:
    {
        CataCameraPtr camera = boost::make_shared<CataCamera>();

        CataCamera::Parameters params = camera->getParameters();
        params.readFromYamlFile(filename);
        camera->setParameters(params);
        return camera;
    }
    }

    return CameraPtr();
}

}

