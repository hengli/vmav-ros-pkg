#ifndef CAMERASYSTEM_H
#define CAMERASYSTEM_H

#include <boost/unordered_map.hpp>
#include <camera_models/Camera.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Pose.h>
#include <vector>

namespace px
{

class CameraSystem
{
public:
    CameraSystem();
    CameraSystem(int cameraCount);

    int cameraCount(void) const;

    void reset(void);

    bool readFromTextFile(const std::string& filename);
    bool readFromTextFile(const std::string& filename,
                          std::vector<std::string>& cameraNames);
    bool writeToTextFile(const std::string& filename) const;

    bool readFromDirectory(const std::string& directory);
    bool writeToDirectory(const std::string& directory) const;

    bool readFromXmlFile(const std::string& filename);
    bool writeToXmlFile(const std::string& filename) const;

    int getCameraIdx(const CameraConstPtr& camera) const;
    CameraPtr getCamera(int idx) const;
    bool isPartOfStereoPair(int idx) const;

    bool setCamera(int idx, CameraPtr& camera);

    bool setReferenceCamera(int idx);

    // global camera pose is the transform from camera frame to system's reference frame
    Eigen::Matrix4d getGlobalCameraPose(int idx) const;
    Eigen::Matrix4d getGlobalCameraPose(const CameraConstPtr& camera) const;

    // local camera pose is the transform from camera frame to reference camera frame
    Eigen::Matrix4d getLocalCameraPose(int idx) const;
    Eigen::Matrix4d getLocalCameraPose(const CameraConstPtr& camera) const;

    bool setGlobalCameraPose(int idx, const Eigen::Matrix4d& pose);
    bool setGlobalCameraPose(int idx, const geometry_msgs::Pose& pose);
    bool setGlobalCameraPose(const CameraConstPtr& camera, const Eigen::Matrix4d& pose);
    bool setLocalCameraPose(int idx, const Eigen::Matrix4d& pose);
    bool setLocalCameraPose(const CameraConstPtr& camera, const Eigen::Matrix4d& pose);

    CameraSystem& operator=(const CameraSystem& other);

private:
    int m_cameraCount;
    int m_referenceCameraIdx;

    std::vector<CameraPtr> m_cameraVec;
    std::vector<bool> m_stereoFlagVec;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > m_globalPoseVec;
    boost::unordered_map<CameraPtr,int> m_cameraMap;
};

typedef boost::shared_ptr<CameraSystem> CameraSystemPtr;
typedef boost::shared_ptr<const CameraSystem> CameraSystemConstPtr;

}

#endif
