/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_CAMERASYSTEM_H
#define POINTCLOUD_CAMERASYSTEM_H

#include <Common.h>
#include <Device.h>
#include <DepthCamera.h>

namespace PointCloud
{
class POINTCLOUD_EXPORT DepthCameraLibrary;
class POINTCLOUD_EXPORT DepthCameraFactory;
// class POINTCLOUD_EXPORT FrameGenerator;
// class POINTCLOUD_EXPORT DepthCamera;
typedef Ptr<DepthCameraLibrary> DepthCameraLibraryPtr;
typedef Ptr<DepthCameraFactory> DepthCameraFactoryPtr;
// typedef Ptr<FrameGenerator>  FrameGeneratorPtr;
// typedef Ptr<DepthCamera> DepthCameraPtr;

/**
 * \defgroup CamSys Camera System Components
 * @{
 * \brief This group of classes provide the higher level API for cameras
 * @}
 */

/**
 * \ingroup CamSys
 * 
 * \brief This class provides ways to instantiate individual cameras and components like filters.
 */
class POINTCLOUD_EXPORT CameraSystem
{
public:
    CameraSystem();
    virtual ~CameraSystem();
    Vector<DevicePtr> scan();
    DepthCameraPtr connect(const DevicePtr &device);
    bool disconnect(const DepthCameraPtr &depthCamera, bool reset = false);
    bool addDepthCameraFactory(DepthCameraFactoryPtr factory);
    bool getFrameGenerator(uint8_t frameType, GeneratorIDType generatorID, FrameGeneratorPtr &frameGenerator);
    bool getDepthCameraFactory(GeneratorIDType generatorID,DepthCameraFactoryPtr &depthFactory);
    void init();
protected:
    void _loadLibraries(const Vector<String> &paths);
protected:
    bool    _init;
    String _deviceID;
    Vector<DepthCameraLibraryPtr> _libraries;
    Map<String, DepthCameraFactoryPtr> _factories; // Key = device ID as returned by Device::id()
    Map<String, DepthCameraPtr> _depthCameras; // Key = device ID as returned by Device::id()
    Map<GeneratorIDType, DepthCameraFactoryPtr> _factoryForGeneratorID; // Key = frame generator ID
};

}

#endif // POINTCLOUD_CAMERASYSTEM_H
