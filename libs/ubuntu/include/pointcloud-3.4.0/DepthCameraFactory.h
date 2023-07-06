/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_DEPTHCAMERA_FACTORY_H
#define POINTCLOUD_DEPTHCAMERA_FACTORY_H

#include "Common.h"
#include "FrameGenerator.h"
#include "FrameStream.h"
namespace PointCloud
{
  
/**
 *  \addtogroup CamSys
 *  @{
 */
class SYMBOL_EXPORT DepthCameraFactory
{
protected:
  String _name;
public:
  DepthCameraFactory(const String &name): _name(name) {}
  inline const String &name() const { return _name; }
  virtual bool getFrameGenerator(FrameType frameType, GeneratorIDType generatorID, FrameGeneratorPtr &frameGenerator) = 0;
  virtual bool getFrameStreamWriter(FrameStreamWriterPtr &frameWriter) = 0;
  virtual bool getFrameStreamReader(FrameStreamReaderPtr &frameReader) = 0;
  virtual Vector<GeneratorIDType> getSupportedGeneratorTypes() = 0;
  virtual ~DepthCameraFactory()
  {
  }
};

typedef Ptr<DepthCameraFactory> DepthCameraFactoryPtr;
typedef void (*GetDepthCameraFactory)(DepthCameraFactoryPtr &depthCameraFactory); // Function type to return DepthCameraFactory
typedef int(*GetABIVersion)(); // Function type to return ABI version

/**
 * @}
 */
}

#endif // DEPTHCAMERA_FACTORY_H
