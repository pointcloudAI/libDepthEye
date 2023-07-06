/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_POINT_CLOUD_FRAME_GENERATOR_H
#define POINTCLOUD_POINT_CLOUD_FRAME_GENERATOR_H

#include <FrameGenerator.h>
#include <PointCloudTransform.h>

namespace PointCloud
{
  
class SYMBOL_EXPORT PointCloudFrameGenerator: public FrameGenerator
{
protected:
  PointCloudTransformPtr _pointCloudTransform;
  
  virtual bool _onReadConfiguration();
  virtual bool _onWriteConfiguration();
public:
  PointCloudFrameGenerator();
  virtual bool setParameters(TofConfSetting *conf_setting);
  virtual bool getParameters(TofConfSetting *conf_setting);
  bool reset();
  bool generate(const FramePtr &in, FramePtr &out);
  bool generateVerticalFrame(const FramePtr &in, FramePtr &out);
  
  virtual ~PointCloudFrameGenerator();
};
}

#endif
