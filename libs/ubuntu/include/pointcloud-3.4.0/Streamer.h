/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_STREAMER_H
#define POINTCLOUD_STREAMER_H

#include "Device.h"
#include "Frame.h"
#include "VideoMode.h"
#include "Timer.h"

namespace PointCloud
{
/**
 * \addtogroup IO
 * @{
 */


class POINTCLOUD_EXPORT Streamer
{
protected:
  DevicePtr _device;
  
  Timer _time;
  
  int _currentID = -1;
  TimeStampType _currentTimeStamp = 0;
  
  bool _isRunning = false;
  bool _isPause = false;
  int  _measureCalibMode = 0;
  bool _isRawDataProvidedOnly = false;
  virtual bool _start(CameraType camType) = 0;
  virtual bool _capture(RawDataFramePtr &p) = 0;
  virtual bool _stop() = 0;
  virtual bool _pause() = 0;
  virtual bool _resume() = 0;
  
public:
  Streamer(DevicePtr device): _device(device) {}
    
  virtual ~Streamer();
  
  virtual bool isInitialized() = 0;
  virtual bool isRunning() { return _isRunning; }
  virtual bool isPause() { return _isPause; }
  
  virtual bool start(CameraType camType);
  virtual bool capture(RawDataFramePtr &p);
  virtual bool stop();
  virtual bool pause();
  virtual bool resume();

  virtual bool getSupportedVideoModes(Vector<VideoMode> &videoModes) = 0;
  virtual bool getCurrentVideoMode(VideoMode &videoMode) = 0;
  virtual bool setVideoMode(const VideoMode &videoMode) = 0;
  virtual bool setMeasureMode(const int measureMode)
    {
        _measureCalibMode = measureMode;
        return true;
    };
   bool setRawDataProvidedOnly(const int RawDataOnly)
    {
        _isRawDataProvidedOnly = RawDataOnly;
        return true;
    };
};
/**
 * @}
 */

}

#endif // STREAMER_H
