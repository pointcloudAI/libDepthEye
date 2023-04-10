/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_FRAME_GENERATOR_H
#define POINTCLOUD_FRAME_GENERATOR_H

#include <Frame.h>
#include <Common.h>

#include "DepthCamera.h"
#include <Serializable.h>
#define NEW_INTERFACE

#define TEMP_SENSOR_QUEUE_SIZE 10
//length of sensor's temperature filter
#define PARAM_FILTER_ENABLE      "filter_en"
#define PARAM_FISHEYE_ENABLE     "fisheye_enable"
#define PARAM_PHASE_OFFSETS_FILE "phaseOffsetsFile"
#define PARAM_PHASE_OFFSETS      "phaseOffsets"
#define PARAM_FRAME_WIDTH        "frameWidth"
#define PARAM_FRAME_HEIGHT       "frameHeight"
#define PARAM_CALIB_SENSOR       "calibSensor"
#define PARAM_CALIB_ILLUM        "calibIllum"
#define PARAM_COEFF_SENSOR       "coeffSensor"
#define PARAM_COEFF_ILLUM        "coeffIllum"
#define PARAM_PHASE_MAX_CORR     "maxPhaseCorr"
#define PARAM_PHASE_MIN_CORR     "minPhaseCorr"
#define PARAM_MEASURE_MODE       "measureMode"
#define PARAM_OUTPUT_MODE        "outputMode"
#define PARAM_MAX_FREQ           "maxFreq"
#define PARAM_MIN_FREQ           "minFreq"
#define PARAM_MAX_NONLINEARITY_COEFF "phaseMaxLinCoeff"
#define PARAM_MIN_NONLINEARITY_COEFF "phaseMinLinCoeff"
#define PARAM_PIXCELWISE_ENABLE      "pixelwiseCalibEnable"
#define PARAM_CALIB_DISABLE          "calib_disable"
#define PARAM_NONLINEAR_ENABLE       "nonlinearityCalibEnable"
#define PARAM_DEALIAS_PHASEMERGE_ENABLE       "dealiasPhaseMerge"
#define PARAM_COMMON_PHASE_ENABLE    "commonPhaseCalibEnable"

#define PARAM_UNAMBIGUOUS_RANGE      "unambiguous_range"
#define PARAM_NEAR_DISTANCE          "near_distance"

#define PARAM_TRANS_THRESHOLD        "transThreshold"
#define PARAM_AMPLITUDE_SCALING_FACTOR  "amplitudeScalingFactor"
#define PARAM_DEPTH_SCALING_FACTOR      "depthScalingFactor"
#define PARAM_ROI_X                  "roiX"
#define PARAM_ROI_Y                  "roiY"
#define PARAM_ROI_WIDTH              "roiWidth"
#define PARAM_ROI_HEIGHT             "roiHeight"
#define PARAM_ROWS_TO_MERGE          "rowsToMerge"
#define PARAM_COLUMNS_TO_MERGE       "columnsToMerge"
#define PARAM_AMP_THREHOLD           "ampThrehold"
#define PARAM_MAX_RANGE              "maxRange"
#define PARAM_CX "cx"
#define PARAM_CY "cy"
#define PARAM_FX "fx"
#define PARAM_FY "fy"
#define PARAM_K1 "k1"
#define PARAM_K2 "k2"
#define PARAM_K3 "k3"
#define PARAM_K4 "k4"
#define PARAM_P1 "p1"
#define PARAM_P2 "p2"


namespace PointCloud
{
  
class FrameStreamWriter;
class POINTCLOUD_EXPORT FrameGenerator
{
protected:
  GeneratorIDType _id;
  int _frameType; // Generated frame type. Value as in PointCloud::DepthCamera::FrameType
  Ptr<FrameStreamWriter> _frameStreamWriter;
  StringKeySerializableMap _frameGeneratorParameters; // Primarily for the purpose of serializing/deserializing these parameters
  uint8_t _majorVersion, _minorVersion; // Used for storing/reading configuration
  TofConfSetting _confSetting;
protected:
  virtual bool _onReadConfiguration() = 0; // On read configuration
  virtual bool _onWriteConfiguration() = 0; // On write configuration
  inline bool _getParam(const String &name, SerializablePtr &param) const;
  template <typename T>
  inline bool _set(const String &name, const T &value); // Only for internal use
public:
  FrameGenerator(GeneratorIDType id, int frameType, uint8_t majorVersion, uint8_t minorVersion): _id(id), _frameType(frameType),
  _majorVersion(majorVersion), _minorVersion(minorVersion) 
  {
      _frameGeneratorParameters["version"] =
        SerializablePtr(new SerializableUnsignedInt((majorVersion << 8) + minorVersion));
  }
  
  inline const GeneratorIDType &id() const { return _id; }
  virtual void getParameters(TofConfSetting* confSetting)const = 0;
  virtual bool setParameters(DepthCamera* depthCamera,TofConfSetting* conf_setting,int calib_disable) = 0;
  inline  void setFrameStreamWriter(Ptr<FrameStreamWriter> &writer) { _frameStreamWriter = writer; }
  inline  void removeFrameStreamWriter() { _frameStreamWriter = nullptr; }
  virtual bool readConfiguration(SerializedObject &object); // Read configuration from serialized data object
  virtual bool writeConfiguration(); // Write configuration to FrameStreamWriter
  virtual bool writeConfiguration(SerializedObject &object) // Write configuration to serialized data object
    {
        object.resize(_frameGeneratorParameters.serializedSize());
        if(!_frameGeneratorParameters.write(object))
        {
            return false;
        }
        
        if(!_onWriteConfiguration()){
            return false;
        }
        
        return true;
    }

  virtual bool generate(const FramePtr &in, FramePtr &out) = 0;
  
  template <typename T>
  inline bool get(const String &name, T &value) const;
  
  virtual ~FrameGenerator() {}
};

typedef Ptr<FrameGenerator> FrameGeneratorPtr;
inline bool FrameGenerator::_getParam(const String &name, SerializablePtr &param) const
{
  auto x = _frameGeneratorParameters.find(name);
  
  if(x != _frameGeneratorParameters.end())
  {
    param = x->second;
    return true;
  }
  
  return false;
}

template <typename T>
inline bool FrameGenerator::_set(const String &name, const T &value)
{
  SerializablePtr param;
  
  if(!_getParam(name, param))
    return false;
  
  BasicSerializable<T> *p = dynamic_cast<BasicSerializable<T> *>(param.get());
  
  if(p)
  {
    p->value = value;
    return true;
  }
  return false;
}

template <>
inline bool FrameGenerator::_set<String>(const String &name, const String &value)
{
  SerializablePtr param;
  
  if(!_getParam(name, param))
    return false;
  
  SerializableString *p = dynamic_cast<SerializableString *>(param.get());
  
  if(p)
  {
    *p = value;
    return true;
  }
  return false;
}

template <typename T>
inline bool FrameGenerator::get(const String &name, T &value) const
{
  SerializablePtr param;
  
  if(!_getParam(name, param))
    return false;
  
  const BasicSerializable<T> *p = dynamic_cast<BasicSerializable<T> *>(param.get());
  
  if(p)
  {
    value = p->value;
    return true;
  }
  return false;
}

template <>
inline bool FrameGenerator::get<String>(const String &name, String &value) const
{
  SerializablePtr param;
  
  if(!_getParam(name, param))
    return false;
  
  SerializableString *p = dynamic_cast<SerializableString *>(param.get());
  
  if(p)
  {
    value = *p;
    return true;
  }
  return false;
}


class POINTCLOUD_EXPORT DepthFrameGenerator: public FrameGenerator
{
public:
  DepthFrameGenerator(GeneratorIDType id, int frameType, uint8_t majorVersion, uint8_t minorVersion): FrameGenerator(id, frameType, majorVersion, minorVersion) {}
  virtual bool setProcessedFrameGenerator(FrameGeneratorPtr &p) = 0;
  
  virtual ~DepthFrameGenerator() {}
};

typedef Ptr<DepthFrameGenerator> DepthFrameGeneratorPtr;
  
}

#endif
