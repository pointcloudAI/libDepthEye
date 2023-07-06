/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_FRAME_GENERATOR_H
#define POINTCLOUD_FRAME_GENERATOR_H

#include <Frame.h>
#include <Common.h>
#include <Serializable.h>
#include "GlobalSetting.h"
namespace PointCloud
{
  class KalmanInfo;
  class SYMBOL_EXPORT KalmanInfo
  {
  public:
	  float filterValue;  //k-1时刻的滤波值，即是k-1时刻的值
	  double kalmanGain;   //   Kalamn增益
	  double A;   // x(n)=A*x(n-1)+u(n),u(n)~N(0,Q)
	  double H;   // z(n)=H*x(n)+w(n),w(n)~N(0,R)
	  double Q;   //预测过程噪声偏差的方差
	  double R;   //测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
	  double P;   //估计误差协方差

	  KalmanInfo(double _R = 1000, double _Q = 0.01, double _P = 1, double _A = 1, double _H = 1) {
		  R = _R; P = _P; Q = _Q;  A = _A; H = _H; filterValue = -99; kalmanGain = 0;
	  }
  };

  class FrameStreamWriter;
  class SYMBOL_EXPORT FrameGenerator
  {
  protected:
    GeneratorIDType _id;
    uint32_t _frameTypes; // Generated frame type. Value as in PointCloud::DepthCamera::FrameType(×) --> new Value as in PointCloud::FrameType(√)
    Ptr<FrameStreamWriter> _frameStreamWriter;
    StringKeySerializableMap _frameGeneratorParameters; // Primarily for the purpose of serializing/deserializing these parameters
    uint8_t _majorVersion, _minorVersion; // Used for storing/reading configuration
    TofConfSetting *_confSetting;
  protected:
    virtual bool _onReadConfiguration() = 0; // On read configuration
    virtual bool _onWriteConfiguration() = 0; // On write configuration
    inline bool _getParam(const String &name, SerializablePtr &param) const;
    template <typename T>
    bool _set(const String &name, const T &value); // Only for internal use
    template <typename T>
    bool _get(const String &name, T &value) const;
  public:
    FrameGenerator(GeneratorIDType id, uint32_t frameTypes, uint8_t majorVersion, uint8_t minorVersion): _id(id), _frameTypes(frameTypes),
    _majorVersion(majorVersion), _minorVersion(minorVersion) 
    {
        _confSetting = new TofConfSetting;
        _frameGeneratorParameters["version"] = SerializablePtr(new SerializableUnsignedInt((majorVersion << 8) + minorVersion));
    }
    inline const uint32_t getFrameTypes() const { return _frameTypes;};
    inline const GeneratorIDType &id() const { return _id; }
    virtual bool  setParameters(TofConfSetting* conf_setting) = 0;
    virtual bool  getParameters(TofConfSetting* conf_setting) = 0;
    inline  void  setFrameStreamWriter(Ptr<FrameStreamWriter> &writer) { _frameStreamWriter = writer; }
    inline  void  removeFrameStreamWriter() { _frameStreamWriter = nullptr; }
    virtual bool  reset(){return false;};
    virtual bool  readConfiguration(SerializedObject &object); // Read configuration from serialized data object
    virtual bool  writeConfiguration(); // Write configuration to FrameStreamWriter
    virtual bool  writeConfiguration(SerializedObject &object) // Write configuration to serialized data object
    {
        object.resize(_frameGeneratorParameters.serializedSize());
        if(!_frameGeneratorParameters.write(object)){
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
  virtual ~FrameGenerator();
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

template <typename T>
inline bool FrameGenerator::_get(const String &name, T &value) const
{
  SerializablePtr param;

  if (!_getParam(name, param))
    return false;
  const BasicSerializable<T> *p = dynamic_cast<BasicSerializable<T> *>(param.get());
  if (p)
  {
    value = p->value;
    return true;
  }
  return false;
}

template <>
inline bool FrameGenerator::_get<String>(const String &name, String &value) const
{
  SerializablePtr param;
  if (!_getParam(name, param))
    return false;
  SerializableString *p = dynamic_cast<SerializableString *>(param.get());
  if (p)
  {
    value = *p;
    return true;
  }
  return false;
}

class SYMBOL_EXPORT DepthFrameGenerator: public FrameGenerator
{
public:
  DepthFrameGenerator(GeneratorIDType id, int frameType, uint8_t majorVersion, uint8_t minorVersion): FrameGenerator(id, frameType, majorVersion, minorVersion) {}
  virtual bool setProcessedFrameGenerator(FrameGeneratorPtr &p) = 0;
  
  virtual ~DepthFrameGenerator() {}
};

typedef Ptr<DepthFrameGenerator> DepthFrameGeneratorPtr;
  
class PointCloudTransform;
typedef Ptr<PointCloudTransform> PointCloudTransformPtr;

class SYMBOL_EXPORT PhaseFrameGenerator : public FrameGenerator
{
public:
    PhaseFrameGenerator(GeneratorIDType id, int frameType, uint8_t majorVersion, uint8_t minorVersion);
    virtual bool setParameters(TofConfSetting* conf_setting);
    virtual bool getParameters(TofConfSetting* conf_setting);
    virtual ~PhaseFrameGenerator();
    
protected:
    bool getDirectionCorr(std::vector<int32_t>&  directionCorr);
    int  getTempOffset(float tSensor, float tIllum,float tempIllumCoeff);
    float KalmanFilter(int lastMeasurement);
    bool configFilterSetting(FilterParam& filterParam,bool bWrite);
    bool configCalibSetting(bool bWrite);
    bool configLensSetting(bool bWrite);
    virtual bool _setParameters() = 0;
    virtual bool _onReadConfiguration();
    virtual bool _onWriteConfiguration();
private:
    void calcPhaseCorrRate();
    bool initLensCalib();
protected:
    FrameSize           _size;
    // TofConfSetting*     _confSetting;
    bool                _bReadConfig;
    KalmanInfo*         _kalmanParameter;
    PointCloudTransformPtr _pointCloudTransform;
};
}

#endif
