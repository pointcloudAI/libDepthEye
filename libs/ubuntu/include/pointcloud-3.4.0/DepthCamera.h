/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */
#ifndef POINTCLOUD_DEPTHCAMERA_H
#define POINTCLOUD_DEPTHCAMERA_H

#include <Device.h>
#include <Frame.h>
#include "VideoMode.h"
#include "FrameStream.h"
#include "Timer.h"
#include "Configuration.h"
#include "md5.h"

#define SPEED_OF_LIGHT 299792458.0
#define BULK_XFER_EXTRA_SIZE  255
#define EBD_RAW12_DATA_LENGTH 396
#define EBD_INT16_DATA_LENGTH 528

#define MIN_UNAMBIGUOUS_RANGE 1.48
#define MIN_NEAR_DISTANCE  0.1
#define UNAMBIGUOUS_RANGE "unambiguous_range"
#define NEAR_DISTANCE "near_distance"
#define MEASURE_MODE "measure_mode"
#define VIRTUAL_CHANNEL "virtual_channel"
#define CHANNEL_STATUS "channel_status"
#define OUTPUT_MODE "output_mode"
#define FILTER_EN "filter_en"

#define	DUTOF_MODE "dutof_mode"
#define COMPAT_DATA_EN "compat_data_en"
#define INTG_SCALE "intg_scale"
#define INTG_TIME "intg_time"
#define TSENSOR_INDIVIDUAL_COMPATIBLE "tsensor_individual_compatible"

#define CALIB_SECT_LENS "lens"
#define CALIB_SECT_LENS_ID 0
#define TSENSOR "tsensor"
#define TILLUM  "tillum"
#define EBD_DEPTH_MAP_ID_OFFSET 87
#define EBD_FRAME_COUNT_OFFSET 63
#define EBD_SUB_FRAME_ID_OFFSET 54
#define EBD_TSENSOR_START_OFFSET 117
#define EBD_TILLUM_OFFSET 2
#define EBD_POWER_SUPPLIER 3

#define ToF_CALIB_SECT_FREQUENCY_ID 0
#define ToF_CALIB_SECT_CROSS_TALK_ID 1
#define ToF_CALIB_SECT_NON_LINEARITY_ID 2
#define ToF_CALIB_SECT_TEMPERATURE_ID 3
#define ToF_CALIB_SECT_COMMON_PHASE_OFFSET_ID 4
#define ToF_CALIB_SECT_PIXELWISE_PHASE_OFFSET_ID 5

namespace PointCloud
{
	class POINTCLOUD_EXPORT Streamer;
	class POINTCLOUD_EXPORT Parameter;
	class POINTCLOUD_EXPORT FrameGenerator;
	class POINTCLOUD_EXPORT PointCloudFrameGenerator;
	class POINTCLOUD_EXPORT RegisterProgrammer;
	typedef Ptr<Streamer>  StreamerPtr;
	typedef Ptr<Parameter>  ParameterPtr;
	typedef Ptr<RegisterProgrammer> RegisterProgrammerPtr;
	typedef Ptr<FrameGenerator> FrameGeneratorPtr;
	typedef Ptr<PointCloudFrameGenerator> PointCloudFrameGeneratorPtr;
	/**
	  * \ingroup CamSys
	  *
	  * \brief This is primary class which provides API for a depth camera.
	  *
	  * DepthCamera is an abstract class which needs to be derived and implemented
	  * for individual depth camera types.
	  */

	  //enum FreqIndex
	  //{
	  //	MAX_FREQ = 0,
	  //	MIN_FREQ = 1
	  //};

	  //enum ChipType
	  //{
	  //	UNKNOW_TYPE = 0,
	  //	MLX75027 = 1,
	  //	MLX75026 = 2,
	  //	IMX556 = 3,
	  //	IMX570 = 4
	  //};



	class POINTCLOUD_EXPORT DepthCamera
	{
	public:
		/*
		enum FrameType
		{
			FRAME_RAW_FRAME_UNPROCESSED = 0,
			FRAME_RAW_FRAME_PROCESSED = 1,
			FRAME_DEPTH_FRAME = 2,
			FRAME_XYZI_POINT_CLOUD_FRAME = 3,
			FRAME_XYZI_POINT_CLOUD_FISHEYE_FRAME = 4,
			FRAME_VERTICAL_FRAME = 5,
			FRAME_RGB_FRAME = 6,
			FRAME_TYPE_COUNT = 7 // This is just used for number of callback types
		};
		*/
		enum FrameType
		{
			FRAME_RAW_FRAME_UNPROCESSED = 0,
			FRAME_RAW_FRAME_PROCESSED = 1,
			FRAME_DEPTH_FRAME = 2,
			FRAME_XYZI_POINT_CLOUD_FRAME = 3,
			FRAME_VERTICAL_FRAME = 4,
			FRAME_RGB_FRAME = 5,
			FRAME_TYPE_COUNT = 6 // This is just used for number of callback types
		};

		enum InitStatus
		{
			ST_UNINIT = 0,
			ST_PARAMS_INIT = 1,
			ST_DEVICE_INIT = 2,
			ST_PROFILE_INIT = 3,
			ST_FINISH_INIT = 4
		};
		typedef Function<void(DepthCamera &camera, const Frame &frame, FrameType callBackType)> CallbackType;
		typedef Function<void(DepthCamera &camera, int isOn12V)> PowerChangedCallback;
	public:
		DepthCamera(const String &name, const String &chipset, DevicePtr device);
		virtual ~DepthCamera();
		virtual bool Init() { return true; }
		virtual bool UnInit() { return true; }
		virtual bool isInitialized() const;
		virtual bool online() = 0;
		inline bool isRunning() const { return _running; }
		inline bool isPaused() const { return _isPaused; }
		virtual bool start(CameraType camType = TOF_CAMERA);
		bool stop();
		//void wait();
		bool reset();
		bool pause();
		bool resume();
		bool close();

		virtual bool registerCallback(FrameType type, CallbackType f);
		virtual bool clearAllCallbacks();
		virtual bool clearCallback(FrameType type);
		bool isRawDataProvidedOnly();

		virtual bool registerPowerSupplierChangedCallback(PowerChangedCallback f);
		virtual bool setPowerLevel(int level) = 0;

		bool setFrameRate(const FrameRate &r);
		bool getFrameRate(FrameRate &r) const;
		bool setFrameSize(const FrameSize &s);
		bool getFrameSize(FrameSize &s) const;
		bool getMaximumFrameSize(FrameSize &s) const;
		bool getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const;
		bool getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const;
		bool getMaximumVideoMode(VideoMode &videoMode) const;

		bool getROI(RegionOfInterest &roi);
		bool setROI(const RegionOfInterest &roi);
		bool allowedROI(String &message);

		bool getFieldOfView(float &fovHalfAngle) const;

		virtual bool saveFrameStream(const String &fileName);
		virtual bool isSavingFrameStream();
		virtual bool closeFrameStream();

		template <typename T>
		bool get(const String &name, T &value, bool refresh = false) const;
		template <typename T>
		bool set(const String &name, const T &value);

		inline const String &name() const { return _name; }
		inline const String &id() const { return _id; }
		inline const String &chipset() const { return _chipset; }
		inline const ChipType &sensorChipType() const { return _sensorChipType; }



		virtual bool getSerialNumber(String &serialNumber) const;
		virtual bool setSerialNumber(const String &serialNumber);
		virtual bool getModuleID(String &mod_id) { return false; };
		virtual bool setModuleID(ChipType chipType, uint16_t opt_id = 0, uint32_t id = 0) { return false; };

		virtual int saveConfToModule(String readFileName, String savedFileName) = 0;
		virtual int getUploadState(void) = 0;

		virtual uint getBatchNo() = 0;
		virtual unsigned long long getSensorID() = 0;
		virtual String getFull_SN() = 0;

		virtual bool checkBoardOnline() = 0;
		virtual bool changeToChannelX(int chnId) = 0;
		virtual uint16_t getChannelStatus(int isFirstStart) = 0;

		virtual bool confSettingReload(int initType = 0) = 0;
		virtual bool setIntegralTimeScale(uint8_t scale) = 0;
		virtual uint8_t getIntegralTimeScale(void) = 0;
		virtual bool setIntegralTime(uint value) = 0;
		virtual uint getIntegralTime(bool refresh = false) = 0;

		virtual bool readConfFromModule(void) = 0;
		virtual String getReadConfFromModuleState(void) = 0;

		virtual bool controlOneTakeEightBoard(bool isOn) = 0;
		virtual uint32_t checkOneTakeEightBoardShortCircuit(void) = 0;
		virtual bool letModuleWorkOn(void) = 0;
		virtual uint32_t getFirmwareVersion() = 0;
		virtual uint32_t getMipiResetCount() = 0;
		virtual uint32_t getPhaseLoseCount() = 0;
		virtual uint32_t checkOneTakeEightBoardChannelStatus() = 0;
		virtual uint32_t getOneTakeEightBoardVerison() = 0;

		inline const DevicePtr &getDevice() const { return _device; }
		template <typename T>
		bool getStreamParam(const String &name, T &value) const;
		bool refreshParams();

		// WARNING: Avoid using get() and set() on ParameterPtr, obtained via getParam() or getParameters(). It is not thread-safe. Instead use get() and set() on DepthCamera
		const ParameterPtr getParam(const String &name) const;
		inline const Map<String, ParameterPtr> &getParameters() const { return _parameters; }
		inline uint32_t getRegisterCallbackType() const { return _callBackTypesRegistered; };
		bool getBytesPerPixel(uint &bpp) const;
		bool setBytesPerPixel(const uint &bpp);
		bool getFrameGeneratorConfig(FrameType type, SerializedObject &object);
		// beforeFilterIndex = -1 => at the end, otherwise at location before the given filter index.
		// Return value:
		//   >= 0 => add successfully with return value as filter ID.
		//   -1 => failed to add filter

		// RegisterProgrammer is usually thread-safe to use outside directly
		inline RegisterProgrammerPtr getProgrammer() { return _programmer; }
		// Streamer may not be thread-safe
		inline StreamerPtr getStreamer() { return _streamer; }

		inline bool reloadConfiguration() { return configFile.read(_name + ".conf"); }
		inline const Map<int, String> &getCameraProfileNames() { return configFile.getCameraProfileNames(); }
		inline int getCurrentCameraProfileID() { return configFile.getCurrentProfileID(); }

		int  addCameraProfile(const String &profileName, const int parentID);
		bool setCameraProfile(const int id, bool softApply = false);
		bool removeCameraProfile(const int id);

		virtual bool _onPowerChangedCallback(int isOn12V);
		inline bool saveCameraProfileToHardware(int &id, bool saveParents = false, bool setAsDefault = false, const String &namePrefix = "") { return configFile.saveCameraProfileToHardware(id, saveParents, setAsDefault, namePrefix); }

		bool setLensParameters(float fx, float fy, float cx, float cy, float k1, float k2, float k3, float k4);
		virtual bool testParametersSet() = 0;
	protected:
		bool _addParameters(const Vector<ParameterPtr> &params);
		// Callback the registered function for 'type' if present and decide whether continue processing or not
		virtual bool _callbackAndContinue(uint32_t &callBackTypesToBeCalled, FrameType type, const Frame &frame);
		bool _refeshProfile();
		virtual bool _start(CameraType camType) = 0;
		virtual bool _stop() = 0;
		virtual bool _close() = 0;

		virtual bool _captureRawUnprocessedFrame(RawDataFramePtr &rawFrame) = 0;
		virtual bool _processRawFrame(const RawDataFramePtr &rawFrameInput, PhaAmpFramePtr &rawFrameOutput) = 0; // here output raw frame will have processed data, like ToF data for ToF cameras
		virtual bool _convertToDepthFrame(const PhaAmpFramePtr &rawFrame, DepthFramePtr &depthFrame) = 0;
		virtual bool _convertToPointCloudFrame(const DepthFramePtr &depthFrame, PointCloudFramePtr &pointCloudFrame);
		virtual bool _convertToPointCloudFrame(const PhaAmpFramePtr &rawFrame, PointCloudFramePtr &pointCloudFrame);
		virtual bool _convertToVerticalFrame(const PhaAmpFramePtr &rawFrame, RawDataFramePtr &verticalFrame);
		bool _writeToFrameStream(RawDataFramePtr &rawUnprocessed);

		// These protected getters and setters are not thread-safe. These are to be directly called only when nested calls are to be done from getter/setter to another.
		// Otherwise use the public functions
		template <typename T>
		bool _get(const String &name, T &value, bool refresh = false) const;
		template <typename T>
		bool _set(const String &name, const T &value);

		virtual bool _setFrameRate(const FrameRate &r) = 0;
		virtual bool _getFrameRate(FrameRate &r) const = 0;
		virtual bool _setFrameSize(const FrameSize &s) = 0;
		virtual bool _getFrameSize(FrameSize &s) const = 0;
		virtual bool _getMaximumFrameSize(FrameSize &s) const = 0;
		virtual bool _getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const = 0;
		virtual bool _getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const = 0;
		virtual bool _getMaximumVideoMode(VideoMode &videoMode) const = 0;

		virtual bool _getBytesPerPixel(uint &bpp) const = 0;
		virtual bool _setBytesPerPixel(const uint &bpp) = 0;

		virtual bool _getROI(RegionOfInterest &roi) const = 0;
		virtual bool _setROI(const RegionOfInterest &roi) = 0;
		virtual bool _allowedROI(String &message) = 0;
		virtual bool _getFieldOfView(float &fovHalfAngle) const = 0;
		inline  void _makeID() { _id = _name + "(" + _device->id() + ")"; }
		virtual bool _reset() = 0;
		virtual bool _onReset() = 0;
		virtual bool _applyConfigParams(const ConfigSet *params);
		virtual bool _saveCurrentProfileID(const int id) = 0;
		virtual bool _getCurrentProfileID(int &id) = 0;
		//inline Map<String, CalibrationInformation> &_getCalibrationInformationStructure() { return configFile._calibrationInformation; }
	private:
		mutable Mutex _accessMutex; // This is locked by getters and setters which are public
		mutable Mutex _frameStreamWriterMutex;
		mutable Mutex _stopStartMutex;
	protected:
		FrameSize   _frameSize;
		CameraType  _camType;
		DevicePtr   _device;
		ChipType    _sensorChipType;
		String      _name, _id, _chipset;

		Map<String, ParameterPtr> _parameters;
		RegisterProgrammerPtr     _programmer;
		StreamerPtr               _streamer;

		FrameGeneratorPtr _frameGenerators[3];
		InitStatus _initStatus;
		//bool _connectInit;
		//bool _parameterInit;
		bool _running, _isPaused; // is capture running?

		FrameStreamWriterPtr _frameStreamWriter;
		//PointCloudFrameGeneratorPtr _pointCloudFrameGenerator;

		CallbackType _callback[FRAME_TYPE_COUNT];
		PowerChangedCallback _powerChangedCallback;

		uint32_t _callBackTypesRegistered = 0;
		//ThreadPtr _captureThread = 0;
	public:
		MainConfigurationFile configFile; // This corresponds to camera specific configuration file
	private:
		friend class UVC;
	};
	typedef Ptr<DepthCamera> DepthCameraPtr;
}

#endif // DEPTHCAMERA_H
