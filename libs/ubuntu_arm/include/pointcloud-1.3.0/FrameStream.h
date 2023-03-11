/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_FRAME_STREAM_H
#define POINTCLOUD_FRAME_STREAM_H

#include <Frame.h>
#include <SerializedObject.h>
#include "DataPacket.h"

#define PACKET_MAJOR 0
#define PACKET_MINOR  3


namespace PointCloud
{

    struct LensCalib
    {
        float   fx = 0.0;
        float   fy = 0.0;
        float   cx = 0.0;
        float   cy = 0.0;
        float   k1 = 0.0;
        float   k2 = 0.0;
        float   k3 = 0.0;
		float   k4 = 0.0;
        float   p1 = 0.0;
        float   p2 = 0.0;
        float   pha_scale = 0.0;
        bool    fisheye_enable = false;
        float   trans_threshold = 1e-6;
    };

    enum MeasureMode
    {
        USE_MOD_F1 = 1,
        USE_MOD_F2 = 2,
        USE_DEALIAS = 3
    };

    enum OutputMode
    {
        AMP_PHA =0,
        II_IQ,
        A_B,
        AandB,
		AplusB,
        RGB
    };

    struct FilterParams
    {
        float  btl_ab_threshold = 3.0f/0.6666667f;
        float  btl_max_edge = 2.5f;
        float  btl_exp = 100.0f;//5.0f;
        float gaussian_kernel[9] ={
                        0.1069973f,
                        0.1131098f,
                        0.1069973f,
                        0.1131098f,
                        0.1195716f,
                        0.1131098f,
                        0.1069973f,
                        0.1131098f,
                        0.1069973f
                };
        float  kde_sigma_sqr = 0.0239282226563f; //the scale of the kernel in the KDE, h in eq (13).
        float  individual_ab_threshold  = 3.0f;
        float  ab_threshold = 10.0f;
        float  ab_confidence_slope = -0.5330578f;
        float  ab_confidence_offset = 0.7694894f;
        float  min_dealias_confidence = 0.3490659f;
        float  max_dealias_confidence = 0.6108653f;
        
        float  edge_ab_avg_min_value = 50.0f;
        float  edge_ab_std_dev_threshold = 0.05f;
        float  edge_close_delta_threshold = 50.0f;
        float  edge_far_delta_threshold = 30.0f;
        float  edge_max_delta_threshold = 100.0f;
        float  edge_avg_delta_threshold = 30.0f;//for test 0.0f;
        float  max_edge_count  = 3;//for test 5.0f;
        
        float  unwrapping_likelihood_scale = 2.0f; //scale parameter for the unwrapping likelihood, s_1^2 in eq (15).
        float  phase_confidence_scale = 3.0f; //scale parameter for the phase likelihood, s_2^2 in eq (23)
        float  kde_threshold = 0.5f; //threshold on the KDE output in eq (25), defines the inlier/outlier rate trade-off
        
        size_t  kde_neigborhood_size = 5; //spatial support of the KDE, defines a filter size of (2*kde_neigborhood_size+1 x 2*kde_neigborhood_size+1)
        size_t  num_hyps = 2;
    };

    struct TofConfSetting
    {
        uint16_t version = 0;
        uint16_t firmware_version = 0;
        uint16_t width = 0;
        uint16_t height = 0;
        uint16_t amp_threhold = 0;
        //bool    disableOffsetCorr = true;
        int16_t maxPhaseCorr = 0;
        int16_t minPhaseCorr = 0;
        uint8_t maxFreq = 100;
        uint8_t minFreq = 80;
        int32_t tillumCalib = 0;
        int32_t tsensorCalib = 0;
        float   coeffIllum = 0;
        float   coeffSensor = 0;
        bool    compat_data_en = false;
        MeasureMode measure_mode = USE_DEALIAS;
        std::vector<uint16_t> maxNLPhaseCorr;
        std::vector<uint16_t> minNLPhaseCorr;
        std::vector<float>    maxNLPhaseCorrRate;
        std::vector<float>    minNLPhaseCorrRate;
        std::vector<int16_t>  pixelwiseCorr;
        int                   phasePeriod = 0;
        int                   phase_duration = 0;
        OutputMode            output_mode = AMP_PHA;
        //calibrateion setting
        LensCalib             lens_calib;
        int        calib_disable;
        bool       pixelwiseCalibEnable = false;
        bool       nonlinearityCalibEnable;
        bool       commonPhaseCalibEnable;
        bool       tempCorrCalibEnable = false;
		bool       dealiasPhaseMergeEnable = true;
        bool       filterEnable = false;
//        bool       fishEyeEnable = false;
        float      unambiguousRange = 0;
        float      near_distance = 0;
        bool       factory_mode = 0;  //for calibration
        FilterParams filterParams;
    };
    class CameraSystem;
    class POINTCLOUD_EXPORT FrameGenerator;
    typedef Ptr<FrameGenerator> FrameGeneratorPtr;
    
    
    struct POINTCLOUD_EXPORT FrameStreamHeader
    {
        char version[2]; // 0 -> major, 1 -> minor
        GeneratorIDType generatorIDs[3]; // For raw (processed), depth and point cloud, in that order
        uint32_t frameCount;
        uint32_t frameRate;
    };
    
    struct POINTCLOUD_EXPORT FrameStreamPacket: public DataPacket
    {
        enum PacketType
        {
            PACKET_DATA = 0,
            PACKET_GENERATOR_CONFIG = 1
        };
        
        FrameStreamPacket(): DataPacket() {}
    };
    
    struct POINTCLOUD_EXPORT GeneratorConfigurationSubPacket
    {
        uint8_t frameType;
        uint32_t size;
        SerializedObject config;
        
        bool read(SerializedObject &object);
        bool write(SerializedObject &object);
    };
    
    class POINTCLOUD_EXPORT FrameStreamWriter
    {
        OutputFileStream &_stream;
        OutputFileStream _internalStream;
        
        Mutex _mutex;
        
        bool _isPaused = false;
        
        size_t _frameCount;
        FrameStreamHeader _header;
        FrameStreamPacket _rawpacket, _configPacket;
        GeneratorConfigurationSubPacket _generatorConfigSubPacket;
        
        bool _writeHeader();
        
        bool _init(GeneratorIDType processedRawFrameGeneratorID, GeneratorIDType depthFrameGeneratorID, GeneratorIDType pointCloudFrameGeneratorID);
        
    public:
        FrameStreamWriter(const String &filename, GeneratorIDType processedRawFrameGeneratorID, GeneratorIDType depthFrameGeneratorID, GeneratorIDType pointCloudFrameGeneratorID);
        FrameStreamWriter(OutputFileStream &stream, GeneratorIDType processedRawFrameGeneratorID, GeneratorIDType depthFrameGeneratorID, GeneratorIDType pointCloudFrameGeneratorID);
        
        inline bool isStreamGood() { return _stream.good(); };
		bool setHeader(FrameStreamHeader& header) { memcpy((char*)&_header, (char*)&header, sizeof(FrameStreamHeader)); return true; };
        bool pause();
        bool resume();
        inline bool isPaused() { return _isPaused; }
        
        bool write(FramePtr rawUnprocessed);
        
        inline SerializedObject &getConfigObject() { return _generatorConfigSubPacket.config; }
        bool writeGeneratorConfiguration(uint frameType);
        // Assumes the config sub-packet has been populated by using getConfigObject()
        
        bool close();
        
        virtual ~FrameStreamWriter() { close(); }
    };
    
    typedef Ptr<FrameStreamWriter> FrameStreamWriterPtr;
    
    class POINTCLOUD_EXPORT FrameStreamReader
    {
        InputFileStream &_stream;
        InputFileStream _internalStream;
        
        Vector<FileOffsetType> _allPacketOffsets;
        
        Vector<IndexType> _dataPacketLocation, _configPacketLocation;
        bool _isInit;
        FrameStreamHeader _header;
        size_t _currentPacketIndex; // index on _allPacketOffsets
        size_t _currentFrameIndex; // index on _dataPacketLocation
        size_t _startPacketIndex;
        CameraSystem &_sys;
        
        FrameGeneratorPtr _frameGenerator[3]; // for processed raw, depth and point cloud
        FrameStreamPacket _dataPacket, _configPacket;
        GeneratorConfigurationSubPacket _configSubPacket;
        
        bool _init();
        bool _getPacket(size_t packetIndex, FrameStreamPacket &packet);
        bool _readConfigPacket(size_t packetIndex);
        
    public:
        
        FrameStreamReader(const String &fileName, CameraSystem &sys);
        FrameStreamReader(InputFileStream &stream, CameraSystem &sys);
        
        inline bool isStreamGood() { return _stream.good() && _isInit; }
        
        Vector<FramePtr> frames; // 4 entries - raw (2 types), depth and point cloud corresponding to currently read frame index
        
        bool readNext();
        bool seekTo(size_t position);
        
        
        FrameStreamHeader *getFrameHeader()
        {
            return &_header;
        }
         FrameStreamPacket *getFrameConfigPacket(int idx)
        {
            if(idx<0||idx>4)
            {
                return NULL;
            }
            bool ret= _getPacket(idx, _configPacket);
            if(!ret)
            {
                return NULL;
            }
            return &_configPacket;
            
        }
         FrameStreamPacket *getFrameCurrDataPacket()
        {
            return &_dataPacket;
        }
        
        inline size_t currentPosition() { return _currentFrameIndex; }
        inline size_t size() {
                size_t total = _dataPacketLocation.size();
                return  total>_startPacketIndex?total - _startPacketIndex:0;
            
        }
        
        bool  getParameters(class TofConfSetting* confSetting) const;
        template <typename T>
        bool getStreamParam(const String &name, T &value) const;
        
        bool close();
        
        virtual ~FrameStreamReader() {}
    };
    
    typedef Ptr<FrameStreamReader> FrameStreamReaderPtr;
    
    class POINTCLOUD_EXPORT SocketStreamReader
    {
        CameraSystem &_sys;
        FrameGeneratorPtr _frameGenerator[3]; // for processed raw, depth and point cloud
        FrameStreamPacket _dataPacket;
        bool _init();
        uint _id1;
        uint _id2;
        uint _id3;
        SerializedObject object;
        PointCloud::String _config1;
        PointCloud::String _config2;
        PointCloud::String _config3;
    public:
        SocketStreamReader(CameraSystem &sys,uint id1,uint id2,uint id3,char *config1,int len1,char *config2,int len2,char *config3,int len3);
        
        bool readNext(char *ptr,int tsize);
        
        template <typename T>
        bool getStreamParam(const String &name, T &value) const;
        Vector<FramePtr> frames;
        virtual ~SocketStreamReader() {}
    };
    
    typedef Ptr<SocketStreamReader> SocketStreamReaderPtr;
}

#endif
