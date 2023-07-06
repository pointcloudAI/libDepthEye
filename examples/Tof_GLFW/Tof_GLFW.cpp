/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */
#include "CameraSystem.h"
#include "Timer.h"
#include "SimpleOpt.h"
#include "Common.h"
#include "Logger.h"
#include <iomanip>
#include <fstream>
#include <math.h>
#define GLFW_INCLUDE_GLU
#define GL_SILENCE_DEPRECATION
#define MAX_FRAME_WITDH     640
#define MAX_FRAME_HEIGHT    480

#define MAX_PHASE_VALUE 4095
#define RGB_FRAME_WITDH     1920
#define RGB_FRAME_HEIGHT    1080
#define SPEED_OF_LIGHT      299792458.0
#define M_PI_FL             3.14159265358979323846264338327950288
#define TOF_ANGLE_TO_PHASE_FACTOR 4096/(2*M_PI_FL)
#define RAW_PHASE_MAX_VALUE 4096
#include "GLFW/glfw3.h"
#include "glfw_window.h"
//#define CHECK_PARSER


using namespace PointCloud;


//#define VXL_FILE "/Volumes/WorkSpace/vxl2/test_0414.vxl"
//#define VXL_FILE "C:\\Users\\swortian\\OneDrive\\pointcloud\\deptheye-tools\\DeptheyeViewer\\export_images\\3.vxl"

#define VXL_FILE "C:\\Users\\wuxia\\Desktop\\test330.vxl"
//#define TEST_VXL

#ifdef TEST_VXL
	ThreadPtr	_vxlDrawThread = 0;
	static	Mutex	_vxlDrawAccessMutex;
	static	bool	_vxlDrawFlag = false;
#endif // TEST_VXL

#define TEST_IDX            240*640 + 320
int         smin = 0;
int         smax = 0xfff;//0x7ff;
uint8_t     Rainbow[3][65536] = {{0}};
FrameSize   g_tofFrameSize;
Mutex       _dataAccessMutex;
OutputMode g_output_mode  = AMP_PHA;
bool        autoStart = false;
//RegionOfInterest roi;
//uint binning_mod    = 0;
//uint scale_binning  = 1;
std::vector<uint16_t>               amplitude_data,phase_data;
std::vector<IntensityPoint>         xyz_data;
std::vector<int16_t>        g_pha_amp;
texture                     view_tex[4];
RawDataFrame                raw_frame[4];
std::vector<vertex>         vertices;
static PointCloud::CameraSystem    cameraSys;
PointCloud::DepthCameraPtr  depthCamera;
PointCloud::DevicePtr       device;
ThreadPtr                   _captureThread = 0;
ThreadPtr					_hotplugThread = 0;
ThreadPtr					_testThread = 0;
TimeStampType s1,e1;

bool		isSelectChnnl = false;
bool		isReadConfFromModule = false;
bool		isSetIntergralTime = false;
uint		intgTime = 0;
bool		isSetIntergralTimeScale = false;
uint		intgTimeScale = 0;
bool		isWithOneTakeEight = false;
bool		isTransportFileToModule = false;
String		fileOriginLocation = "";
String		fileDestinationLocation = "";
bool		isGetAllID = false;
bool		isRunLoop = false;
ThreadPtr	_loopRunThread = 0;
uint		timeRunLoop = 5*1000;
bool		isDetectHotPlug = false; 
ThreadPtr	_detectHotPlugThread = 0;
bool		isLetModuleWorkOn = false;

void onPowerChangedHandle(DepthCamera &dc,int isOn12V){

   std::cout << " --------  onPowerChangedHandle:  ------- " << isOn12V  << std::endl;
   if (isOn12V == 1){
       dc.setPowerLevel(1);
   }else{
       dc.setPowerLevel(7);
   }
}

void MakeColorTbl()
{
    for(int i = smin; i < smax; i++){
      //short ii = ((double)(i-smin) / (double)(smax-smin))*dsrange > dsrange+512 ? srange+512 : (short)(((double)(i-smin) / (double)(smax-smin))*dsrange) + 255;
        short ii =  (double)(i-smin)/(smax-smin)*(1276-255) + 255;
        if(ii < 0)
            {Rainbow[0][i] = 255; Rainbow[1][i] = 255; Rainbow[2][i] = 255;}
        else if(ii < 255)
            {Rainbow[0][i] = 255; Rainbow[1][i] = 255; Rainbow[2][i] = 255;}
        else if(ii < 511)
            {Rainbow[0][i] = (char)(     0); Rainbow[1][i] = (char)( ii-255); Rainbow[2][i] = (char)(   255);}
        else if(ii < 766)
            {Rainbow[0][i] = (char)(     0); Rainbow[1][i] = (char)(    255); Rainbow[2][i] = (char)(765-ii);}
        else if(ii < 1021)
            {Rainbow[0][i] = (char)(ii-765); Rainbow[1][i] = (char)(    255); Rainbow[2][i] = (char)(     0);}
        else if(ii < 1276)
            {Rainbow[0][i] = (char)(   255); Rainbow[1][i] = (char)(1275-ii); Rainbow[2][i] = (char)(     0);}
        else
            {Rainbow[0][i] = (char)(     0); Rainbow[1][i] = (char)(      0); Rainbow[2][i] = (char)(     0);}
    }
    //for(int i = imMax * 2 - 2; i < 65536; i++) {Rainbow[0][i] = 0; Rainbow[1][i] = 0; Rainbow[2][i] = 0;}
    for(int i = 0; i < smin; i++) {Rainbow[0][i] = 255; Rainbow[1][i] = 255; Rainbow[2][i] = 255;}
    for(int i = smax; i < 65536; i++) {Rainbow[0][i] = 0; Rainbow[1][i] = 0; Rainbow[2][i] = 0;}
}

void rgbCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c)
{
    Timer t;
    Lock<Mutex> _(_dataAccessMutex);
    const RawDataFrame *d = dynamic_cast<const RawDataFrame *>(&frame);
    int length = d->data.size();
    if(raw_frame[2].data.size() < length)
    {
        std::cout << "RGB Frame: buffer length:" << raw_frame[2].data.size() << " receive length:" << length << std::endl;
        return ;
    }
    memcpy(raw_frame[2].data.data(),d->data.data(),length);
    static int count = 0;
    count++;
    if(count%30 == 0)
    {
        e1 =  t.getCurrentRealTime();
        float dur = (e1 - s1);
        int idx = RGB_FRAME_WITDH*RGB_FRAME_HEIGHT/2 + RGB_FRAME_WITDH/2;
        
        std::cout << "RGB Frame:" << length  << " time:" <<  dur  << "ms fps:"  << 30*1e6/dur << " idx:"<< idx << "  center point[" << std::hex << (uint8_t) d->data[idx*3] << "," <<(uint8_t) d->data[idx*3+1]<< "," << (uint8_t)d->data[idx*3+2] << "]" << std::dec<< std::endl;
        s1 = e1;
    }
}

void verticalPhaseCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c)
{
	const RawDataFrame *d = dynamic_cast<const RawDataFrame *>(&frame);
	size_t idx = 240 * 640 + 320;
	size_t pha_len = 480 * 640;
	int16_t* v = (int16_t*)d->data.data();
	int16_t* v_amp = v + idx;
	int16_t* v_x = v + pha_len + idx;
	int16_t* v_y = v + 2 * pha_len + idx;
	int16_t* v_z = v + 3 * pha_len + idx;
	int16_t* phz = v + 3 * pha_len;
	static int count3 = 0;
	count3++;
	if (count3 % 30 == 0)
	{
		std::cout << __func__ << " index:" << idx << " v_amp:" << (int16_t)*v_amp << " v_x:" << (int16_t)*v_x << " v_y:" << (int16_t)*v_y << " v_z:" << (int16_t)*v_z << std::endl;
	}

	// save rawdata 
	static int count4 = 0;
	if(count4 % 30 == 0 )
	{
		char file_name[100] = {0};
		snprintf(file_name, 100 ,"data/vergray_%d.raw",count4);
		FILE* gray_file = fopen(file_name, "wb");
		if (gray_file) 
		{
			fwrite(v, pha_len, 2 ,gray_file);
			fclose(gray_file);
		}

		char depth_file_name[100] = {0};
		snprintf(depth_file_name, 100, "data/depth_%d.raw",count4);
		FILE* depth_file = fopen(depth_file_name, "wb");
		if (depth_file) 
		{
			fwrite(phz, pha_len, 2 ,depth_file);
			fclose(depth_file);
		}
	}
	count4++;
	//  ~~~~~~~~~~~~~~~~~~~~~~~~~`
}

void pointcloudCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c)
{
	Timer t;
	static TimeStampType s1 = 0, e1 = 0;
	//Lock<Mutex> _(_dataAccessMutex);
	const XYZIPointCloudFrame *d = dynamic_cast<const XYZIPointCloudFrame *>(&frame);
	// uint x_offset = roi.x;
	// uint y_offset = roi.y;

	for (int i = 0; i < g_tofFrameSize.height; i++)
	{
		for (int j = 0; j < g_tofFrameSize.width; j++)
		{
			uint index = i * g_tofFrameSize.width + j;
			/*
			if(i < y_offset || j < x_offset || j >= x_offset + roi.width || i >= y_offset + roi.height)
			{
				IntensityPoint point;
				point.i = 0;
				point.x = 0;
				point.y = 0;
				point.z = 0;
				xyz_data[index] = point;
			}else
				*/
			{
				//uint buf_index = i*frameSize.width + j  ;//(i-y_offset)/scale_binning*frameSize.width + (j-x_offset)/scale_binning;
				xyz_data[index] = d->points[index];

			}
			//if(index == TEST_IDX)
			//    std::cout << "pointcloudCallback: i:"<< xyz_data[index].i <<" x:" << xyz_data[index].x << " y:" << xyz_data[index].y << " z:" << xyz_data[index].z<< std::endl;
		}
	}
	static int count2 = 0;
	count2++;
	if (count2 % 30 == 0)
	{
		uint idx = TEST_IDX;
		std::cout << __func__ << " idx:" << idx << " i:" << xyz_data[idx].i << " x:" << xyz_data[idx].x << " y:" << xyz_data[idx].y << " z:" << xyz_data[idx].z << std::endl;
	}
}


void rawdataCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c)
{
    Timer t;
    static TimeStampType s=0,e=0;
   // Lock<Mutex> _(_dataAccessMutex);
    const PhaAmpFrame *d = dynamic_cast<const PhaAmpFrame *>(&frame);
    
    uint16_t* amp = (uint16_t *)d->amplitude();
    uint16_t* ph = (uint16_t *)d->phase();
    //uint x_offset = roi.x;
    //uint y_offset = roi.y;
    for(int i = 0;i < g_tofFrameSize.height;i++ )
    {
        for(int j = 0;j < g_tofFrameSize.width;j++ )
        {
            uint index = i*g_tofFrameSize.width +j;
            amplitude_data[index] = amp[index];
            phase_data[index] = ph[index];
        }
    }
    uint index = g_tofFrameSize.height/2*g_tofFrameSize.width + g_tofFrameSize.width/2;
    static int count1 = 0;
	static float Tillum;
	static float Tsensor;
    count1++;
	if(count1%30 == 0)
    {
		depthCamera->get(TILLUM, Tillum);
		depthCamera->get(TSENSOR, Tsensor);

        e = t.getCurrentRealTime();
        float dur = (e - s)/1e6;
		std::cout << __func__ << " index:" << index << " phase:" << std::dec << phase_data[index] << " amp:" << std::dec << amplitude_data[index] << " time:" << dur << "ms fps:" << (float)count1/dur
			<< " curFreq:" << d->id << " Tsensor:" << Tsensor << " Tillum:" << Tillum << std::endl;
		count1 = 0;
        s = e;
    }
}

void unprocCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c)
{
	Lock<Mutex> _(_dataAccessMutex);
	const RawDataFrame* rawDataFrame = dynamic_cast<const RawDataFrame*>(&frame);

	int32_t id = rawDataFrame->id;
	FrameSize size = g_tofFrameSize;
	int32_t idx = size.width * size.height / 2 + size.width / 2;
	size_t raw_len = rawDataFrame->data.size();
	int16_t *data = (int16_t *)rawDataFrame->data.data();
	size_t length = g_tofFrameSize.width*g_tofFrameSize.height;
	static TimeStampType s = 0, e = 0;
	Timer t;


	if (g_output_mode == AMP_PHA)
	{
		static uint count = 0;
		count++;
		if (count % 30 == 0)
		{
			e = t.getCurrentRealTime();
			float dur = (e - s) / 1e6;
			std::cout << __func__ << " idx:" << idx << " pha:" << data[idx] << " amp:" << data[idx + length] << " freq:" << id << " tsensor:" << rawDataFrame->tsensor << " tillum:" << rawDataFrame->tillum << " fps:" << (float)count/dur << std::endl;
			count = 0;
			s = e;
		}
	}
	else if (g_output_mode == A_B)
	{
		size_t phase_length = length;

		if (raw_len == phase_length * 8)//sizeof(int16_t))
		{
			uint index = 240 * 640 + 320;

		}
		else
		{
			std::cout << __func__ << " Error Data length =" << raw_len << std::endl;
		}
		/*
		if( total > 0 && total_rate > 0)
		{
		avg_phase = total/total_rate;
		//uint index = frameSize.height/2*frameSize.width + frameSize.width/2;
		//std::cout << " unprocCallback avg phase:" << avg_phase << " center point phase:" << data[2*index +1] << " amp:" <<  data[2*index] << " max_phase:" << max_phase  << " max_amp:" << max_amp<< std::endl;
		}*/
	}
	else if (g_output_mode == AplusB)
	{
		if (raw_len == length * 2)//sizeof(int16_t))
		{
			uint index = 240 * 640 + 320;
			memcpy((uint16_t*)amplitude_data.data(), (uint16_t*)data, raw_len);
			std::cout << __func__ << " idx:" << index << " amp:" << amplitude_data[index] << " freq:" << id << std::endl;
		}
	}
	else
	{
		uint16_t* pha = (uint16_t *)rawDataFrame->data.data();
		uint16_t* amp = (uint16_t *)rawDataFrame->data.data() + size.width * size.height;
		std::cout << __func__ << "idx:" << idx << " pha:" << pha[idx] << " amp:" << amp[idx] << " id:" << id << std::endl;
	}
}

#define Test_HotPlug		0
#define	Test_OneTakeEight	1
#define	Test_Reload			0
#define Test_StartStop		0
#define Test_PointedChannel	0
#define Test_Api			0
void ThreadHotplug()
{
	while (1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(5000));
#if (Test_HotPlug)
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		bool bOnline = depthCamera->online();
		//std::cout << "online: " << bOnline << std::endl;
		if (!bOnline && depthCamera->isRunning())
		{
			depthCamera->stop();
			autoStart = false;
		}
		else if (bOnline && autoStart == false && !depthCamera->isRunning())
		{
			autoStart = true;
			depthCamera->getFrameSize(g_tofFrameSize);//for the case that not connet before start 
			depthCamera->start();
		}
#endif // (Test_HotPlug)
	}
}

void ThreadWrapper()
{
	uint currentChnnl = 0;
	uint loopCnt = 0;
	uint8_t chnlOn = 0;

	uint currCase = 0;
	bool mycase = true;
	uint restart_time = 0;

    while (true) {
#if (Test_OneTakeEight)
		loopCnt++;
		if (loopCnt % 1 == 0)
		{
			currentChnnl = loopCnt / 1;
			if ( !((1<<(currentChnnl-1))&chnlOn) || currentChnnl == 9)
			{
				if (currentChnnl == 9)
				{
					//depthCamera->controlOneTakeEightBoard(false);
					//std::this_thread::sleep_for(std::chrono::milliseconds(5000));
					//depthCamera->controlOneTakeEightBoard(true);
					//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
					std::cout << "------------start get status-------------" << std::endl;
					
					chnlOn = 0;
					depthCamera->getChannelStatus(1);
					int retry_cnt = 5;
					while (chnlOn == 0 && retry_cnt--)
					{
						chnlOn = depthCamera->getChannelStatus(0);
						std::this_thread::sleep_for(std::chrono::milliseconds(2000));
					}
					loopCnt = 0;
					currentChnnl = 0;
				}
				continue;
			}
		}

		if (loopCnt % 1 == 0)
		{
			depthCamera->stop();
			std::cout << "after stop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<ready start" << std::endl;
			currentChnnl = loopCnt / 1;
			depthCamera->changeToChannelX(currentChnnl);
			depthCamera->checkBoardOnline();
			depthCamera->start();
			std::this_thread::sleep_for(std::chrono::milliseconds(180000));

			//depthCamera->readConfFromModule();
			//String conf_name = "";
			//conf_name = depthCamera->getReadConfFromModuleState();
			//int retry = 0;
			//while (conf_name == "" && retry < 5)
			//{
			//	conf_name = depthCamera->getReadConfFromModuleState();
			//	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			//	retry++;
			//}

			//depthCamera->getFull_SN();
			//depthCamera->getBatchNo();
			//depthCamera->getSensorID();
		}
#endif // (Test_OneTakeEight)

#if (Test_Api)
		uint sensor_manuf_id = 0;
		unsigned long long SensorID = 0;
		String full_sn = "full_sn_test";
		std::this_thread::sleep_for(std::chrono::milliseconds(15000));

		full_sn = depthCamera->getFull_SN();
		logger(LOG_INFO) << "Full_SN: " << full_sn << std::endl;

		sensor_manuf_id = depthCamera->getBatchNo();
		logger(LOG_INFO) << "sensor_manuf_id: 0x" << std::hex << std::setw(6) << std::setfill('0') << sensor_manuf_id << std::endl;

		SensorID = depthCamera->getSensorID();
		logger(LOG_INFO) << "SensorID: 0x" << std::hex << std::setw(14) << std::setfill('0') << SensorID << std::endl;


#endif // (Test_Api)

#if (Test_PointedChannel)
		if (mycase)
		{
			currentChnnl = 4;
			depthCamera->changeToChannelX(currentChnnl);
			mycase = false;
		}
#endif // (Test_PointedChannel)

#if (Test_Reload)
		loopCnt++;
		if (loopCnt % 1000 == 0)
		{
			currCase = loopCnt / 1000;
			std::this_thread::sleep_for(std::chrono::milliseconds(10000));
			depthCamera->stop();

			if (currCase == 1)
			{
				int output_mode = AplusB;
				depthCamera->set("output_mode", output_mode);
				bool compat_data_en = 0;
				depthCamera->set("compat_data_en", compat_data_en);
				uint dutof_mode = 1;
				depthCamera->set("dutof_mode", dutof_mode);
				depthCamera->confSettingReload();
			}
			else if (currCase == 2)
			{
				int output_mode = AMP_PHA;
				depthCamera->set("output_mode", output_mode);
				bool compat_data_en = 1;
				depthCamera->set("compat_data_en", compat_data_en);
				uint dutof_mode = 0;
				depthCamera->set("dutof_mode", dutof_mode);
				depthCamera->confSettingReload();
				loopCnt = 0;
			}
			depthCamera->getFrameSize(g_tofFrameSize);
			int output = 4;
			depthCamera->get(OUTPUT_MODE, output,true);
			g_output_mode = (OutputMode)output;
			depthCamera->start();
		}
#endif // Test_Reload

#if (Test_StartStop)
		depthCamera->start();
		std::this_thread::sleep_for(std::chrono::milliseconds(6000));
		depthCamera->stop();
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));
#endif // Test_StartStop

    }
}

#define	TEST_RELOAD	1
#define	TEST_SYS_CAMERA	0
#define	TEST_UVC	0
#define TEST_ONE_TAKE_EIGHT_API	0
void ThreadTest()
{
	//if (isReadConfFromModule)
	//{
	//	depthCamera->readConfFromModule();
	//	String conf_name = "";
	//	conf_name = depthCamera->getReadConfFromModuleState();
	//	while (conf_name == "")
	//	{
	//		conf_name = depthCamera->getReadConfFromModuleState();
	//		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//	}
	//}
	uint currentChnnl = 0;
	uint loopCnt = 0;
	uint8_t chnlOn = 0;
	bool isFirstStart = true;
	while (1)
	{
		//std::this_thread::sleep_for(std::chrono::milliseconds(20000));
		////depthCamera->checkOneTakeEightBoardShortCircuit();
		//depthCamera->getMipiResetCount();
		//depthCamera->getPhaseLoseCount();
		//depthCamera->checkBoardOnline();

#if(TEST_RELOAD)
		std::this_thread::sleep_for(std::chrono::milliseconds(10000));
		depthCamera->confSettingReload(1);
#endif

	}
}

void loopRunThread()
{
	uint curChn = 8;
	uint16_t chnStatus = 65280;
	bool isFirstRun = true;
	// while (isRunLoop)
	while(0)
	{
		curChn++;
		if (!((1 << (curChn - 1))&chnStatus) || curChn == 9)
		{
			if (curChn == 9)
			{
				/*if (!isFirstRun)
				{
					isRunLoop = false;
					curChn = 0;
					continue;
				}
				else
					isFirstRun = false;*/
				std::cout << "------------get status start-------------" << std::endl;
				chnStatus = 65280;
				depthCamera->getChannelStatus(1);
				int retry_cnt = 5;
				//while (chnStatus == 65280 && retry_cnt--)
				while (chnStatus == 65280)
				{
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));
					chnStatus = depthCamera->getChannelStatus(0);
				}
				if (chnStatus < 255)
				{
					depthCamera->checkOneTakeEightBoardShortCircuit();
				}
				curChn = 0;
				std::cout << "------------get status finished-------------" << std::endl;
			}
			continue;
		}

		if (depthCamera->changeToChannelX(curChn))
		//if (1)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
			if (isGetAllID)
			{
				depthCamera->getFull_SN();
				depthCamera->getBatchNo();
				depthCamera->getSensorID();
			}

			//if (isReadConfFromModule)
			//{
			//	depthCamera->readConfFromModule();
			//	String conf_name = "";
			//	int retry_cnt = 5;
			//	conf_name = depthCamera->getReadConfFromModuleState();
			//	while (conf_name == "" && retry_cnt--)
			//	{
			//		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			//		conf_name = depthCamera->getReadConfFromModuleState();
			//	}
			//}
			//depthCamera->changeToChannelX(curChn);
			//depthCamera->getMipiResetCount();
			//depthCamera->getPhaseLoseCount();
			std::cout << "------------ready start-------------" << std::endl;
			depthCamera->getFrameSize(g_tofFrameSize);//for the case that disconncet before start 
			depthCamera->start();
			FrameRate r;
			r.numerator = 25;
			r.denominator = 1;
			bool res = depthCamera->setFrameRate(r);
			if (!res)
				std::cout << "Failed to set frame rate:" << r.numerator << std::endl;
			//std::this_thread::sleep_for(std::chrono::milliseconds(timeRunLoop));

			//depthCamera->stop();
			//std::this_thread::sleep_for(std::chrono::milliseconds(2000));
			//std::cout << "------------after stop-------------" << std::endl;
			//depthCamera->getMipiResetCount();
			//depthCamera->getPhaseLoseCount();

			//if (isLetModuleWorkOn)
			//{
			//	depthCamera->letModuleWorkOn();
			//	std::this_thread::sleep_for(std::chrono::milliseconds(3*1000));
			//}
			break;
		}
		else
		{
			//for new hardware version function
			//depthCamera->checkOneTakeEightBoardShortCircuit();
		}
	}
	//test set freq
	//while(isRunLoop)
	while(0)
	{
		depthCamera->stop();

		std::this_thread::sleep_for(std::chrono::milliseconds(5000));

		static int cnt = 0;
		if(cnt%2 == 0)
		{
			int freq1 = 0;
			int freq2 = 0;
			
			depthCamera->get("mod_freq1",freq1);
			depthCamera->get("mod_freq2",freq2);
			printf("freq1:%d, freq2:%d\n",freq1,freq2);
			std::this_thread::sleep_for(std::chrono::milliseconds(5000));
			
			freq1 = 80;
			freq2 = 50;
			depthCamera->set("mod_freq1",freq1);
			depthCamera->set("mod_freq2",freq2);

			freq1 = 0;
			freq2 = 0;
			depthCamera->get("mod_freq1",freq1);
			depthCamera->get("mod_freq2",freq2);
			printf("freq1:%d, freq2:%d\n",freq1,freq2);
		}
		else
		{
			int freq1 = 0;
			int freq2 = 0;
			
			depthCamera->get("mod_freq1",freq1);
			depthCamera->get("mod_freq2",freq2);
			printf("freq1:%d, freq2:%d\n",freq1,freq2);
			std::this_thread::sleep_for(std::chrono::milliseconds(5000));
			
			freq1 = 100;
			freq2 = 20;
			depthCamera->set("mod_freq1",freq1);
			depthCamera->set("mod_freq2",freq2);

			freq1 = 0;
			freq2 = 0;
			depthCamera->get("mod_freq1",freq1);
			depthCamera->get("mod_freq2",freq2);
			printf("freq1:%d, freq2:%d\n",freq1,freq2);
		}
		cnt++;
		//start
		FrameRate r;
		r.numerator = 25;
		r.denominator = 1;
		bool res = depthCamera->setFrameRate(r);
		if (!res)
			std::cout << "Failed to set frame rate:" << r.numerator << std::endl;
		depthCamera->getFrameSize(g_tofFrameSize);
		if (!depthCamera->start(TOF_CAMERA))
		{
			logger(LOG_ERROR) << "Could not start the depth camera " << depthCamera->id() << std::endl;
			//return -1;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	}
	while (0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		//static int cnt = 0;
		//cnt++;
		//if (cnt == 2)
		//{
		//	depthCamera->testParametersSet();
		//}
		//else if(cnt == 4)
		//{
			depthCamera->testParametersSet();
		//	cnt = 0;
		//}
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	while (1)
	{
		float tillum = -40.0f;
		float tsensor = -40.0f;
		depthCamera->get(TILLUM, tillum, true);
		depthCamera->get(TSENSOR, tsensor,true);

		printf("[%s] tillum:%.04f, tsensor:%.04f\n",__func__,tillum, tsensor);
	}
}

void detectHotPlugThread()
{
	bool autoStart = true;
	while (isDetectHotPlug)
	{
		bool bOnline = depthCamera->online();
		if (!bOnline && depthCamera->isRunning())
		{
			depthCamera->stop();
			autoStart = false;
		}
		else if (bOnline && autoStart == false && !depthCamera->isRunning())
		//else if (bOnline && !depthCamera->isRunning())
		{
			autoStart = true;
			depthCamera->getFrameSize(g_tofFrameSize);//for the case that disconncet before start 
			depthCamera->start();
		}
	}
}

void render_textures(int cols, int rows, float view_width, float view_height,int channel_size)
{
    Lock<Mutex> _(_dataAccessMutex);
    for(int stream_no = 0;stream_no< channel_size;stream_no++)
    {
        rect frame_location{ view_width * (stream_no % cols), view_height * (1 - stream_no / rows), view_width, view_height };
        int width = g_tofFrameSize.width;
        int height = g_tofFrameSize.height;
        if(stream_no == 2)
        {
            width = RGB_FRAME_WITDH;
            height = RGB_FRAME_HEIGHT;
        }
        view_tex[stream_no].render(&raw_frame[stream_no],width,height, frame_location);
    }
}

void draw_pointcloud(float width, float height, glfw_state& app_state, const vertex* vertices,int vertices_count)
{
    //glColor3f(1.0, 1.0, 1.0);
    glViewport(width/2, 0,width/2 , height/2);
    // OpenGL commands that prep screen for the pointcloud
    glLoadIdentity();
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    
    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    gluPerspective(60, width / height, 0.01f, 10.0f);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);
    
    glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
    glRotated(app_state.pitch, 1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    glTranslatef(0, 0, -0.5f);
    
    glPointSize(width / 640);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, app_state.tex.get_gl_handle());
    float tex_border_color[] = { 0.8f, 0.8f, 0.8f, 0.8f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, tex_border_color);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 0x812F); // GL_CLAMP_TO_EDGE
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 0x812F); // GL_CLAMP_TO_EDGE
    glBegin(GL_POINTS);
    
    
    /* this segment actually prints the pointcloud */
    //auto vertices = points.get_vertices();              // get vertices
    //auto tex_coords = points.get_texture_coordinates(); // and texture coordinates
    for (int i = 0; i < vertices_count; i++)
        //for (int i = 0; i < points.size(); i++)
    {
        if (vertices[i].z)
        {
            // upload the point and texture coordinates only for points we have depth data for
            glVertex3fv(vertices[i]);
            //    glTexCoord2fv(tex_coords[i]);
        }
    }
    
    // OpenGL cleanup
    glEnd();
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
}
#ifdef TEST_VXL
void vxlDrawThread()
{
	const std::string no_camera_message = "No camera connected, please connect 1 or more";
	MakeColorTbl();
	window      app(1280, 960, "Pointcloud.ai Tof Example");
	glfw_state  app_state;
	register_glfw_callbacks(app, app_state);
	int channel_size = 3;
	static std::condition_variable _dataAvailableCondition;

	while (app)
	{
		std::cout << "*-*-*-*-*-*-*-*-*-*-*-*-*-*" << std::endl;
		if (1)
		{
			Lock<Mutex> lck(_vxlDrawAccessMutex);
			_dataAvailableCondition.wait(lck, [] {return _vxlDrawFlag; });
			_vxlDrawFlag = false;
		}
		draw_text(int(std::max(0.f, (app.width() / 2) - no_camera_message.length() * 3)),
			int(app.height() / 2), no_camera_message.c_str());

		int cols = 2;// int(std::ceil(std::sqrt(total_number_of_streams)));
		int rows = 2;// int(std::ceil(total_number_of_streams / static_cast<float>(cols)));
		float view_width = (app.width() / cols);
		float view_height = (app.height() / rows);
		for (int index = 0; index < g_tofFrameSize.width*g_tofFrameSize.height; index++)
		{
			raw_frame[1].data[index] = (amplitude_data[index] * 256) / 4096;
			if (amplitude_data[index] < 0)
			{
				raw_frame[0].data[index * 3] = 0;    //B
				raw_frame[0].data[index * 3 + 1] = 0;    //G
				raw_frame[0].data[index * 3 + 2] = 0;
			}
			else
			{
				raw_frame[0].data[index * 3] = Rainbow[0][phase_data[index]];    //B
				raw_frame[0].data[index * 3 + 1] = Rainbow[1][phase_data[index]];    //G
				raw_frame[0].data[index * 3 + 2] = Rainbow[2][phase_data[index]];    //R
			}
		}
		render_textures(cols, rows, view_width, view_height, channel_size);
		draw_pointcloud(app.width(), app.height(), app_state, vertices.data(), (int)vertices.size());
	}
}
#endif // TEST_VXL


 OutputFileStream out("/Users/hejie/Documents/PointcloudAI/log.tzt", std::ios::out );
void logerfile( const String & str)
{
    out << str ;
}

enum Options
{
	TEST_HELP,
	TEST_WITH_ONE_TAKE_EIGHT_BOARD,
	TEST_SELECTED_CHANNEL,
	TEST_READ_CONF_FROM_MODULE,
	TEST_SET_INTERGRAL_TIME,
	TEST_SET_INTERGRAL_TIME_SCALE,
	TEST_TRANSPORT_FILE_TO_MODULE,
	TEST_TRANSPORT_FILE_ORIGIN,
	TEST_TRANSPORT_FILE_DESTINATION,
	TEST_GET_ALL_KINDS_OF_ID,
	TEST_RUN_LOOP,
	TEST_RUN_LOOP_TIME,
	TEST_DETECT_HOTPLUG,
	TEST_LET_MODULE_WORK_ON,
};

Vector<CSimpleOpt::SOption> argumentSpecifications =
{
	{	TEST_HELP,							"-h",	SO_NONE,	"get help message"},
	{	TEST_WITH_ONE_TAKE_EIGHT_BOARD,		"-v",	SO_NONE,	"if oneTakeEight board parent"},
	{	TEST_SELECTED_CHANNEL,				"-s",	SO_REQ_SEP,	"select channel number[1,8]"},
	{	TEST_READ_CONF_FROM_MODULE,			"-r",	SO_NONE,	"read conf from module"},
	{	TEST_SET_INTERGRAL_TIME,			"-I",	SO_REQ_SEP,	"set intergraltion time[0,100]"},
	{	TEST_SET_INTERGRAL_TIME_SCALE,		"-S",	SO_REQ_SEP,	"set intergraltion time scale[0,100]"},
	{	TEST_TRANSPORT_FILE_TO_MODULE,		"-t",	SO_NONE,	"if transport file to module"},
	{	TEST_TRANSPORT_FILE_ORIGIN,			"-f",	SO_REQ_SEP,	"the original location of file"},
	{	TEST_TRANSPORT_FILE_DESTINATION,	"-n",	SO_REQ_SEP,	"the destination location of file"},
	{	TEST_GET_ALL_KINDS_OF_ID,			"-i",	SO_NONE,	"get all kinds of id"},
	{	TEST_RUN_LOOP,						"-L",	SO_NONE,	"run eight channel in loop"},
	{	TEST_RUN_LOOP_TIME,					"-T",	SO_REQ_SEP,	"time cost at echo channel in loop test, unit second"},
	{	TEST_DETECT_HOTPLUG,				"-H",	SO_NONE,	"detect usb hotplug to stop start uvc"},
	{	TEST_LET_MODULE_WORK_ON,			"-w",	SO_NONE,	"let module work on in verification calibration"},
	SO_END_OF_OPTIONS
};

void help()
{
	std::cout << "TofTest Guide v1.0" << std::endl;

	CSimpleOpt::SOption *option = argumentSpecifications.data();

	while (option->nId >= 0)
	{
		std::cout << option->pszArg << " " << option->helpInfo << std::endl;
		option++;
	}
}


int main(int argc, char *argv[])
{
	CSimpleOpt s(argc, argv, argumentSpecifications);
	char *endptr;
	int curr_chnnl = 1;
	String vxlFile = "C:\\Users\\wuxia\\Desktop\\test3495.vxl";
	while (s.Next())
	{
		if (s.LastError() != SO_SUCCESS)
		{
			std::cout << s.GetLastErrorText(s.LastError()) << ": '" << s.OptionText() << "' (use -h to get command line help)" << std::endl;
			help();
			return -1;
		}

		Vector<String> splits;
		switch (s.OptionId())
		{
		case TEST_HELP:
			help();
			return 0;
		case TEST_SELECTED_CHANNEL:
			curr_chnnl = atoi(s.OptionArg());
			isSelectChnnl = true;
			break;
		case TEST_READ_CONF_FROM_MODULE:
			isReadConfFromModule = true;
			break;
		case TEST_SET_INTERGRAL_TIME:
			isSetIntergralTime = true;
			intgTime = atoi(s.OptionArg());
			break;
		case TEST_SET_INTERGRAL_TIME_SCALE:
			isSetIntergralTimeScale = true;
			intgTimeScale = atoi(s.OptionArg());
			break;
		case TEST_WITH_ONE_TAKE_EIGHT_BOARD:
			isWithOneTakeEight = true;
			break;
		case TEST_TRANSPORT_FILE_TO_MODULE:
			isTransportFileToModule = true;
			break;
		case TEST_TRANSPORT_FILE_ORIGIN:
			fileOriginLocation = s.OptionArg();
			break;
		case TEST_TRANSPORT_FILE_DESTINATION:
			fileDestinationLocation = s.OptionArg();
			break;
		case TEST_GET_ALL_KINDS_OF_ID:
			isGetAllID = true; 
			break;
		case TEST_RUN_LOOP:
			isRunLoop = true;
			break;
		case TEST_RUN_LOOP_TIME:
			timeRunLoop = (uint)atoi(s.OptionArg()) * 1000;
			break;
		case TEST_DETECT_HOTPLUG:
			isDetectHotPlug = true;
			break;
		case TEST_LET_MODULE_WORK_ON:
			isLetModuleWorkOn = true;
			break;
		default:
			help();
			break;
		};
	}

    logger.setDefaultLogLevel(LOG_DEBUG);
    logger.addOutputStream(logerfile);
    logger(LOG_DEBUG) <<  "Log DEBUG TEst: "  << std::endl;
    g_tofFrameSize.width    = MAX_FRAME_WITDH;
    g_tofFrameSize.height   = MAX_FRAME_HEIGHT;
    size_t buffer_size      = MAX_FRAME_WITDH*MAX_FRAME_HEIGHT;
    phase_data.resize(buffer_size);
    amplitude_data.resize(buffer_size);
    xyz_data.resize(buffer_size);
    vertices.resize(buffer_size);
    raw_frame[0].data.resize(buffer_size*3 );
    raw_frame[0].id = FORMAT_RGB8;
    raw_frame[1].data.resize(buffer_size );
    raw_frame[1].id = FORMAT_Y8;
    raw_frame[2].data.resize(RGB_FRAME_WITDH*RGB_FRAME_HEIGHT*3 );
    raw_frame[2].id = FORMAT_RGB8;
    raw_frame[3].data.resize(buffer_size*3 );
    raw_frame[3].id = FORMAT_RGB8;
    int channel_size = 3;

    /*
     for(int x = 0; x <MAX_FRAME_WITDH;x++)
     {
         for(int y = 0; y< MAX_FRAME_HEIGHT;y++)
         {
             int idx = y*MAX_FRAME_WITDH + x;
             raw_frame[0].data[idx*3] = x%256;
             raw_frame[0].data[idx*3 + 1] = y%256;
             raw_frame[0].data[idx*3 + 2] = 128;
             raw_frame[1].data[idx] = idx%256;
             raw_frame[2].data[idx*3] =y%256;
             raw_frame[2].data[idx*3 + 1] = 128;
             raw_frame[2].data[idx*3 + 2] = x%256;
     
             vertices[idx].x = -1.0 + 2.0/MAX_FRAME_WITDH*x;
             vertices[idx].y = -0.5 + 1.0/MAX_FRAME_HEIGHT*y;                        
         }
     }
     */
    
    const Vector<DevicePtr> &devices = cameraSys.scan();
    bool found = false;
    for (auto &d: devices){
        logger(LOG_INFO) <<  "Detected devices: "  << d->id() << std::endl;
        device = d;
        found = true;
    }
   
    if(!found)
    {
        #define DUAL_CDK_VENDOR_ID  0x1d6bU
        #define DUAL_CDK_PRODUCT_ID 0x0102U
        device = DevicePtr(new USBDevice(DUAL_CDK_VENDOR_ID, DUAL_CDK_PRODUCT_ID, "", -1, "", ""));
        found = true;
        autoStart = false;
    }
    if(!found)
    {
        logger(LOG_DEBUG) <<  "Can't find TOF  device Load File  " << vxlFile << std::endl;
        PointCloud::FrameStreamReaderPtr  frameStream = new PointCloud::FrameStreamReader(vxlFile);
        if(!frameStream->isStreamGood())
            return -1;
        
        uint32_t width,height;
        TofConfSetting confSetting;
        frameStream->getParameters(&confSetting);
        width = confSetting.width;
        height = confSetting.height;
        logger(LOG_DEBUG) << " MeasureMode:" << confSetting.measure_mode << " width:" << width << " height:" << height << " outputmode:" << confSetting.output_mode << " mod_f1:" << (int) confSetting.maxFreq << " mod_f2:" << (int) confSetting.minFreq << std::endl;
        uint numFrames = frameStream->size();

#ifdef TEST_VXL
		_vxlDrawThread = ThreadPtr(new Thread(vxlDrawThread));
#endif // TEST_VXL

		//const std::string no_camera_message = "No camera connected, please connect 1 or more";
		//MakeColorTbl();
		//window      app(1280, 960, "Pointcloud.ai Tof Example");
		//glfw_state  app_state;
		//register_glfw_callbacks(app, app_state);
		//int channel_size = 3;

		for (int i = 0; i < numFrames; i++)
		{
			if (!frameStream->readNext())
			{
				printf("Failed to read frame %d\n", i);
				break;
			}

			RawDataFramePtr rawFrame = std::dynamic_pointer_cast<RawDataFrame>(frameStream->frames[FRAME_RAW_FRAME_UNPROCESSED]);
			//PhaAmpFramePtr phaAmpFrame = std::dynamic_pointer_cast<PhaAmpFrame>(frameStream->frames[FRAME_RAW_FRAME_PROCESSED]);
			//Ptr<XYZIPointCloudFrame> xyzFrame = std::dynamic_pointer_cast<XYZIPointCloudFrame>(frameStream->frames[FRAME_XYZI_POINT_CLOUD_FRAME]);
			//uint index = height * width / 2 + width / 2;
			//size_t pha_len = phaAmpFrame->pha_amp.size() / 2;
			//size_t raw_len = rawFrame->data.size();
			//uint16_t* amp = 0;
			//uint16_t* pha = 0;

#ifdef CHECK_PARSER
			//if(ii_iq.size() != pha_len)
			//    ii_iq.resize(pha_len);
			//        
			//if(phase_data.size() != pha_len)
			//    phase_data.resize(pha_len);

			//if(amplitude_data.size() != pha_len)
			//    amplitude_data.resize(pha_len);

			//convertRawDataToIIIQ((uint8_t*)rawFrame->data.data(),ii_iq.data(),width,height);
			//convertIIIQToAmpPha(ii_iq.data(),pha_len,phase_data.data(),amplitude_data.data());
#endif

#ifdef TEST_VXL

#endif // TEST_VXL

			//if (confSetting.output_mode == A_B)
			//{
			//	pha = (uint16_t *)phase_data.data();
			//	amp = (uint16_t *)amplitude_data.data();
			//}
			//else
			//{
			//	uint16_t* rawdata = (uint16_t*)rawFrame->data.data();
			//	pha = (uint16_t *)rawdata;
			//	amp = (uint16_t *)rawdata + pha_len;
			//}
			
			//draw_text(int(std::max(0.f, (app.width() / 2) - no_camera_message.length() * 3)),
			//	int(app.height() / 2), no_camera_message.c_str());

			//int cols = 2;// int(std::ceil(std::sqrt(total_number_of_streams)));
			//int rows = 2;// int(std::ceil(total_number_of_streams / static_cast<float>(cols)));
			//float view_width = (app.width() / cols);
			//float view_height = (app.height() / rows);
			//for (int index = 0; index < g_tofFrameSize.width*g_tofFrameSize.height; index++)
			//{
			//	raw_frame[1].data[index] = (amp[index] * 256) / 4096;
			//	if (amp[index] < 0)
			//	{
			//		raw_frame[0].data[index * 3] = 0;    //B
			//		raw_frame[0].data[index * 3 + 1] = 0;    //G
			//		raw_frame[0].data[index * 3 + 2] = 0;
			//	}
			//	else
			//	{
			//		raw_frame[0].data[index * 3] = Rainbow[0][pha[index]];    //B
			//		raw_frame[0].data[index * 3 + 1] = Rainbow[1][pha[index]];    //G
			//		raw_frame[0].data[index * 3 + 2] = Rainbow[2][pha[index]];    //R
			//	}
			//}
			//render_textures(cols, rows, view_width, view_height, channel_size);
			//draw_pointcloud(app.width(), app.height(), app_state, vertices.data(), (int)vertices.size());
			//std::this_thread::sleep_for(std::chrono::milliseconds(1000));

			//std::cout << " index:"<< i <<  " raw[pha:" <<  pha[index] << " amp:" << amp[index] << "] calib[pha:" << phaAmpFrame->pha_amp[index] << " amp:" << phaAmpFrame->pha_amp[index + pha_len] << "] tsensor:" <<  (int)rawFrame->tsensor << " tillum:" << (int)rawFrame->tillum << " freq:" << rawFrame->id;
   //         std::cout << " x:" <<xyzFrame->points[index].x << " y:" <<xyzFrame->points[index].y << " z:" <<xyzFrame->points[index].z  << std::endl ;
		}
        return -1;
    }
	else
    {
        depthCamera = cameraSys.connect(device);
        if (!depthCamera) {
            logger(LOG_ERROR) << "Could not load depth camera for device "<< device->id() << std::endl;
            return -1;
        }

		CameraType camType = TOF_CAMERA;// TOF_CAMERA; // RGB_CAMERA; DUAL_CAMERA;
        if(camType == DUAL_CAMERA)
            channel_size = 3;
        
        if(!depthCamera->Init())
        {
            logger(LOG_ERROR) << "Could not load Init Depth Camera "<< device->id() << std::endl;
            return -1;
        }
        
        if (!depthCamera->isInitialized()) {
            logger(LOG_ERROR) << "Depth camera not initialized for device "<< device->id() << std::endl;
            return -1;
        }

		depthCamera->controlOneTakeEightBoard(true);
		std::this_thread::sleep_for(std::chrono::milliseconds(200));

		String name = depthCamera->configFile.getConfigFileName();
		std::cout << "config file: " << name << std::endl;

        if(camType == DUAL_CAMERA || camType == RGB_CAMERA)
        {
            depthCamera->registerCallback(DepthCamera::FRAME_RGB_FRAME,rgbCallback);
        }
        
        if(camType == DUAL_CAMERA || camType == TOF_CAMERA)
        {
            // depthCamera->registerPowerSupplierChangedCallback(onPowerChangedHandle);
            int output_mode = 0;
            depthCamera->get(OUTPUT_MODE, output_mode);
            g_output_mode = (OutputMode)output_mode;
            if(g_output_mode != 0)
            { //a_b
                depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_UNPROCESSED,unprocCallback);
            }else
            {//amp_pha
                depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_UNPROCESSED,unprocCallback);
                depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_PROCESSED,rawdataCallback);
                depthCamera->registerCallback(DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME,pointcloudCallback);
                depthCamera->registerCallback(DepthCamera::FRAME_VERTICAL_FRAME,verticalPhaseCallback);
            }
                        
            //FrameRate r;
			//r.numerator = 25;
			//r.denominator = 1;
            //bool res = depthCamera->setFrameRate(r);
            //if(!res)
            //   std::cout << "Failed to set frame rate:" <<  r.numerator << std::endl;

            //depthCamera->set("micr_lnum",160);
            //depthCamera->getFrameSize(g_tofFrameSize);
            //float range,near_distance;
            //depthCamera->get(UNAMBIGUOUS_RANGE, range);
            //depthCamera->get(NEAR_DISTANCE,near_distance);
            //depthCamera->saveFrameStream(VXL_FILE);
            //float value = 0.0;
            //depthCamera->getStreamParam("cx",value);
            //std::cout << " cx:" << value << std::endl;
            
            /*
             FrameSize size;
             size.width = 80;
             size.height = 60;
             depthCamera->setFrameSize(size);
             int temp;
             depthCamera->get("tsensor",temp);
             binning_mod = 0;
             depthCamera->set("binning_mode",binning_mod);
             bool v = 1;
             depthCamera->set("img_orientation_v",v);
             bool h = 1;
             depthCamera->set("img_orientation_h",h);
             scale = pow(2,binning_mod);
             roi.x = 40;
             roi.y = 30;
             roi.width = 400;
             roi.height = 300;
             depthCamera->setROI(roi);*/
        }
		
        //if (!depthCamera->start(camType))
        //{
        //    logger(LOG_ERROR) << "Could not start the depth camera "<< depthCamera->id() << std::endl;
        //    //return -1;
        //}
        //depthCamera->saveFrameStream(VXL_FILE);
    }

	uint16_t connectedChannel = 0;
	if (isWithOneTakeEight)
	{
		int retry_cnt = 5;
		depthCamera->getChannelStatus(1);
		uint16_t chnSta = 0xff00;
		while (chnSta==0xff00 && retry_cnt--)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			chnSta = depthCamera->getChannelStatus(0);
		}
		depthCamera->checkOneTakeEightBoardShortCircuit();
		connectedChannel = chnSta;
	}
	if (isSelectChnnl)
	{
		if (1<<(curr_chnnl - 1) & connectedChannel)
		{
			depthCamera->changeToChannelX(curr_chnnl);
		}
		else
		{
			logger(LOG_ERROR) << "input error channel id, module disconnected, auto select connected channel: " << std::endl;
			for (int i = 1; i <= 8; i++)
			{
				if ((1 << (i - 1)) & connectedChannel)
				{
					if (depthCamera->changeToChannelX(i))
					{
						std::this_thread::sleep_for(std::chrono::milliseconds(1000));
						break;
					}
					//depthCamera->letModuleWorkOn();
					//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				}
			}
		}
	}
	if (isSetIntergralTime)
	{
		depthCamera->setIntegralTime(intgTime);
	}
	if (isSetIntergralTimeScale)
	{
		depthCamera->setIntegralTimeScale((uint8_t)intgTimeScale);
	}
	if (isTransportFileToModule) //-r -t -f C:/WorkSpace/ServerProject/tof_calib_06_08_05150c1900000001.conf -n /customer/pointcloud-1.1.1/conf/tof_calib_06_08_05150c1900000001.conf 
	{
		depthCamera->saveConfToModule(fileOriginLocation, fileDestinationLocation);
		int retry_cnt = 5;
		int ret = 12;
		while (ret == 12 && retry_cnt--)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			ret = depthCamera->getUploadState();
		}
		//if (ret == 5 || ret == 7)//write bcs or qt conf file ok
		//{
		//	if (isReadConfFromModule)
		//	{
		//		depthCamera->readConfFromModule();
		//		String conf_name = "";
		//		retry_cnt = 5;
		//		conf_name = depthCamera->getReadConfFromModuleState();
		//		while (conf_name == "" && retry_cnt--)
		//		{
		//			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		//			conf_name = depthCamera->getReadConfFromModuleState();
		//		}
		//	}
		//}
		depthCamera->controlOneTakeEightBoard(false);
		if (depthCamera->isSavingFrameStream())
			depthCamera->closeFrameStream();
		depthCamera->stop();
		depthCamera->UnInit();
		cameraSys.disconnect(depthCamera, true);
		return 0;
	}
	if (isGetAllID)
	{
		depthCamera->getFull_SN();
		depthCamera->getBatchNo();
		depthCamera->getSensorID();
	}
	if (isReadConfFromModule)
	{
		depthCamera->readConfFromModule();
		String conf_name = "";
		int retry_cnt = 5;
		conf_name = depthCamera->getReadConfFromModuleState();
		while (conf_name == "" && retry_cnt--)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			conf_name = depthCamera->getReadConfFromModuleState();
		}
	}
	if (isDetectHotPlug)
	{
		_detectHotPlugThread = ThreadPtr(new Thread(detectHotPlugThread));
	}
	if (isRunLoop)
	{
		_loopRunThread = ThreadPtr(new Thread(loopRunThread));
	}
	//else
	{
		FrameRate r;
		r.numerator = 25;
		r.denominator = 1;
		bool res = depthCamera->setFrameRate(r);
		if (!res)
			std::cout << "Failed to set frame rate:" << r.numerator << std::endl;
		depthCamera->getFrameSize(g_tofFrameSize);
		if (!depthCamera->start(TOF_CAMERA))
		{
			logger(LOG_ERROR) << "Could not start the depth camera " << depthCamera->id() << std::endl;
			//return -1;
		}
        //depthCamera->saveFrameStream(VXL_FILE);
	}
	//_captureThread = ThreadPtr(new Thread(ThreadWrapper));
	//_hotplugThread = ThreadPtr(new Thread(ThreadHotplug));
	//_testThread = ThreadPtr(new Thread(ThreadTest));

	const std::string no_camera_message = "No camera connected, please connect 1 or more";
	MakeColorTbl();
	window      app(1280, 960, "Pointcloud.ai Tof Example");
	glfw_state  app_state;
	register_glfw_callbacks(app, app_state);
	// Create a simple OpenGL window for rendering:
	 //int cnt = 10;
  //   while (cnt --) // Application still alive?
    while (app)
    {
        if(!found)
            draw_text(int(std::max(0.f, (app.width() / 2) - no_camera_message.length() * 3)),
                  int(app.height() / 2), no_camera_message.c_str());
        
        //int total_number_of_streams = 4;
        int cols =2;// int(std::ceil(std::sqrt(total_number_of_streams)));
        int rows =2;// int(std::ceil(total_number_of_streams / static_cast<float>(cols)));
        float view_width = (app.width() / cols);
        float view_height = (app.height() / rows);
        for(int index=0;index < g_tofFrameSize.width*g_tofFrameSize.height;index++)
        {
             raw_frame[1].data[index] = (amplitude_data[index]*256)/4096;
             if(amplitude_data[index] < 0)
             {
                 raw_frame[0].data[index*3] = 0;    //B
                 raw_frame[0].data[index*3 + 1] = 0;    //G
                 raw_frame[0].data[index*3 +2] = 0;
             }else
             {
                 raw_frame[0].data[index*3]     = Rainbow[0][phase_data[index]];    //B
                 raw_frame[0].data[index*3 + 1] = Rainbow[1][phase_data[index]];    //G
                 raw_frame[0].data[index*3 +2]  = Rainbow[2][phase_data[index]];    //R
             }
            
            vertices[index].x = xyz_data[index].x;
            vertices[index].y = xyz_data[index].y;
            vertices[index].z = xyz_data[index].z;
        }
        render_textures(cols, rows, view_width, view_height,channel_size);
        //app_state.tex.upload(&raw_frame[0],640,480);
        // Draw the pointcloud
        //draw_pointcloud(app.width(), app.height(), app_state, xyz_data);
        draw_pointcloud(app.width(), app.height(), app_state, vertices.data(),(int)vertices.size());
        /*
        static int total = 0;
        total++;
        if(total%100 == 0 && depthCamera && (camType == DUAL_CAMERA || camType == TOF_CAMERA))
        {
            int tillum, tsensor;
            depthCamera->get(TILLUM, tillum,true);
            depthCamera->get(TSENSOR,tsensor,true);
            std::cout << " tillum:" << std::dec << tillum << " tsensor:" << std::dec << tsensor << std::endl;
        }*/
    }

    if(depthCamera->isSavingFrameStream())
        depthCamera->closeFrameStream();
    depthCamera->stop();
	depthCamera->controlOneTakeEightBoard(false);
	depthCamera->UnInit();
    cameraSys.disconnect(depthCamera,true);

  return 0;
}
