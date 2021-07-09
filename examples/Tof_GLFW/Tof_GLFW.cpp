/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */


#include "CameraSystem.h"
#include "Timer.h"
#include "Common.h"
#include "Logger.h"
#include <iomanip>
#include <fstream>
#include <math.h>
//#include "framestream.h"
#define GLFW_INCLUDE_GLU
#define GL_SILENCE_DEPRECATION
#include "GLFW/glfw3.h"
#include "glfw_window.h"
using namespace PointCloud;

#define MAX_FRAME_WITDH 640
#define MAX_FRAME_HEIGHT 480

#define RGB_FRAME_WITDH 1280
#define RGB_FRAME_HEIGHT 720
#define SPEED_OF_LIGHT 299792458.0
#define VXL_FILE "/Users/hejie/Documents/PointcloudAI/workspace/test.vxl"
int smin = 0;
int smax = 0xfff;//0x7ff;
uint8_t Rainbow[3][65536] = {{0}};

#define TEST_IDX 240*640 + 320

void onPowerChangedHandle(DepthCamera &dc,int isOn12V){

   std::cout << " --------  onPowerChangedHandle:  ------- " << isOn12V  << std::endl;
    /*
   if (isOn12V == 1){
       dc.setPowerLevel(1);
   }else{
       dc.setPowerLevel(7);
   }*/

}
void MakeColorTbl()
{
   // int imMax = smax;
   // int srange = 1020*3;//*4;
   // double dsrange = (double)srange;
    //for(int i = 0; i < imMax * 2 - 2; i++){
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

void MakeDepthColor(uint8_t * depth, uint8_t* depthView, int sizeH, int sizeV)
{
    unsigned short *pDepth;
    pDepth = (unsigned short *)depth;
    unsigned short dat;
    for(int j = 0; j < sizeV; j++){
        for(int i = 0; i < sizeH; i++){
            dat = pDepth[j*sizeH+i];
            depthView[j*sizeH*3+i*3+0] = Rainbow[0][dat];    //B
            depthView[j*sizeH*3+i*3+1] = Rainbow[1][dat];    //G
            depthView[j*sizeH*3+i*3+2] = Rainbow[2][dat];    //R
        }
    }
}

Mutex _dataAccessMutex;
ConditionVariable _dataAvailableCondition;
FrameSize frameSize;
RegionOfInterest roi;
uint binning_mod = 0;
uint scale_binning = 1;
std::vector<uint16_t> amplitude_data,phase_data;
std::vector<IntensityPoint> xyz_data;
Map<uint,std::vector<uint16_t> > map_temp_phase;
char rawFrameQueue[42496 * 2];

PointCloud::PointCloudFramePtr ptFramePtr ;
texture view_tex[4];
RawDataFrame a_b_frame[4];
std::vector<vertex> vertices;

TimeStampType s1,e1;
void rgbCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c)
{
    Timer t;
    Lock<Mutex> _(_dataAccessMutex);
    const RawDataFrame *d = dynamic_cast<const RawDataFrame *>(&frame);
    int length = d->data.size();
    if(a_b_frame[2].data.size() < length)
    {
        std::cout << "RGB Frame: buffer length:" << a_b_frame[2].data.size() << " receive length:" << length << std::endl;
        return ;
    }
    memcpy(a_b_frame[2].data.data(),d->data.data(),length);
    static int count = 0;
    count++;
    if(count%30 == 0)
    {
        e1 =  t.getCurrentRealTime();
        float dur = (e1 - s1);
        std::cout << "RGB Frame:" << d->data.size()  << " time:" <<  dur  << "ms fps:"  << 30*1e6/dur << std::endl;
        s1 = e1;
    }
}

void pointcloudCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c)
{
    Timer t;
    static TimeStampType s=0,e=0;
    //Lock<Mutex> _(_dataAccessMutex);
    const XYZIPointCloudFrame *d = dynamic_cast<const XYZIPointCloudFrame *>(&frame);
   // uint step_i = MAX_FRAME_HEIGHT/frameSize.height;
   // uint step_j = MAX_FRAME_WITDH/frameSize.width;
   // uint x_offset = roi.x;
   // uint y_offset = roi.y;
    
    for(int i = 0;i < 480;i++ )
    {
        for(int j = 0;j < 640;j++ )
        {
            uint index = i*640 +j;
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
    if(count2%30 == 0)
    {
        e = t.getCurrentRealTime();
        float dur = (e - s);
        std::cout << " pointcloudCallback: center point phase  time:" << dur  << "ms fps:"  << (float)30*1e6/dur << " timestamp:" << d->timestamp<< std::endl;
        s = e;
    }
}

uint avg_phase = 0;
void unprocCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c)
{
    
    Lock<Mutex> _(_dataAccessMutex);
    //const RawDataFrame* rawDataFrame = dynamic_cast<const RawDataFrame*>(&frame);
    //size_t raw_len = rawDataFrame->data.size();
    //int16_t *data = (int16_t *)rawDataFrame->data.data();
    /*
    double total = 0.0;
    float total_rate = 0.0;
    uint16_t max_phase = 0;
    uint16_t max_amp = 0;
    */
    
    //size_t length = frameSize.width*frameSize.height;
    /*
    if(raw_len == length*2*sizeof(int16_t))
    {
        for(int i=0;i<frameSize.width*frameSize.height;)
        {
            ii_data[i] = data[i];
            iq_data[i] = data[i + length];
            i++;
        }
    }else if(raw_len == length*4*sizeof(int16_t))
    {
        uint index = 320*640 +240;
     
        memcpy((int8_t*)a_b_frame[0].data.data(),rawDataFrame->data.data(),length*2);
        memcpy((int8_t*)a_b_frame[1].data.data(),rawDataFrame->data.data()+2*length,length*2);
        memcpy((int8_t*)a_b_frame[2].data.data(),rawDataFrame->data.data()+4*length,length*2);
        memcpy((int8_t*)a_b_frame[3].data.data(),rawDataFrame->data.data()+6*length,length*2);
        //std::cout << " A_B  center point 0 value:" << (int)a_b_data[0][index] << " 90 Value:" <<  (int)a_b_data[1][index]  << " 180 value:" << (int)a_b_data[2][index] << " 270 Value:" <<  (int)a_b_data[3][index] << std::endl;
    }else
    {
        std::cout << "Error Data length =" << raw_len << std::endl;
    }*/
    /*
    if( total > 0 && total_rate > 0)
    {
        avg_phase = total/total_rate;
        //uint index = frameSize.height/2*frameSize.width + frameSize.width/2;
        //std::cout << " unprocCallback avg phase:" << avg_phase << " center point phase:" << data[2*index +1] << " amp:" <<  data[2*index] << " max_phase:" << max_phase  << " max_amp:" << max_amp<< std::endl;
    }*/
}


void rawdataCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c)
{
    Timer t;
    static TimeStampType s=0,e=0;
   // Lock<Mutex> _(_dataAccessMutex);
    const ToFRawFrame *d = dynamic_cast<const ToFRawFrame *>(&frame);
    
    uint16_t* amp = (uint16_t *)d->amplitude();
    uint16_t* ph = (uint16_t *)d->phase();
    //uint x_offset = roi.x;
    //uint y_offset = roi.y;
    for(int i = 0;i < 480;i++ )
    {
        for(int j = 0;j < 640;j++ )
        {
            uint index = i*640 +j;
            amplitude_data[index] = amp[index];
            phase_data[index] = ph[index];
        }
    }
    uint index = 640*240+320;//frameSize.height/2*frameSize.width + frameSize.width/2;
    static int count1 = 0;
    count1++;
    if(count1%30 == 0)
    {
        e = t.getCurrentRealTime();
        float dur = (e - s);
        std::cout << " rawdataCallback: center point phase:"<< std::dec << phase_data[index] << " amp:" << std::dec <<  amplitude_data[index] << " time:" << dur  << "ms fps:"  << (float)30*1e6/dur << std::endl;
        s = e;
    }
}

PointCloud::CameraSystem      cameraSys;
PointCloud::DepthCameraPtr depthCamera;

PointCloud::DevicePtr       device;
ThreadPtr _captureThread = 0;
uint mode = 0;
void ThreadWrapper()
{
    while (true) {
        if(depthCamera->isRunning())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10000));
            std::cout << "disconnect>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << mode << std::endl;
            
            depthCamera->stop();
            //cameraSys.disconnect(depthCamera);
        }
        else{
            
            std::cout << "connect<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
            /*
            depthCamera = cameraSys.connect(device);
            if (!depthCamera->isInitialized()) {
                logger(LOG_ERROR) << "Depth camera not initialized for device "<< device->id() << std::endl;
                return ;
            }
            //depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_UNPROCESSED,unprocCallback);
            depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_PROCESSED,rawdataCallback);
            //depthCamera->registerCallback(DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME,pointcloudCallback);
            mode++;
            mode = mode%3;
            depthCamera->set("measure_mode",mode);
            */
            /*
            mode++;
            uint binning = mode%3 + 1;
            depthCamera->set("binning_mode",binning);
            
            depthCamera->getFrameSize(frameSize);
            scale_binning = 640/frameSize.width;*/
            
            
            static int i = 5;
            i++;
            FrameRate r;
            r.numerator = i%30 < 20? 20:i%30;//7;
             depthCamera->setFrameRate(r);
            depthCamera->start();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            
        }
        
    }
    
}

void render_textures(int cols, int rows, float view_width, float view_height,int channel_size)
{
    Lock<Mutex> _(_dataAccessMutex);
    for(int stream_no = 0;stream_no< channel_size;stream_no++)
    {
        rect frame_location{ view_width * (stream_no % cols), view_height * (1 - stream_no / rows), view_width, view_height };
        int width = MAX_FRAME_WITDH;
        int height = MAX_FRAME_HEIGHT;
        if(stream_no == 2)
        {
            width = RGB_FRAME_WITDH;
            height = RGB_FRAME_HEIGHT;
        }
        view_tex[stream_no].render(&a_b_frame[stream_no],width,height, frame_location);
    }
}

void draw_pointcloud1(float width, float height, glfw_state& app_state, const vertex* vertices,int vertices_count)
{
    //  if (!points)
    //       return;
    
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

int main(int argc, char *argv[])
{
    /*
    int16_t d  = -2;
    printf("0x%04x\n",(uint16_t)d);
    uint32_t  val = 0x26200000;
    int8_t a = val >> 16;
    int b = a;
    
    a = val >> 24;
    b = a;
    time_t t;
    t=time(NULL) ;//- 1614778538;
    
    time_t lt1 = t;//id + 1614778538;
    struct tm *newtime=localtime(&lt1);
    char tmpbuf[128];
    strftime( tmpbuf, 128, "Today is %A, day %d of %B in the year %Y.\n", newtime);
    printf("to test %s , %d_%d_%d_%d%d%d\n",tmpbuf,newtime->tm_year,newtime->tm_mon,newtime->tm_mday,newtime->tm_hour,newtime->tm_min,newtime->tm_sec);
    */
    logger.setDefaultLogLevel(LOG_DEBUG);
    // Create a simple OpenGL window for rendering:
    window app(1280, 960, "Pointcloud.ai Tof Example");
    glfw_state app_state;
    register_glfw_callbacks(app, app_state);
    phase_data.resize(MAX_FRAME_WITDH*MAX_FRAME_HEIGHT);
    amplitude_data.resize(MAX_FRAME_WITDH*MAX_FRAME_HEIGHT);
    
    xyz_data.resize(MAX_FRAME_WITDH*MAX_FRAME_HEIGHT);
    
    size_t buffer_size = MAX_FRAME_WITDH*MAX_FRAME_HEIGHT;
    vertices.resize(buffer_size);
    a_b_frame[0].data.resize(buffer_size*3 );
    a_b_frame[0].id = FORMAT_RGB8;
    a_b_frame[1].data.resize(buffer_size );
    a_b_frame[1].id = FORMAT_Y8;
    a_b_frame[2].data.resize(RGB_FRAME_WITDH*RGB_FRAME_HEIGHT*3 );
    a_b_frame[2].id = FORMAT_RGB8;
    a_b_frame[3].data.resize(buffer_size*3 );
    a_b_frame[3].id = FORMAT_RGB8;
    int channel_size = 2;
    MakeColorTbl();
    logger.setDefaultLogLevel(LOG_DEBUG);
  
    const Vector<DevicePtr> &devices = cameraSys.scan();
    bool found = false;
    int output_mode = 0;
    for (auto &d: devices){
        logger(LOG_INFO) <<  "Detected devices: "  << d->id() << std::endl;
        device = d;
        found = true;
    }
    //found = false;
    if(!found)
    {
        logger(LOG_ERROR) <<  "Can't find TOF  device Load File:" << VXL_FILE << std::endl;
        
        PointCloud::FrameStreamReaderPtr  frameStream = new PointCloud::FrameStreamReader(VXL_FILE, cameraSys);
        if(!frameStream->isStreamGood())
            return -1;
        uint numFrames = frameStream->size();
        for(int i=0;i<numFrames;i++)
        {
            if(!frameStream->readNext())
            {
                printf("Failed to read frame %d",i);
                break;
            }
            RawDataFramePtr rawFrame = std::dynamic_pointer_cast<RawDataFrame>(frameStream->frames[DepthCamera::FRAME_RAW_FRAME_UNPROCESSED]);
            
            uint index = 240*640+320;
            
            size_t raw_len = rawFrame->data.size();
            uint16_t* rawdata = (uint16_t*) rawFrame->data.data();
            std::cout << " index:"<< i << " phase:" << rawdata[2*index+1] << " amplitude:" << rawdata[2*index] << " tsensor:" <<  (int)rawFrame->data[raw_len - 2] << " tillum:" << (int)rawFrame->data[raw_len - 1] << std::endl;
            //Ptr<XYZIPointCloudFrame> xyzFrame = std::dynamic_pointer_cast<XYZIPointCloudFrame>(frameStream->frames[DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME]);
            //std::cout << " index:" << i << " size:" << rawFrame->size.height << "," <<rawFrame->size.width  << " x:" <<xyzFrame->points[120*320+160].x << " y:" <<xyzFrame->points[120*320+160].y << " z:" <<xyzFrame->points[120*320+160].z  << std::endl ;
        }
        //return -1;
    
    }else
    {
        depthCamera = cameraSys.connect(device);
        if (!depthCamera) {
            logger(LOG_ERROR) << "Could not load depth camera for device "<< device->id() << std::endl;
            return -1;
        }
        uint id = depthCamera->getCurrentCameraProfileID();
        CameraType camType = TOF_CAMERA;//RGB_CAMERA;//TOF_CAMERA;//DUAL_CAMERA;
        
        if(camType == DUAL_CAMERA)
            channel_size = 3;
        
        if(!depthCamera->Init(camType))
            logger(LOG_ERROR) << "Could not load Init Depth Camera "<< device->id() << std::endl;
        
        if (!depthCamera->isInitialized()) {
            logger(LOG_ERROR) << "Depth camera not initialized for device "<< device->id() << std::endl;
            return -1;
        }
    
        if(camType == DUAL_CAMERA || camType == RGB_CAMERA)
        {
            depthCamera->registerCallback(DepthCamera::FRAME_RGB_FRAME,rgbCallback);
        }
        
        if(camType == DUAL_CAMERA || camType == TOF_CAMERA)
        {
            FrameRate r1;
            depthCamera->getFrameRate(r1);
            // depthCamera->registerPowerSupplierChangedCallback(onPowerChangedHandle);
            //output_mode = AMP_PHA;
            // depthCamera->set(OUTPUT_MODE, output_mode);
            depthCamera->get(OUTPUT_MODE, output_mode);
            if(output_mode != 0)
            {
                depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_UNPROCESSED,unprocCallback);
            }else{
                
                //depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_UNPROCESSED,unprocCallback);
                depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_PROCESSED,rawdataCallback);
                depthCamera->registerCallback(DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME,pointcloudCallback);
            }
            
            uint32_t integrationTime = 50;
            depthCamera->set(INTG_TIME, integrationTime);
            /*
            int32_t measure_mode = USE_MOD_F2;
            depthCamera->set(MEASURE_MODE,measure_mode);*/
    
            FrameRate r;
            r.numerator = 24;
            bool res = depthCamera->setFrameRate(r);
            if(!res)
                std::cout << "Failed to set frame rate:" <<  r.numerator << std::endl;
            
            //float range,near_distance;
            //depthCamera->get(UNAMBIGUOUS_RANGE, range);
            //depthCamera->get(NEAR_DISTANCE,near_distance);
            
            //depthCamera->saveFrameStream(VXL_FILE);
            //float value = 0.0;
            //depthCamera->getStreamParam("cx",value);
            //std::cout << " cx:" << value << std::endl;
        }
        //uint val = 3;
        //depthCamera->set("binning_mode",val);
        
        //depthCamera->set("intg_time",val);
        //int abc = 30;
        //depthCamera->set(MOD_F,abc);
        /*
        FrameSize size;
        size.width = 80;
        size.height = 60;
        depthCamera->setFrameSize(size);
        int temp;
        depthCamera->get("tsensor",temp);
        */
        /*
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
        
        if (!depthCamera->start(camType))
        {
            logger(LOG_ERROR) << "Could not start the depth camera "<< depthCamera->id() << std::endl;
            //return -1;
        }
    }
    
    for(int x = 0; x <640;x++)
    {
        for(int y = 0; y< 480;y++)
        {
            int idx = y*640 + x;
            a_b_frame[0].data[idx*3] = x%256;
            a_b_frame[0].data[idx*3 + 1] = y%256;
            a_b_frame[0].data[idx*3 + 2] = 128;
            a_b_frame[1].data[idx] = idx%256;
            a_b_frame[2].data[idx*3] =y%256;
            a_b_frame[2].data[idx*3 + 1] = 128;
            a_b_frame[2].data[idx*3 + 2] = x%256;

            vertices[idx].x = -1.0 + 2.0/640*x;
            vertices[idx].y = -0.5 + 1.0/480*y;
            vertices[idx].z = 0.50;
        }
    }
    const std::string no_camera_message = "No camera connected, please connect 1 or more";
    //int total_number_of_streams = 4;
    //_captureThread = ThreadPtr(new Thread(ThreadWrapper));
    
    while (app) // Application still alive?
    {
        if(!found)
            draw_text(int(std::max(0.f, (app.width() / 2) - no_camera_message.length() * 3)),
                  int(app.height() / 2), no_camera_message.c_str());
        
        int cols =2;// int(std::ceil(std::sqrt(total_number_of_streams)));
        int rows =2;// int(std::ceil(total_number_of_streams / static_cast<float>(cols)));
        float view_width = (app.width() / cols);
        float view_height = (app.height() / rows);
        for(int index=0;index < MAX_FRAME_WITDH*MAX_FRAME_HEIGHT;index++)
        {
             a_b_frame[1].data[index] = (amplitude_data[index]*256)/4096;
             if(amplitude_data[index] < 0)
             {
                 a_b_frame[0].data[index*3] = 0;    //B
                 a_b_frame[0].data[index*3 + 1] = 0;    //G
                 a_b_frame[0].data[index*3 +2] = 0;
             }else
             {
                 a_b_frame[0].data[index*3]     = Rainbow[0][phase_data[index]];    //B
                 a_b_frame[0].data[index*3 + 1] = Rainbow[1][phase_data[index]];    //G
                 a_b_frame[0].data[index*3 +2]  = Rainbow[2][phase_data[index]];    //R
             }
            
            vertices[index].x = xyz_data[index].x;
            vertices[index].y = xyz_data[index].y;
            vertices[index].z = xyz_data[index].z;
        }
        render_textures(cols, rows, view_width, view_height,channel_size);
        //app_state.tex.upload(&a_b_frame[0],640,480);
        // Draw the pointcloud
        //draw_pointcloud(app.width(), app.height(), app_state, xyz_data);
       
        draw_pointcloud1(app.width(), app.height(), app_state, vertices.data(),(int)vertices.size());
        static int total = 0;
        
        total++;
        if(total%100 == 0 && depthCamera)
        {
            int tillum, tsensor;
            depthCamera->get(TILLUM, tillum,true);
            depthCamera->get(TSENSOR,tsensor,true);
            std::cout << " tillum:" << std::dec << tillum << " tsensor:" << std::dec << tsensor << std::endl;
        }
    }
    
    if(depthCamera->isSavingFrameStream())
        depthCamera->closeFrameStream();
    depthCamera->stop();
    depthCamera->UnInit();
    
    cameraSys.disconnect(depthCamera,true);
    ///////////////////////

  return 0;
}
