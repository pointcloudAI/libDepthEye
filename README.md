# Overview

libDepthEye SDK  is a cross-platform library for PointCloud.AI® DepthEye™ depth cameras.

Developer kits containing the necessary hardware to use this library are available for purchase at 
[pointcloud.ai/buy](http://pointcloud.ai/buy) . 

## Supported TOF chip
### Sony IMX556
The IMX556 is a fully integrated optical Time-of-Flight (ToF) camera sensor. The sensor features 640 x 480 (VGA) time-of-flight pixels based on the DepthSense® pixel technology. Combined with a modulated light source, this sensor is capable of measuring distance and reflectivity with VGA resolution. 
#### Functions and Features
* Back-illuminated Time-of-flight image sensor ◆High signal to noise ratio (SNR)
* Full resolution @max60 frame/s (4phase/frame) ◆Pixel binning readout function
* Independent flipping and mirroring
* Built-in temperature sensor
* Unit cell size : 10.0 um (H) × 10.0 um (V)
* Operation temperature : -30 to +75  ̊C
* Performance guarantee temperature : -10 to +60  ̊C

## What’s included in the SDK:

|What | Description |  
|- | :-: | 
|[Dependent library files](https://github.com/pointcloudAI/libDepthEye/tree/master/libs) |We provide SDK for those platforms : Linux \ Mac OS \ Windows .|
|[Code Samples](https://github.com/pointcloudAI/libDepthEye/tree/master/examples) | These simple examples demonstrate how to easily use the SDK to include code snippets that access the camera into your applications. Check some of the C++ examples including capture, pointcloud parsing.
|Tools | [DepthEyeViewer](http://www.pointcloud.ai/doc/DepthEyeViewer_1.3.2_210807.exe.zip) is a visualization tool which is based on the libDepthEye SDK API. |
|Wrappers | [Python](https://github.com/pointcloudAI/libDepthEye/tree/master/wrappers/python) API, ROS, OpenCV and more to come |

## Visualization tool Snapshot

![PointCloudTool is a visualization tool ](https://raw.githubusercontent.com/pointcloudAI/libDepthEye/master/doc/image/snapshotMain.jpg) 


## Getting started


### 0) Hardware Install

Please add one usb hub with Independent power supply. Then insert the double head USB cable together on the USB hub and connect the usb hub to your PC.


### 1) SET POINTCLOUD_SDK_PATH

To install vim and cmake, please run:

`sudo apt-get install cmake vim `

Please run `# uname -a` to check system platform version first.

|Plateform | SDK Path |
|- | :-: | 
|MacOs |/libs/macos|
|Ubuntu | /libs/ubuntu|
|Ubuntu arm64 | /libs/ubuntu_arm|
|Windows | /libs/windows|

Modify .bashrc to set environment variables：

`# vim ~/.bashrc`

add below source code to the end of bashrc file：
```
export POINTCLOUD_SDK_PATH="your_directory/libs/ubuntu"
export PATH=$POINTCLOUD_SDK_PATH/lib:$POINTCLOUD_SDK_PATH/bin:$PATH
```

PS：Please remember to replace ubuntu with your system platform.

We need to make above changes come into effect：
 
`# source ~/.bashrc`

Finally, echo the constant to verify ：

`# echo $POINTCLOUD_SDK_PATH`

### 2) SET USB Driver
(For Ubuntu only)
```
./config/setupDriverOnLinux.sh
```

### 3) Make

In the root directory of SDK, run below commands one by one：

```
$ mkdir build

$ cd build/ 

$ cmake ..

$ make 
```

You can get message as below : 

`[100%] Built target DepthMeasure`

Plug the module and run it:

`./bin/DepthMeasure`

You can see the result!



## Wiki

Learn more to use libDepthEye at our [wiki](https://github.com/pointcloudAI/libDepthEye/wiki).

## Others

Please send mail to sdk@pointcloud.ai. We will reply you asap.

Check or raise issue on [Issue tracking] (https://github.com/pointcloudAI/libDepthEye/issues)
