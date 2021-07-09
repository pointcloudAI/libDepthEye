/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_TI_POINTCLOUD_UVC_PROGRAMMER_H
#define POINTCLOUD_TI_POINTCLOUD_UVC_PROGRAMMER_H

#include "UVC.h"
#include "GeneralProgrammerBase.h"
#define GRP_PARAM_HOLD "grp_param_hold"
#define EXTERNEL_FREQ_CLOCK 8
#define SYSTEM_FREQ_CLOCK 120
#define MAX_FRAME_WITDH 640
#define MAX_FRAME_HEIGHT 480
#define MAX_FPS 30

#define DEALIAS_EN "dealias_en"
//if hdr_en is true , delalias_en will be set false
#define HDR_EN "hdr_en"
#define MOD_F1 "mod_freq1"
#define MOD_F2 "mod_freq2"

#define SOFTWARE_RESET "power_supply"

#define STANDBY "standby"
#define MODE_SEL "mode_sel"


#define Y_ADD_STA "y_add_sta"
#define Y_ADD_END "y_add_end"
#define HMAX "hmax"
#define DPTH_SU_NUM_WAIT_A "dpth_su_num_wait_a"
#define SUB_FNUM "sub_fnum"
#define DPTH_SU_NUM "dpth_su_num"
#define SUB_RSEQ_LNUM "sub_rseq_lnum"
#define SUB_VBLK1 "sub_vblk1"
#define MICR_LNUM "micr_lnum"

#define INTG_DUTY_CYCLE "intg_duty_cycle"
#define AMP_THREHOLD "amp_threhold"
#define INTG_SCALE "intg_scale"
#define INTG_TIME "intg_time"  // Integration time
#define EXAREA_INTG00 "exarea_intg_00"
/* COMMAND CODE */
#define IMX556_I2C_ADDRESS  0x57
#define MOD_FREQ_CMD 0x68
#define IMX556_CTRL_CMD 0x69
#define DEALIAS_CMD 0x70
#define POWER_LEVEL_CMD 0x70
#define IMX556_RESET_CMD 0x71
#define HDR_CMD 0x72
#define DEPTHEYE_DEV_CMD 0x73

#define CURRENT_PROFILE "scratch"
namespace PointCloud
{
  
namespace TOF
{

class TOFCAM_EXPORT GeneralUVCProgrammer: public GeneralProgrammerBase
{
public:
  struct RequestParams
  {
    uint8_t readRequestCode, writeRequestCode;
    uint8_t leftShiftBits;
  };
  
  typedef Map<uint, RequestParams> SlaveAddressToRequestParamsMap;
protected:
  UVC  _uvc;
  bool _isBigEnd;
  SlaveAddressToRequestParamsMap _slaveAddressToRequestParamsMap;

  virtual bool _readRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t &value, uint8_t length) const;
  virtual bool _writeRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t value, uint8_t length);
private:
    bool getDataFromDevice(uint16_t slaveAddress, uint16_t registerAddress, uint8_t* value, uint8_t length) const;
    bool setDataToDevice(uint16_t slaveAddress, uint16_t registerAddress, uint8_t* value, uint8_t length);
public:
  GeneralUVCProgrammer(const SlaveAddressToByteMap &map, const SlaveAddressToRequestParamsMap &slaveAddressToRequestParamsMap, DevicePtr device,bool isBigEnd = false);
  
  virtual bool isInitialized() const;
  
  virtual bool reset();
  UVC*  getUVC(){return &_uvc;}
  //inline USBIOPtr &getUSBIO() { return _usbIO; }
  
  virtual ~GeneralUVCProgrammer() {}
};

}
}

#endif // POINTCLOUD_TI_VOXELXU_PROGRAMMER_H
