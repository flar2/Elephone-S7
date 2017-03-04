/*****************************************************************************
 *
 * Filename:
 * ---------
 *     T4K37mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
//#include <linux/xlog.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "t4k37mipiraw_Sensor.h"

/****************************Modify following Strings for debug****************************/
#define PFX "t4k37_camera_sensor"
#define LOG_1 LOG_INF("t4k37,MIPI 2LANE\n")
#define LOG_2 LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n")
/****************************   Modify end    *******************************************/
#define LOG_INF(fmt, args...)   pr_debug(PFX "[%s] " fmt, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

#define MIPI_SETTLEDELAY_AUTO     0
#define MIPI_SETTLEDELAY_MANNUAL  1
#define ANALOG_GAIN_1X  (0x0048)


kal_uint16 sensorGlobalGain;//sensor gain read from 0x350a 0x350b;
	kal_uint16 ispBaseGain;//64
	kal_uint16 realGain;//ispBaseGain as 1x

static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = T4K37_SENSOR_ID,
	
	.checksum_value = 0x9e088612,
	
	.pre = {
		.pclk =  211200000,				//record different mode's pclk
		.linelength = 4480,//2368   /1116=	21219		//record different mode's linelength 7024,3294
		.framelength = 1580,//2480			//record different mode's framelength   2560,1312
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2104,		//record different mode's width of grabwindow
		.grabwindow_height = 1560,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk =  211200000,
		.linelength = 4480,
		.framelength = 3144,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 15,
	},
	.cap1 = {							//capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
		.pclk = 259200000,
		.linelength = 5408,
		.framelength = 3144,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 15,	//less than 13M(include 13M),cap1 max framerate is 24fps,16M max framerate is 20fps, 20M max framerate is 15fps  
	},
	.normal_video = {
		.pclk = 211200000,
		.linelength = 4480,
		.framelength = 1572,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 24,//unit , ns
		.max_framerate = 30,
	},
	.hs_video = {
		 .pclk = 211200000,
		.linelength = 4480,
		.framelength = 1572,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 24,//unit , ns
		.max_framerate = 30,
	},
	.slim_video = {
		.pclk = 211200000,
		.linelength = 4480,
		.framelength = 1572,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 24,//unit , ns
		.max_framerate = 30,

	},
	.margin = 6,			//sensor framelength & shutter margin
	.min_shutter = 1,		//min shutter
	.max_frame_length = 0xffff,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 1,
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 3,	  //support sensor mode num ,don't support Slow motion
	
	.cap_delay_frame = 2,		//enter capture delay frame num
	.pre_delay_frame = 2, 		//enter preview delay frame num
	.video_delay_frame = 2,		//enter video delay frame num
	.hs_video_delay_frame = 2,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 2,//enter slim video delay frame num
	
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,//R
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x6c, 0x20, 0xff},
    .i2c_speed = 300, // i2c read/write speed
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x4EA,	//3d0				//current shutter
	.gain = 0x100,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x20,
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =	 
{{ 4208, 3120,	 0,	0, 4208, 3120, 2104,  1560, 0000, 0000, 2104, 1560,	  0,	0, 2096,  1552}, // Preview 
 { 4208, 3120,	 0,	0, 4208, 3120, 4208,  3120, 0000, 0000, 4208, 3120,	  8,	8, 4192,  3104}, // capture 
 { 4208, 3120,	 0,	0, 4208, 3120, 4208,  3120, 0000, 0000, 4208, 3120,	  8,	8, 4192,  3104}, // video 
 { 4208, 3120,	 0, 0, 4208, 3120, 2104,  1560, 0000, 0000, 2104, 1560,	  8,	8, 2096,  1552}, //hight speed video 
 { 4208, 3120,	 0, 0, 4208, 3120, 2104,  1560, 0000, 0000, 2104, 1560,	  8,  8, 2096,  1552},// slim video 
 };// slim video  



static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    //kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
	//iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);
	iReadRegI2CTiming(pu_send_cmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id,imgsensor_info.i2c_speed);	
	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    //kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
	iWriteRegI2CTiming(pu_send_cmd , 3, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);	
}

#if 0
static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iReadRegI2C(pu_send_cmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
    return get_byte;
}


static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}   
#endif
static void set_dummy(void)
{
	//LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	//write_cmos_sensor(0x0104, 1); 
  write_cmos_sensor(0x0340, (imgsensor.frame_length >>8) & 0xFF);
  write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);	
  write_cmos_sensor(0x0342, (imgsensor.line_length >>8) & 0xFF);
  write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);
  //write_cmos_sensor(0x0104, 0);
  
}	/*	set_dummy  */


static kal_uint32 return_sensor_id(void)
{
    //return (((read_cmos_sensor(0x0000)&&0x0f)<<8)  | (read_cmos_sensor(0x0001)) );;
    kal_uint32 sensor_id;
    //sensor_id = (((read_cmos_sensor(0x0000)&&0x0f)<<8)  | (read_cmos_sensor(0x0001)) );
    sensor_id = ((read_cmos_sensor(0x0000)<<8)  | (read_cmos_sensor(0x0001)) );
    LOG_INF("return_sensor_id() id: 0x%x\n", sensor_id);
    return (sensor_id);
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	//kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;
	//LOG_INF("framerate = %d, min framelength should enable? \n", framerate,min_framelength_en);   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length)?frame_length:imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
	//	imgsensor.dummy_line = 0;
	//else
	//	imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */



/*************************************************************************
* FUNCTION
*    set_shutter
*
* DESCRIPTION
*    This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*    iShutter : exposured lines
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    //kal_uint32 frame_length = 0;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
	//shutter = shutter * 14 / 10;
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    //write_shutter(shutter);
    /* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
    /* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

    // OV Recommend Solution
    // if shutter bigger than frame_length, should extend frame length first
    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
    shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    // Framelength should be an even number
   // shutter = (shutter >> 1) << 1;
   // imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
		else {
		// Extend frame length
 		write_cmos_sensor(0x0104, 0x01);
		        write_cmos_sensor(0x0340, (imgsensor.frame_length >> 8) & 0xFF);
                write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
              write_cmos_sensor(0x0104, 0x00); 
            }
    } else {
        // Extend frame length
		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0340, (imgsensor.frame_length >> 8) & 0xFF);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
    }

    // Update Shutter
                write_cmos_sensor(0x0104, 0x01);
                write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
                write_cmos_sensor(0x0203, shutter  & 0xFF);
                write_cmos_sensor(0x0104, 0x00);
             LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}    /*    set_shutter */


/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
/*static kal_uint16 Reg2Gain(const kal_uint8 iReg)
{

    kal_uint16 iGain;
	kal_uint8  BASEGAIN=ANALOG_GAIN_1X;

	iGain=(read_cmos_sensor(0x0204)<<8)|read_cmos_sensor(0x0205);///to fan debug
    iGain=(iGain*64)/BASEGAIN;
	
	return iGain;


}*/

static kal_uint16  Reg2Gain(const kal_uint16 iReg)
{
	kal_uint16    sensorGain=0x0000;	

    sensorGain = iReg/ANALOG_GAIN_1X;  //get sensor gain multiple
    return sensorGain*BASEGAIN; 
}
	
/*static kal_uint16 Gain2Reg(kal_uint16 iGain)
{
    kal_uint16 Gain;
	kal_uint8  BASEGAIN=ANALOG_GAIN_1X;
	//T4K37DB("[T4K37MIPI_SetGain] iGain is :%d \n ",iGain);///to fan debug
	Gain=(iGain*BASEGAIN)/64;
	//T4K37DB("[T4K37MIPI_SetGain] Gain is :%d \n ",Gain);
	//T4K37_write_cmos_sensor(0x0104, 1);      	
   
    write_cmos_sensor(0x0204, (Gain >> 8) & 0x0F);
    write_cmos_sensor(0x0205, Gain  & 0xFF);
	
   // T4K37_write_cmos_sensor(0x0104, 0); 


}*/

static kal_uint16 Gain2Reg(const kal_uint16 Gain)
{
    kal_uint32 iReg = 0x0000;
	//kal_uint16 sensorGain=0x0000;	

	iReg= Gain*ANALOG_GAIN_1X/BASEGAIN;
	
	LOG_INF("Gain2Reg iReg =%d",iReg);
    return (kal_uint16)iReg;
}


/*************************************************************************
* FUNCTION
*    T4K37MIPI_SetGain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    gain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
void write_gain(kal_uint16 gain)
{

//	T4K37_write_cmos_sensor(0x0104,0x0001);// );//-/-/-/-/-/-/-/GROUP_PARA_HOLD 

	write_cmos_sensor(0x0204,(gain>>8));
	write_cmos_sensor(0x0205,(gain&0xff));


//	T4K37_write_cmos_sensor(0x0104,0x0000);// );//-/-/-/-/-/-/-/GROUP_PARA_HOLD 

	return;
}
void set_gain(UINT16 iGain)
{
	//UINT16 gain_reg=0;
	unsigned long flags;

	
	LOG_INF("4K04:featurecontrol--iGain=%d\n",iGain);


	spin_lock_irqsave(&imgsensor_drv_lock,flags);
	realGain = iGain;//64 Base
	sensorGlobalGain = Gain2Reg(iGain);
	spin_unlock_irqrestore(&imgsensor_drv_lock,flags);
	
	write_gain(sensorGlobalGain);	
	LOG_INF("[T4K37_SetGain]t4k37.sensorGlobalGain=0x%x,t4k37.realGain=%d\n",sensorGlobalGain,realGain);
	
}   

kal_uint16 read_gain(void)
{
    //kal_uint8  temp_reg;
	kal_uint16 read_gain=0;

	read_gain=((read_cmos_sensor(0x0204) << 8) | read_cmos_sensor(0x0205));
	
	spin_lock(&imgsensor_drv_lock);
	sensorGlobalGain = read_gain; 
	realGain = Reg2Gain(sensorGlobalGain);
	spin_unlock(&imgsensor_drv_lock);
	
	//LOG_INF("t4k37.sensorGlobalGain=0x%x,t4k37.realGain=%d\n",t4k37.sensorGlobalGain,t4k37.realGain);
	
	return sensorGlobalGain;
} 

#if 0
static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
    LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
    if (imgsensor.ihdr_en) {

        spin_lock(&imgsensor_drv_lock);
        if (le > imgsensor.min_frame_length - imgsensor_info.margin)
            imgsensor.frame_length = le + imgsensor_info.margin;
        else
            imgsensor.frame_length = imgsensor.min_frame_length;
        if (imgsensor.frame_length > imgsensor_info.max_frame_length)
            imgsensor.frame_length = imgsensor_info.max_frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
        if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;


        // Extend frame length first
		    write_cmos_sensor(0x0340, (imgsensor.frame_length >> 8)& 0xFF);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);

		write_cmos_sensor(0x0202, (le >> 8) & 0xFF);
		write_cmos_sensor(0x0203, le & 0xFF);	 

        
		//iReg = gain2reg(gain);	
		
		write_cmos_sensor(0x0234, (se >> 8) & 0xFF);
         write_cmos_sensor(0x0235, se & 0x0F); 


        set_gain(gain);
    }

}
#endif

#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	kal_uint8  iTemp; 
	LOG_INF("image_mirror = %d\n", image_mirror);

	LOG_INF("set_mirror_flip function\n");
    iTemp = read_cmos_sensor(0x0101) & 0x03;	//Clear the mirror and flip bits.
    switch (image_mirror)
    {
        case IMAGE_NORMAL:
            write_cmos_sensor(0x0101, 0x03);	//Set normal
            break;
        case IMAGE_V_MIRROR:
            write_cmos_sensor(0x0101, iTemp | 0x01);	//Set flip
            break;
        case IMAGE_H_MIRROR:
            write_cmos_sensor(0x0101, iTemp | 0x02);	//Set mirror
            break;
        case IMAGE_HV_MIRROR:
            write_cmos_sensor(0x0101, 0x00);	//Set mirror and flip
            break;
    }
	LOG_INF("Error image_mirror setting\n");

}
#endif

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/

static void sensor_init(void)
{
	LOG_INF("E\n");
	//write_cmos_sensor(0x0100,0x00);//20160407  fan

    write_cmos_sensor(0x0101,0x03);////20160407  fan
    write_cmos_sensor(0x0103, 0x00);    //-/-/-/-/-/-/MIPI_RST/SOFTWARE_RESET;
    write_cmos_sensor(0x0104, 0x00);    //-/-/-/-/-/-/-/GROUP_PARA_HOLD;
    write_cmos_sensor(0x0105, 0x01);    //-/-/-/-/-/-/-/MSK_CORRUPT_FR;
    write_cmos_sensor(0x0106, 0x00);    //-/-/-/-/-/-/-/FAST_STANDBY_CTRL;
    write_cmos_sensor(0x0110, 0x00);    //-/-/-/-/-/CSI_CHAN_IDNTF[2:0];
    write_cmos_sensor(0x0111, 0x02);    //-/-/-/-/-/-/CSI_SIGNAL_MOD[1:0];
    write_cmos_sensor(0x0112, 0x0a);    //CSI_DATA_FORMAT[15:8];
    write_cmos_sensor(0x0113, 0x0a);    //CSI_DATA_FORMAT[7:0];
    write_cmos_sensor(0x0114, 0x03);    //-/-/-/-/-/-/CSI_LANE_MODE[1:0];
    write_cmos_sensor(0x0115, 0x30);    //-/-/CSI_10TO8_DT[5:0];
    write_cmos_sensor(0x0117, 0x32);    //-/-/CSI_10TO6_DT[5:0];
    write_cmos_sensor(0x0130, 0x18);    //EXTCLK_FRQ_MHZ[15:8];
    write_cmos_sensor(0x0131, 0x00);    //EXTCLK_FRQ_MHZ[7:0];
    write_cmos_sensor(0x0141, 0x00);    //-/-/-/-/-/CTX_SW_CTL[2:0];
    write_cmos_sensor(0x0142, 0x00);    //-/-/-/-/CONT_MDSEL_FRVAL[1:0]/CONT_FRCNT_MSK/CONT_GRHOLD_MSK;
    write_cmos_sensor(0x0202, 0x0c);    //COAR_INTEGR_TIM[15:8];
    write_cmos_sensor(0x0203, 0x42);    //COAR_INTEGR_TIM[7:0];
    write_cmos_sensor(0x0204, 0x00);    //-/-/-/-/ANA_GA_CODE_GL[11:8];
    write_cmos_sensor(0x0205, 0x45);    //ANA_GA_CODE_GL[7:0];
    write_cmos_sensor(0x0210, 0x01);    //-/-/-/-/-/-/DG_GA_GREENR[9:8];
    write_cmos_sensor(0x0211, 0x00);    //DG_GA_GREENR[7:0];
    write_cmos_sensor(0x0212, 0x01);    //-/-/-/-/-/-/DG_GA_RED[9:8];
    write_cmos_sensor(0x0213, 0x00);    //DG_GA_RED[7:0];
    write_cmos_sensor(0x0214, 0x01);    //-/-/-/-/-/-/DG_GA_BLUE[9:8];
    write_cmos_sensor(0x0215, 0x00);    //DG_GA_BLUE[7:0];
    write_cmos_sensor(0x0216, 0x01);    //-/-/-/-/-/-/DG_GA_GREENB[9:8];
    write_cmos_sensor(0x0217, 0x00);    //DG_GA_GREENB[7:0];
    write_cmos_sensor(0x0230, 0x00);    //-/-/-/HDR_MODE[4:0];
    write_cmos_sensor(0x0301, 0x01);    //-/-/-/-/VT_PIX_CLK_DIV[3:0];
    write_cmos_sensor(0x0303, 0x0A);    //-/-/-/-/VT_SYS_CLK_DIV[3:0];
    write_cmos_sensor(0x0305, 0x07);    //-/-/-/-/-/PRE_PLL_CLK_DIV[2:0];
    write_cmos_sensor(0x0306, 0x00);    //-/-/-/-/-/-/-/PLL_MULTIPLIER[8];
    write_cmos_sensor(0x0307, 0xB0);    //PLL_MULTIPLIER[7:0];
    write_cmos_sensor(0x030B, 0x01);    //-/-/-/-/OP_SYS_CLK_DIV[3:0];
    write_cmos_sensor(0x030D, 0x03);    //-/-/-/-/-/PRE_PLL_ST_CLK_DIV[2:0];
    write_cmos_sensor(0x030E, 0x00);    //-/-/-/-/-/-/-/PLL_MULT_ST[8];
    write_cmos_sensor(0x030F, 0x64); //70   //PLL_MULT_ST[7:0];
    write_cmos_sensor(0x0310, 0x01);    //-/-/-/-/-/-/-/OPCK_PLLSEL;
    write_cmos_sensor(0x0340, 0x0c);    //FR_LENGTH_LINES[15:8];
    write_cmos_sensor(0x0341, 0x48);    //FR_LENGTH_LINES[7:0];
    write_cmos_sensor(0x0342, 0x11);    //LINE_LENGTH_PCK[15:8];
    write_cmos_sensor(0x0343, 0x80);    //LINE_LENGTH_PCK[7:0];
    write_cmos_sensor(0x0344, 0x00);    //-/-/-/-/H_CROP[3:0];
    write_cmos_sensor(0x0346, 0x00);    //Y_ADDR_START[15:8];
    write_cmos_sensor(0x0347, 0x00);    //Y_ADDR_START[7:0];
    write_cmos_sensor(0x034A, 0x0c);    //Y_ADDR_END[15:8];
    write_cmos_sensor(0x034B, 0x2f);    //Y_ADDR_END[7:0];
    write_cmos_sensor(0x034C, 0x10);    //X_OUTPUT_SIZE[15:8];
    write_cmos_sensor(0x034D, 0x70);    //X_OUTPUT_SIZE[7:0];
    write_cmos_sensor(0x034E, 0x0c);    //Y_OUTPUT_SIZE[15:8];
    write_cmos_sensor(0x034F, 0x30);    //Y_OUTPUT_SIZE[7:0];
    write_cmos_sensor(0x0401, 0x00);    //-/-/-/-/-/-/SCALING_MODE[1:0];
    write_cmos_sensor(0x0403, 0x00);    //-/-/-/-/-/-/SPATIAL_SAMPLING[1:0];
    write_cmos_sensor(0x0404, 0x10);    //SCALE_M[7:0];
    write_cmos_sensor(0x0601, 0x00);    //TEST_PATT_MODE[7:0];
    write_cmos_sensor(0x0800, 0x88);    //TCLK_POST[7:3]/-/-/-;
    write_cmos_sensor(0x0801, 0x38);    //THS_PREPARE[7:3]/-/-/-;
    write_cmos_sensor(0x0802, 0x78);    //THS_ZERO[7:3]/-/-/-;
    write_cmos_sensor(0x0803, 0x48);    //THS_TRAIL[7:3]/-/-/-;
    write_cmos_sensor(0x0804, 0x48);    //TCLK_TRAIL[7:3]/-/-/-;
    write_cmos_sensor(0x0805, 0x40);    //TCLK_PREPARE[7:3]/-/-/-;
    write_cmos_sensor(0x0806, 0x00);    //TCLK_ZERO[7:3]/-/-/-;
    write_cmos_sensor(0x0807, 0x48);    //TLPX[7:3]/-/-/-;
    write_cmos_sensor(0x0808, 0x01);    //-/-/-/-/-/-/DPHY_CTRL[1:0];
    write_cmos_sensor(0x0820, 0x0b);    //MSB_LBRATE[31:24];
    write_cmos_sensor(0x0821, 0x40);    //MSB_LBRATE[23:16];
    write_cmos_sensor(0x0822, 0x00);    //MSB_LBRATE[15:8];
    write_cmos_sensor(0x0823, 0x00);    //MSB_LBRATE[7:0];
    write_cmos_sensor(0x0900, 0x00);    //-/-/-/-/-/-/H_BIN[1:0];
    write_cmos_sensor(0x0901, 0x00);    //-/-/-/-/-/-/V_BIN_MODE[1:0];
    write_cmos_sensor(0x0902, 0x00);    //-/-/-/-/-/-/BINNING_WEIGHTING[1:0];
    write_cmos_sensor(0x0A05, 0x01);    //-/-/-/-/-/-/-/MAP_DEF_EN;
    write_cmos_sensor(0x0A06, 0x01);    //-/-/-/-/-/-/-/SGL_DEF_EN;
    write_cmos_sensor(0x0A07, 0x98);    //SGL_DEF_W[7:0];
    write_cmos_sensor(0x0A0A, 0x01);    //-/-/-/-/-/-/-/COMB_CPLT_SGL_DEF_EN;
    write_cmos_sensor(0x0A0B, 0x98);    //COMB_CPLT_SGL_DEF_W[7:0];
    write_cmos_sensor(0x0F00, 0x00);    //-/-/-/-/-/ABF_LUT_CTL[2:0];
    write_cmos_sensor(0x3000, 0x30);    //BSC_ULMT_AGRNG3[7:0];
    write_cmos_sensor(0x3001, 0x14);    //BSC_ULMT_AGRNG2[7:0];
    write_cmos_sensor(0x3002, 0x0e);    //BSC_ULMT_AGRNG1[7:0];
    write_cmos_sensor(0x3007, 0x08);    //BSC_OFF/-/-/-/LIMIT_BSC_ON/-/-/-;
    write_cmos_sensor(0x3008, 0x80);    //DRESET_HIGH/FTLSNS_HIGH/-/DRESET_LBSC_LOW/FTLSNS_HIGH_RNG4/FTLSNS_HIGH_RNG3/FTLSNS_HIGH_RNG2/FTLSNS_HIGH_RNG1;
    write_cmos_sensor(0x3009, 0x0f);    //-/-/-/-/LIMIT_BSC_RNG4/LIMIT_BSC_RNG3/LIMIT_BSC_RNG2/LIMIT_BSC_RNG1;
    write_cmos_sensor(0x300A, 0x00);    //-/-/-/-/VSIG_CLIP_BSC_RNG4/VSIG_CLIP_BSC_RNG3/VSIG_CLIP_BSC_RNG2/VSIG_CLIP_BSC_RNG1;
    write_cmos_sensor(0x300B, 0x08);    //-/-/-/DRESET_LBSC_U[4:0];
    write_cmos_sensor(0x300C, 0x20);    //DRESET_LBSC_D[7:0];
    write_cmos_sensor(0x300D, 0x00);    //DRESET_CONJ_U[11:8]/-/-/-/-;
    write_cmos_sensor(0x300E, 0x1c);    //DRESET_CONJ_U[7:0];
    write_cmos_sensor(0x300F, 0x05);    //-/-/-/-/DRESET_CONJ_D[3:0];
    write_cmos_sensor(0x3010, 0x00);    //-/-/-/-/-/FTLSNS_LBSC_U[10:8];
    write_cmos_sensor(0x3011, 0x46);    //FTLSNS_LBSC_U[7:0];
    write_cmos_sensor(0x3012, 0x00);    //-/-/-/-/FTLSNS_LBSC_D[3:0];
    write_cmos_sensor(0x3013, 0x18);    //-/FTLSNS_CONJ_W[6:0];
    write_cmos_sensor(0x3014, 0x05);    //-/-/-/-/FTLSNS_CONJ_D[3:0];
    write_cmos_sensor(0x3015, 0x00);    //-/-/-/-/-/SADR_LBSC_U[10:8];
    write_cmos_sensor(0x3016, 0x4e);    //SADR_LBSC_U[7:0];
    write_cmos_sensor(0x3017, 0x00);    //-/-/-/-/SADR_LBSC_D[3:0];
    write_cmos_sensor(0x3018, 0x4e);    //SADR_CONJ_U[7:0];
    write_cmos_sensor(0x3019, 0x00);    //-/-/-/-/SADR_CONJ_D[3:0];
    write_cmos_sensor(0x301A, 0x44);    //KBIASCNT_RNG4[3:0]/KBIASCNT_RNG3[3:0];
    write_cmos_sensor(0x301B, 0x44);    //KBIASCNT_RNG2[3:0]/KBIASCNT_RNG1[3:0];
    write_cmos_sensor(0x301C, 0x88);    //FIX_VSIGLMTEN[3:0]/VSIGLMTEN_TIM_CHG/-/-/-;
    write_cmos_sensor(0x301D, 0x00);    //-/-/-/-/-/-/-/VSIGLMTEN_U[8];
    write_cmos_sensor(0x301E, 0x00);    //VSIGLMTEN_U[7:0];
    write_cmos_sensor(0x301F, 0x00);    //-/-/-/-/-/-/-/VSIGLMTEN_D[8];
    write_cmos_sensor(0x3020, 0x00);    //VSIGLMTEN_D[7:0];
    write_cmos_sensor(0x3021, 0x0a);    //-/-/VSIGLMTEN_TIM_CHG_D[5:0];
    write_cmos_sensor(0x3023, 0x08);    //AGADJ_CALC_MODE/-/-/-/MPS_AGADJ_MODE/-/-/-;
    write_cmos_sensor(0x3024, 0x00);    //-/-/-/-/AGADJ_FIX_COEF[11:8];
    write_cmos_sensor(0x3025, 0x7c);    //AGADJ_FIX_COEF[7:0];
    write_cmos_sensor(0x3027, 0x00);    //ADSW1_HIGH/-/ADSW1DMX_LOW/-/ADSW1LK_HIGH/-/ADSW1LKX_LOW/-;
    write_cmos_sensor(0x3028, 0x02);    //ADSW2_HIGH/-/ADSW2DMX_LOW/-/-/-/FIX_ADENX[1:0];
    write_cmos_sensor(0x3029, 0x0c);    //ADSW1_D[7:0];
    write_cmos_sensor(0x302A, 0x00);    //ADSW_U[7:0];
    write_cmos_sensor(0x302B, 0x06);    //-/-/-/ADSW1DMX_U[4:0];
    write_cmos_sensor(0x302C, 0x18);    //-/ADSW1LK_D[6:0];
    write_cmos_sensor(0x302D, 0x18);    //-/ADSW1LKX_U[6:0];
    write_cmos_sensor(0x302E, 0x0c);    //-/-/ADSW2_D[5:0];
    write_cmos_sensor(0x302F, 0x06);    //-/-/-/ADSW2DMX_U[4:0];
    write_cmos_sensor(0x3030, 0x00);    //ADENX_U[7:0];
    write_cmos_sensor(0x3031, 0x80);    //ADENX_D[8]/-/-/-/-/-/-/-;
    write_cmos_sensor(0x3032, 0xfc);    //ADENX_D[7:0];
    write_cmos_sensor(0x3033, 0x00);    //CMPI_GCR_CHG[7:0];
    write_cmos_sensor(0x3034, 0x00);    //CMPI_NML_RET[8]/-/-/-/-/-/-/-;
    write_cmos_sensor(0x3035, 0x04);    //CMPI_NML_RET[7:0];
    write_cmos_sensor(0x3036, 0x00);    //-/CMPAMPCAPEN_RNG4/-/CMPAMPCAPEN_RNG3/-/CMPAMPCAPEN_RNG2/-/CMPAMPCAPEN_RNG1;
    write_cmos_sensor(0x3037, 0x0d);    //-/-/-/CMPI1_NML_RNG4[4:0];
    write_cmos_sensor(0x3038, 0x0d);    //-/-/-/CMPI1_NML_RNG3[4:0];
    write_cmos_sensor(0x3039, 0x0d);    //-/-/-/CMPI1_NML_RNG2[4:0];
    write_cmos_sensor(0x303A, 0x0d);    //-/-/-/CMPI1_NML_RNG1[4:0];
    write_cmos_sensor(0x303B, 0x0f);    //-/-/-/CMPI2_NML_VAL[4:0];
    write_cmos_sensor(0x303C, 0x03);    //-/-/-/CMPI1_GCR_VAL[4:0];
    write_cmos_sensor(0x303D, 0x03);    //-/-/-/CMPI2_GCR_VAL[4:0];
    write_cmos_sensor(0x303E, 0x00);    //ADCKEN_MASK[1:0]/-/-/-/-/-/ADCKEN_LOW;
    write_cmos_sensor(0x303F, 0x0b);    //ADCKEN_NML_RS_U[7:0];
    write_cmos_sensor(0x3040, 0x15);    //-/-/-/ADCKEN_RS_D[4:0];
    write_cmos_sensor(0x3042, 0x0b);    //ADCKEN_NML_AD_U[7:0];
    write_cmos_sensor(0x3043, 0x15);    //-/-/-/ADCKEN_AD_D[4:0];
    write_cmos_sensor(0x3044, 0x00);    //EDCONX_RS_HIGH/EDCONX_AD_HIGH/-/-/-/-/-/-;
    write_cmos_sensor(0x3045, 0x14);    //EDCONX_RS_D[7:0];
    write_cmos_sensor(0x3046, 0x00);    //-/-/-/-/-/-/-/EDCONX_AD_D[8];
    write_cmos_sensor(0x3047, 0x28);    //EDCONX_AD_D[7:0];
    write_cmos_sensor(0x3048, 0x0e);    //-/-/-/ADCKSTT_D[4:0];
    write_cmos_sensor(0x3049, 0x20);    //-/CNTEDSTP_U[6:0];
    write_cmos_sensor(0x304A, 0x09);    //-/-/-/-/CNTRSTX_U[3:0];
    write_cmos_sensor(0x304B, 0x1f);    //-/-/CNT0RSTX_RS_D[5:0];
    write_cmos_sensor(0x304C, 0x09);    //-/-/-/-/CNT0RSTX_AD_U[3:0];
    write_cmos_sensor(0x304D, 0x21);    //-/-/CNT0RSTX_AD_D[5:0];
    write_cmos_sensor(0x304E, 0x00);    //CNTRD_HCNT_DRV/-/-/-/-/-/-/-;
    write_cmos_sensor(0x304F, 0x29);    //CNTRD_HCNT_U[7:0];
    write_cmos_sensor(0x3050, 0x1d);    //-/-/-/CNTINVX_START[4:0];
    write_cmos_sensor(0x3051, 0x00);    //ADTESTCK_ON/COUNTER_TEST/-/-/-/-/-/-;
    write_cmos_sensor(0x3052, 0x00);    //-/-/-/-/ADTESTCK_INTVL[3:0];
    write_cmos_sensor(0x3053, 0xe0);    //BSDIGITAL_RNG4/BSDIGITAL_RNG3/BSDIGITAL_RNG2/BSDIGITAL_RNG1/-/-/-/-;
    write_cmos_sensor(0x3055, 0x00);    //TEST_ST_HREGSW/-/-/-/-/-/-/-;
    write_cmos_sensor(0x3056, 0x00);    //DATA_LATCH_DLY/-/-/-/NONHEF_PS_ORDER/-/-/-;
    write_cmos_sensor(0x3057, 0x00);    //HOBINV/-/-/-/-/-/-/-;
    write_cmos_sensor(0x3059, 0x01);    //-/-/-/-/EN_HSFBC_PERIOD_U[3:0];
    write_cmos_sensor(0x305A, 0x0a);    //-/-/-/-/EN_HSFBC_PERIOD_W[3:0];
    write_cmos_sensor(0x305C, 0x01);    //-/-/-/-/-/FBC_STOP_CND[2:0];
    write_cmos_sensor(0x305D, 0x10);    //-/FBC_IN_RANGE[6:0];
    write_cmos_sensor(0x305E, 0x06);    //FBC_AG_CONT_COEF[7:0];
    write_cmos_sensor(0x305F, 0x04);    //-/-/-/FBC_IN_LINES[4:0];
    write_cmos_sensor(0x3061, 0x01);    //-/-/-/-/-/-/FBC_SUSP_CND[1:0];
    write_cmos_sensor(0x3062, 0x01);    //-/-/-/-/-/FBC_SUSP_RANGE[2:0];
    write_cmos_sensor(0x3063, 0x60);    //FBC_SUSP_AGPARAM[7:0];
    write_cmos_sensor(0x3065, 0x02);    //-/-/-/-/FBC_START_CND[3:0];
    write_cmos_sensor(0x3066, 0x01);    //-/-/-/-/-/-/FBC_OUT_RANGE[1:0];
    write_cmos_sensor(0x3067, 0x03);    //-/-/-/-/-/-/FBC_OUT_CNT_FRMS[1:0];
    write_cmos_sensor(0x3068, 0x24);    //-/-/FBC_OUT_LINES[5:0];
    write_cmos_sensor(0x306A, 0x80);    //HS_FBC_ON/-/-/-/-/-/-/-;
    write_cmos_sensor(0x306B, 0x08);    //-/-/HS_FBC_RANGE[5:0];
    write_cmos_sensor(0x306E, 0x10);    //-/-/-/VSIGLMTBIASSEL/-/-/-/IREFVSEL;
    write_cmos_sensor(0x306F, 0x02);    //-/-/-/-/-/-/FBC_VREF_RNG_SEL[1:0];
    write_cmos_sensor(0x3070, 0x0e);    //-/-/EN_PS_VREFI_FB[5:0];
    write_cmos_sensor(0x3071, 0x00);    //PS_VFB_GBL_VAL[9:8]/-/-/-/-/-/-;
    write_cmos_sensor(0x3072, 0x00);    //PS_VFB_GBL_VAL[7:0];
    write_cmos_sensor(0x3073, 0x26);    //-/PS_VFB_10B_RNG4[6:0];
    write_cmos_sensor(0x3074, 0x1a);    //-/PS_VFB_10B_RNG3[6:0];
    write_cmos_sensor(0x3075, 0x0f);    //-/PS_VFB_10B_RNG2[6:0];
    write_cmos_sensor(0x3076, 0x03);    //-/PS_VFB_10B_RNG1[6:0];
	write_cmos_sensor(0x3077, 0x01);
    write_cmos_sensor(0x3078, 0x01);    //-/-/-/-/VFB_STEP_RNG3[3:0];
    write_cmos_sensor(0x3079, 0x01);    //-/-/-/-/VFB_STEP_RNG2[3:0];
    write_cmos_sensor(0x307A, 0x01);    //-/-/-/-/VFB_STEP_RNG1[3:0];
    write_cmos_sensor(0x307B, 0x04);    //-/-/-/-/HSVFB_STEP_RNG4[3:0];
    write_cmos_sensor(0x307C, 0x02);    //-/-/-/-/HSVFB_STEP_RNG3[3:0];
    write_cmos_sensor(0x307D, 0x01);    //-/-/-/-/HSVFB_STEP_RNG2[3:0];
    write_cmos_sensor(0x307E, 0x02);    //-/-/-/-/HSVFB_STEP_RNG1[3:0];
    write_cmos_sensor(0x3080, 0x00);    //EXT_VREFI_FB_ON/-/-/-/-/-/-/-;
    write_cmos_sensor(0x3081, 0x00);    //VREFI_FB_FIXVAL[9:8]/-/-/-/-/-/-;
    write_cmos_sensor(0x3082, 0x00);    //VREFI_FB_FIXVAL[7:0];
    write_cmos_sensor(0x3085, 0x00);    //EXT_VREFI_ZS_ON/-/-/-/-/-/-/-;
    write_cmos_sensor(0x3086, 0x00);    //-/-/-/-/-/-/VREFI_ZS_FIXVAL[9:8];
    write_cmos_sensor(0x3087, 0x00);    //VREFI_ZS_FIXVAL[7:0];
    write_cmos_sensor(0x3089, 0xa0);    //ZSV_LEVEL_RNG4[7:0];
    write_cmos_sensor(0x308A, 0xa0);    //ZSV_LEVEL_RNG3[7:0];
    write_cmos_sensor(0x308B, 0xa0);    //ZSV_LEVEL_RNG2[7:0];
    write_cmos_sensor(0x308C, 0xa0);    //ZSV_LEVEL_RNG1[7:0];
    write_cmos_sensor(0x308D, 0x03);    //-/-/-/-/-/-/ZSV_STOP_CND[1:0];
    write_cmos_sensor(0x308E, 0x20);    //-/-/ZSV_IN_RANGE[5:0];
    write_cmos_sensor(0x3090, 0x04);    //-/-/-/-/ZSV_IN_LINES[3:0];
    write_cmos_sensor(0x3091, 0x04);    //-/-/-/-/ZSV_FORCE_END[3:0];
    write_cmos_sensor(0x3092, 0x01);    //-/-/-/-/-/-/ZSV_SUSP_CND[1:0];
    write_cmos_sensor(0x3093, 0x01);    //-/-/-/-/-/-/ZSV_SUSP_RANGE[1:0];
    write_cmos_sensor(0x3094, 0x60);    //ZSV_SUSP_AGPARAM[7:0];
    write_cmos_sensor(0x3095, 0x0e);    //-/-/-/EN_PS_VREFI_ZS[4:0];
    write_cmos_sensor(0x3096, 0x75);    //PS_VZS_10B_COEF[7:0];
    write_cmos_sensor(0x3097, 0x7e);    //-/PS_VZS_10B_INTC[6:0];
    write_cmos_sensor(0x3098, 0x20);    //VZS_STEP_10B[7:0];
    write_cmos_sensor(0x3099, 0x01);    //-/-/MIN_VZS_STEP[5:0];
    write_cmos_sensor(0x309C, 0x00);    //EXT_HCNT_MAX_ON/-/-/-/HCNT_MAX_MODE/-/-/-;
    write_cmos_sensor(0x309D, 0x08);    //HCNT_MAX_FIXVAL[15:8];
    write_cmos_sensor(0x309E, 0x2c);    //HCNT_MAX_FIXVAL[7:0];
    write_cmos_sensor(0x30A0, 0x82);    //REV_INT_MODE/-/-/-/RI_LOWVOL_MODE/-/RI_FINE_FBC/-;
    write_cmos_sensor(0x30A1, 0x02);    //-/-/-/-/-/AGADJ_EXEC_MODE[2:0];
    write_cmos_sensor(0x30A2, 0x00);    //-/-/-/-/-/-/-/AGADJ_REV_INT;
    write_cmos_sensor(0x30A3, 0x0e);    //-/-/-/ZSV_EXEC_MODE[4:0];
    write_cmos_sensor(0x30A6, 0x00);    //-/-/-/-/-/-/ROUND_VREF_CODE[1:0];
    write_cmos_sensor(0x30A7, 0x20);    //EXT_VCD_ADJ_ON/-/AUTO_BIT_SFT_VCD/-/AG_SEN_SHIFT/-/-/-;
    write_cmos_sensor(0x30A8, 0x04);    //-/-/-/-/VCD_COEF_FIXVAL[11:8];
    write_cmos_sensor(0x30A9, 0x00);    //VCD_COEF_FIXVAL[7:0];
    write_cmos_sensor(0x30AA, 0x00);    //-/-/VCD_INTC_FIXVAL[5:0];
    write_cmos_sensor(0x30AB, 0x30);    //-/-/VCD_RNG_TYPE_SEL[1:0]/-/-/VREFIC_MODE[1:0];
    write_cmos_sensor(0x30AC, 0x32);    //-/-/VREFAD_RNG4_SEL[1:0]/-/-/VREFAD_RNG3_SEL[1:0];
    write_cmos_sensor(0x30AD, 0x10);    //-/-/VREFAD_RNG2_SEL[1:0]/-/-/VREFAD_RNG1_SEL[1:0];
    write_cmos_sensor(0x30AE, 0x00);    //FIX_AGADJ_VREFI/-/-/-/-/-/-/-;
    write_cmos_sensor(0x30AF, 0x00);    //-/-/-/-/-/-/AGADJ1_VREFI_ZS[9:8];
    write_cmos_sensor(0x30B0, 0x3e);    //AGADJ1_VREFI_ZS[7:0];
    write_cmos_sensor(0x30B1, 0x00);    //-/-/-/-/-/-/AGADJ2_VREFI_ZS[9:8];
    write_cmos_sensor(0x30B2, 0x1f);    //AGADJ2_VREFI_ZS[7:0];
    write_cmos_sensor(0x30B3, 0x00);    //-/-/-/-/-/-/-/AGADJ1_VREFI_AD[8];
    write_cmos_sensor(0x30B4, 0x3e);    //AGADJ1_VREFI_AD[7:0];
    write_cmos_sensor(0x30B5, 0x00);    //-/-/-/-/-/-/-/AGADJ2_VREFI_AD[8];
    write_cmos_sensor(0x30B6, 0x1f);    //AGADJ2_VREFI_AD[7:0];
    write_cmos_sensor(0x30B7, 0x70);    //-/AGADJ_VREFIC[2:0]/-/AGADJ_VREFC[2:0];
    write_cmos_sensor(0x30B9, 0x80);    //RI_VREFAD_COEF[7:0];
    write_cmos_sensor(0x30BA, 0x00);    //EXT_VREFC_ON/-/-/-/-/-/-/-;
    write_cmos_sensor(0x30BB, 0x00);    //-/-/-/-/-/VREFC_FIXVAL[2:0];
    write_cmos_sensor(0x30BC, 0x00);    //EXT_VREFIC_ON/-/-/-/-/-/-/-;
    write_cmos_sensor(0x30BD, 0x00);    //-/-/-/-/-/VREFIC_FIXVAL[2:0];
    write_cmos_sensor(0x30BE, 0x80);    //VREFIC_CHG_SEL/-/VREFICAID_OFF[1:0]/-/FIX_VREFICAID[2:0];
    write_cmos_sensor(0x30BF, 0x00);    //-/-/-/-/-/-/VREFICAID_W[9:8];
    write_cmos_sensor(0x30C0, 0xe6);    //VREFICAID_W[7:0];
    write_cmos_sensor(0x30C1, 0x00);    //EXT_PLLFREQ_ON/-/-/-/-/-/-/-;
    write_cmos_sensor(0x30C2, 0x00);    //-/-/-/-/PLLFREQ_FIXVAL[3:0];
    write_cmos_sensor(0x30C4, 0x00);    //-/-/-/-/-/-/ST_BLACK_LEVEL[9:8];
    write_cmos_sensor(0x30C5, 0x40);    //ST_BLACK_LEVEL[7:0];
    write_cmos_sensor(0x30C6, 0x00);    //-/-/ST_CKI[5:0];
    write_cmos_sensor(0x30C7, 0x00);    //EN_TAIL_DATA/-/-/-/-/-/-/-;
    write_cmos_sensor(0x30C8, 0x10);    //-/-/-/EN_REV_CNT_DATA/-/-/-/EXT_BCODEREF_ON;
    write_cmos_sensor(0x30C9, 0x00);    //-/BCODEREF_FIXVAL[6:0];
    write_cmos_sensor(0x30CC, 0x04);    //CNTSTTMODE/CNTSTPMODE/-/-/ADCKNUM[1:0]/-/-;
    write_cmos_sensor(0x30CD, 0xf0);    //POSBSTEN/POSBSTEN2/POSLFIX/NEGLFIX/-/NEGBST2EN/NEGBST2RNG/NEGBST2CKSEL;
    write_cmos_sensor(0x30CE, 0x65);    //NEGBSTCNT[3:0]/NEGBST2CNT[3:0];
    write_cmos_sensor(0x30CF, 0x55);    //-/POSBSTCNT[2:0]/-/POSBST2CNT[2:0];
    write_cmos_sensor(0x30D0, 0x22);    //-/POSBSTHG[2:0]/-/POSBST2HG[2:0];
    write_cmos_sensor(0x30D1, 0x05);    //-/-/READVDSEL[1:0]/GDMOSBGREN/POSBSTGA[2:0];
    write_cmos_sensor(0x30D2, 0xb3);    //NEGLEAKCUT/POSBSTEV/POSBSTRNG[1:0]/-/-/POSBST2RNG[1:0];
    write_cmos_sensor(0x30D3, 0x80);    //KBIASSEL/-/-/BSVBPSEL[4:0];
    write_cmos_sensor(0x30D5, 0x09);    //-/-/-/VREFV[4:0];
    write_cmos_sensor(0x30D6, 0xcc);    //ADSW2WEAK/ADSW1WEAK/-/-/VREFAI[3:0];
    write_cmos_sensor(0x30D7, 0x10);    //ADCKSEL/-/ADCKDIV[1:0]/-/-/-/-;
    write_cmos_sensor(0x30D8, 0x00);    //-/-/-/-/ANAMON1_SEL[3:0];
    write_cmos_sensor(0x30D9, 0x27);    //HREG_TEST/TESTCROP/BSDETSEL/HOBSIGIN/VBINVSIG/BINVSIG/BINED/BINCMP;
    write_cmos_sensor(0x30DB, 0x00);    //-/-/-/-/-/-/VSIGLMTGSEL/GCR_MODE;
    write_cmos_sensor(0x30DC, 0x21);    //-/-/GDMOS2CNT[1:0]/-/-/-/GDMOSCNTX4;
    write_cmos_sensor(0x30DD, 0x07);    //-/-/-/-/-/GBLRSTVREFSTPEN/GBLRSTDDSTPEN/GBLRSTGDSTPEN;
    write_cmos_sensor(0x30DE, 0x00);    //-/-/-/VREFALN/-/-/-/VREFIMBC;
    write_cmos_sensor(0x30DF, 0x10);    //-/-/VREFTEST[1:0]/-/-/-/VREFIMX4_SEL;
    write_cmos_sensor(0x30E0, 0x13);    //-/-/-/ADCMP1SRTSEL/-/SINTSELFB[2:0];
    write_cmos_sensor(0x30E1, 0xc0);    //SINTLSEL2/SINTLSEL/SINTSELPH2/SINTSELPH1/-/-/-/-;
    write_cmos_sensor(0x30E2, 0x00);    //VREFAMPSHSEL/-/-/-/-/-/-/-;
    write_cmos_sensor(0x30E3, 0x40);    //SINTSELOUT2[3:0]/SINTSELOUT1[3:0];
    write_cmos_sensor(0x30E4, 0x00);    //POSBSTCKDLYOFF/NEGBSTCKDLYOFF/-/-/-/-/-/-;
    write_cmos_sensor(0x30E5, 0x00);    //SENSEMODE[7:0];
    write_cmos_sensor(0x30E6, 0x00);    //SPARE_BL[3:0]/SPARE_BR[3:0];
    write_cmos_sensor(0x30E7, 0x00);    //SPARE_TL[3:0]/SPARE_TR[3:0];
    write_cmos_sensor(0x30E8, 0x10);    //-/-/-/BGRDVSTPEN/-/-/-/TSIGEN;
    write_cmos_sensor(0x30E9, 0x00);    //-/-/-/ST_VOB_VALID_TYPE/-/-/-/BLMD_VALID_ON;
    write_cmos_sensor(0x30EA, 0x00);    //-/-/-/SADR_HIGH/DCLAMP_LEVEL[1:0]/DCLAMP_LINE_MODE/DCLAMP_ON;
    write_cmos_sensor(0x3100, 0x07);    //-/-/-/-/FRAC_EXP_TIME_10B[11:8];
    write_cmos_sensor(0x3101, 0xed);    //FRAC_EXP_TIME_10B[7:0];
    write_cmos_sensor(0x3104, 0x00);    //ES_1PULSE/-/-/-/-/-/-/-;
    write_cmos_sensor(0x3105, 0x00);    //ESTGRESET_LOW/-/-/-/ESTGRESET_U_SEL/-/-/-;
    write_cmos_sensor(0x3106, 0xa0);    //AUTO_READ_W/-/AUTO_ESREAD_2D/-/-/-/-/-;
    write_cmos_sensor(0x3107, 0x06);    //-/-/ESTGRESET_U[5:0];
    write_cmos_sensor(0x3108, 0x10);    //ESTGRESET_D[7:0];
    write_cmos_sensor(0x310A, 0x54);    //ESREAD_1W[7:0];
    write_cmos_sensor(0x310B, 0x00);    //-/-/-/-/-/-/-/ESREAD_1D[8];
    write_cmos_sensor(0x310C, 0x46);    //ESREAD_1D[7:0];
    write_cmos_sensor(0x310E, 0x79);    //ESREAD_2W[7:0];
    write_cmos_sensor(0x310F, 0x01);    //-/-/-/ESREAD_2D[12:8];
    write_cmos_sensor(0x3110, 0x04);    //ESREAD_2D[7:0];
    write_cmos_sensor(0x3111, 0x00);    //EXTD_ROTGRESET/-/-/-/-/-/-/-;
    write_cmos_sensor(0x3112, 0x00);    //-/-/-/-/-/-/ROTGRESET_U[9:8];
    write_cmos_sensor(0x3113, 0x0c);    //ROTGRESET_U[7:0];
    write_cmos_sensor(0x3114, 0x2a);    //-/ROTGRESET_W[6:0];
    write_cmos_sensor(0x3116, 0x00);    //ROREAD_U[7:0];
    write_cmos_sensor(0x3118, 0x79);    //ROREAD_W[7:0];
    write_cmos_sensor(0x311B, 0x00);    //-/-/-/-/-/-/-/ZEROSET_U[8];
    write_cmos_sensor(0x311C, 0x0e);    //ZEROSET_U[7:0];
    write_cmos_sensor(0x311E, 0x30);    //ZEROSET_W[7:0];
    write_cmos_sensor(0x311F, 0x1e);    //-/-/AZSZEROSET_U[5:0];
    write_cmos_sensor(0x3121, 0x64);    //AZSZEROSET_W[7:0];
    write_cmos_sensor(0x3124, 0xfc);    //ESROWVD_U[7:0];
    write_cmos_sensor(0x3126, 0x02);    //ESROWVD_D[7:0];
    write_cmos_sensor(0x3127, 0x00);    //-/-/-/-/-/-/-/ROROWVD_U[8];
    write_cmos_sensor(0x3128, 0x08);    //ROROWVD_U[7:0];
    write_cmos_sensor(0x312A, 0x04);    //ROROWVD_D[7:0];
    write_cmos_sensor(0x312C, 0x00);    //PDRSTDRN_HIGH/-/-/-/-/-/-/-;
    write_cmos_sensor(0x312D, 0x42);    //-/PDRSTDRN_U[6:0];
    write_cmos_sensor(0x312E, 0x00);    //PDRSTDRN_D[8]/-/-/-/-/-/-/-;
    write_cmos_sensor(0x312F, 0x00);    //PDRSTDRN_D[7:0];
    write_cmos_sensor(0x3130, 0x42);    //-/AZSRSTDRN_U[6:0];
    write_cmos_sensor(0x3131, 0x14);    //-/AZSRSTDRN_D[6:0];
    write_cmos_sensor(0x3132, 0x08);    //FIX_RSTNWEAK[1:0]/-/-/FIX_RSTNWEAK_AZS/-/-/-;
    write_cmos_sensor(0x3133, 0x00);    //-/READ_VDD_MODE[2:0]/ESREAD1_VDD_MDOE/-/-/-;
    write_cmos_sensor(0x3134, 0x01);    //-/-/-/-/-/-/-/ROROWVD_MODE;
    write_cmos_sensor(0x3135, 0x04);    //-/-/-/VDDRD28EN_RORE_U[4:0];
    write_cmos_sensor(0x3136, 0x04);    //-/-/-/VDDRD28EN_RORE_D[4:0];
    write_cmos_sensor(0x3137, 0x04);    //-/-/-/VDDRD28EN_ROFE_U[4:0];
    write_cmos_sensor(0x3138, 0x04);    //-/-/-/VDDRD28EN_ROFE_D[4:0];
    write_cmos_sensor(0x3139, 0x04);    //-/-/-/VDDRD28EN_ES1RE_U[4:0];
    write_cmos_sensor(0x313A, 0x04);    //-/-/-/VDDRD28EN_ES1RE_D[4:0];
    write_cmos_sensor(0x313B, 0x04);    //-/-/-/VDDRD28EN_ES1FE_U[4:0];
    write_cmos_sensor(0x313C, 0x04);    //-/-/-/VDDRD28EN_ES1FE_D[4:0];
    write_cmos_sensor(0x313D, 0x04);    //-/-/-/VDDRD28EN_ES2RE_U[4:0];
    write_cmos_sensor(0x313E, 0x04);    //-/-/-/VDDRD28EN_ES2RE_D[4:0];
    write_cmos_sensor(0x313F, 0x04);    //-/-/-/VDDRD28EN_ES2FE_U[4:0];
    write_cmos_sensor(0x3140, 0x04);    //-/-/-/VDDRD28EN_ES2FE_D[4:0];
    write_cmos_sensor(0x3141, 0x02);    //-/-/-/BSTRDCUT_RORE_U[4:0];
    write_cmos_sensor(0x3142, 0x02);    //-/-/-/BSTRDCUT_RORE_D[4:0];
    write_cmos_sensor(0x3143, 0x02);    //-/-/-/BSTRDCUT_ROFE_U[4:0];
    write_cmos_sensor(0x3144, 0x02);    //-/-/-/BSTRDCUT_ROFE_D[4:0];
    write_cmos_sensor(0x3145, 0x02);    //-/-/-/BSTRDCUT_ES1RE_U[4:0];
    write_cmos_sensor(0x3146, 0x02);    //-/-/-/BSTRDCUT_ES1RE_D[4:0];
    write_cmos_sensor(0x3147, 0x02);    //-/-/-/BSTRDCUT_ES1FE_U[4:0];
    write_cmos_sensor(0x3148, 0x02);    //-/-/-/BSTRDCUT_ES1FE_D[4:0];
    write_cmos_sensor(0x3149, 0x02);    //-/-/-/BSTRDCUT_ES2RE_U[4:0];
    write_cmos_sensor(0x314A, 0x02);    //-/-/-/BSTRDCUT_ES2RE_D[4:0];
    write_cmos_sensor(0x314B, 0x02);    //-/-/-/BSTRDCUT_ES2FE_U[4:0];
    write_cmos_sensor(0x314C, 0x02);    //-/-/-/BSTRDCUT_ES2FE_D[4:0];
    write_cmos_sensor(0x314D, 0x80);    //VSIGSRT_LOW/-/VSIGSRT_U[5:0];
    write_cmos_sensor(0x314E, 0x00);    //-/VSIGSRT_D_RNG4[6:0];
    write_cmos_sensor(0x314F, 0x3d);    //-/VSIGSRT_D_RNG3[6:0];
    write_cmos_sensor(0x3150, 0x3b);    //-/VSIGSRT_D_RNG2[6:0];
    write_cmos_sensor(0x3151, 0x36);    //-/VSIGSRT_D_RNG1[6:0];
    write_cmos_sensor(0x3154, 0x00);    //RO_DMY_CR_MODE/-/-/-/DRCUT_SIGIN/-/-/-;
    write_cmos_sensor(0x3155, 0x00);    //INTERMIT_DRCUT/-/-/DRCUT_TMG_CHG/DRCUT_DMY_OFF/-/-/-;
    write_cmos_sensor(0x3156, 0x00);    //DRCUT_INTERMIT_U[7:0];
    write_cmos_sensor(0x3159, 0x00);    //GDMOSCNT_GCR_RET[8]/-/-/-/-/-/-/-;
    write_cmos_sensor(0x315A, 0x04);    //GDMOSCNT_GCR_RET[7:0];
    write_cmos_sensor(0x315B, 0x88);    //GDMOSCNT_NML_RNG4[3:0]/GDMOSCNT_NML_RNG3[3:0];
    write_cmos_sensor(0x315C, 0x88);    //GDMOSCNT_NML_RNG2[3:0]/GDMOSCNT_NML_RNG1[3:0];
    write_cmos_sensor(0x315D, 0x02);    //-/-/-/-/GDMOSCNT_GCR_VAL[3:0];
    write_cmos_sensor(0x315F, 0x00);    //RSTDRNAMPSTP_LOW/-/-/-/-/-/-/-;
    write_cmos_sensor(0x3160, 0x10);    //-/-/RSTDRNAMPSTP_U[5:0];
    write_cmos_sensor(0x3162, 0x7c);    //RSTDRNAMPSTP_D[7:0];
    write_cmos_sensor(0x3163, 0x00);    //BGRDVSTP_LOW/-/-/-/-/-/-/-;
    write_cmos_sensor(0x3165, 0x27);    //RSTVDSEL_NML[1:0]/RSTVDSEL_AZS[1:0]/RSTDRNVDSEL_NML[1:0]/RSTDRNVDSEL_AZS[1:0];
    write_cmos_sensor(0x3166, 0x11);    //VSIGLMTCNT_RNG4[3:0]/VSIGLMTCNT_RNG3[3:0];
    write_cmos_sensor(0x3167, 0x11);    //VSIGLMTCNT_RNG2[3:0]/VSIGLMTCNT_RNG1[3:0];
    write_cmos_sensor(0x3168, 0xf1);    //GDMOS2EN_RNG4/GDMOS2EN_RNG3/GDMOS2EN_RNG2/GDMOS2EN_RNG1/DRADRVLV_AZS[3:0];
    write_cmos_sensor(0x3169, 0x55);    //DRADRVLV_RNG4[3:0]/DRADRVLV_RNG3[3:0];
    write_cmos_sensor(0x316A, 0x55);    //DRADRVLV_RNG2[3:0]/DRADRVLV_RNG1[3:0];
    write_cmos_sensor(0x316B, 0x61);    //-/DRADRVI_AZS[2:0]/-/DRADRVI_NML[2:0];
    write_cmos_sensor(0x316C, 0x01);    //-/-/-/-/-/-/-/FIX_RSTDRNAMPSTP_GES;
    write_cmos_sensor(0x316D, 0x0c);    //-/-/-/-/VSIGLMTCNT_EZ_VAL[3:0];
    write_cmos_sensor(0x316E, 0x04);    //-/-/VSIGLMTCNT_EZ_U[5:0];
    write_cmos_sensor(0x316F, 0x04);    //-/-/VSIGLMTCNT_EZ_D[5:0];
    write_cmos_sensor(0x3172, 0x41);    //-/FBC_LONG_REF/-/FBC_SUBSMPL/-/-/BIN_MODE[1:0];
    write_cmos_sensor(0x3173, 0x30);    //-/-/ES_MODE[1:0]/ESREAD_ALT_OFF[3:0];
    write_cmos_sensor(0x3174, 0x00);    //-/-/-/-/-/-/-/DIS_MODE;
    write_cmos_sensor(0x3175, 0x00);    //-/-/-/-/-/-/-/ALLZEROSET_ON;
    write_cmos_sensor(0x3176, 0x11);    //-/-/ALLZEROSET_1ST_ON[1:0]/-/-/-/ALLZEROSET_CHG_ON;
    write_cmos_sensor(0x3177, 0x0a);    //-/-/LTCH_POS[1:0]/DME_ON/RODATA_U/DMR_ON/ALLREAD_ON;
    write_cmos_sensor(0x3178, 0x01);    //-/-/-/-/ES_LOAD_BAL[3:0];
    write_cmos_sensor(0x3180, 0x02);    //-/-/-/-/-/-/GBL_HCNT_MAX_SEL[1:0];
    write_cmos_sensor(0x3181, 0x1e);    //-/GBLESTGRESET_U[6:0];
    write_cmos_sensor(0x3182, 0x00);    //-/-/-/-/-/GBLESTGRESET_W[10:8];
    write_cmos_sensor(0x3183, 0xfa);    //GBLESTGRESET_W[7:0];
    write_cmos_sensor(0x3184, 0x00);    //-/-/-/-/-/-/GBLESREAD_1W[9:8];
    write_cmos_sensor(0x3185, 0x88);    //GBLESREAD_1W[7:0];
    write_cmos_sensor(0x3186, 0x00);    //-/-/-/-/-/-/GBLESREAD_1D[9:8];
    write_cmos_sensor(0x3187, 0x9c);    //GBLESREAD_1D[7:0];
    write_cmos_sensor(0x3188, 0x00);    //-/-/-/-/-/-/GBLESREAD_2W[9:8];
    write_cmos_sensor(0x3189, 0xc2);    //GBLESREAD_2W[7:0];
    write_cmos_sensor(0x318A, 0x10);    //-/-/-/GBLESREAD_2D[4:0];
    write_cmos_sensor(0x318B, 0x01);    //-/-/-/-/-/-/FIX_GBLRSTSTP_AZS/FIX_GBLRSTSTP_GES;
    write_cmos_sensor(0x318E, 0x88);    //GBLVDDRD28EN_ES1RE_U[3:0]/GBLVDDRD28EN_ES1RE_D[3:0];
    write_cmos_sensor(0x318F, 0x88);    //GBLVDDRD28EN_ES1FE_U[3:0]/GBLVDDRD28EN_ES1FE_D[3:0];
    write_cmos_sensor(0x3190, 0x88);    //GBLVDDRD28EN_ES2RE_U[3:0]/GBLVDDRD28EN_ES2RE_D[3:0];
    write_cmos_sensor(0x3191, 0x88);    //GBLVDDRD28EN_ES2FE_U[3:0]/GBLVDDRD28EN_ES2FE_D[3:0];
    write_cmos_sensor(0x3192, 0x44);    //GBLBSTRDCUT_ES1RE_U[3:0]/GBLBSTRDCUT_ES1RE_D[3:0];
    write_cmos_sensor(0x3193, 0x44);    //GBLBSTRDCUT_ES1FE_U[3:0]/GBLBSTRDCUT_ES1FE_D[3:0];
    write_cmos_sensor(0x3194, 0x44);    //GBLBSTRDCUT_ES2RE_U[3:0]/GBLBSTRDCUT_ES2RE_D[3:0];
    write_cmos_sensor(0x3195, 0x44);    //GBLBSTRDCUT_ES2FE_U[3:0]/GBLBSTRDCUT_ES2FE_D[3:0];
    write_cmos_sensor(0x3197, 0x00);    //SENDUM_1U[7:0];
    write_cmos_sensor(0x3198, 0x00);    //SENDUM_1W[7:0];
    write_cmos_sensor(0x3199, 0x00);    //SENDUM_2U[7:0];
    write_cmos_sensor(0x319A, 0x00);    //SENDUM_2W[7:0];
    write_cmos_sensor(0x319B, 0x00);    //SENDUM_3U[7:0];
    write_cmos_sensor(0x319C, 0x00);    //SENDUM_3W[7:0];
    write_cmos_sensor(0x31A8, 0x00);    //-/-/-/ACT_TESTDAC/-/-/-/TESTDACEN;
    write_cmos_sensor(0x31A9, 0xff);    //TDAC_INT[7:0];
    write_cmos_sensor(0x31AA, 0x00);    //TDAC_MIN[7:0];
    write_cmos_sensor(0x31AB, 0x10);    //TDAC_STEP[3:0]/-/-/TDAC_SWD[1:0];
    write_cmos_sensor(0x31AC, 0x00);    //-/-/-/-/-/-/-/AG_TEST;
    write_cmos_sensor(0x31AD, 0x00);    //DACS_INT[7:0];
    write_cmos_sensor(0x31AE, 0xff);    //DACS_MAX[7:0];
    write_cmos_sensor(0x31AF, 0x10);    //DACS_STEP[3:0]/-/-/DACS_SWD[1:0];
    write_cmos_sensor(0x31B0, 0x80);    //TESTDAC_RSVOL[7:0];
    write_cmos_sensor(0x31B1, 0x40);    //TESTDAC_ADVOL[7:0];
    write_cmos_sensor(0x31B5, 0x03);    //-/-/-/-/SINT_ZS_U[3:0];
    write_cmos_sensor(0x31B6, 0x27);    //-/SINT_10B_ZS_W[6:0];
    write_cmos_sensor(0x31B7, 0x00);    //SINT_10B_MZ_U[7:0];
    write_cmos_sensor(0x31B8, 0x3b);    //-/SINT_10B_MZ_W[6:0];
    write_cmos_sensor(0x31B9, 0x1b);    //SINT_NML_RS_U[7:0];
    write_cmos_sensor(0x31BA, 0x3b);    //SINT_10B_RS_W[7:0];
    write_cmos_sensor(0x31BB, 0x11);    //SINT_NML_RF_U[7:0];
    write_cmos_sensor(0x31BC, 0x3b);    //SINT_10B_RF_W[7:0];
    write_cmos_sensor(0x31BD, 0x0c);    //-/-/SINT_FF_U[5:0];
    write_cmos_sensor(0x31BE, 0x0b);    //-/-/SINT_10B_FF_W[5:0];
    write_cmos_sensor(0x31BF, 0x13);    //SINT_FB_U[7:0];
    write_cmos_sensor(0x31C1, 0x27);    //-/SINT_10B_FB_W[6:0];
    write_cmos_sensor(0x31C2, 0x00);    //-/-/-/-/-/-/SINT_10B_AD_U[9:8];
    write_cmos_sensor(0x31C3, 0xa3);    //SINT_10B_AD_U[7:0];
    write_cmos_sensor(0x31C4, 0x00);    //-/-/-/-/-/-/SINT_10B_AD_W[9:8];
    write_cmos_sensor(0x31C5, 0xbb);    //SINT_10B_AD_W[7:0];
    write_cmos_sensor(0x31C6, 0x11);    //-/-/SINTX_USHIFT[1:0]/-/-/SINTX_DSHIFT[1:0];
    write_cmos_sensor(0x31C7, 0x00);    //-/-/-/-/-/-/-/SRST_10B_ZS_U[8];
    write_cmos_sensor(0x31C8, 0x59);    //SRST_10B_ZS_U[7:0];
    write_cmos_sensor(0x31C9, 0x0e);    //-/-/SRST_ZS_W[5:0];
    write_cmos_sensor(0x31CA, 0x00);    //-/-/-/-/-/-/SRST_10B_RS_U[9:8];
    write_cmos_sensor(0x31CB, 0xee);    //SRST_10B_RS_U[7:0];
    write_cmos_sensor(0x31CC, 0x10);    //-/-/SRST_RS_W[5:0];
    write_cmos_sensor(0x31CD, 0x07);    //-/-/-/-/SRST_AD_U[3:0];
    write_cmos_sensor(0x31CE, 0x00);    //-/-/-/-/-/-/-/SRST_10B_AD_D[8];
    write_cmos_sensor(0x31CF, 0x5b);    //SRST_10B_AD_D[7:0];
    write_cmos_sensor(0x31D0, 0x08);    //FIX_VREFSHBGR[1:0]/-/-/EN_VREFC_ZERO/-/-/-;
    write_cmos_sensor(0x31D1, 0x00);    //-/-/-/-/-/-/-/VREFSHBGR_U[8];
    write_cmos_sensor(0x31D2, 0x02);    //VREFSHBGR_U[7:0];
    write_cmos_sensor(0x31D3, 0x07);    //-/-/-/-/VREFSHBGR_D[3:0];
    write_cmos_sensor(0x31D4, 0x01);    //VREF_H_START_U[7:0];
    write_cmos_sensor(0x31D6, 0x00);    //ADCMP1SRT_LOW/-/-/-/-/-/-/-;
    write_cmos_sensor(0x31D7, 0x00);    //ADCMP1SRT_NML_RS_U[7:0];
    write_cmos_sensor(0x31D8, 0x00);    //ADCMP1SRT_NML_AD_U[9:8]/-/-/-/-/-/-;
    write_cmos_sensor(0x31D9, 0x00);    //ADCMP1SRT_NML_AD_U[7:0];
    write_cmos_sensor(0x31DA, 0x15);    //-/ADCMP1SRT_RS_D[6:0];
    write_cmos_sensor(0x31DB, 0x15);    //-/ADCMP1SRT_AD_D[6:0];
    write_cmos_sensor(0x31DC, 0xe0);    //CDS_STOPBST/CDS_CMP_BSTOFF/CDS_ADC_BSTOFF/CDS_ADC_BSTOFF_TEST/BSTCKLFIX_HIGH/-/BSTCKLFIX_MERGE/-;
    write_cmos_sensor(0x31DD, 0x10);    //-/BSTCKLFIX_CMP_U[6:0];
    write_cmos_sensor(0x31DE, 0x0e);    //-/BSTCKLFIX_CMP_D[6:0];
    write_cmos_sensor(0x31DF, 0x09);    //-/-/BSTCKLFIX_RS_U[5:0];
    write_cmos_sensor(0x31E0, 0x01);    //-/-/-/BSTCKLFIX_RS_D[4:0];
    write_cmos_sensor(0x31E1, 0x09);    //-/-/BSTCKLFIX_AD_U[5:0];
    write_cmos_sensor(0x31E2, 0x01);    //-/-/-/BSTCKLFIX_AD_D[4:0];
    write_cmos_sensor(0x3200, 0x12);    //-/-/-/BK_ON/-/-/BK_HOKAN_MODE/DANSA_ON;
    write_cmos_sensor(0x3201, 0x12);    //-/-/BK_START_LINE[1:0]/-/-/BK_MODE[1:0];
    write_cmos_sensor(0x3203, 0x00);    //BK_FRAME[7:0];
    write_cmos_sensor(0x3204, 0x00);    //-/-/-/BK_LEV_AGDELTA[4:0];
    write_cmos_sensor(0x3205, 0x80);    //BK_LEVEL_ALL[7:0];
    write_cmos_sensor(0x3206, 0x00);    //-/-/-/-/BK_LEVEL_COL1[11:8];
    write_cmos_sensor(0x3207, 0x00);    //BK_LEVEL_COL1[7:0];
    write_cmos_sensor(0x3208, 0x00);    //-/-/-/-/BK_LEVEL_COL2[11:8];
    write_cmos_sensor(0x3209, 0x00);    //BK_LEVEL_COL2[7:0];
    write_cmos_sensor(0x320A, 0x00);    //-/-/-/-/BK_LEVEL_COL3[11:8];
    write_cmos_sensor(0x320B, 0x00);    //BK_LEVEL_COL3[7:0];
    write_cmos_sensor(0x320C, 0x00);    //-/-/-/-/BK_LEVEL_COL4[11:8];
    write_cmos_sensor(0x320D, 0x00);    //BK_LEVEL_COL4[7:0];
    write_cmos_sensor(0x320E, 0x00);    //-/-/-/-/BK_LEVEL_COL5[11:8];
    write_cmos_sensor(0x320F, 0x00);    //BK_LEVEL_COL5[7:0];
    write_cmos_sensor(0x3210, 0x00);    //-/-/-/-/BK_LEVEL_COL6[11:8];
    write_cmos_sensor(0x3211, 0x00);    //BK_LEVEL_COL6[7:0];
    write_cmos_sensor(0x3212, 0x00);    //-/-/-/-/BK_LEVEL_COL7[11:8];
    write_cmos_sensor(0x3213, 0x00);    //BK_LEVEL_COL7[7:0];
    write_cmos_sensor(0x3214, 0x00);    //-/-/-/-/BK_LEVEL_COL8[11:8];
    write_cmos_sensor(0x3215, 0x00);    //BK_LEVEL_COL8[7:0];
    write_cmos_sensor(0x3216, 0x00);    //-/BK_MPY_COL1[6:0];
    write_cmos_sensor(0x3217, 0x00);    //-/BK_MPY_COL2[6:0];
    write_cmos_sensor(0x3218, 0x00);    //-/BK_MPY_COL3[6:0];
    write_cmos_sensor(0x3219, 0x00);    //-/BK_MPY_COL4[6:0];
    write_cmos_sensor(0x321A, 0x00);    //-/BK_MPY_COL5[6:0];
    write_cmos_sensor(0x321B, 0x00);    //-/BK_MPY_COL6[6:0];
    write_cmos_sensor(0x321C, 0x00);    //-/BK_MPY_COL7[6:0];
    write_cmos_sensor(0x321D, 0x00);    //-/BK_MPY_COL8[6:0];
    write_cmos_sensor(0x321E, 0x0c);    //-/-/-/-/AG_LEVEL_MAX[3:0];
    write_cmos_sensor(0x321F, 0x84);    //AG_LEVEL_MID[3:0]/AG_LEVEL_MIN[3:0];
    write_cmos_sensor(0x3220, 0x00);    //-/-/-/VOB_DISP/-/-/-/HOB_DISP;
    write_cmos_sensor(0x3221, 0x00);    //-/MPY_LIPOL/MPY_CSPOL/MPY_2LPOL/-/LVL_LIPOL/LVL_CSPOL/LVL_2LPOL;
    write_cmos_sensor(0x3222, 0x00);    //BK_TEST_FCNT/TEST_CUP/-/CB_VALID_SEL/-/-/TEST_CHART_H/TEST_CHART_W;
    write_cmos_sensor(0x3223, 0x03);    //-/-/VE_CURS_TEMP/VE_CURS_SCRL_MAX/-/-/VE_CURS_SPEED[1:0];
    write_cmos_sensor(0x3224, 0x20);    //-/AGMAX_OB_REFLEV[6:0];
    write_cmos_sensor(0x3225, 0x20);    //-/AGMIN_OB_REFLEV[6:0];
    write_cmos_sensor(0x3226, 0x20);    //-/-/AGMAX_OB_WIDTH[5:0];
    write_cmos_sensor(0x3227, 0x18);    //-/-/AGMIN_OB_WIDTH[5:0];
    write_cmos_sensor(0x3230, 0x00);    //-/-/-/-/-/-/DAMP_LIPOL/DAMP_CSPOL;
    write_cmos_sensor(0x3231, 0x00);    //PWB_RG[7:0];  0x23  jackymao
    write_cmos_sensor(0x3232, 0x00);    //PWB_GRG[7:0];
    write_cmos_sensor(0x3233, 0x00);    //PWB_GBG[7:0];
    write_cmos_sensor(0x3234, 0x00);    //PWB_BG[7:0];  0x44 jackymao
    write_cmos_sensor(0x3235, 0x80);    //DAMP_BLACK[7:0];
    write_cmos_sensor(0x3237, 0x00);    //LSSC_EN/-/-/TEST_LSSC/LSSC_DISP/-/LSSC_LIPOL/LSSC_CSPOL;  0x80 jackymao
    write_cmos_sensor(0x3238, 0x00);    //LSSC_HCNT_ADJ[7:0];
    write_cmos_sensor(0x3239, 0x80);    //LSSC_HCNT_MPY[7:0];
    write_cmos_sensor(0x323A, 0x80);    //LSSC_HCEN_ADJ[7:0];
    write_cmos_sensor(0x323B, 0x00);    //LSSC_VCNT_ADJ[7:0];
    write_cmos_sensor(0x323C, 0x81);    //LSSC_VCNT_MPYSW/-/-/-/LSSC_VCNT_MPY[11:8];
    write_cmos_sensor(0x323D, 0x00);    //LSSC_VCNT_MPY[7:0];
    write_cmos_sensor(0x323E, 0x02);    //LSSC_VCEN_WIDTH/-/-/-/-/-/LSSC_VCEN_ADJ[9:8];
    write_cmos_sensor(0x323F, 0x00);    //LSSC_VCEN_ADJ[7:0];
    write_cmos_sensor(0x3240, 0x00);    //LSSC_TOPL_PM1RG[7:0];
    write_cmos_sensor(0x3241, 0x00);    //LSSC_TOPL_PM1GRG[7:0];
    write_cmos_sensor(0x3242, 0x00);    //LSSC_TOPL_PM1GBG[7:0];
    write_cmos_sensor(0x3243, 0x00);    //LSSC_TOPL_PM1BG[7:0];
    write_cmos_sensor(0x3244, 0x00);    //LSSC_TOPR_PM1RG[7:0];
    write_cmos_sensor(0x3245, 0x00);    //LSSC_TOPR_PM1GRG[7:0];
    write_cmos_sensor(0x3246, 0x00);    //LSSC_TOPR_PM1GBG[7:0];
    write_cmos_sensor(0x3247, 0x00);    //LSSC_TOPR_PM1BG[7:0];
    write_cmos_sensor(0x3248, 0x00);    //LSSC_BOTL_PM1RG[7:0];
    write_cmos_sensor(0x3249, 0x00);    //LSSC_BOTL_PM1GRG[7:0];
    write_cmos_sensor(0x324A, 0x00);    //LSSC_BOTL_PM1GBG[7:0];
    write_cmos_sensor(0x324B, 0x00);    //LSSC_BOTL_PM1BG[7:0];
    write_cmos_sensor(0x324C, 0x00);    //LSSC_BOTR_PM1RG[7:0];
    write_cmos_sensor(0x324D, 0x00);    //LSSC_BOTR_PM1GRG[7:0];
    write_cmos_sensor(0x324E, 0x00);    //LSSC_BOTR_PM1GBG[7:0];
    write_cmos_sensor(0x324F, 0x00);    //LSSC_BOTR_PM1BG[7:0];
    write_cmos_sensor(0x3250, 0x00);    //-/-/-/-/LSSC1BG_PMSW/LSSC1GBG_PMSW/LSSC1GRG_PMSW/LSSC1RG_PMSW;
    write_cmos_sensor(0x3251, 0x57);    //LSSC_LEFT_P2RG[7:0];
    write_cmos_sensor(0x3252, 0x70);    //LSSC_LEFT_P2GRG[7:0];
    write_cmos_sensor(0x3253, 0x70);    //LSSC_LEFT_P2GBG[7:0];
    write_cmos_sensor(0x3254, 0x70);    //LSSC_LEFT_P2BG[7:0];
    write_cmos_sensor(0x3255, 0x65);    //LSSC_RIGHT_P2RG[7:0];
    write_cmos_sensor(0x3256, 0x70);    //LSSC_RIGHT_P2GRG[7:0];
    write_cmos_sensor(0x3257, 0x70);    //LSSC_RIGHT_P2GBG[7:0];
    write_cmos_sensor(0x3258, 0x70);    //LSSC_RIGHT_P2BG[7:0];
    write_cmos_sensor(0x3259, 0x51);    //LSSC_TOP_P2RG[7:0];
    write_cmos_sensor(0x325A, 0x70);    //LSSC_TOP_P2GRG[7:0];
    write_cmos_sensor(0x325B, 0x70);    //LSSC_TOP_P2GBG[7:0];
    write_cmos_sensor(0x325C, 0x64);    //LSSC_TOP_P2BG[7:0];
    write_cmos_sensor(0x325D, 0x3f);    //LSSC_BOTTOM_P2RG[7:0];
    write_cmos_sensor(0x325E, 0x70);    //LSSC_BOTTOM_P2GRG[7:0];
    write_cmos_sensor(0x325F, 0x70);    //LSSC_BOTTOM_P2GBG[7:0];
    write_cmos_sensor(0x3260, 0x57);    //LSSC_BOTTOM_P2BG[7:0];
    write_cmos_sensor(0x3261, 0x00);    //LSSC_LEFT_PM4RG[7:0];
    write_cmos_sensor(0x3262, 0x00);    //LSSC_LEFT_PM4GRG[7:0];
    write_cmos_sensor(0x3263, 0x00);    //LSSC_LEFT_PM4GBG[7:0];
    write_cmos_sensor(0x3264, 0x00);    //LSSC_LEFT_PM4BG[7:0];
    write_cmos_sensor(0x3265, 0x40);    //LSSC_RIGHT_PM4RG[7:0];
    write_cmos_sensor(0x3266, 0x00);    //LSSC_RIGHT_PM4GRG[7:0];
    write_cmos_sensor(0x3267, 0x00);    //LSSC_RIGHT_PM4GBG[7:0];
    write_cmos_sensor(0x3268, 0x24);    //LSSC_RIGHT_PM4BG[7:0];
    write_cmos_sensor(0x3269, 0x1c);    //LSSC_TOP_PM4RG[7:0];
    write_cmos_sensor(0x326A, 0x00);    //LSSC_TOP_PM4GRG[7:0];
    write_cmos_sensor(0x326B, 0x00);    //LSSC_TOP_PM4GBG[7:0];
    write_cmos_sensor(0x326C, 0x24);    //LSSC_TOP_PM4BG[7:0];
    write_cmos_sensor(0x326D, 0x40);    //LSSC_BOTTOM_PM4RG[7:0];
    write_cmos_sensor(0x326E, 0x00);    //LSSC_BOTTOM_PM4GRG[7:0];
    write_cmos_sensor(0x326F, 0x00);    //LSSC_BOTTOM_PM4GBG[7:0];
    write_cmos_sensor(0x3270, 0x00);    //LSSC_BOTTOM_PM4BG[7:0];
    write_cmos_sensor(0x3271, 0x80);    //LSSC_MGSEL[1:0]/-/-/LSSC4BG_PMSW/LSSC4GBG_PMSW/LSSC4GRG_PMSW/LSSC4RG_PMSW;;
    write_cmos_sensor(0x3272, 0x00);    //-/-/-/-/-/-/-/LSSC_BLACK[8];
    write_cmos_sensor(0x3273, 0x80);    //LSSC_BLACK[7:0];
    write_cmos_sensor(0x3274, 0x01);    //-/-/-/-/-/-/-/OBOFFSET_MPY_LSSC;
    write_cmos_sensor(0x3275, 0x00);    //-/-/-/-/-/-/-/LSSC_HGINV;
    write_cmos_sensor(0x3276, 0x00);    //-/-/-/-/-/-/-/LSSC_TEST_STRK;
    write_cmos_sensor(0x3282, 0xc0);    //ABPC_EN/ABPC_CK_EN/-/-/-/-/-/-;
    write_cmos_sensor(0x3284, 0x06);    //-/-/-/DM_L_GAIN_WH[4:0];
    write_cmos_sensor(0x3285, 0x03);    //-/-/-/DM_M_GAIN_WH[4:0];
    write_cmos_sensor(0x3286, 0x02);    //-/-/-/DM_H_GAIN_WH[4:0];
    write_cmos_sensor(0x3287, 0x40);    //DM_L_THRS_WH[7:0];
    write_cmos_sensor(0x3288, 0x80);    //DM_H_THRS_WH[7:0];
    write_cmos_sensor(0x3289, 0x06);    //-/-/-/DM_L_GAIN_BK[4:0];
    write_cmos_sensor(0x328A, 0x03);    //-/-/-/DM_M_GAIN_BK[4:0];
    write_cmos_sensor(0x328B, 0x02);    //-/-/-/DM_H_GAIN_BK[4:0];
    write_cmos_sensor(0x328C, 0x40);    //DM_L_THRS_BK[7:0];
    write_cmos_sensor(0x328D, 0x80);    //DM_H_THRS_BK[7:0];
    write_cmos_sensor(0x328E, 0x13);    //-/-/DARK_LIMIT[5:0];
    write_cmos_sensor(0x328F, 0x80);    //ABPC_M_THRS[7:0];
    write_cmos_sensor(0x3290, 0x20);    //ABPC_DSEL/ABPC_MSEL/ABPC_RSEL/USE_ABPC_AVE/TEST_ABPC/-/-/TEST_PPSRAM_ON;
    write_cmos_sensor(0x3291, 0x00);    //-/-/-/-/-/-/ABPC_VBLK_SEL/ABPC_TEST_STRK;
    write_cmos_sensor(0x3294, 0x10);    //-/-/-/MAP_MEMLESS/-/-/TEST_MAP_DISP_H/HDR_WCLIP_ON;
    write_cmos_sensor(0x3295, 0x20);    //TEST_WCLIP_ON/HDR_WCLIP_W[6:0];
    write_cmos_sensor(0x3296, 0x10);    //-/-/-/DFCT_XADJ[4:0];
    write_cmos_sensor(0x3297, 0x00);    //-/-/-/-/-/DFCT_YDLY_VAL[1:0]/DFCT_YDLY_SW;
    write_cmos_sensor(0x329E, 0x00);    //-/-/-/-/-/-/-/MEMIF_RWPOL;
    write_cmos_sensor(0x329F, 0x04);    //-/-/-/-/-/MEMIF_MAX_ADR1[10:8];
    write_cmos_sensor(0x32A0, 0x3e);    //MEMIF_MAX_ADR1[7:0];
    write_cmos_sensor(0x32A1, 0x02);    //-/-/-/-/-/MEMIF_MAX_ADR2[10:8];
    write_cmos_sensor(0x32A2, 0x1f);    //MEMIF_MAX_ADR2[7:0];
    write_cmos_sensor(0x32A8, 0x84);    //CNR_SW/-/-/-/-/CNR_USE_FB/CNR_USE_FB_1ST/CNR_USE_FB_ABF;
    write_cmos_sensor(0x32A9, 0x02);    //-/-/-/-/-/-/CNR_UPDATE_FB/CNR_UPDATE_FB_MASK;
    write_cmos_sensor(0x32AB, 0x00);    //-/-/-/-/-/CNR_ABF_DGR[2:0];
    write_cmos_sensor(0x32AC, 0x00);    //-/-/-/-/-/CNR_ABF_DGG[2:0];
    write_cmos_sensor(0x32AD, 0x00);    //-/-/-/-/-/CNR_ABF_DGB[2:0];
    write_cmos_sensor(0x32AF, 0x00);    //-/-/CNR_CNF_DARK_AG0[5:0];
    write_cmos_sensor(0x32B0, 0x00);    //-/-/CNR_CNF_DARK_AG1[5:0];
    write_cmos_sensor(0x32B1, 0x00);    //-/-/CNR_CNF_DARK_AG2[5:0];
    write_cmos_sensor(0x32B3, 0x10);    //-/-/-/CNR_CNF_RATIO_AG0[4:0];
    write_cmos_sensor(0x32B4, 0x1f);    //-/-/-/CNR_CNF_RATIO_AG1[4:0];
    write_cmos_sensor(0x32B5, 0x1f);    //-/-/-/CNR_CNF_RATIO_AG2[4:0];
    write_cmos_sensor(0x32B7, 0x3b);    //-/-/CNR_CNF_CLIP_GAIN_R[1:0]/CNR_CNF_CLIP_GAIN_G[1:0]/CNR_CNF_CLIP_GAIN_B[1:0];
    write_cmos_sensor(0x32B8, 0xff);    //CNR_CNF_DITHER_LEVEL[7:0];
    write_cmos_sensor(0x32BA, 0x04);    //-/-/CNR_A1L_DARK_AG0[5:0];
    write_cmos_sensor(0x32BB, 0x0f);    //-/-/CNR_A1L_DARK_AG1[5:0];
    write_cmos_sensor(0x32BC, 0x0f);    //-/-/CNR_A1L_DARK_AG2[5:0];
    write_cmos_sensor(0x32BE, 0x04);    //-/-/-/CNR_A1L_RATIO_AG0[4:0];
    write_cmos_sensor(0x32BF, 0x0f);    //-/-/-/CNR_A1L_RATIO_AG1[4:0];
    write_cmos_sensor(0x32C0, 0x0f);    //-/-/-/CNR_A1L_RATIO_AG2[4:0]; 
    write_cmos_sensor(0x32C1, 0x00);    //-/CNR_INF_ZERO_CLIP_AG0[6:0];
    write_cmos_sensor(0x32C2, 0x00);    //-/CNR_INF_ZERO_CLIP_AG1[6:0];
    write_cmos_sensor(0x32C3, 0x00);    //-/CNR_INF_ZERO_CLIP_AG2[6:0];
    write_cmos_sensor(0x32C5, 0x01);    //CNR_TOUT_SEL[3:0]/-/-/CNR_CNF_SHIFT[1:0];
    write_cmos_sensor(0x32C6, 0x50);    //-/CNR_DCOR_SW[2:0]/-/-/-/CNR_MERGE_YLSEL;
    write_cmos_sensor(0x32C8, 0x0e);    //-/-/-/CNR_MERGE_HLBLEND_MIN_AG0[4:0];
    write_cmos_sensor(0x32C9, 0x0e);    //-/-/-/CNR_MERGE_HLBLEND_MIN_AG1[4:0];
    write_cmos_sensor(0x32CA, 0x0e);    //-/-/-/CNR_MERGE_HLBLEND_MIN_AG2[4:0];
    write_cmos_sensor(0x32CB, 0x0e);    //-/-/-/CNR_MERGE_HLBLEND_MAX_AG0[4:0];
    write_cmos_sensor(0x32CC, 0x0e);    //-/-/-/CNR_MERGE_HLBLEND_MAX_AG1[4:0];
    write_cmos_sensor(0x32CD, 0x0e);    //-/-/-/CNR_MERGE_HLBLEND_MAX_AG2[4:0];
    write_cmos_sensor(0x32CE, 0x08);    //-/-/-/-/CNR_MERGE_HLBLEND_DARK_AG0[3:0];
    write_cmos_sensor(0x32CF, 0x08);    //-/-/-/-/CNR_MERGE_HLBLEND_DARK_AG1[3:0];
    write_cmos_sensor(0x32D0, 0x08);    //-/-/-/-/CNR_MERGE_HLBLEND_DARK_AG2[3:0];
    write_cmos_sensor(0x32D1, 0x0f);    //-/-/-/-/CNR_MERGE_HLBLEND_RATIO_AG0[3:0];
    write_cmos_sensor(0x32D2, 0x0f);    //-/-/-/-/CNR_MERGE_HLBLEND_RATIO_AG1[3:0];
    write_cmos_sensor(0x32D3, 0x0f);    //-/-/-/-/CNR_MERGE_HLBLEND_RATIO_AG2[3:0];
    write_cmos_sensor(0x32D4, 0x08);    //CNR_MERGE_HLBLEND_SLOPE_AG0[7:0];
    write_cmos_sensor(0x32D5, 0x08);    //CNR_MERGE_HLBLEND_SLOPE_AG1[7:0];
    write_cmos_sensor(0x32D6, 0x08);    //CNR_MERGE_HLBLEND_SLOPE_AG2[7:0];
    write_cmos_sensor(0x32D8, 0x00);    //-/-/-/CNR_MERGE_D2BLEND_AG0[4:0];
    write_cmos_sensor(0x32D9, 0x00);    //-/-/-/CNR_MERGE_D2BLEND_AG1[4:0];
    write_cmos_sensor(0x32DA, 0x00);    //-/-/-/CNR_MERGE_D2BLEND_AG2[4:0];
    write_cmos_sensor(0x32DD, 0x02);    //-/-/CNR_MERGE_MINDIV_AG0[5:0];
    write_cmos_sensor(0x32DE, 0x04);    //-/-/CNR_MERGE_MINDIV_AG1[5:0];
    write_cmos_sensor(0x32DF, 0x04);    //-/-/CNR_MERGE_MINDIV_AG2[5:0];
    write_cmos_sensor(0x32E0, 0x00);    //-/CNR_MERGE_BLACK_AG0[6:0];
    write_cmos_sensor(0x32E1, 0x00);    //-/CNR_MERGE_BLACK_AG1[6:0];
    write_cmos_sensor(0x32E2, 0x00);    //-/CNR_MERGE_BLACK_AG2[6:0];
    write_cmos_sensor(0x32E4, 0x00);    //-/-/CNR_ABCY_DARK_AG0[5:0];
    write_cmos_sensor(0x32E5, 0x00);    //-/-/CNR_ABCY_DARK_AG1[5:0];
    write_cmos_sensor(0x32E6, 0x00);    //-/-/CNR_ABCY_DARK_AG2[5:0];
    write_cmos_sensor(0x32E7, 0x00);    //-/-/-/-/CNR_ABCY_RATIO_AG0[3:0];
    write_cmos_sensor(0x32E8, 0x00);    //-/-/-/-/CNR_ABCY_RATIO_AG1[3:0];
    write_cmos_sensor(0x32E9, 0x00);    //-/-/-/-/CNR_ABCY_RATIO_AG2[3:0];
    write_cmos_sensor(0x32EA, 0x00);    //-/-/-/-/-/CNR_ABCY_WEIGHT_AG0[2:0];
    write_cmos_sensor(0x32EB, 0x00);    //-/-/-/-/-/CNR_ABCY_WEIGHT_AG1[2:0];
    write_cmos_sensor(0x32EC, 0x00);    //-/-/-/-/-/CNR_ABCY_WEIGHT_AG2[2:0];
    write_cmos_sensor(0x32ED, 0x00);    //-/-/-/-/-/-/-/CNR_USE_FIXED_SIZE;
    write_cmos_sensor(0x32EE, 0x00);    //-/-/-/CNR_IMAGE_WIDTH[12:8];
    write_cmos_sensor(0x32EF, 0x00);    //CNR_IMAGE_WIDTH[7:0];
    write_cmos_sensor(0x32F0, 0x00);    //-/-/-/-/CNR_IMAGE_HEIGHT[11:8];
    write_cmos_sensor(0x32F1, 0x00);    //CNR_IMAGE_HEIGHT[7:0];
    write_cmos_sensor(0x32F4, 0x03);    //-/-/-/-/-/-/PDS_SIGSEL[1:0];
    write_cmos_sensor(0x32F5, 0x80);    //AGMAX_PED[7:0];
    write_cmos_sensor(0x32F6, 0x80);    //AGMIN_PED[7:0];
    write_cmos_sensor(0x32F7, 0x00);    //-/-/-/-/-/-/-/PP_DCROP_SW;
    write_cmos_sensor(0x32F8, 0x03);    //-/PP_HCNT_MGN[2:0]/-/PP_VCNT_MGN[2:0];
    write_cmos_sensor(0x32F9, 0x03);    //-/-/-/-/-/PP_VPIX_MGN[2:0];
    write_cmos_sensor(0x32FA, 0x01);    //-/-/-/-/-/PP_VBLK_SEL[2:0];
    write_cmos_sensor(0x32FD, 0xf0);    //PP_RESV[7:0];
    write_cmos_sensor(0x32FE, 0x00);    //PP_RESV_A[7:0];
    write_cmos_sensor(0x32FF, 0x00);    //PP_RESV_B[7:0];
    write_cmos_sensor(0x3300, 0x0a);    //-/-/-/-/WKUP_WAIT_ON/VCO_CONV[2:0];
    write_cmos_sensor(0x3301, 0x05);    //ES_MARGIN[7:0];
    write_cmos_sensor(0x3302, 0x00);    //-/-/-/-/-/SP_COUNT[10:8];
    write_cmos_sensor(0x3303, 0x70);    //SP_COUNT[7:0];
    write_cmos_sensor(0x3304, 0x00);    //-/-/-/-/-/HREG_HRST_POS[10:8];
    write_cmos_sensor(0x3305, 0x00);    //HREG_HRST_POS[7:0];
    write_cmos_sensor(0x3307, 0x45);    //AG_MIN[7:0];
    write_cmos_sensor(0x3308, 0x44);    //AG_MAX[7:0];
    write_cmos_sensor(0x3309, 0x0d);    //-/-/-/-/AGCHG_BK_PERIOD[1:0]/-/INCLUDE_ESCHG;
    write_cmos_sensor(0x330A, 0x01);    //-/-/-/-/-/-/BIN_MODE[1:0];
    write_cmos_sensor(0x330B, 0x01);    //-/-/-/-/-/-/VLAT_GLBFRM_RO/VBLK_RGIF_2H;
    write_cmos_sensor(0x330C, 0x02);    //-/-/-/-/GRST_ALLZEROSET_NUM[3:0];
    write_cmos_sensor(0x330D, 0x00);    //-/-/-/-/-/HFV_ES_REF[2:0];
    write_cmos_sensor(0x330E, 0x00);    //-/-/-/-/-/-/-/USE_OP_FRM_END;
    write_cmos_sensor(0x3310, 0x03);    //-/-/-/-/-/ESYNC_SW/VSYNC_PH/HSYNC_PH;
    write_cmos_sensor(0x3311, 0x05);    //-/HC_PRESET[14:8];
    write_cmos_sensor(0x3312, 0x42);    //HC_PRESET[7:0];
    write_cmos_sensor(0x3313, 0x00);    //VC_PRESET[15:8];
    write_cmos_sensor(0x3314, 0x00);    //VC_PRESET[7:0];
    write_cmos_sensor(0x3316, 0x00);    //TEST_TG_MODE[7:0];
    write_cmos_sensor(0x3318, 0x40);    //LTCH_CYCLE[1:0]/-/-/-/-/-/-;
    write_cmos_sensor(0x3319, 0x0a);    //-/-/-/V_ADR_ADJ[4:0];
    write_cmos_sensor(0x3380, 0x00);    //-/-/-/TEST_SCCLK_ON/-/-/TEST_ABCLK_ON/TEST_PPCLK_ON;
    write_cmos_sensor(0x3381, 0x01);    //-/-/-/-/-/AD_CNTL[2:0];
    write_cmos_sensor(0x3383, 0x08);    //-/-/-/-/ST_CNTL[3:0];
    write_cmos_sensor(0x3384, 0x00);    //-/-/-/VTCK_PLLSEL/-/-/-/PLL_SINGLE_SW;
    write_cmos_sensor(0x338B, 0x00);    //-/-/-/-/-/-/-/REGVD_SEL;
    write_cmos_sensor(0x338C, 0x05);    //-/-/-/-/BST_CNTL[3:0];
    write_cmos_sensor(0x338D, 0x03);    //PLL_SNR_CNTL[1:0]/PLL_SYS_CNTL[1:0]/-/VCO_EN_SEL/VCO_EN/DIVRSTX;
    write_cmos_sensor(0x338E, 0x00);    //-/ICP_SEL[1:0]/LPFR_SEL[1:0]/AMON0_SEL[2:0];
    write_cmos_sensor(0x338F, 0x20);    //PCMODE/-/ICP_PCH/ICP_NCH/-/-/-/VCO_TESTSEL;
    write_cmos_sensor(0x3390, 0x00);    //-/-/AUTO_ICP_R_SEL_LG/AUTO_ICP_R_SEL_ST/-/-/PLLEV_EN/PLLEV_SEL;
    write_cmos_sensor(0x3391, 0x00);    //-/-/-/-/-/-/-/ADCK_DIV2_CNTL;
    write_cmos_sensor(0x339B, 0x00);    //-/-/-/-/-/SC_MPYOFFSET[2:0];
    write_cmos_sensor(0x339C, 0x00);    //IPHASE_DIR_INPUT/IPHASE_EVN[6:0];
    write_cmos_sensor(0x339D, 0x00);    //-/IPHASE_ODD[6:0];
    write_cmos_sensor(0x33B2, 0x00);    //-/-/-/-/-/-/-/PISO_MSKN;
    write_cmos_sensor(0x33B3, 0x00);    //SLEEP_SW/VCO_STP_SW/PHY_PWRON_SW/SLEEP_CKSP_SW/SLEEP_MN/VCO_STP_MN/PHY_PWRON_MN/SLEEP_CKSP_MN;
    write_cmos_sensor(0x33B4, 0x01);    //-/-/-/-/-/-/-/SMIAPP_SW;
    write_cmos_sensor(0x33B5, 0x03);    //-/-/-/-/-/-/CLR_TRG/B_FLS_MSKGPH;
    write_cmos_sensor(0x33B6, 0x00);    //SENSTP_SW/-/-/-/SENSTP_MN/-/-/-;
    write_cmos_sensor(0x33D0, 0x00);    //-/-/DRVUP[5:0];
    write_cmos_sensor(0x33E0, 0x00);    //-/-/-/DCLK_POL/-/PARA_SW[2:0];
    write_cmos_sensor(0x33E1, 0x01);    //-/-/-/STBIO_HZ/-/-/-/PARA_HZ;
    write_cmos_sensor(0x33E2, 0x00);    //-/-/OPCK_SEL[1:0]/-/-/VTCK_SEL[1:0];
    write_cmos_sensor(0x33E4, 0x00);    //-/-/-/-/TM_MODE_SEL[3:0];
    write_cmos_sensor(0x33E5, 0x00);    //-/-/-/-/-/-/-/PLL_BIST_CK_EN;
    write_cmos_sensor(0x33F1, 0x00);    //-/-/-/-/REG18_CNT[3:0];
    write_cmos_sensor(0x33FF, 0x00);    //RG_RSVD_REG[7:0];
    write_cmos_sensor(0x3405, 0x00);    //-/-/SC_TEST[1:0]/EMB_OUT_SW/HFCORROFF/EQ_MONI[1:0];
    write_cmos_sensor(0x3420, 0x00);    //-/-/B_LB_LANE_SEL[1:0]/B_LBTEST_CLR/B_LB_TEST_EN/-/B_LB_MODE;
    write_cmos_sensor(0x3424, 0x00);    //-/-/-/-/B_TRIG_Z5_X/B_TX_TRIGOPT/B_CLKULPS/B_ESCREQ;
    write_cmos_sensor(0x3425, 0x78);    //B_ESCDATA[7:0];
    write_cmos_sensor(0x3426, 0xff);    //B_TRIG_DUMMY[7:0];
    write_cmos_sensor(0x3427, 0xc0);    //B_MIPI_CLKVBLK/B_MIPI_CLK_MODE/-/-/B_HS_SR_CNT[1:0]/B_LP_SR_CNT[1:0];
    write_cmos_sensor(0x3428, 0x00);    //-/-/-/-/B_LANE_OFF[3:0];
    write_cmos_sensor(0x3429, 0x40);    //-/B_PHASE_ADJ[2:0]/-/-/-/-;
    write_cmos_sensor(0x342A, 0x00);    //-/B_CK_DELAY[2:0]/-/-/-/-;
    write_cmos_sensor(0x342B, 0x00);    //-/B_D1_DELAY[2:0]/-/B_D2_DELAY[2:0];
    write_cmos_sensor(0x342C, 0x00);    //-/B_D3_DELAY[2:0]/-/B_D4_DELAY[2:0];
    write_cmos_sensor(0x342D, 0x00);    //-/-/-/B_READ_TRIG/-/-/B_EN_PHASE_SEL[1:0];
    write_cmos_sensor(0x342E, 0x00);    //-/-/-/-/-/-/B_FIFODLY[9:8];
    write_cmos_sensor(0x342F, 0x00);    //B_FIFODLY[7:0];
    write_cmos_sensor(0x3430, 0xa7);    //B_NUMWAKE[7:0];
    write_cmos_sensor(0x3431, 0x60);    //B_NUMINIT[7:0];
    write_cmos_sensor(0x3432, 0x11);    //-/-/-/B_CLK0_M/-/-/-/B_LNKBTWK_ON;
    write_cmos_sensor(0x3433, 0x00);    //-/-/-/-/B_OP_TEST[3:0];
    write_cmos_sensor(0x3434, 0x00);    //B_T_VALUE1[7:0];
    write_cmos_sensor(0x3435, 0x00);    //B_T_VALUE2[7:0];
    write_cmos_sensor(0x3436, 0x00);    //B_T_VALUE3[7:0];
    write_cmos_sensor(0x3437, 0x00);    //B_T_VALUE4[7:0];
    write_cmos_sensor(0x3438, 0x00);    //-/-/-/-/-/B_MIPI_LANE_SEL[2:0];
    write_cmos_sensor(0x3439, 0x00);    //B_TLPX_LINKOFF/-/B_TCLK_ZERO_LINKOFF/B_TCLK_PREPARE_LINKOFF/B_TCLK_TRAIL_LINKOFF/B_THS_TRAIL_LINKOFF/B_THS_ZERO_LINKOFF/B_THS_PREPARE_LINKOFF;
    write_cmos_sensor(0x343A, 0x00);    //-/-/-/-/-/-/-/B_HFVCOUNT_MODE;
    //write_cmos_sensor(0x0100, 0x01);    //-/-/-/-/-/-/-/MODE_SELECT;
}	/*	sensor_init  */


static void preview_setting(void)
{
	//5.1.2 FQPreview 1640x1232 30fps 24M MCLK 4lane 256Mbps/lane
	LOG_INF("E\n");
	//write_cmos_sensor(0x0100,0x00);
		
	//write_cmos_sensor(0x0101,0x03);
	write_cmos_sensor(0x0103,0x00);
	write_cmos_sensor(0x0104,0x01);// -/-/-/-/-/-/-/GROUP_PARA_HOLD;////20160407  fan
	write_cmos_sensor(0x0340,0x06);// FR_LENGTH_LINES[15:8];
	write_cmos_sensor(0x0341,0x2C);// FR_LENGTH_LINES[7:0];
	write_cmos_sensor(0x0342,0x11);// LINE_LENGTH_PCK[15:8];
	write_cmos_sensor(0x0343,0x80);// LINE_LENGTH_PCK[7:0];
	write_cmos_sensor(0x0346,0x00);// Y_ADDR_START[15:8];
	write_cmos_sensor(0x0347,0x00);// Y_ADDR_START[7:0];
	write_cmos_sensor(0x034A,0x0C);// Y_ADDR_END[15:8];
	write_cmos_sensor(0x034B,0x2F);// Y_ADDR_END[7:0];
	write_cmos_sensor(0x034C,0x08);// X_OUTPUT_SIZE[15:8];//2104
	write_cmos_sensor(0x034D,0x38);// X_OUTPUT_SIZE[7:0];
	write_cmos_sensor(0x034E,0x06);// Y_OUTPUT_SIZE[15:8];//1560
	write_cmos_sensor(0x034F,0x18);// Y_OUTPUT_SIZE[7:0];
	write_cmos_sensor(0x0401,0x01);// -/-/-/-/-/-/SCALING_MODE[1:0];
	write_cmos_sensor(0x0404,0x20);// SCALE_M[7:0];
	write_cmos_sensor(0x0900,0x00);// -/-/-/-/-/-/H_BIN[1:0];
	write_cmos_sensor(0x0901,0x01);// -/-/-/-/-/-/V_BIN_MODE[1:0];
	write_cmos_sensor(0x0104,0x00);// -/-/-/-/-/-/-/GROUP_PARA_HOLD;////20160407  fan
	write_cmos_sensor(0x0100,0x01);
// The register only need to enable 1 time.     
}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
	if (currefps == 240) { //24fps for PIP
		//@@full_132PCLK_24.75
	write_cmos_sensor(0x0104,0x01);// -/-/-/-/-/-/-/GROUP_PARA_HOLD;
	write_cmos_sensor(0x0340,0x0C);// FR_LENGTH_LINES[15:8];//3144
	write_cmos_sensor(0x0341,0x48);// FR_LENGTH_LINES[7:0];
	write_cmos_sensor(0x0342,0x11);// LINE_LENGTH_PCK[15:8];//4480
	write_cmos_sensor(0x0343,0x80);// LINE_LENGTH_PCK[7:0];
	write_cmos_sensor(0x0346,0x00);// Y_ADDR_START[15:8];
	write_cmos_sensor(0x0347,0x00);// Y_ADDR_START[7:0];
	write_cmos_sensor(0x034A,0x0C);// Y_ADDR_END[15:8];
	write_cmos_sensor(0x034B,0x2F);// Y_ADDR_END[7:0];
	write_cmos_sensor(0x034C,0x10);// X_OUTPUT_SIZE[15:8];
	write_cmos_sensor(0x034D,0x70);// X_OUTPUT_SIZE[7:0];
	write_cmos_sensor(0x034E,0x0C);// Y_OUTPUT_SIZE[15:8];
	write_cmos_sensor(0x034F,0x30);// Y_OUTPUT_SIZE[7:0];
	write_cmos_sensor(0x0401,0x00);// -/-/-/-/-/-/SCALING_MODE[1:0];
	write_cmos_sensor(0x0404,0x10);// SCALE_M[7:0];
	write_cmos_sensor(0x0900,0x00);// -/-/-/-/-/-/H_BIN[1:0];
	write_cmos_sensor(0x0901,0x00);// -/-/-/-/-/-/V_BIN_MODE[1:0];
	write_cmos_sensor(0x0104,0x00);// -/-/-/-/-/-/-/GROUP_PARA_HOLD;
		
	


	} else {   //30fps			//30fps for Normal capture & ZSD
		  
		//write_cmos_sensor(0x0100,0x00);
		write_cmos_sensor(0x0101,0x03);
		write_cmos_sensor(0x0103,0x00);
		write_cmos_sensor(0x0104,0x01);
		write_cmos_sensor(0x0105,0x00);
		write_cmos_sensor(0x0106,0x00);
		write_cmos_sensor(0x0110,0x00);
		write_cmos_sensor(0x0111,0x02);
		write_cmos_sensor(0x0112,0x0A);
		write_cmos_sensor(0x0113,0x0A);
		write_cmos_sensor(0x0114,0x03);
		write_cmos_sensor(0x0115,0x30);
		write_cmos_sensor(0x0117,0x32);
		write_cmos_sensor(0x0130,0x18);
		write_cmos_sensor(0x0131,0x00);
		write_cmos_sensor(0x0141,0x00);
		write_cmos_sensor(0x0142,0x00);
		write_cmos_sensor(0x0143,0x00);
		write_cmos_sensor(0x0202,0x03);
		write_cmos_sensor(0x0203,0xA6);
		write_cmos_sensor(0x0204,0x00);
		write_cmos_sensor(0x0205,0x3D);
		write_cmos_sensor(0x0210,0x01);
		write_cmos_sensor(0x0211,0x00);
		write_cmos_sensor(0x0212,0x01);
		write_cmos_sensor(0x0213,0x00);
		write_cmos_sensor(0x0214,0x01);
		write_cmos_sensor(0x0215,0x00);
		write_cmos_sensor(0x0216,0x01);
		write_cmos_sensor(0x0217,0x00);
		write_cmos_sensor(0x0230,0x00);
		write_cmos_sensor(0x0232,0x04);
		write_cmos_sensor(0x0234,0x00);
		write_cmos_sensor(0x0235,0x19);
		write_cmos_sensor(0x0301,0x01);
		write_cmos_sensor(0x0303,0x0A);
		write_cmos_sensor(0x0305,0x07);
		write_cmos_sensor(0x0306,0x00);
		write_cmos_sensor(0x0307,0xd8);//0x61
		write_cmos_sensor(0x030B,0x01);
		write_cmos_sensor(0x030D,0x03);
		write_cmos_sensor(0x030E,0x00);
		write_cmos_sensor(0x030F,0x6c);
		write_cmos_sensor(0x0310,0x00);
		write_cmos_sensor(0x0340,((imgsensor_info.cap.framelength >> 8) & 0xFF)); 
		write_cmos_sensor(0x0341,(imgsensor_info.cap.framelength & 0xFF));        
		write_cmos_sensor(0x0342,((imgsensor_info.cap.linelength >> 8) & 0xFF));	 
		write_cmos_sensor(0x0343,(imgsensor_info.cap.linelength & 0xFF));         
		write_cmos_sensor(0x0344,0x00);
		write_cmos_sensor(0x0345,0x00);
		write_cmos_sensor(0x0346,0x00);
		write_cmos_sensor(0x0347,0x00);
		write_cmos_sensor(0x034A,0x0C);
		write_cmos_sensor(0x034B,0x2F);
		write_cmos_sensor(0x034C,0x10);
		write_cmos_sensor(0x034D,0x70);
		write_cmos_sensor(0x034E,0x0C);
		write_cmos_sensor(0x034F,0x30);
		write_cmos_sensor(0x0401,0x00);
		write_cmos_sensor(0x0403,0x00);
		write_cmos_sensor(0x0404,0x10);
		write_cmos_sensor(0x0408,0x00);
		write_cmos_sensor(0x0409,0x00);
		write_cmos_sensor(0x040A,0x00);
		write_cmos_sensor(0x040B,0x00);
		write_cmos_sensor(0x040C,0x10);
		write_cmos_sensor(0x040D,0x70);
		write_cmos_sensor(0x040E,0x0C);
		write_cmos_sensor(0x040F,0x30);
		write_cmos_sensor(0x0601,0x00);
		write_cmos_sensor(0x0602,0x02);
		write_cmos_sensor(0x0603,0xC0);
		write_cmos_sensor(0x0604,0x02);
		write_cmos_sensor(0x0605,0xC0);
		write_cmos_sensor(0x0606,0x02);
		write_cmos_sensor(0x0607,0xC0);
		write_cmos_sensor(0x0608,0x02);
		write_cmos_sensor(0x0609,0xC0);
		write_cmos_sensor(0x060A,0x00);
		write_cmos_sensor(0x060B,0x00);
		write_cmos_sensor(0x060C,0x00);
		write_cmos_sensor(0x060D,0x00);
		write_cmos_sensor(0x060E,0x00);
		write_cmos_sensor(0x060F,0x00);
		write_cmos_sensor(0x0610,0x00);
		write_cmos_sensor(0x0611,0x00);
		write_cmos_sensor(0x0800,0x88);
		write_cmos_sensor(0x0801,0x38);
		write_cmos_sensor(0x0802,0x78);
		write_cmos_sensor(0x0803,0x48);
		write_cmos_sensor(0x0804,0x48);
		write_cmos_sensor(0x0805,0x40);
		write_cmos_sensor(0x0806,0x00);
		write_cmos_sensor(0x0807,0x48);
		write_cmos_sensor(0x0808,0x01);
		write_cmos_sensor(0x0820,0x08);
		write_cmos_sensor(0x0821,0x40);
		write_cmos_sensor(0x0822,0x00);
		write_cmos_sensor(0x0823,0x00);
		write_cmos_sensor(0x0900,0x00);
		write_cmos_sensor(0x0901,0x00);
		write_cmos_sensor(0x0902,0x00);
		write_cmos_sensor(0x0104,0x00);
		write_cmos_sensor(0x0100,0x01);


		if (imgsensor.ihdr_en) {
		
	} else {
		
	}
		
	}
		
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
	   write_cmos_sensor(0x0104,0x01);// -/-/-/-/-/-/-/GROUP_PARA_HOLD;
	write_cmos_sensor(0x0340,0x06);// FR_LENGTH_LINES[15:8];//1572
	write_cmos_sensor(0x0341,0x24);// FR_LENGTH_LINES[7:0];
	write_cmos_sensor(0x0342,0x11);// LINE_LENGTH_PCK[15:8];//4480
	write_cmos_sensor(0x0343,0x80);// LINE_LENGTH_PCK[7:0];
	write_cmos_sensor(0x0346,0x00);// Y_ADDR_START[15:8];
	write_cmos_sensor(0x0347,0x00);// Y_ADDR_START[7:0];
	write_cmos_sensor(0x034A,0x0A);// Y_ADDR_END[15:8];
	write_cmos_sensor(0x034B,0x4F);// Y_ADDR_END[7:0];
	write_cmos_sensor(0x034C,0x07);// X_OUTPUT_SIZE[15:8];
	write_cmos_sensor(0x034D,0x80);// X_OUTPUT_SIZE[7:0];
	write_cmos_sensor(0x034E,0x04);// Y_OUTPUT_SIZE[15:8];
	write_cmos_sensor(0x034F,0x38);// Y_OUTPUT_SIZE[7:0];
	write_cmos_sensor(0x0401,0x02);// -/-/-/-/-/-/SCALING_MODE[1:0];
	write_cmos_sensor(0x0404,0x11);// SCALE_M[7:0];
	write_cmos_sensor(0x0900,0x01);// -/-/-/-/-/-/H_BIN[1:0];
	write_cmos_sensor(0x0901,0x01);// -/-/-/-/-/-/V_BIN_MODE[1:0];
	write_cmos_sensor(0x0104,0x00);// -/-/-/-/-/-/-/GROUP_PARA_HOLD;
	if (imgsensor.ihdr_en) {
	} else {
	}

}
static void hs_video_setting(void)
{
	LOG_INF("E\n");
write_cmos_sensor(0x0104,0x01);// -/-/-/-/-/-/-/GROUP_PARA_HOLD;
	write_cmos_sensor(0x0340,0x06);// FR_LENGTH_LINES[15:8];//1572
	write_cmos_sensor(0x0341,0x24);// FR_LENGTH_LINES[7:0];
	write_cmos_sensor(0x0342,0x11);// LINE_LENGTH_PCK[15:8];//4480
	write_cmos_sensor(0x0343,0x80);// LINE_LENGTH_PCK[7:0];
	write_cmos_sensor(0x0346,0x00);// Y_ADDR_START[15:8];
	write_cmos_sensor(0x0347,0x00);// Y_ADDR_START[7:0];
	write_cmos_sensor(0x034A,0x0A);// Y_ADDR_END[15:8];
	write_cmos_sensor(0x034B,0x4F);// Y_ADDR_END[7:0];
	write_cmos_sensor(0x034C,0x07);// X_OUTPUT_SIZE[15:8];
	write_cmos_sensor(0x034D,0x80);// X_OUTPUT_SIZE[7:0];
	write_cmos_sensor(0x034E,0x04);// Y_OUTPUT_SIZE[15:8];
	write_cmos_sensor(0x034F,0x38);// Y_OUTPUT_SIZE[7:0];
	write_cmos_sensor(0x0401,0x02);// -/-/-/-/-/-/SCALING_MODE[1:0];
	write_cmos_sensor(0x0404,0x11);// SCALE_M[7:0];
	write_cmos_sensor(0x0900,0x01);// -/-/-/-/-/-/H_BIN[1:0];
	write_cmos_sensor(0x0901,0x01);// -/-/-/-/-/-/V_BIN_MODE[1:0];
	write_cmos_sensor(0x0104,0x00);// -/-/-/-/-/-/-/GROUP_PARA_HOLD;
	if (imgsensor.ihdr_en) {
	} else {
	}

}

static void slim_video_setting(void)
{
	LOG_INF("E\n");

 		write_cmos_sensor(0x0104,0x01);// -/-/-/-/-/-/-/GROUP_PARA_HOLD;
	write_cmos_sensor(0x0340,0x06);// FR_LENGTH_LINES[15:8];//1572
	write_cmos_sensor(0x0341,0x24);// FR_LENGTH_LINES[7:0];
	write_cmos_sensor(0x0342,0x11);// LINE_LENGTH_PCK[15:8];//4480
	write_cmos_sensor(0x0343,0x80);// LINE_LENGTH_PCK[7:0];
	write_cmos_sensor(0x0346,0x00);// Y_ADDR_START[15:8];
	write_cmos_sensor(0x0347,0x00);// Y_ADDR_START[7:0];
	write_cmos_sensor(0x034A,0x0A);// Y_ADDR_END[15:8];
	write_cmos_sensor(0x034B,0x4F);// Y_ADDR_END[7:0];
	write_cmos_sensor(0x034C,0x07);// X_OUTPUT_SIZE[15:8];
	write_cmos_sensor(0x034D,0x80);// X_OUTPUT_SIZE[7:0];
	write_cmos_sensor(0x034E,0x04);// Y_OUTPUT_SIZE[15:8];
	write_cmos_sensor(0x034F,0x38);// Y_OUTPUT_SIZE[7:0];
	write_cmos_sensor(0x0401,0x02);// -/-/-/-/-/-/SCALING_MODE[1:0];
	write_cmos_sensor(0x0404,0x11);// SCALE_M[7:0];
	write_cmos_sensor(0x0900,0x01);// -/-/-/-/-/-/H_BIN[1:0];
	write_cmos_sensor(0x0901,0x01);// -/-/-/-/-/-/V_BIN_MODE[1:0];
	write_cmos_sensor(0x0104,0x00);// -/-/-/-/-/-/-/GROUP_PARA_HOLD;
	//@@video_720p_30fps_800Mbps
	
	if (imgsensor.ihdr_en) {
	} else {
	}
}
//
static kal_uint8  test_pattern_flag=0;

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);
	if(imgsensor.current_scenario_id != MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG)
	   {
		   if(enable) 
		   {   
			   //1640 x 1232
			   // enable color bar
			   test_pattern_flag=TRUE;
			   write_cmos_sensor(0x0600, 0x00); 
			   write_cmos_sensor(0x0601, 0x02); 	 
			   write_cmos_sensor(0x0624, 0x06); //W:3280---h
			   write_cmos_sensor(0x0625, 0x68); //		  l
			   write_cmos_sensor(0x0626, 0x04); //H:2464   h
			   write_cmos_sensor(0x0627, 0xd0); //		  l
			   write_cmos_sensor(0x6128, 0x00); 
			   write_cmos_sensor(0x6129, 0x02); 		 
			   write_cmos_sensor(0x613C, 0x06); //W 		h
			   write_cmos_sensor(0x613D, 0x68); //		   l
			   write_cmos_sensor(0x613E, 0x04); //H 		h
			   write_cmos_sensor(0x613F, 0xd0); //			   l
			   write_cmos_sensor(0x6506, 0x00); 
			   write_cmos_sensor(0x6507, 0x00);
	
		   } 
		   else 
		   {   
			   //1640 x 1232
			   test_pattern_flag=FALSE;
			   write_cmos_sensor(0x0600, 0x00); 
			   write_cmos_sensor(0x0601, 0x00); 	 
			   write_cmos_sensor(0x0624, 0x06); //W:3280---h
			   write_cmos_sensor(0x0625, 0x68); //		  l
			   write_cmos_sensor(0x0626, 0x04); //H:2464   h
			   write_cmos_sensor(0x0627, 0xd0); //		  l
			   write_cmos_sensor(0x6128, 0x00); 
			   write_cmos_sensor(0x6129, 0x02); 		 
			   write_cmos_sensor(0x613C, 0x06); //W 		h
			   write_cmos_sensor(0x613D, 0x68); //		   l
			   write_cmos_sensor(0x613E, 0x04); //H 		h
			   write_cmos_sensor(0x613F, 0xd0); //			   l
			   write_cmos_sensor(0x6506, 0x00); 
			   write_cmos_sensor(0x6507, 0x00);
	
		   }
	   }
	   else
	   {
		   if(enable) 
		   {   
			   //3280 x 2464
			   // enable color bar
			   test_pattern_flag=TRUE;
			   write_cmos_sensor(0x0600, 0x00); 
			   write_cmos_sensor(0x0601, 0x02); 	 
			   write_cmos_sensor(0x0624, 0x0C); //W:3280---h
			   write_cmos_sensor(0x0625, 0xD0); //		  l
			   write_cmos_sensor(0x0626, 0x09); //H:2464   h
			   write_cmos_sensor(0x0627, 0xA0); //		  l
			   write_cmos_sensor(0x6128, 0x00); 
			   write_cmos_sensor(0x6129, 0x02); 		 
			   write_cmos_sensor(0x613C, 0x0C); //W 		h
			   write_cmos_sensor(0x613D, 0xD0); //		   l
			   write_cmos_sensor(0x613E, 0x09); //H 		h
			   write_cmos_sensor(0x613F, 0xA0); //			   l
			   write_cmos_sensor(0x6506, 0x00); 
			   write_cmos_sensor(0x6507, 0x00);
	
		   } 
		   else 
		   {   
			   test_pattern_flag=FALSE;
			   write_cmos_sensor(0x0600, 0x00); 
			   write_cmos_sensor(0x0601, 0x02); 	 
			   write_cmos_sensor(0x0624, 0x0C); //W:3280---h
			   write_cmos_sensor(0x0625, 0xD0); //		  l
			   write_cmos_sensor(0x0626, 0x09); //H:2464   h
			   write_cmos_sensor(0x0627, 0xA0); //		  l
			   write_cmos_sensor(0x6128, 0x00); 
			   write_cmos_sensor(0x6129, 0x02); 		 
			   write_cmos_sensor(0x613C, 0x0C); //W 		h
			   write_cmos_sensor(0x613D, 0xD0); //		   l
			   write_cmos_sensor(0x613E, 0x09); //H 		h
			   write_cmos_sensor(0x613F, 0xA0); //			   l
			   write_cmos_sensor(0x6506, 0x00); 
			   write_cmos_sensor(0x6507, 0x00);
	
	
		   }
	   }
		   
	   return ERROR_NONE;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
            *sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  
				return ERROR_NONE;
			}	
			LOG_INF("Read sensor id fail, i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0; 
	LOG_1;	
	LOG_2;
	printk("by zx:t4k37 open \n");
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
            sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);	  
				break;
			}	
			LOG_INF("Read sensor id fail, write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}		 
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	
	/* initail sequence write in  */
	sensor_init();
	/*
	iTemp = read_cmos_sensor(0x0101);
	iTemp&= ~0x03; //Clear the mirror and flip bits.
	write_cmos_sensor_8(0x0101, iTemp | 0x03); //Set mirror and flip
*/
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = KAL_FALSE;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	printk("by zx:t4k37 open exit\n");
	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*	
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
	printk("by zx:t4k37preview\n");
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
	
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    } else {
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    spin_unlock(&imgsensor_drv_lock);
    capture_setting(imgsensor.current_fps);
	if(imgsensor.test_pattern == KAL_TRUE)
	{
		set_test_pattern_mode(TRUE);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.test_pattern = KAL_FALSE;
		spin_unlock(&imgsensor_drv_lock);
	}

	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
	
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
    return ERROR_NONE;
}   /*  slim_video   */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;		

	
	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;
	
	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	
	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;	
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num; 
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x 
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc; 

			break;	 
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			
			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
	   
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc; 

			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:			
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc; 

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc; 

			break;
		default:			
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}
	
	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;	  
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);
	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate) 
{
	kal_uint32 frame_length; 
	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength):0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength):0;			
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            }
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}	
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate) 
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;		
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO: 
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

    return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
	
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;	
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
 
	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:	 
			LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;		   
		case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
			break;
		case SENSOR_FEATURE_SET_GAIN:		
            set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
			break; 
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32); 
			break; 
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing			 
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;							 
			break;				
		case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
			spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
			spin_unlock(&imgsensor_drv_lock);		
			break;
		case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
			spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
			spin_unlock(&imgsensor_drv_lock);		
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		
			switch (*feature_data_32) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;	  
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
			}
            break;
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
			//LOG_INF("SENSOR_SET_SENSOR_IHDR is no support");
			//LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data_32,(UINT16)*(feature_data_32+1),(UINT16)*(feature_data_32+2)); 
			//ihdr_write_shutter_gain((UINT16)*feature_data_32,(UINT16)*(feature_data_32+1),(UINT16)*(feature_data_32+2));	
			break;
		default:
			break;
	}
  
	return ERROR_NONE;
}	/*	feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 T4K37_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	T4K37_MIPI_RAW_SensorInit	*/
