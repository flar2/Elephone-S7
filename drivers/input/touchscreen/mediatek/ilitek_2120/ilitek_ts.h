#include "tpd.h"

#include <linux/module.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/proc_fs.h>

#include <linux/dma-mapping.h>
#include <linux/of_irq.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
//#include <cust_eint.h>

//#include <mach/mt_pm_ldo.h>
//#include <mach/mt_typedefs.h>
//#include <mach/mt_boot.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include <asm/atomic.h>

//#include "cust_gpio_usage.h"

//#define MT6735M

//driver information
#define DERVER_VERSION_MAJOR 		4
#define DERVER_VERSION_MINOR 		0
#define CUSTOMER_ID 				0
#define MODULE_ID					0
#define PLATFORM_ID					0
#define PLATFORM_MODULE				0
#define ENGINEER_ID					0
#define TPD_I2C_NUMBER           	2


#define IC2120						1

//#define TPD_HAVE_BUTTON

#define ILI_HAVE_TOUCH_KEY    0


#define CLOCK_INTERRUPT

//#define TOUCH_PROTOCOL_B

#define TOOL


//#define ILI_UPDATE_FW

//#define ILITEK_ESD_CHECK 

#define GESTURE_2120		1
#define GESTURE_DRIVER		2
#if IC2120
//#define GESTURE				GESTURE_2120
#endif

#if !IC2120
//#define GESTURE				GESTURE_DRIVER
#endif

#define HALL_CHECK_HW		1
#define HALL_CHECK_OTHER	2
//#define HALL_CHECK			HALL_CHECK_HW
//#define HALL_CHECK			HALL_CHECK_OTHER


//#define SENSOR_TEST

//#define REPORT_PRESSURE

#ifdef ILI_UPDATE_FW
#define UPDATE_THREADE
//#define FORCE_UPDATE
#endif

#ifdef CLOCK_INTERRUPT
//#define REPORT_THREAD
#endif

#if IC2120
#define NEW_PROTOCOL_2120
#endif

#ifdef GESTURE
#define GESTURE_FUN_1	1 //Study gesture function
#define GESTURE_FUN_2	2 //A function
#define GESTURE_FUN GESTURE_FUN_1
#endif

//#ifdef TPD_HAVE_BUTTON
#define TPD_KEY_COUNT   4
#define key_1           60,2850              //auto define  
#define key_2           180,2850
#define key_3           300,2850
#define key_4           420,2850

#define TPD_KEYS        {KEY_MENU, KEY_HOMEPAGE, KEY_BACK, KEY_SEARCH}
#define TPD_KEYS_DIM    {{key_1,50,30},{key_2,50,30},{key_3,50,30},{key_4,50,30}}
//#endif
//#define DBG_FLAG 
#define DBG(fmt, args...)   if (DBG_FLAG)printk("%s(%d): " fmt, __func__,__LINE__,  ## args)
#define DBG_CO(fmt, args...)   if (DBG_FLAG||DBG_COR)printk("%s: " fmt, "ilitek",  ## args)



//key
struct key_info {
	int id;
	int x;
	int y;
	int status;
	int flag;
};

struct touch_info {
	int id;
	int x;
	int y;
	int status;
	int flag;
};

// declare i2c data member
struct i2c_data {
	// input device
	struct input_dev *input_dev;
	// i2c client
	struct i2c_client *client;
	// polling thread
	struct task_struct *thread;

	// maximum x
	int max_x;
	// maximum y
	int max_y;
	// minimum x
	int min_x;
	// minimum y
	int min_y;
	// maximum touch point
	int max_tp;
	// maximum key button
	int max_btn;
	// the total number of x channel
	int x_ch;
	// the total number of y channel
	int y_ch;
	// check whether i2c driver is registered success
	int valid_i2c_register;
	// check whether input driver is registered success
	int valid_input_register;
	// check whether the i2c enter suspend or not
	int stop_polling;
	// read semaphore
	struct semaphore wr_sem;
	// protocol version
	int protocol_ver;
	int set_polling_mode;
	// valid irq request
	int valid_irq_request;
	struct regulator *vdd;//0424
	struct regulator *vcc_i2c;
	int irq;
	int irq_gpio;
	int vci_gpio;
	u32 irq_gpio_flags;
	int rst;
	u32 reset_gpio_flags;
	//firmware version
	unsigned char firmware_ver[8];
	//reset request flag
	int reset_request_success;
	// work queue for interrupt use only
	struct workqueue_struct *irq_work_queue;
	// work struct for work queue
	struct work_struct irq_work;

	struct timer_list timer;
	int report_status;
	int reset_gpio;
	int irq_status;
	//irq_status enable:1 disable:0
	struct completion complete;
	int keyflag;
	int keycount;
	int key_xlen;
	int key_ylen;
	struct key_info keyinfo[10];

	// touch_flag release:0 touch:1
	int touch_flag ;
	struct touch_info touchinfo[10];
	int release_flag[10];
};

#define TPD_RES_X		1080
#define TPD_RES_Y		1920

#define VIRTUAL_FUN_1	1	//0X81 with key_id
#define VIRTUAL_FUN_2	2	//0x81 with x position
#define VIRTUAL_FUN_3	3	//Judge x & y position
#define VIRTUAL_FUN		VIRTUAL_FUN_1
#define BTN_DELAY_TIME	500 //ms

#define TOUCH_POINT    0x80
#define TOUCH_KEY      0xC0
#define RELEASE_KEY    0x40
#define RELEASE_POINT    0x00
#define DRIVER_VERSION "aimvF"



//define key pad range
#define KEYPAD01_X1	0
#define KEYPAD01_X2	1000
#define KEYPAD02_X1	1000
#define KEYPAD02_X2	2000
#define KEYPAD03_X1	2000
#define KEYPAD03_X2	3000
#define KEYPAD04_X1	3000
#define KEYPAD04_X2	3968
#define KEYPAD_Y	2100
// definitions
#define ILITEK_I2C_RETRY_COUNT			3
#define ILITEK_I2C_DRIVER_NAME			"ilitek_i2c"
#define ILITEK_FILE_DRIVER_NAME			"ilitek_file"
#define ILITEK_DEBUG_LEVEL			KERN_INFO
#define ILITEK_ERROR_LEVEL			KERN_ALERT

// i2c command for ilitek touch screen

#define ILITEK_TP_CMD_READ_DATA			    0x10
#define ILITEK_TP_CMD_READ_SUB_DATA		    0x11
#define ILITEK_TP_CMD_GET_RESOLUTION		0x20
#define ILITEK_TP_CMD_GET_KEY_INFORMATION	0x22
#define ILITEK_TP_CMD_SOFTRESET				0x60
#define	ILITEK_TP_CMD_CALIBRATION			0xCC
#define	ILITEK_TP_CMD_CALIBRATION_STATUS	0xCD
#define ILITEK_TP_CMD_ERASE_BACKGROUND		0xCE

#if IC2120
#define ILITEK_TP_CMD_SLEEP                 0x02
#define ILITEK_TP_CMD_GET_FIRMWARE_VERSION	0x21
#define ILITEK_TP_CMD_READ_DATA_CONTROL		0xF6
#define ILITEK_TP_CMD_GET_PROTOCOL_VERSION	0x22
#else
#define OP_MODE_APPLICATION					0x5A
#define OP_MODE_BOOTLOADER					0x55
#define ILITEK_TP_CMD_WRITE_ENABLE			0xC4
#define ILITEK_TP_CMD_WRITE_DATA			0xC3
#define ILITEK_TP_CMD_SLEEP                 0x30
#define ILITEK_TP_CMD_GET_FIRMWARE_VERSION	0x40
#define ILITEK_TP_CMD_GET_PROTOCOL_VERSION	0x42
#define ILITEK_TP_CMD_GET_KERNEL_VERSION	0x61
#endif

#define ILITEK_TP_CMD_ESD_CHECK	 			0x42


// module information
MODULE_AUTHOR("Steward_Fu");
MODULE_DESCRIPTION("ILITEK I2C touch driver for Android platform");
MODULE_LICENSE("GPL");

extern int ilitek_i2c_write(struct i2c_client *client, uint8_t * cmd, int length);
extern int ilitek_i2c_read_tp_info(void);
extern int ilitek_i2c_write_and_read(struct i2c_client *client, uint8_t *cmd,
		int write_len, int delay, uint8_t *data, int read_len);

// i2c functions
extern int ilitek_i2c_transfer(struct i2c_client*, struct i2c_msg*, int);
extern void ilitek_i2c_suspend(struct device *h);
extern void ilitek_i2c_resume(struct device *h);
extern int ilitek_i2c_read(struct i2c_client *client, uint8_t *data, int length);

extern void ilitek_i2c_irq_enable(void);
extern void ilitek_i2c_irq_disable(void);

extern void ilitek_reset(void);
extern void ilitek_set_finish_init_flag(void);
