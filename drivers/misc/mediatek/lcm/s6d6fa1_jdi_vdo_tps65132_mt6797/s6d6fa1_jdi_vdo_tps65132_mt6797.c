#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#include <mach/gpio_const.h>
#include <mt-plat/mt_gpio.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/upmu_hw.h>

#if 0
#include <platform/upmu_common.h>
#include <platform/mt_pmic.h>
#endif

#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else

#if 0
#include <mach/mt_pm_ldo.h>
#endif
#include <upmu_common.h>

#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#include <mach/gpio_const.h>
#include <cust_gpio_usage.h>
#endif

#endif

#ifdef CONFIG_MTK_LEGACY
#include <cust_i2c.h>
#endif

#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL, fmt)
#else
#define LCD_DEBUG(fmt)  pr_debug(fmt)
#endif

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define LCM_DSI_CMD_MODE 0
#define FRAME_WIDTH (1080)
#define FRAME_HEIGHT (1920)
#define GPIO_65132_EN GPIO65
#define TPS_I2C_BUSNUM  0 //I2C_I2C_LCD_BIAS_CHANNEL

#define REGFLAG_DELAY 0xFFFC
/* END OF REGISTERS MARKER */
#define REGFLAG_END_OF_TABLE 0xFFFD

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif
/* static unsigned int lcm_esd_test = FALSE;      ///only for ESD test */
/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

static const unsigned int BL_MIN_LEVEL = 20;
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))
#define MDELAY(n) (lcm_util.mdelay(n))

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
#define dsi_set_cmd_by_cmdq_dual(handle, cmd, count, ppara, force_update) \
    lcm_util.dsi_set_cmdq_V23(handle, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V3(para_tbl, size, force_update)\
    lcm_util.dsi_set_cmdq_V3(para_tbl, size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
    lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) \
    lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
    lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
    lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
    lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define dsi_swap_port(swap) \
    lcm_util.dsi_swap_port(swap)
#define set_gpio_lcd_enp(cmd) \
	lcm_util.set_gpio_lcd_enp_bias(cmd)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
#include <linux/uaccess.h>
/* #include <linux/delay.h> */
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
/*****************************************************************************
* Define
*****************************************************************************/
/* for I2C channel 0 */
#define I2C_ID_NAME "tps65132"
#define TPS_ADDR 0x3E

/*****************************************************************************
* GLobal Variable
*****************************************************************************/
static struct i2c_board_info tps65132_board_info __initdata = { I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR) };

static const struct of_device_id lcm_of_match[] = {
		{ .compatible = "mediatek,i2c_lcd_bias" },
		{},
};

static struct i2c_client *tps65132_i2c_client;


/*****************************************************************************
* Function Prototype
*****************************************************************************/
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);
/*****************************************************************************
* Data Structure
*****************************************************************************/

struct tps65132_dev {
	struct i2c_client *client;

};

static const struct i2c_device_id tps65132_id[] = {
	{I2C_ID_NAME, 0},
	{}
};

/* #if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)) */
/* static struct i2c_client_address_data addr_data = { .forces = forces,}; */
/* #endif */
static struct i2c_driver tps65132_iic_driver = {
	.id_table = tps65132_id,
	.probe = tps65132_probe,
	.remove = tps65132_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "tps65132",
		.of_match_table = lcm_of_match,
	},
};

/*****************************************************************************
* Extern Area
*****************************************************************************/

/*****************************************************************************
* Function
*****************************************************************************/
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	pr_debug("tps65132_iic_probe\n");
	pr_debug("TPS: info==>name=%s addr=0x%x\n", client->name, client->addr);
	tps65132_i2c_client = client;
	return 0;
}


static int tps65132_remove(struct i2c_client *client)
{
	pr_debug("tps65132_remove\n");
	tps65132_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}


int tps65132_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	char write_data[2] = {0};

	if (client == NULL) {
		pr_debug("ERROR!!tps65132_i2c_client is null\n");
		return 0;
	}

	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		pr_debug("tps65132 write data fail !!\n");
	return ret;
}

EXPORT_SYMBOL_GPL(tps65132_write_bytes);


/*
* module load/unload record keeping
*/

static int __init tps65132_iic_init(void)
{
	pr_debug("tps65132_iic_init\n");
	i2c_register_board_info(TPS_I2C_BUSNUM, &tps65132_board_info, 1);
	pr_debug("tps65132_iic_init2\n");
	i2c_add_driver(&tps65132_iic_driver);
	pr_debug("tps65132_iic_init success\n");
	return 0;
}

static void __exit tps65132_iic_exit(void)
{
	pr_debug("tps65132_iic_exit\n");
	i2c_del_driver(&tps65132_iic_driver);
}

module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");

#endif

#ifdef BUILD_LK
void __attribute__((weak)) mt6331_upmu_set_rg_vgp1_en(kal_uint32 en)
{
	return;
}
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[90];
};

#if 0//jackymao
static LCM_setting_table_V3 lcm_initialization_setting[] = {
	{0x15,0x11,1,{0x00}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,120,{}},	  
	{0x15,0x29,1,{0x00}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,20,{}},	  
};
#elif 0
static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xF0, 2, {0x5A,0x5A} },
	{REGFLAG_DELAY, 5, {}},  
	{0xF1, 2, {0x5A,0x5A} },
	{REGFLAG_DELAY, 5, {}},  	
	{0xFC, 2, {0x5A,0x5A} },
	{REGFLAG_DELAY, 5, {}},  
	{0xF5, 7, {0x10,0x18,0x00,	  0xD1,0xA7,0x11,0x08} },
	{REGFLAG_DELAY, 5, {}},  	
	{0xB2, 16, {0x00,0x00,0x00,  0x10,0x55,0x0B, 0x00,  0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,	0x00} },
	{REGFLAG_DELAY, 5, {}},  	
	{0xB3, 6, {0x10,0xF0,0x00,	 0xBB,0x04,0x08} }, 
	{REGFLAG_DELAY, 5, {}},  
	{0xB6, 6, {0x29,0x10,0x2C,	0x64,0x64,0x01} }, 
	{REGFLAG_DELAY, 5, {}},  
	{0xB7, 14, {0x00,0x00,0x00,  0x00,0x00,0xF8,0x00,	0x09,0x03,0xF5,0x16,  0x0F,0x00,0x00} },
	{REGFLAG_DELAY, 5, {}},  
	{0xB8, 27, {0x41,0x00,0x03,  0x03,0x03,0x24,0x11,	0x00,0x83,0xF4,0xBB,  0x00,0x03,0x21,0x24,
				0x00,0xAB,0xA9,0x0A,  0xB4,0x70,0xFF,0xFF,	0x00,0x1D,0x00,0x00} }, 
	{REGFLAG_DELAY, 5, {}},  
	{0xB9, 27, {0x20,0x00,  0x03,0x03,0x03,0x00,  0x11,0x00,0x02,0xF4,  0xBB,0x00,0x03,0x00, 
				0x00,0x11,0x5A,0x58,  0x15,0xA5,0x20,0xFF,  0xFF,0x00,0x39,0x00,  0x00} },	
	{REGFLAG_DELAY, 5, {}},  
	{0xBA, 27, {0x41,0x00,  0x03,0x03,0x03,0x24,  0x11,0x00,0x83,0xF4,  0xBB,0x00,0x03,0x21,  
				0x24,0x00,0xAB,0xA9,  0x0A,0xB4,0x70,0xFF,  0xFF,0x00,0x1D,0x00,  0x00} },
	{REGFLAG_DELAY, 5, {}},  
	{0xBB, 27, {0x41,0x00,0x03,0x03,0x03,0x24,0x11,0x00,0x83,0xF4,0xBB,0x00,0x03,0x21,
				0x24,0x00,0xAB,0xA9,  0x0A,0xB4,0x70,0xFF,  0xFF,0x00,0x1D,0x00,  0x00} },
	{REGFLAG_DELAY, 5, {}},  
	{0xBC, 23, {0x20,0x00,  0x3C,0x19,0x31,0x02,  0x07,0x00,0x00,0x2D,  0x08,0x0F,0x0E,0x07,
				0x1E,0x04,0x02,0x2A,  0x00,0x00,0x02,0x00, 0x9E} },
	{REGFLAG_DELAY, 5, {}},  
	{0xBD, 20, {0x01,0x07,  0x07,0x07,0x00,0x00,  0x00,0x00,0x05,0x01,  0x00,0x00,0x00,0x40, 
				0x00,0x05,0x2D,0x05,  0x05,0x05} },
	{REGFLAG_DELAY, 5, {}},  
	{0xBE, 43, {0x00,0x00,  0x00,0x00,0x00,0x00,  0x00,0x0B,0x0B,0x0B,  0x0B,0x07,0x0A,0x06,
				0x02,0x01,0x00,0x09,  0x0B,0x0E,0x0D,0x0C,  0x11,0x10,0x0F,0x0B,  0x0B,0x0B,0x0B,0x0B,  0x0B,0x05,0x02,0x01,
				0x00,0x08,0x12,0x0E,  0x0D,0x0C,0x11,0x10,  0x0F} },
	{REGFLAG_DELAY, 5, {}},  
	{0xBF, 14, {0x00,0x00,  0x00,0x00,0xCC,0xCC,  0xCC,0xCC,0x0C,0xAC,  0x67,0xCC,0xCC,0x0C} },
	{REGFLAG_DELAY, 5, {}},  
	{0xD7, 3, {0xA1,0x75,0x20} },
	{REGFLAG_DELAY, 5, {}},  
	{0xE3, 1, {0x22} },
	{REGFLAG_DELAY, 5, {}},  
	{0xE8, 7, {0x00,0x03,0x1A,	0x0E,0x02,0x06,0xD0} },
	{REGFLAG_DELAY, 5, {}},  
	{0xF2, 6, {0x46,0x43,0x13,	0x33,0xC1,0x18} },
	{REGFLAG_DELAY, 5, {}},  
	{0xF4, 27, {0x20,0x20,  0x70,0x20,0x14,0x20,  0x14,0x06,0x10,0x1C,  0x00,0x6E,0x19,0x10,  0x0E,0xC9,0x02,0x55,
				0x35,0x54,0xAA,0x55,  0x05,0x04,0x44,0x48,  0x30} },
	{REGFLAG_DELAY, 5, {}},  
	{0xF7, 4, {0x00,0x00,0x3F,	0xFF} },
	{REGFLAG_DELAY, 5, {}},  
	{0xFA, 82, {0x05,0x54,   0x0E,0x0E,0x12,0x17,	 0x18,0x1C,0x1E,0x1E,	0x2D,0x34,0x16,0x19,   0x19,0x24,0x22,0x1F,
				0x23,0x22,0x1F,0x1F,	  0x0C,0x0D,0x14,0x1D,	 0x0E,0x05,0x54,0x10,	0x11,0x15,0x1A,0x1C,   0x1F,0x20,0x20,0x2F, 
				0x35,0x18,0x1A,0x1B,	  0x26,0x24,0x21,0x25,	 0x23,0x21,0x21,0x0D,	0x0E,0x15,0x1D,0x0F,   0x05,0x54,0x0C,0x0B,
				0x0F,0x14,0x16,0x1A,	  0x1C,0x1C,0x2C,0x33,	 0x16,0x18,0x18,0x23,	0x21,0x1E,0x22,0x20,   0x1E,0x1E,0x0B,0x0C,
				0x13,0x1C,0x0E,0x00} }, 
	{REGFLAG_DELAY, 5, {}},  
	{0xFB, 81, {0x05,0x54,  0x0E,0x0E,0x12,0x17,  0x18,0x1C,0x1E,0x1E,  0x2D,0x34,0x16,0x19,  0x19,0x24,0x22,0x1F,
				0x23,0x22,0x1F,0x1F,  0x0C,0x0D,0x14,0x1D,  0x0E,0x05,0x54,0x10,  0x11,0x15,0x1A,0x1C,  0x1F,0x20,0x20,0x2F,
				0x35,0x18,0x1A,0x1B,  0x26,0x24,0x21,0x25,  0x23,0x21,0x21,0x0D,  0x0E,0x15,0x1D,0x0F,  0x05,0x54,0x0C,0x0B,
				0x0F,0x14,0x16,0x1A,  0x1C,0x1C,0x2C,0x33,  0x16,0x18,0x18,0x23,  0x21,0x1E,0x22,0x20,  0x1E,0x1E,0x0B,0x0C,
				0x13,0x1C,0x0E} }, 
	{REGFLAG_DELAY, 5, {}},  
	{0xF0, 2, {0xA5,0xA5} },
	{REGFLAG_DELAY, 5, {}},  
	{0xF1, 2, {0xA5,0xA5} },
	{REGFLAG_DELAY, 5, {}},  
	{0xFC, 2, {0xA5,0xA5} },
	{REGFLAG_DELAY, 5, {}},  
	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 100, {}},  
	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 150, {}},  
	{REGFLAG_END_OF_TABLE, 0x00, {}} 
};
#else
#endif

#if 0
static struct LCM_setting_table lcm_set_window[] = {
	{0x2A, 4, {0x00, 0x00, (FRAME_WIDTH >> 8), (FRAME_WIDTH & 0xFF)} },
	{0x2B, 4, {0x00, 0x00, (FRAME_HEIGHT >> 8), (FRAME_HEIGHT & 0xFF)} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
	{0x29, 1, {0x00} },
	{REGFLAG_DELAY, 120, {} },

	{0x11, 1, {0x00} },
	{REGFLAG_DELAY, 20, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_normal_sleep_mode_in_setting[] = {
	/* Display off sequence */
	{0x28, 1, {0x00} },
	{REGFLAG_DELAY, 20, {} },

	/* Sleep Mode On */
	{0x10, 1, {0x00} },
	{REGFLAG_DELAY, 120, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

#endif

#if 0
static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	/* Display off sequence */
	{0x28, 1, {0x00} },
	{REGFLAG_DELAY, 20, {} },

	/* Sleep Mode On */
	{0x10, 1, {0x00} },
	{REGFLAG_DELAY, 120, {} },

	{0xB0, 1, {0x00} },
	{0xB1, 1, {0x01} },
	{REGFLAG_DELAY, 20, {} },

	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
#endif

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {}},
	{0x10, 0, {}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}} 
};

#if 0
static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
#endif

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++)
	{
		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY :
				if(table[i].count <= 10)
					MDELAY(table[i].count);
				else
					MDELAY(table[i].count);
				break;

			case REGFLAG_END_OF_TABLE :
				break;

			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}


/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
	params->dsi.mode = SYNC_EVENT_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
#endif
	params->dsi.switch_mode_enable = 0;


	params->dsi.LANE_NUM				= LCM_FOUR_LANE;

	params->dsi.data_format.color_order 	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq		= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding 		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format				= LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size=256;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 2; //2
	params->dsi.vertical_backporch = 10; //10
	params->dsi.vertical_frontporch = 100; //100
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 20; //20
	params->dsi.horizontal_backporch = 50; //50
	params->dsi.horizontal_frontporch = 114; //114
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable = 1;

	params->dsi.PLL_CLOCK = 440;

	params->dsi.CLK_HS_POST = 36;
	params->dsi.clk_lp_per_line_enable = 0;
}

#ifdef BUILD_LK

#define TPS65132_SLAVE_ADDR_WRITE  0x7C
static struct mt_i2c_t TPS65132_i2c;

static int TPS65132_write_byte(kal_uint8 addr, kal_uint8 value)
{
	kal_uint32 ret_code = I2C_OK;
	kal_uint8 write_data[2];
	kal_uint16 len;

	write_data[0] = addr;
	write_data[1] = value;

	TPS65132_i2c.id = TPS_I2C_BUSNUM; /* I2C2; */
	/* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
	TPS65132_i2c.addr = (TPS65132_SLAVE_ADDR_WRITE >> 1);
	TPS65132_i2c.mode = ST_MODE;
	TPS65132_i2c.speed = 100;
	len = 2;

	ret_code = i2c_write(&TPS65132_i2c, write_data, len);
	/* printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code); */

	return ret_code;
}

#else

/* extern int mt8193_i2c_write(u16 addr, u32 data); */
/* extern int mt8193_i2c_read(u16 addr, u32 *data); */

/* #define TPS65132_write_byte(add, data)  mt8193_i2c_write(add, data) */
/* #define TPS65132_read_byte(add)  mt8193_i2c_read(add) */


#endif

static void lcm_init_power(void)
{
#if 0
#ifdef BUILD_LK
	mt6331_upmu_set_rg_vgp1_en(1);
#else
	pr_debug("%s, begin\n", __func__);
	hwPowerOn(MT6331_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");
	pr_debug("%s, end\n", __func__);
#endif
#endif
}


static void lcm_resume_power(void)
{
#if 0
#ifdef BUILD_LK
	mt6331_upmu_set_rg_vgp1_en(1);
#else
	pr_debug("%s, begin\n", __func__);
	hwPowerOn(MT6331_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");
	pr_debug("%s, end\n", __func__);
#endif
#endif
}


static void lcm_suspend_power(void)
{
#if 0
#ifdef BUILD_LK
	mt6331_upmu_set_rg_vgp1_en(0);
#else
	pr_debug("%s, begin\n", __func__);
	hwPowerDown(MT6331_POWER_LDO_VGP1, "LCM_DRV");
	pr_debug("%s, end\n", __func__);
#endif
#endif
}

#if 0 //jackymao
static struct LCM_setting_table lcm_read_reg_table[] = {
	{0xFF,3,{0x98,0x81,0x00}},
	{0x36,1,{0x08}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}} 	
};

static void lcm_read_reg(void)
{
	unsigned char buffer[2];
	unsigned int array[16];

	push_table(lcm_read_reg_table, sizeof(lcm_read_reg_table) / sizeof(struct LCM_setting_table),1);

	array[0] = 0x00023700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x36, buffer, 2);
	MDELAY(20);

#ifdef BUILD_LK
	dprintf(0, "%s, LK s6d6fa1 debug: buffer[0] = 0x%08x,buffer[1] = 0x%08x\n", __func__, buffer[0],buffer[1]);
#endif
}
#endif

#ifdef BUILD_LK
extern void pmic_set_register_value(PMU_FLAGS_LIST_ENUM flagname,kal_uint32 val);
#endif
static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret = 0;
	unsigned int data_array[24];


	pmic_set_register_value(PMIC_RG_VLDO28_EN_0,1);
   #ifdef BUILD_LK
	mt_set_gpio_mode(GPIO_65132_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_EN, GPIO_OUT_ONE);
   #else	
	set_gpio_lcd_enp(1);
   #endif
	MDELAY(5);

	cmd = 0x00;
	data = 0x0E;
#ifdef BUILD_LK
	ret = TPS65132_write_byte(cmd, data);
	if (ret)
		dprintf(0, "[LK]s6d6fa1----tps6132----cmd=%0x--i2c write error----\n", cmd);
	else
		dprintf(0, "[LK]s6d6fa1----tps6132----cmd=%0x--i2c write success----\n", cmd);
#else
	ret = tps65132_write_bytes(cmd, data);
	if (ret < 0)
		pr_err("[KERNEL]s6d6fa1----tps6132---cmd=%0x-- i2c write error-----\n", cmd);
	else
		pr_debug("[KERNEL]s6d6fa1----tps6132---cmd=%0x-- i2c write success-----\n", cmd);
#endif

	cmd = 0x01;
	data = 0x0E;
#ifdef BUILD_LK
	ret = TPS65132_write_byte(cmd, data);
	if (ret)
		dprintf(0, "[LK]s6d6fa1----tps6132----cmd=%0x--i2c write error----\n", cmd);
	else
		dprintf(0, "[LK]s6d6fa1----tps6132----cmd=%0x--i2c write success----\n", cmd);
#else
	ret = tps65132_write_bytes(cmd, data);
	if (ret < 0)
		pr_err("[KERNEL]s6d6fa1----tps6132---cmd=%0x-- i2c write error-----\n", cmd);
	else
		pr_debug("[KERNEL]s6d6fa1----tps6132---cmd=%0x-- i2c write success-----\n", cmd);
#endif

	SET_RESET_PIN(1);
	MDELAY(5);
	/* MDELAY(10); */
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);

	/* when phone initial , config output high, enable backlight drv chip */
#if 0 //jackymao
	dsi_set_cmdq_V3(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);
#elif 1 //jackymao
 
#if 1  //register setting table open	
	data_array[0] = 0x00110500; //0x29,Display On,1byte
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(150);
	data_array[0] = 0x00290500; //0x29,Display On,1byte
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(100);

	 // {0xF0, 2, {0x5A,0x5A} },	/* Return  To	   CMD1 */	
	  data_array[0]=0x00033902;
	  data_array[1]=0x005A5AF0;
	  dsi_set_cmdq(data_array,2,1);
	  MDELAY(1);
	  
	//	{0xF1, 2, {0x5A,0x5A} },	/* Return  To	   CMD1 */
	  data_array[0]=0x00033902;
	  data_array[1]=0x005A5AF1;
	  dsi_set_cmdq(data_array,2,1);
	  MDELAY(1);
	
	//	{0xFC, 2, {0x5A,0x5A} },	/* Return  To	   CMD1 */
	  data_array[0]=0x00033902;
	  data_array[1]=0x005A5AFC;
	  dsi_set_cmdq(data_array,2,1);
	  MDELAY(1);
	  #if 0
	//	{0xF5, 7, {0x10,0x18,0x00,	  0xD1,0xA7,0x11,0x08} },	/* Return  To	   CMD1 */
	  data_array[0]=0x00083902;
	  data_array[1]=0x001810F5;
	  data_array[2]=0x0811A7D1;
	  dsi_set_cmdq(data_array,3,1);
	  MDELAY(1);  
	  
	//	{0xB2, 17, {0xB2,0x00,0x00,0x00,  0x10,0x55,0x0B, 0x00,  0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,	0x00} },	/* Return  To	   CMD1 */ 
	  data_array[0]=0x00113902;
	  data_array[1]=0x000000B2;
	  data_array[2]=0x000B5510;
	  data_array[3]=0x00000000;
	  data_array[4]=0x00000000;
	  data_array[5]=0x00000000;
	  dsi_set_cmdq(data_array,6,1);
	  MDELAY(1); 
	  
	//	{0xB3, 6, {0x10,0xF0,0x00,	 0xBB,0x04,0x08} }, 
	  data_array[0]=0x00073902;
	  data_array[1]=0x00F010B3;
	  data_array[2]=0x000804BB;
	  dsi_set_cmdq(data_array,3,1);
	  MDELAY(1);  
	  
	 // {0xB6, 6, {0x29,0x10,0x2C,	0x64,0x64,0x01} }, 
	  data_array[0]=0x00073902;
	  data_array[1]=0x2C1029B6;
	  data_array[2]=0x00016464;
	  dsi_set_cmdq(data_array,3,1);
	  MDELAY(1); 
	 
	//	{0xB7, 15, {0xB7,0x00,0x00,0x00,  0x00,0x00,0xF8,0x00,	0x09,0x03,0xF5,0x16,  0x0F,0x00,0x00} },
	  data_array[0]=0x00103902;
	  data_array[1]=0x000000B7;
	  data_array[2]=0x00F80000;
	  data_array[3]=0x16F50309;
	  data_array[4]=0x0000000F;
	  dsi_set_cmdq(data_array,5,1);
	  MDELAY(1);  
	 
	//	{0xB8, 28, {0xB8,0x41,0x00,0x03,  0x03,0x03,0x24,0x11,	0x00,0x83,0xF4,0xBB,  0x00,0x03,0x21,0x24,
	//				0x00,0xAB,0xA9,0x0A,  0xB4,0x70,0xFF,0xFF,	0x00,0x1D,0x00,0x00} }, 
	  data_array[0]=0x001C3902;
	  data_array[1]=0x030041B8;
	  data_array[2]=0x11240303;
	  data_array[3]=0xBBF48300;
	  data_array[4]=0x24210300;
	  data_array[5]=0x0AA9AB00;
	  data_array[6]=0xFFFF70B4;
	  data_array[7]=0x00001D00;
	  dsi_set_cmdq(data_array,8,1);
	  MDELAY(1);
	  
	//	{0xB9, 28, {0xB9,0x20,0x00,  0x03,0x03,0x03,0x00,  0x11,0x00,0x02,0xF4,  0xBB,0x00,0x03,0x00, 
	// 0x00,0x11,0x5A,0x58,  0x15,0xA5,0x20,0xFF,  0xFF,0x00,0x39,0x00,  0x00} },	/* CMD2 Page4 Entrance */
	  data_array[0]=0x001C3902;
	  data_array[1]=0x030020B9;
	  data_array[2]=0x11000303;
	  data_array[3]=0xBBF40200;
	  data_array[4]=0x00000300;
	  data_array[5]=0x15585A11;
	  data_array[6]=0xFFFF20A5;
	  data_array[7]=0x00003900;
	  dsi_set_cmdq(data_array,8,1);
	  MDELAY(1);  
	  
	//	{0xBA, 28, {0xBA,0x41,0x00,  0x03,0x03,0x03,0x24,  0x11,0x00,0x83,0xF4,  0xBB,0x00,0x03,0x21,  
	// 0x24,0x00,0xAB,0xA9,  0x0A,0xB4,0x70,0xFF,  0xFF,0x00,0x1D,0x00,  0x00} },
	  data_array[0]=0x001C3902;
	  data_array[1]=0x030041BA;
	  data_array[2]=0x11240303;
	  data_array[3]=0xBBF48300;
	  data_array[4]=0x24210300;
	  data_array[5]=0x0AA9AB00;
	  data_array[6]=0xFFFF70B4;
	  data_array[7]=0x00001D00;
	  dsi_set_cmdq(data_array,8,1);
	  MDELAY(1);  
	  
	//	{0xBB, 28, {0xBB,0x41,0x00,0x03,0x03,0x03,0x24,0x11,0x00,0x83,0xF4,0xBB,0x00,0x03,0x21,
	// 0x24,0x00,0xAB,0xA9,  0x0A,0xB4,0x70,0xFF,  0xFF,0x00,0x1D,0x00,  0x00} },
	  data_array[0]=0x001C3902;
	  data_array[1]=0x030041BB;
	  data_array[2]=0x11240303;
	  data_array[3]=0xBBF48300;
	  data_array[4]=0x24210300;
	  data_array[5]=0x0AA9AB00;
	  data_array[6]=0xFFFF70B4;
	  data_array[7]=0x00001D00;
	  dsi_set_cmdq(data_array,8,1);
	  MDELAY(1); 
	  
	//	{0xBC, 24, {0xBC,0x20,0x00,  0x3C,0x19,0x31,0x02,  0x07,0x00,0x00,0x2D,  0x08,0x0F,0x0E,0x07,
	// 0x1E,0x04,0x02,0x2A,  0x00,0x00,0x02,0x00, 0x9E} },
	  data_array[0]=0x00183902;
	  data_array[1]=0x3C0020BC;
	  data_array[2]=0x07023119;
	  data_array[3]=0x082D0000;
	  data_array[4]=0x1E070E0F;
	  data_array[5]=0x002A0204;
	  data_array[6]=0x9E000200;
	  dsi_set_cmdq(data_array,7,1);
	  MDELAY(1); 
	
	//	{0xBD, 21, {0xBD,0x01,0x07,  0x07,0x07,0x00,0x00,  0x00,0x00,0x05,0x01,  0x00,0x00,0x00,0x40, 
	//	0x00,0x05,0x2D,0x05,  0x05,0x05} },
	  data_array[0]=0x00153902;
	  data_array[1]=0x070701BD;
	  data_array[2]=0x00000007;
	  data_array[3]=0x00010500;
	  data_array[4]=0x00400000;
	  data_array[5]=0x05052D05;
	  data_array[6]=0x00000005;
	  dsi_set_cmdq(data_array,7,1);
	  MDELAY(1); 
	  
	//	{0xBE, 44, {0xBE,0x00,0x00,  0x00,0x00,0x00,0x00,  0x00,0x0B,0x0B,0x0B,  0x0B,0x07,0x0A,0x06,
	// 0x02,0x01,0x00,0x09,  0x0B,0x0E,0x0D,0x0C,  0x11,0x10,0x0F,0x0B,  0x0B,0x0B,0x0B,0x0B,  0x0B,0x05,0x02,0x01,
	// 0x00,0x08,0x12,0x0E,  0x0D,0x0C,0x11,0x10,  0x0F} },
	  data_array[0]=0x002C3902;
	  data_array[1]=0x000000BE;
	  data_array[2]=0x00000000;
	  data_array[3]=0x0B0B0B0B;
	  data_array[4]=0x02060A07;
	  data_array[5]=0x0B090001;
	  data_array[6]=0x110C0D0E;
	  data_array[7]=0x0B0B0F10;
	  data_array[8]=0x0B0B0B0B;
	  data_array[9]=0x00010205;
	  data_array[10]=0x0D0E1208;
	  data_array[11]=0x0F10110C;
	  dsi_set_cmdq(data_array,12,1);
	  MDELAY(1); 
	  
	//	{0xBF, 15, {0xBF,0x00,0x00,  0x00,0x00,0xCC,0xCC,  0xCC,0xCC,0x0C,0xAC,  0x67,0xCC,0xCC,0x0C} },
	  data_array[0]=0x000F3902;
	  data_array[1]=0x000000BF;
	  data_array[2]=0xCCCCCC00;
	  data_array[3]=0x67AC0CCC;
	  data_array[4]=0x000CCCCC;
	  dsi_set_cmdq(data_array,5,1);
	  MDELAY(1); 
	
	//	{0xD7, 3, {0xA1,0x75,0x20} },
	  data_array[0]=0x00043902;
	  data_array[1]=0x2075A1D7;
	  dsi_set_cmdq(data_array,2,1);
	  MDELAY(1);
	
	//	{0xE3, 1, {0x22} },
	  data_array[0]=0x00023902;
	  data_array[1]=0x000022E3;
	  dsi_set_cmdq(data_array,2,1);
	  MDELAY(1);
		
	//	{0xE8, 7, {0x00,0x03,0x1A,	0x0E,0x02,0x06,0xD0} },
	  data_array[0]=0x00083902;
	  data_array[1]=0x1A0300E8;
	  data_array[2]=0xD006020E; 		 //20160303
	  dsi_set_cmdq(data_array,3,1);
	  MDELAY(1);	
		
	//	{0xF2, 6, {0x46,0x43,0x13,	0x33,0xC1,0x18} },
	  data_array[0]=0x00073902;
	  data_array[1]=0x134346F2;
	  data_array[2]=0x0018C133;
	  dsi_set_cmdq(data_array,3,1);
	  MDELAY(1);	
	
	//	{0xF4, 28, {0xF4,0x20,0x20,  0x70,0x20,0x14,0x20,  0x14,0x06,0x10,0x1C,  0x00,0x6E,0x19,0x10,  0x0E,0xC9,0x02,0x55,
	// 0x35,0x54,0xAA,0x55,  0x05,0x04,0x44,0x48,  0x30} },
	  data_array[0]=0x001C3902;
	  data_array[1]=0x702020F4;
	  data_array[2]=0x14201420;
	  data_array[3]=0x001C1006;
	  data_array[4]=0x0E10196E;
	  data_array[5]=0x355502C9;
	  data_array[6]=0x0555AA54;
	  data_array[7]=0x30484404;
	  dsi_set_cmdq(data_array,8,1);
	  MDELAY(1); 
	  
	//	{0xF7, 4, {0x00,0x00,0x3F,	0xFF} },
	  data_array[0]=0x00053902;
	  data_array[1]=0x3F0000F7;
	  data_array[2]=0x000000FF;
	  dsi_set_cmdq(data_array,3,1);
	  MDELAY(1);  
	#endif
	//	{0xFA, 83, {0xFA,0x05,0x54,   0x1C,0x1E,0x23,0x27,	 0x29,0x2B,0x2C,0x2A,	0x39,0x3D,0x1F,0x20,   0x20,0x29,0x28,0x23,
	//		  0x26,0x21,0x1F,0x1F,	  0x08,0x07,0x0B,0x12,	 0x0B,0x05,0x54,0x1C,	0x1E,0x23,0x27,0x29,   0x2B,0x2C,0x2A,0x39, 
	//		  0x3D,0x1F,0x20,0x20,	  0x29,0x28,0x23,0x26,	 0x21,0x1F,0x1F,0x08,	0x07,0x0B,0x12,0x0B,   0x05,0x54,0x1C,0x1E,
	//		  0x23,0x27,0x29,0x2B,	  0x2C,0x2A,0x39,0x3D,	 0x1F,0x20,0x20,0x29,	0x28,0x23,0x26,0x21,   0x1F,0x1F,0x08,0x07,
	//		  0x0B,0x12,0x0B,0x00} },  //20160303 old
	
	  data_array[0]=0x00533902;
	  data_array[1]=0x0E5405FA;
	  data_array[2]=0x1817120E;
	  data_array[3]=0x2D1E1E1C;
	  data_array[4]=0x19191634;
	  data_array[5]=0x231F2224;
	  data_array[6]=0x0C1F1F22;
	  data_array[7]=0x0E1D140D;
	  data_array[8]=0x11105405;
	  data_array[9]=0x1F1C1A15;
	  data_array[10]=0x352F2020;
	  data_array[11]=0x261B1A18;
	  data_array[12]=0x23252124;
	  data_array[13]=0x0E0D2121;
	  data_array[14]=0x050F1D15;
	  data_array[15]=0x0F0B0C54;
	  data_array[16]=0x1C1A1614;
	  data_array[17]=0x16332C1C;
	  data_array[18]=0x21231818;
	  data_array[19]=0x1E20221E;
	  data_array[20]=0x130C0B1E;
	  data_array[21]=0x00000E1C; 
	  //data_array[1]=0x5f5462FA;
	  //data_array[2]=0x40485057;
	  //data_array[3]=0x392f333c;
	  //data_array[4]=0x1c1c1b3b;
	  //data_array[5]=0x24222528;
	  //data_array[6]=0x071d1d21;
	  //data_array[7]=0x1e290b06;
	  //data_array[8]=0x525a545c;
	  //data_array[9]=0x3c3f454c;
	  //data_array[10]=0x403d3235;
	  //data_array[11]=0x2e232221;
	  //data_array[12]=0x282a292c;
	  //data_array[13]=0x0b0c2323;
	  //data_array[14]=0x050d170e;
	  //data_array[15]=0x0d090C54;
	  //data_array[16]=0x17161210;
	  //data_array[17]=0x102e2717;
	  //data_array[18]=0x1c201313;
	  //data_array[19]=0x15191b1a;
	  //data_array[20]=0x04000015;
	  //data_array[21]=0x00001c0d;  
	  dsi_set_cmdq(data_array,22,1);
	  MDELAY(1); 
	
	//	{0xFB, 82, {0xFB,0x05,0x54,  0x1C,0x1E,0x23,0x27,  0x29,0x2B,0x2C,0x2A,  0x39,0x3D,0x1F,0x20,  0x20,0x29,0x28,0x23,
	//		   0x26,0x21,0x1F,0x1F,  0x08,0x07,0x0B,0x12,  0x0B,0x05,0x54,0x1C,  0x1E,0x23,0x27,0x29,  0x2B,0x2C,0x2A,0x39,
	//		   0x3D,0x1F,0x20,0x20,  0x29,0x28,0x23,0x26,  0x21,0x1F,0x1F,0x08,  0x07,0x0B,0x12,0x0B,  0x05,0x54,0x1C,0x1E,
	//		   0x23,0x27,0x29,0x2B,  0x2C,0x2A,0x39,0x3D,  0x1F,0x20,0x20,0x29,  0x28,0x23,0x26,0x21,  0x1F,0x1F,0x08,0x07,
	//		   0x0B,0x12,0x0B} },  //20160303 old
	  data_array[0]=0x00523902;
	  data_array[1]=0x0E5405FB;
	  data_array[2]=0x1817120E;
	  data_array[3]=0x2D1E1E1C;
	  data_array[4]=0x19191634;
	  data_array[5]=0x231F2224;
	  data_array[6]=0x0C1F1F22;
	  data_array[7]=0x0E1D140D;
	  data_array[8]=0x11105405;
	  data_array[9]=0x1F1C1A15;
	  data_array[10]=0x352F2020;
	  data_array[11]=0x261B1A18;
	  data_array[12]=0x23252124;
	  data_array[13]=0x0E0D2121;
	  data_array[14]=0x050F1D15;
	  data_array[15]=0x0F0B0C54;
	  data_array[16]=0x1C1A1614;
	  data_array[17]=0x16332C1C;
	  data_array[18]=0x21231818;
	  data_array[19]=0x1E20221E;
	  data_array[20]=0x130C0B1E;
	  data_array[21]=0x00000E1C;  
	  //data_array[1]=0x5f5462FB;
	  //data_array[2]=0x40485057;
	  //data_array[3]=0x392f333c;
	  //data_array[4]=0x1c1c1b3b;
	  //data_array[5]=0x24222528;
	  //data_array[6]=0x071d1d21;
	  //data_array[7]=0x1e290b06;
	  //data_array[8]=0x525a545c;
	  //data_array[9]=0x3c3f454c;
	  //data_array[10]=0x403d3235;
	  //data_array[11]=0x2e232221;
	  //data_array[12]=0x282a292c;
	  //data_array[13]=0x0b0c2323;
	  //data_array[14]=0x050d170e;
	  //data_array[15]=0x0d090C54;
	  //data_array[16]=0x17161210;
	  //data_array[17]=0x102e2717;
	  //data_array[18]=0x1c201313;
	  //data_array[19]=0x15191b1a;
	  //data_array[20]=0x04000015;
	  //data_array[21]=0x00001c0d;
	  dsi_set_cmdq(data_array,22,1);
	  MDELAY(1); 
	
	 // {0xF0, 2, {0x5A,0x5A} },	/* Return  To	   CMD1 */	
	  data_array[0]=0x00033902;
	  data_array[1]=0x00A5A5F0; 	  //20160303
	  dsi_set_cmdq(data_array,2,1);
	  MDELAY(1);
	  
	//	{0xF1, 2, {0x5A,0x5A} },	/* Return  To	   CMD1 */
	  data_array[0]=0x00033902;
	  data_array[1]=0x00A5A5F1; 	  //20160303
	  dsi_set_cmdq(data_array,2,1);
	  MDELAY(1);
	
	//	{0xFC, 2, {0x5A,0x5A} },	/* Return  To	   CMD1 */
	  data_array[0]=0x00033902;
	  data_array[1]=0x00A5A5FC; 	 //20160303
	  dsi_set_cmdq(data_array,2,1);
	  MDELAY(1); 
#endif   ///end of register setting


#else
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table),1);
#endif
   //lcm_read_reg();//jackymao

#ifdef BUILD_LK
	dprintf(0, "[LK]push_table end\n");
#endif
}

//jackymao
//extern int gesture_enabled;
static void lcm_suspend(void)
{
	#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO_65132_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_EN, GPIO_OUT_ZERO);
	#else
	set_gpio_lcd_enp(0);
	#endif
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),1);
	SET_RESET_PIN(0);

	//if(gesture_enabled==0)
	//{
		pmic_set_register_value(PMIC_RG_VLDO28_EN_0,0);
	//}
}

static void lcm_resume(void)
{
	lcm_init();
}

#if 0
static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	/*BEGIN PN:DTS2013013101431 modified by s00179437 , 2013-01-31 */
	/* delete high speed packet */
	/* data_array[0]=0x00290508; */
	/* dsi_set_cmdq(data_array, 1, 1); */
	/*END PN:DTS2013013101431 modified by s00179437 , 2013-01-31 */

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}
#endif

static unsigned int lcm_compare_id(void)
{
	return 1;
}

#if 0
static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	return 1;
#endif
}
#endif

#if 0
static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
#ifdef BUILD_LK
	dprintf(0, "%s,lk s6d6fa1 backlight: level = %d\n", __func__, level);
#else
	pr_info("%s, kernel s6d6fa1 backlight: level = %d\n", __func__, level);
#endif
	/* Refresh value of backlight level. */

	unsigned int cmd = 0x51;
	unsigned int count = 1;
	unsigned int value = level;
	dsi_set_cmd_by_cmdq_dual(handle, cmd, count, &value, 1);
	/* lcm_backlight_level_setting[0].para_list[0] = level; */
	/* push_table(lcm_backlight_level_setting,
	  sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1); */
}
#endif

LCM_DRIVER s6d6fa1_jdi_vdo_tps65132_lcm_drv = {
	.name = "s6d6fa1_jdi_vdo_tps65132_mt6797",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	//.ata_check = lcm_ata_check,
	//.set_backlight_cmdq = lcm_setbacklight_cmdq,
//#if (LCM_DSI_CMD_MODE)
//	.update = lcm_update,
//#endif

};

/* END PN:DTS2013053103858 , Added by d00238048, 2013.05.31*/
