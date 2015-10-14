/*
 * Copyright (c)  2012 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicensen
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */

#include "displays/otm8018b_vid_a450cg.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include <linux/gpio.h>
#include "psb_drv.h"
#include <linux/lnw_gpio.h>
#include <asm/intel_scu_pmic.h>
#include <linux/HWVersion.h>
#include "asus_panel_id.h"
#include <linux/microp_notify.h>
#include <linux/microp_pin_def.h>
#include <linux/microp_api.h>


extern int Read_HW_ID(void);
static int board_hw_id=0;

#define OTM8018B_PANEL_NAME	"OTM8018B"

#define OTM8018B_DEBUG 1
#define OTM8018B_HDP_TEST 1

static bool splendid_init = false;
/*
 * GPIO pin definition
 */
#define LCD_BL_EN 0x7E           //PMIC:BACKLIGHT_EN
#define LCD_LOGIC_PWR_EN 0x7F    //PMIC:PANEL_EN
#define LCD_ANALONG_PWR_EN 108   //CPU:GP_CORE_012

#define LCD_ANALONG_PWR_EN_ER 0xda    //PMIC:VEMMC2CNT for ER sku
#define LCD_ANALONG_PWR_EN_ER_OFF 0x4
#define LCD_ANALONG_PWR_EN_ER_AUTO 0x6

//change PANEL_ID , RESET_INNO for SR
//#define LCM_ID_SR 165          //GP_CORE_069 =69 + 96 =165
#define LCM_ID_SR 54             //LCM_ID : GP_SPI_3_CLK = GP_AON_054 = 54
#define RESET_INNO_SR 116        //GP_CORE_020 = 20 + 96 =116

//MIPI Control
//#define MIPI_SW_SEL 159 //GP_CORE_063 = 63 + 96 = 159
//#define MIPI_PWR_EN 40 // GP_XDP_BLK_DP = gp_aon_040 = 40

#define PWMCTRL_REG 0xffae9000
#define PWMCTRL_SIZE 0x80
static void __iomem *pwmctrl_mmio;
#define PWM_ENABLE_GPIO 49        //LED_BL_PWM
#define PWM_BASE_UNIT 0x444 //5,000Hz

union sst_pwmctrl_reg {
	struct {
		u32 pwmtd:8;
		u32 pwmbu:22;
		u32 pwmswupdate:1;
		u32 pwmenable:1;
	} part;
	u32 full;
};

static struct mdfld_dsi_config *otm8018b_dsi_config;

#define CMD_SIZE(x) (sizeof((x)) / sizeof((x)[0]))

static int send_mipi_cmd(
				struct mdfld_dsi_pkg_sender * sender,
				u8 * data,
				u32 len);
static int send_mipi_cmd2(
				struct mdfld_dsi_pkg_sender * sender,
				u8 * data,
				u32 len);
static int compare_mipi_reg(
				struct mdfld_dsi_pkg_sender * sender,
				u8 * data,
				u32 len,
				u8 * compare_data);

//NT35510S
static u8 cm0[]={ 0x10 };
static u8 cm1[]={ 0xF0,0x55,0xAA,0x52,0x08,0x01 };
static u8 cm2[]={ 0xB0,0x0D };
static u8 cm3[]={ 0xB6,0x44 };
static u8 cm4[]={ 0xB1,0x0D };
static u8 cm5[]={ 0xB7,0x34 };

static u8 cm6[]={ 0xB2,0x01 };
static u8 cm7[]={ 0xB8,0x24 };

static u8 cm8[]={ 0xB3,0x0C };
static u8 cm9[]={ 0xB9,0x34 };
static u8 cm10[]={ 0xBF,0x00 };
static u8 cm11[]={ 0xB5,0x0A };

static u8 cm12[]={ 0xBA,0x14 };

static u8 cm13[]={  0xBC,0x00,0x7C,0x00 };

static u8 cm14[]={ 0xBD,0x00,0x8F,0x00 };

static u8 cm15[]={ 0xBE,0x00,0x58 };

//static u8 cm16[]={ 0xD0,0x0F,0x0F,0x10,0x10 };

static u8 cm17[]={ 0xD1,0x00,0x01,0x00,0x22,0x00,0x59,0x00,0x76,0x00,0x8C,
	0x00,0xAF,0x00,0xC8,0x01,0x04,0x01,0x1D,0x01,0x59,0x01,0x86,0x01,0xCA,
	0x02,0x03,0x02,0x04,0x02,0x35,0x02,0x6C,0x02,0x8F,0x02,0xC6,0x02,0xF8,
	0x03,0x2A,0x03,0x52,0x03,0x75,0x03,0x8E,0x03,0x98,0x03,0xA0,0x03,0xA4 };

static u8 cm18[]={ 0xD2,0x00,0x01,0x00,0x22,0x00,0x59,0x00,0x76,0x00,0x8C,
	0x00,0xAF,0x00,0xC8,0x01,0x04,0x01,0x1D,0x01,0x59,0x01,0x86,0x01,0xCA,
	0x02,0x03,0x02,0x04,0x02,0x35,0x02,0x6C,0x02,0x8F,0x02,0xC6,0x02,0xF8,
	0x03,0x2A,0x03,0x52,0x03,0x75,0x03,0x8E,0x03,0x98,0x03,0xA0,0x03,0xA4 };

static u8 cm19[]={ 0xD3,0x00,0x01,0x00,0x22,0x00,0x59,0x00,0x76,0x00,0x8C,
	0x00,0xAF,0x00,0xC8,0x01,0x04,0x01,0x1D,0x01,0x59,0x01,0x86,0x01,0xCA,
	0x02,0x03,0x02,0x04,0x02,0x35,0x02,0x6C,0x02,0x8F,0x02,0xC6,0x02,0xF8,
	0x03,0x2A,0x03,0x52,0x03,0x75,0x03,0x8E,0x03,0x98,0x03,0xA0,0x03,0xA4 };

static u8 cm20[]={ 0xD4,0x00,0x01,0x00,0x22,0x00,0x59,0x00,0x76,0x00,0x8C,
	0x00,0xAF,0x00,0xC8,0x01,0x04,0x01,0x1D,0x01,0x59,0x01,0x86,0x01,0xCA,
	0x02,0x03,0x02,0x04,0x02,0x35,0x02,0x6C,0x02,0x8F,0x02,0xC6,0x02,0xF8,
	0x03,0x2A,0x03,0x52,0x03,0x75,0x03,0x8E,0x03,0x98,0x03,0xA0,0x03,0xA4 };

static u8 cm21[]={ 0xD5,0x00,0x01,0x00,0x22,0x00,0x59,0x00,0x76,0x00,0x8C,
	0x00,0xAF,0x00,0xC8,0x01,0x04,0x01,0x1D,0x01,0x59,0x01,0x86,0x01,0xCA,
	0x02,0x03,0x02,0x04,0x02,0x35,0x02,0x6C,0x02,0x8F,0x02,0xC6,0x02,0xF8,
	0x03,0x2A,0x03,0x52,0x03,0x75,0x03,0x8E,0x03,0x98,0x03,0xA0,0x03,0xA4 };

static u8 cm22[]={ 0xD6,0x00,0x01,0x00,0x22,0x00,0x59,0x00,0x76,0x00,0x8C,
0x00,0xAF,0x00,0xC8,0x01,0x04,0x01,0x1D,0x01,0x59,0x01,0x86,0x01,0xCA,0x02,
0x03,0x02,0x04,0x02,0x35,0x02,0x6C,0x02,0x8F,0x02,0xC6,0x02,0xF8,0x03,0x2A,
0x03,0x52,0x03,0x75,0x03,0x8E,0x03,0x98,0x03,0xA0,0x03,0xA4 };

static u8 cm23[]={ 0xF0,0x55,0xAA,0x52,0x08,0x00 };

static u8 cm24[]={ 0xB1,0xF8,0x00 };

static u8 cm25[]={ 0xB5,0x6B };

static u8 cm26[]={ 0xB6,0x05 };

static u8 cm27[]={ 0xB7,0x71,0x71 };

static u8 cm28[]={ 0xB8,0x02,0x05,0x05,0x05 };

static u8 cm29[]={ 0xB9,0x00,0x40 };

static u8 cm30[]={ 0xBA,0x05 };

static u8 cm31[]={ 0xBC,0x00,0x00,0x00 };

static u8 cm32[]={ 0xBD,0x01,0x78,0x14,0x14,0x00 };

static u8 cm33[]={ 0xC9,0xC0,0x02,0x50,0x50,0x50 };

//static u8 cm34[]={ 0x36,0x00 };
//static u8 cm35[]={ 0x2A,0x00,0x00,0x01,0xDF };
//static u8 cm36[]={ 0x2B,0x00,0x00,0x03,0x55 };
static u8 cm37[]={ 0x51,0xFF };
static u8 cm38[]={ 0x53,0x2C };
static u8 cm38_b[]={ 0x54,0x2C };

static u8 cm39[]={ 0x35,0x00 };

static u8 cm40[]={ 0x11 };
static u8 cm41[]={ 0x29 };

//initial code V1.0 start
//otm8018b
static u8 otm8018b_v1_cm1[] = {0x00,0x00};
static u8 otm8018b_v1_cm2[] = {0xff,0x80,0x09,0x01};

static u8 otm8018b_v1_cm3[] = {0x00,0x80};
static u8 otm8018b_v1_cm4[] = {0xFF,0x80,0x09};

static u8 otm8018b_v1_cm5[] = {0x00,0x80};   ///FF03
static u8 otm8018b_v1_cm6[] = {0xF5,0x01,0x18,0x02,0x18,0x10,0x18,0x02,0x18,0x0E,0x18,0x0F,0x20};


static u8 otm8018b_v1_cm7[] = {0x00,0x90};   ///C0B5
static u8 otm8018b_v1_cm8[] = {0xF5,0x02,0x18,0x08,0x18,0x06,0x18,0x0D,0x18,0x0B,0x18};


static u8 otm8018b_v1_cm9[] = {0x00,0xA0};   ///C0B4
static u8 otm8018b_v1_cm10[] = {0xF5,0x10,0x18,0x01,0x18,0x14,0x18,0x14,0x18};

static u8 otm8018b_v1_cm11[] = {0x00,0xB0};     //B391
static u8 otm8018b_v1_cm12[] = {0xF5,0x14,0x18,0x12,0x18,0x13,0x18,0x11,0x18,0x13,0x18,0x00,0x00};

static u8 otm8018b_v1_cm13[] = {0x00,0x80};    ///C489
static u8 otm8018b_v1_cm14[] = {0xc0,0x00,0x58,0x00,0x14,0x16};


static u8 otm8018b_v1_cm15[] = {0x00,0x8B};    //C582
static u8 otm8018b_v1_cm16[] = {0xB0,0x40};

static u8 otm8018b_v1_cm17[] = {0x00,0xB4};    //C590
static u8 otm8018b_v1_cm18[] = {0xc0,0x10};


static u8 otm8018b_v1_cm19[] = {0x00,0x82};    //D800
static u8 otm8018b_v1_cm20[] = {0xc5,0xA3};

static u8 otm8018b_v1_cm21[] = {0x00,0x90};    //C181
static u8 otm8018b_v1_cm22[] = {0xc5,0x96,0x38};

static u8 otm8018b_v1_cm23[] = {0x00,0x00};    //C1A0
static u8 otm8018b_v1_cm24[] = {0xD8,0x7F,0x7F};

static u8 otm8018b_v1_cm25[] = {0x00,0x00};     //C481
static u8 otm8018b_v1_cm26[] = {0xD9,0x68};

static u8 otm8018b_v1_cm27[] = {0x00,0x81};     //C592
static u8 otm8018b_v1_cm28[] = {0xC1,0x66};

static u8 otm8018b_v1_cm29[] = {0x00,0xA0};     //C5B1
static u8 otm8018b_v1_cm30[] = {0xC1,0xEA};

static u8 otm8018b_v1_cm31[] = {0x00,0xA1};     //C1A6
static u8 otm8018b_v1_cm32[] = {0xC1,0x08};

static u8 otm8018b_v1_cm33[] = {0x00,0xA3};     //C5C0
static u8 otm8018b_v1_cm34[] = {0xC0,0x1B};

static u8 otm8018b_v1_cm35[] = {0x00,0x80};      //B08B
static u8 otm8018b_v1_cm36[] = {0xC4,0x30};

static u8 otm8018b_v1_cm37[] = {0x00,0x8A};       //F5B2
static u8 otm8018b_v1_cm38[] = {0xC4,0x40};

static u8 otm8018b_v1_cm39[] = {0x00,0x81};       //C593
static u8 otm8018b_v1_cm40[] = {0xC4,0x83};

static u8 otm8018b_v1_cm41[] = {0x00,0x92};    //C090
static u8 otm8018b_v1_cm42[] = {0xC5,0x01,0x03};

static u8 otm8018b_v1_cm43[] = {0x00,0xB1};    //C1A6
static u8 otm8018b_v1_cm44[] = {0xc5,0xA9,0x15,0x00,0x15,0x00};

static u8 otm8018b_v1_cm45[] = {0x00,0xC0};    //CE80
static u8 otm8018b_v1_cm46[] = {0xc5,0x00};

static u8 otm8018b_v1_cm47[] = {0x00,0x90};     //CEA0
static u8 otm8018b_v1_cm48[] = {0xB3,0x02};

static u8 otm8018b_v1_cm49[] = {0x00,0x92};     //CEB0
static u8 otm8018b_v1_cm50[] = {0xB3,0x45};

static u8 otm8018b_v1_cm51[] = {0x00,0x40};    //CBC0
static u8 otm8018b_v1_cm52[] = {0xB3,0x45};

static u8 otm8018b_v1_cm53[] = {0x00,0x90};    //CBD0
static u8 otm8018b_v1_cm54[] = {0xc0,0x00,0x44,0x00,0x00,0x00,0x03};


static u8 otm8018b_v1_cm55[] = {0x00,0xa6};    //CC80
static u8 otm8018b_v1_cm56[] = {0xc1,0x01,0x00,0x00};

static u8 otm8018b_v1_cm57[] = {0x00,0xC6};    //CC9A
static u8 otm8018b_v1_cm58[] = {0xB0,0x03};

static u8 otm8018b_v1_cm59[] = {0x00,0x80};   //CCA1
static u8 otm8018b_v1_cm60[] = {0xce,0x85,0x03,0x00,0x84,0x03,0x00};

static u8 otm8018b_v1_cm61[] = {0x00,0x90};   //CCB0
static u8 otm8018b_v1_cm62[] = {0xce,0x33,0x5C,0x00,0x33,0x5D,0x00};

static u8 otm8018b_v1_cm63[] = {0x00,0xa0};   //CCB6
static u8 otm8018b_v1_cm64[] = {0xCE,0x38,0x03,0x03,0x56,0x00,0x00,0x00,0x38,0x02,0x03,0x57,0x00,0x00,0x00};

static u8 otm8018b_v1_cm65[] = {0x00,0xb0};
static u8 otm8018b_v1_cm66[] = {0xCE,0x38,0x01,0x03,0x58,0x00,0x00,0x00,0x38,0x00,0x03,0x59,0x00,0x00,0x00};

static u8 otm8018b_v1_cm67[] = {0x00,0xc0};
static u8 otm8018b_v1_cm68[] = {0xCE,0x30,0x00,0x03,0x5A,0x00,0x00,0x00,0x30,0x01,0x03,0x5B,0x00,0x00,0x00};

static u8 otm8018b_v1_cm69[] = {0x00,0xd0};
static u8 otm8018b_v1_cm70[] = {0xCE,0x30,0x02,0x03,0x5C,0x00,0x00,0x00,0x30,0x03,0x03,0x5D,0x00,0x00,0x00};

static u8 otm8018b_v1_cm71[] = {0x00,0xc7};
static u8 otm8018b_v1_cm72[] = {0xCF,0x00};

static u8 otm8018b_v1_cm73[] = {0x00,0xc0};
static u8 otm8018b_v1_cm74[] = {0xCB,0x00,0x00,0x00,0x00,0x54,0x54,0x54,0x54,0x00,0x54,0x00,0x54,0x00,0x00,0x00};

static u8 otm8018b_v1_cm75[] = {0x00,0xD0};
static u8 otm8018b_v1_cm76[] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x54,0x54,0x54,0x54,0x00,0x54};

static u8 otm8018b_v1_cm77[] = {0x00,0xE0};
static u8 otm8018b_v1_cm78[] = {0xCB,0x00,0x54,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

static u8 otm8018b_v1_cm79[] = {0x00,0x80};
static u8 otm8018b_v1_cm80[] = {0xCC,0x00,0x00,0x00,0x00,0x0c,0x0a,0x10,0x0e,0x00,0x02};

static u8 otm8018b_v1_cm81[] = {0x00,0x90};
static u8 otm8018b_v1_cm82[] = {0xCC,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0b};

static u8 otm8018b_v1_cm83[] = {0x00,0xA0};
static u8 otm8018b_v1_cm84[] = {0xCC,0x09,0x0f,0x0d,0x00,0x01,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

static u8 otm8018b_v1_cm85[] = {0x00,0xB0};
static u8 otm8018b_v1_cm86[] = {0xCC,0x00,0x00,0x00,0x00,0x0d,0x0f,0x09,0x0b,0x00,0x05};

static u8 otm8018b_v1_cm87[] = {0x00,0xC0};
static u8 otm8018b_v1_cm88[] = {0xCC,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0e};

static u8 otm8018b_v1_cm89[] = {0x00,0xD0};
static u8 otm8018b_v1_cm90[] = {0xCC,0x10,0x0a,0x0c,0x00,0x06,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

/// GAMMA START
static u8 otm8018b_v1_cm91[] = {0x00,0x00};
static u8 otm8018b_v1_cm92[] = {0xE1,0x01,0x09,0x0e,0x0d,0x06,0x0c,0x0B,0x09,0x04,0x07,0x10,0x07,0x0d,0x0c,0x08,0x06};


static u8 otm8018b_v1_cm93[] = {0x00,0x00};
static u8 otm8018b_v1_cm94[] = {0xE2,0x01,0x08,0x0e,0x0d,0x06,0x0c,0x0B,0x0A,0x04,0x07,0x10,0x08,0x0E,0x0c,0x08,0x06};

// v1.0 end

static u8 otm8018b_orise_off_1[] = {0x00,0x00};
static u8 otm8018b_orise_off_2[] = {0xff,0x00,0x00,0x00};

static u8 otm8018b_orise_off_3[] = {0x00,0x80};
static u8 otm8018b_orise_off_4[] = {0xff,0x00,0x00};

#if 0
static u8 otm8018b_bist1[] = {0x00,0xd0};
static u8 otm8018b_bist2[] = {0xb3,0x10};

static u8 otm8018b_bist3[] = {0x00,0xd1};
static u8 otm8018b_bist4[] = {0xb3,0xff};

static u8 otm8018b_bist5[] = {0x00,0xd2};
static u8 otm8018b_bist6[] = {0xb3,0x00};

static u8 otm8018b_bist7[] = {0x00,0xd3};
static u8 otm8018b_bist8[] = {0xb3,0x00};
#endif

//NT35510
static u8 ccm0[]={ 0x10 };
static u8 ccm1[]={ 0xF0,0x55,0xAA,0x52,0x08,0x01 };
static u8 ccm2[]={ 0xB0,0x0D,0x0D,0x0D };
static u8 ccm3[]={ 0xB6,0x44,0x44,0x44 };
static u8 ccm4[]={ 0xB1,0x0D,0x0D,0x0D };
static u8 ccm5[]={ 0xB7,0x34,0x34,0x34 };
static u8 ccm6[]={ 0xB2,0x01,0x01,0x01 };
static u8 ccm7[]={ 0xB8,0x24,0x24,0x24 };
static u8 ccm8[]={ 0xB3,0x0C,0x0C,0x0C };
static u8 ccm9[]={ 0xB9,0x34,0x34,0x34 };
static u8 ccm10[]={ 0xBF,0x00 };
static u8 ccm11[]={ 0xB5,0x0A,0x0A,0x0A };
static u8 ccm12[]={ 0xBA,0x14,0x14,0x14 };
static u8 ccm13[]={  0xBC,0x00,0x7C,0x00 };
static u8 ccm14[]={ 0xBD,0x00,0x8F,0x00 };
static u8 ccm15[]={ 0xBE,0x00,0x58 };
static u8 ccm16[]={ 0xD0,0x0F,0x0F,0x10,0x10 };
static u8 ccm17[]={ 0xD1,0x00,0x01,0x00,0x22,0x00,0x59,0x00,0x76,0x00,0x8C,
	0x00,0xAF,0x00,0xC8,0x01,0x04,0x01,0x1D,0x01,0x59,0x01,0x86,0x01,0xCA,
	0x02,0x03,0x02,0x04,0x02,0x35,0x02,0x6C,0x02,0x8F,0x02,0xC6,0x02,0xF8,
	0x03,0x2A,0x03,0x52,0x03,0x75,0x03,0x8E,0x03,0x98,0x03,0xA0,0x03,0xA4 };
static u8 ccm18[]={ 0xD2,0x00,0x01,0x00,0x22,0x00,0x59,0x00,0x76,0x00,0x8C,
	0x00,0xAF,0x00,0xC8,0x01,0x04,0x01,0x1D,0x01,0x59,0x01,0x86,0x01,0xCA,
	0x02,0x03,0x02,0x04,0x02,0x35,0x02,0x6C,0x02,0x8F,0x02,0xC6,0x02,0xF8,
	0x03,0x2A,0x03,0x52,0x03,0x75,0x03,0x8E,0x03,0x98,0x03,0xA0,0x03,0xA4 };
static u8 ccm19[]={ 0xD3,0x00,0x01,0x00,0x22,0x00,0x59,0x00,0x76,0x00,0x8C,
	0x00,0xAF,0x00,0xC8,0x01,0x04,0x01,0x1D,0x01,0x59,0x01,0x86,0x01,0xCA,
	0x02,0x03,0x02,0x04,0x02,0x35,0x02,0x6C,0x02,0x8F,0x02,0xC6,0x02,0xF8,
	0x03,0x2A,0x03,0x52,0x03,0x75,0x03,0x8E,0x03,0x98,0x03,0xA0,0x03,0xA4 };
static u8 ccm20[]={ 0xD4,0x00,0x01,0x00,0x22,0x00,0x59,0x00,0x76,0x00,0x8C,
	0x00,0xAF,0x00,0xC8,0x01,0x04,0x01,0x1D,0x01,0x59,0x01,0x86,0x01,0xCA,
	0x02,0x03,0x02,0x04,0x02,0x35,0x02,0x6C,0x02,0x8F,0x02,0xC6,0x02,0xF8,
	0x03,0x2A,0x03,0x52,0x03,0x75,0x03,0x8E,0x03,0x98,0x03,0xA0,0x03,0xA4 };
static u8 ccm21[]={ 0xD5,0x00,0x01,0x00,0x22,0x00,0x59,0x00,0x76,0x00,0x8C,
	0x00,0xAF,0x00,0xC8,0x01,0x04,0x01,0x1D,0x01,0x59,0x01,0x86,0x01,0xCA,
	0x02,0x03,0x02,0x04,0x02,0x35,0x02,0x6C,0x02,0x8F,0x02,0xC6,0x02,0xF8,
	0x03,0x2A,0x03,0x52,0x03,0x75,0x03,0x8E,0x03,0x98,0x03,0xA0,0x03,0xA4 };
static u8 ccm22[]={ 0xD6,0x00,0x01,0x00,0x22,0x00,0x59,0x00,0x76,0x00,0x8C,
	0x00,0xAF,0x00,0xC8,0x01,0x04,0x01,0x1D,0x01,0x59,0x01,0x86,0x01,0xCA,0x02,
	0x03,0x02,0x04,0x02,0x35,0x02,0x6C,0x02,0x8F,0x02,0xC6,0x02,0xF8,0x03,0x2A,
	0x03,0x52,0x03,0x75,0x03,0x8E,0x03,0x98,0x03,0xA0,0x03,0xA4 };
static u8 ccm23[]={ 0xF0,0x55,0xAA,0x52,0x08,0x00 };
static u8 ccm24[]={ 0xB1,0xF8,0x00 };
static u8 ccm25[]={ 0xB5,0x70 };
static u8 ccm26[]={ 0xB6,0x05 };
static u8 ccm27[]={ 0xB7,0x71,0x71 };
static u8 ccm28[]={ 0xB8,0x02,0x05,0x05,0x05 };
static u8 ccm29[]={ 0xB9,0x00,0x40 };
static u8 ccm30[]={ 0xBA,0x05 };
static u8 ccm31[]={ 0xBC,0x00,0x00,0x00 };
static u8 ccm32[]={ 0xBD,0x01,0x78,0x14,0x14,0x00 }; //porch
static u8 ccm33[]={ 0xC9,0xC0,0x02,0x50,0x50,0x50 };
static u8 ccm34[]={ 0x36,0x00 };
static u8 ccm35[]={ 0x2A,0x00,0x00,0x01,0xDF };
static u8 ccm36[]={ 0x2B,0x00,0x00,0x03,0x55 };
static u8 ccm37[]={ 0x51,0xFF };
static u8 ccm38[]={ 0x53,0x2C };
static u8 ccm38b[]={ 0x54,0x2C };
static u8 ccm39[]={ 0x35,0x00 };
static u8 ccm40[]={ 0x11 };
static u8 ccm41[]={ 0x29 };

static int pwm_configure(int duty)
{
	union sst_pwmctrl_reg pwmctrl;

	/*Read the PWM register to make sure there is no pending
	*update.
	*/
	pwmctrl.full = readl(pwmctrl_mmio);

	/*check pwnswupdate bit */
	if (pwmctrl.part.pwmswupdate)
		return -EBUSY;
	pwmctrl.part.pwmswupdate = 0x1;
	pwmctrl.part.pwmbu = PWM_BASE_UNIT;
	pwmctrl.part.pwmtd = duty;
	writel(pwmctrl.full,  pwmctrl_mmio);

	return 0;
}


static void pwm_enable(){
	union sst_pwmctrl_reg pwmctrl;

	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);

	/*Enable the PWM by setting PWM enable bit to 1 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 1;
	writel(pwmctrl.full, pwmctrl_mmio);
}

static void pwm_disable(){
	union sst_pwmctrl_reg pwmctrl;
	/*setting PWM enable bit to 0 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 0;
	writel(pwmctrl.full,  pwmctrl_mmio);

	gpio_set_value(PWM_ENABLE_GPIO, 0);
	lnw_gpio_set_alt(PWM_ENABLE_GPIO, 0);
}

//normal
static int otm8018b_send_mipi_cmd(
				struct mdfld_dsi_pkg_sender * sender,
				u8 * data,
				u32 len){

	int r = 0, i;

	switch(len){
		case 1:
			r = mdfld_dsi_send_gen_short_lp(sender, data[0],0, 1, 0);
			break;
		case 2:
			r = mdfld_dsi_send_gen_short_lp(sender, data[0], data[1], 2, 0);;
			break;
		default:


#if 0
		sender->status = MDFLD_DSI_PKG_SENDER_FREE;
		printk("%s : %d len = %d,0x%02x",__func__,r,len,data[0]);
		for(i=1;i<len;i++){
			printk(",0x%02x", data[i]);
		}
		printk("\n");
#endif
				
			r = mdfld_dsi_send_gen_long_lp(sender, data, len, 0);
			break;
	}
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL){
		printk("%s : MDFLD_DSI_CONTROL_ABNORMAL\n",__func__);
		return -EIO;
	}
	else{
		return 0;
	}
}


static int otm8018b_send_mipi_cmd_long2short(
				struct mdfld_dsi_pkg_sender * sender,
				u8 * data1,u8 * data2,
				u32 len, u32 len2){
	int ret =0;
	u8 data3[20]={0};
	int i,r=0;

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	if(len2<=2){

		otm8018b_send_mipi_cmd(sender,data1,len);
		otm8018b_send_mipi_cmd(sender,data2,len2);

	}else{

#if 0
		printk("%s : %d len = %d,0x%02x,0x%02x\n",__func__,r,len2,data1[0],data1[1]);
		printk("%s : %d len = %d,0x%02x",__func__,r,len2,data2[0]);
		for(i=1;i<len2;i++){
			printk(",0x%02x", data2[i]);
		}
		printk("\n");
#endif

		for(i = 0 ; i<(len2-1) ;i++ ){

			r = mdfld_dsi_send_gen_short_lp(sender, 0x0     , data1[1]+i, 2, 0);
			r = mdfld_dsi_send_gen_short_lp(sender, data2[0], data2[i+1]  , 2, 0);
		}

	}

#if 0
	printk("-----------------------\n");
	r = mdfld_dsi_send_gen_short_lp(sender, 0x0 , data1[1], 2, 0);
	r = mdfld_dsi_read_gen_lp(sender,data2[0],0,1,data3, len-1);

	printk("read: %d, 0x%02x",r,data2[0]);
	for(i=0;i<len-1;i++){
		printk(" 0x%02x", data3[i]);
	}
	printk("\n");
#endif

	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL){
		printk("%s : MDFLD_DSI_CONTROL_ABNORMAL\n",__func__);
		return -EIO;
	}
	else{
		return 0;
	}
}

static int otm8018b_send_mipi_cmd_long2short_gamma(
				struct mdfld_dsi_pkg_sender * sender,
				u8 * data1,u8 * data2,
				u32 len){

	int ret =0;
	u8 data3[20]={0};
	int i,r=0;

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

#if 0
	printk("%s : %d len = %d,0x%02x,0x%02x\n",__func__,r,len,data1[0],data1[1]);
	printk("%s : %d len = %d,0x%02x",__func__,r,len,data2[0]);
	for(i=1;i<len;i++){
		printk(",0x%02x", data2[i]);
	}
	printk("\n");
#endif

	r = mdfld_dsi_send_gen_short_lp(sender, 0x0 , 0x0, 2, 0);

	for(i = 0 ; i<(len-1) ; i++ ){
		r = mdfld_dsi_send_gen_short_lp(sender, data2[0], data2[i+1]  , 2, 0);
	}
	r = mdfld_dsi_send_gen_short_lp(sender, 0x0 , 0x0, 2, 0);

	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL){
		printk("%s : MDFLD_DSI_CONTROL_ABNORMAL\n",__func__);
		return -EIO;
	}
	else{
		return 0;
	}

}


#define LP_RETRY 1

static int otm8018b_mipi_cmd_setting(struct mdfld_dsi_config *dsi_config){

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
    int x0=0;

	int r = 0;
	u8 data[10]={0};
	u8 rdata;
	int retry_times;
	int reset_inno;
	static u8 read_data = 0;
	int read_ret = 0;
	read_data = 0x0;

	reset_inno = RESET_INNO_SR;


	retry_times=LP_RETRY;

otm8018b_retry:

	retry_times--;

    mdfld_dsi_read_gen_lp(sender,0xa,0x0,1,&read_data,1);

	//RESX : L > H > L > H

	if (gpio_direction_output(reset_inno, 0))
		gpio_set_value_cansleep(reset_inno, 0);

	usleep_range(1000, 1000);

	if (gpio_direction_output(reset_inno, 1))
		gpio_set_value_cansleep(reset_inno, 1);

	usleep_range(1000, 1000);

	if (gpio_direction_output(reset_inno, 0))
		gpio_set_value_cansleep(reset_inno, 0);

	usleep_range(1000, 1000);

	if (gpio_direction_output(reset_inno, 1))
		gpio_set_value_cansleep(reset_inno, 1);

	usleep_range(5000, 5000);

	if(asus_panel_id == 0x51) {
		if(board_hw_id == 0x6) {	//ER2
			send_mipi_cmd2(sender,ccm0,CMD_SIZE(ccm0));
			send_mipi_cmd2(sender,ccm1,CMD_SIZE(ccm1));
			send_mipi_cmd2(sender,ccm2,CMD_SIZE(ccm2));
			r = compare_mipi_reg(sender,ccm2,CMD_SIZE(ccm2)-1,ccm2);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm3,CMD_SIZE(ccm3));
			r=compare_mipi_reg(sender,ccm3,CMD_SIZE(ccm3)-1,ccm3);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm4,CMD_SIZE(ccm4));
			r=compare_mipi_reg(sender,ccm4,CMD_SIZE(ccm4)-1,ccm4);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm5,CMD_SIZE(ccm5));
			r=compare_mipi_reg(sender,ccm5,CMD_SIZE(ccm5)-1,ccm5);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm6,CMD_SIZE(ccm6));
			r=compare_mipi_reg(sender,ccm6,CMD_SIZE(ccm6)-1,ccm6);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm7,CMD_SIZE(ccm7));
			r=compare_mipi_reg(sender,ccm7,CMD_SIZE(ccm7)-1,ccm7);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm8,CMD_SIZE(ccm8));
			r=compare_mipi_reg(sender,ccm8,CMD_SIZE(ccm8)-1,ccm8);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm9,CMD_SIZE(ccm9));
			r=compare_mipi_reg(sender,ccm9,CMD_SIZE(ccm9)-1,ccm9);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm10,CMD_SIZE(ccm10));
			r=compare_mipi_reg(sender,ccm10,CMD_SIZE(ccm10)-1,ccm10);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm11,CMD_SIZE(ccm11));
			r=compare_mipi_reg(sender,ccm11,CMD_SIZE(ccm11)-1,ccm11);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm12,CMD_SIZE(ccm12));
			r=compare_mipi_reg(sender,ccm12,CMD_SIZE(ccm12)-1,ccm12);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm13,CMD_SIZE(ccm13));
			r=compare_mipi_reg(sender,ccm13,CMD_SIZE(ccm13)-1,ccm13);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm14,CMD_SIZE(ccm14));
			r=compare_mipi_reg(sender,ccm14,CMD_SIZE(ccm14)-1,ccm14);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm15,CMD_SIZE(ccm15));
			r=compare_mipi_reg(sender,ccm15,CMD_SIZE(ccm15)-1,ccm15);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm16,CMD_SIZE(ccm16));
			r=compare_mipi_reg(sender,ccm16,CMD_SIZE(ccm16)-1,ccm16);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm17,CMD_SIZE(ccm17));
			r=compare_mipi_reg(sender,ccm17,CMD_SIZE(ccm17)-1,ccm17);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm18,CMD_SIZE(ccm18));
			r=compare_mipi_reg(sender,ccm18,CMD_SIZE(ccm18)-1,ccm18);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm19,CMD_SIZE(ccm19));
			r=compare_mipi_reg(sender,ccm19,CMD_SIZE(ccm19)-1,ccm19);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm20,CMD_SIZE(ccm20));
			r=compare_mipi_reg(sender,ccm20,CMD_SIZE(ccm20)-1,ccm20);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm21,CMD_SIZE(ccm21));
			r=compare_mipi_reg(sender,ccm21,CMD_SIZE(ccm21)-1,ccm21);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm22,CMD_SIZE(ccm22));
			r=compare_mipi_reg(sender,ccm22,CMD_SIZE(ccm22)-1,ccm22);
			if(r) return -EIO;
			//========== page1 relative ==========
			send_mipi_cmd2(sender,ccm23,CMD_SIZE(ccm23));
			send_mipi_cmd2(sender,ccm24,CMD_SIZE(ccm24));
			r=compare_mipi_reg(sender,ccm24,CMD_SIZE(ccm24)-1,ccm24);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm25,CMD_SIZE(ccm25));
			r=compare_mipi_reg(sender,ccm25,CMD_SIZE(ccm25)-1,ccm25);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm26,CMD_SIZE(ccm26));
			r=compare_mipi_reg(sender,ccm26,CMD_SIZE(ccm26)-1,ccm26);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm27,CMD_SIZE(ccm27));
			r=compare_mipi_reg(sender,ccm27,CMD_SIZE(ccm27)-1,ccm27);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm28,CMD_SIZE(ccm28));
			r=compare_mipi_reg(sender,ccm28,CMD_SIZE(ccm28)-1,ccm28);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm29,CMD_SIZE(ccm29));
			r=compare_mipi_reg(sender,ccm29,CMD_SIZE(ccm29)-1,ccm29);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm30,CMD_SIZE(ccm30));
			r=compare_mipi_reg(sender,ccm30,CMD_SIZE(ccm30)-1,ccm30);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm31,CMD_SIZE(ccm31));
			r=compare_mipi_reg(sender,ccm31,CMD_SIZE(ccm31)-1,ccm31);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm32,CMD_SIZE(ccm32));
			r=compare_mipi_reg(sender,ccm32,CMD_SIZE(ccm32)-1,ccm32);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm33,CMD_SIZE(ccm33));
			r=compare_mipi_reg(sender,ccm33,CMD_SIZE(ccm33)-1,ccm33);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm34,CMD_SIZE(ccm34));
			r=compare_mipi_reg(sender,ccm34,CMD_SIZE(ccm34)-1,ccm34);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm35,CMD_SIZE(ccm35));
			r=compare_mipi_reg(sender,ccm35,CMD_SIZE(ccm35)-1,ccm35);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm36,CMD_SIZE(ccm36));
			r=compare_mipi_reg(sender,ccm36,CMD_SIZE(ccm36)-1,ccm36);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm37,CMD_SIZE(ccm37));
			r=compare_mipi_reg(sender,ccm37,CMD_SIZE(ccm37)-1,ccm37);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm38,CMD_SIZE(ccm38));
			r=compare_mipi_reg(sender,ccm38b,CMD_SIZE(ccm38b)-1,ccm38b);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm39,CMD_SIZE(ccm39));
			r=compare_mipi_reg(sender,ccm39,CMD_SIZE(ccm39)-1,ccm39);
			if(r) return -EIO;
			send_mipi_cmd2(sender,ccm40,CMD_SIZE(ccm40));
			usleep_range(160000, 160000);
			send_mipi_cmd2(sender,ccm41,CMD_SIZE(ccm41));
			usleep_range(5000, 5000);
			mdfld_dsi_read_gen_lp(sender,0xa,0x0,1,&read_data,1);
			printk("TCON IC : NT35510\n");
		}
		else {
			send_mipi_cmd(sender,cm0,CMD_SIZE(cm0));
			send_mipi_cmd2(sender,cm1,CMD_SIZE(cm1));
			send_mipi_cmd(sender,cm2,CMD_SIZE(cm2));
			send_mipi_cmd(sender,cm3,CMD_SIZE(cm3));
			send_mipi_cmd(sender,cm4,CMD_SIZE(cm4));
			send_mipi_cmd(sender,cm5,CMD_SIZE(cm5));
			send_mipi_cmd(sender,cm6,CMD_SIZE(cm6));
			send_mipi_cmd(sender,cm7,CMD_SIZE(cm7));
			send_mipi_cmd(sender,cm8,CMD_SIZE(cm8));
			send_mipi_cmd(sender,cm9,CMD_SIZE(cm9));
			send_mipi_cmd(sender,cm10,CMD_SIZE(cm10));
			send_mipi_cmd(sender,cm11,CMD_SIZE(cm11));
			send_mipi_cmd(sender,cm12,CMD_SIZE(cm12));
			send_mipi_cmd(sender,cm13,CMD_SIZE(cm13));
			send_mipi_cmd(sender,cm14,CMD_SIZE(cm14));
			send_mipi_cmd(sender,cm15,CMD_SIZE(cm15));
			send_mipi_cmd2(sender,cm17,CMD_SIZE(cm17));
			r = compare_mipi_reg(sender,cm17,CMD_SIZE(cm17)-1,cm17);
			if(r) return -EIO;
			send_mipi_cmd2(sender,cm18,CMD_SIZE(cm18));
			r = compare_mipi_reg(sender,cm18,CMD_SIZE(cm18)-1,cm18);
			if(r) return -EIO;
			send_mipi_cmd2(sender,cm19,CMD_SIZE(cm19));
			r = compare_mipi_reg(sender,cm19,CMD_SIZE(cm19)-1,cm19);
			if(r) return -EIO;
			send_mipi_cmd2(sender,cm20,CMD_SIZE(cm20));
			r = compare_mipi_reg(sender,cm20,CMD_SIZE(cm20)-1,cm20);
			if(r) return -EIO;
			send_mipi_cmd2(sender,cm21,CMD_SIZE(cm21));
			r = compare_mipi_reg(sender,cm21,CMD_SIZE(cm21)-1,cm21);
			if(r) return -EIO;
			send_mipi_cmd2(sender,cm22,CMD_SIZE(cm22));
			r = compare_mipi_reg(sender,cm22,CMD_SIZE(cm22)-1,cm22);
			if(r) return -EIO;

			//page
			send_mipi_cmd2(sender,cm23,CMD_SIZE(cm23));
			send_mipi_cmd(sender,cm24,CMD_SIZE(cm24));
			send_mipi_cmd(sender,cm25,CMD_SIZE(cm25));
			send_mipi_cmd(sender,cm26,CMD_SIZE(cm26));
			send_mipi_cmd(sender,cm27,CMD_SIZE(cm27));
			send_mipi_cmd(sender,cm28,CMD_SIZE(cm28));
			send_mipi_cmd(sender,cm29,CMD_SIZE(cm29));
			send_mipi_cmd(sender,cm30,CMD_SIZE(cm30));
			send_mipi_cmd(sender,cm31,CMD_SIZE(cm31));
			send_mipi_cmd(sender,cm32,CMD_SIZE(cm32));
			send_mipi_cmd(sender,cm33,CMD_SIZE(cm33));
			send_mipi_cmd(sender,cm37,CMD_SIZE(cm37));
			send_mipi_cmd(sender,cm38,CMD_SIZE(cm38));
			r = compare_mipi_reg(sender,cm38_b,CMD_SIZE(cm38_b)-1,cm38_b);
			if(r) return -EIO;
			send_mipi_cmd(sender,cm39,CMD_SIZE(cm39));
			send_mipi_cmd(sender,cm40,CMD_SIZE(cm40));

			usleep_range(120000, 120000);
			send_mipi_cmd(sender,cm41,CMD_SIZE(cm41));

			usleep_range(5000, 5000);
			printk("TCON IC : NT35510S\n");
		}
	}
	else {
		otm8018b_send_mipi_cmd(sender,otm8018b_v1_cm1,CMD_SIZE(otm8018b_v1_cm1));
		otm8018b_send_mipi_cmd(sender,otm8018b_v1_cm2,CMD_SIZE(otm8018b_v1_cm2));

		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm3,otm8018b_v1_cm4,CMD_SIZE(otm8018b_v1_cm3),CMD_SIZE(otm8018b_v1_cm4));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm5,otm8018b_v1_cm6,CMD_SIZE(otm8018b_v1_cm5),CMD_SIZE(otm8018b_v1_cm6));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm7,otm8018b_v1_cm8,CMD_SIZE(otm8018b_v1_cm7),CMD_SIZE(otm8018b_v1_cm8));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm9,otm8018b_v1_cm10,CMD_SIZE(otm8018b_v1_cm9),CMD_SIZE(otm8018b_v1_cm10));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm11,otm8018b_v1_cm12,CMD_SIZE(otm8018b_v1_cm11),CMD_SIZE(otm8018b_v1_cm12));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm13,otm8018b_v1_cm14,CMD_SIZE(otm8018b_v1_cm13),CMD_SIZE(otm8018b_v1_cm14));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm15,otm8018b_v1_cm16,CMD_SIZE(otm8018b_v1_cm15),CMD_SIZE(otm8018b_v1_cm16));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm17,otm8018b_v1_cm18,CMD_SIZE(otm8018b_v1_cm17),CMD_SIZE(otm8018b_v1_cm18));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm19,otm8018b_v1_cm20,CMD_SIZE(otm8018b_v1_cm19),CMD_SIZE(otm8018b_v1_cm20));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm21,otm8018b_v1_cm22,CMD_SIZE(otm8018b_v1_cm21),CMD_SIZE(otm8018b_v1_cm22));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm23,otm8018b_v1_cm24,CMD_SIZE(otm8018b_v1_cm23),CMD_SIZE(otm8018b_v1_cm24));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm25,otm8018b_v1_cm26,CMD_SIZE(otm8018b_v1_cm25),CMD_SIZE(otm8018b_v1_cm26));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm27,otm8018b_v1_cm28,CMD_SIZE(otm8018b_v1_cm27),CMD_SIZE(otm8018b_v1_cm28));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm29,otm8018b_v1_cm30,CMD_SIZE(otm8018b_v1_cm29),CMD_SIZE(otm8018b_v1_cm30));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm31,otm8018b_v1_cm32,CMD_SIZE(otm8018b_v1_cm31),CMD_SIZE(otm8018b_v1_cm32));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm33,otm8018b_v1_cm34,CMD_SIZE(otm8018b_v1_cm33),CMD_SIZE(otm8018b_v1_cm34));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm35,otm8018b_v1_cm36,CMD_SIZE(otm8018b_v1_cm35),CMD_SIZE(otm8018b_v1_cm36));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm37,otm8018b_v1_cm38,CMD_SIZE(otm8018b_v1_cm37),CMD_SIZE(otm8018b_v1_cm38));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm39,otm8018b_v1_cm40,CMD_SIZE(otm8018b_v1_cm39),CMD_SIZE(otm8018b_v1_cm40));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm41,otm8018b_v1_cm42,CMD_SIZE(otm8018b_v1_cm41),CMD_SIZE(otm8018b_v1_cm42));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm43,otm8018b_v1_cm44,CMD_SIZE(otm8018b_v1_cm43),CMD_SIZE(otm8018b_v1_cm44));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm45,otm8018b_v1_cm46,CMD_SIZE(otm8018b_v1_cm45),CMD_SIZE(otm8018b_v1_cm46));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm47,otm8018b_v1_cm48,CMD_SIZE(otm8018b_v1_cm47),CMD_SIZE(otm8018b_v1_cm48));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm49,otm8018b_v1_cm50,CMD_SIZE(otm8018b_v1_cm49),CMD_SIZE(otm8018b_v1_cm50));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm51,otm8018b_v1_cm52,CMD_SIZE(otm8018b_v1_cm51),CMD_SIZE(otm8018b_v1_cm52));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm53,otm8018b_v1_cm54,CMD_SIZE(otm8018b_v1_cm53),CMD_SIZE(otm8018b_v1_cm54));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm55,otm8018b_v1_cm56,CMD_SIZE(otm8018b_v1_cm55),CMD_SIZE(otm8018b_v1_cm56));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm57,otm8018b_v1_cm58,CMD_SIZE(otm8018b_v1_cm57),CMD_SIZE(otm8018b_v1_cm58));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm59,otm8018b_v1_cm60,CMD_SIZE(otm8018b_v1_cm59),CMD_SIZE(otm8018b_v1_cm60));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm61,otm8018b_v1_cm62,CMD_SIZE(otm8018b_v1_cm61),CMD_SIZE(otm8018b_v1_cm62));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm63,otm8018b_v1_cm64,CMD_SIZE(otm8018b_v1_cm63),CMD_SIZE(otm8018b_v1_cm64));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm65,otm8018b_v1_cm66,CMD_SIZE(otm8018b_v1_cm65),CMD_SIZE(otm8018b_v1_cm66));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm67,otm8018b_v1_cm68,CMD_SIZE(otm8018b_v1_cm67),CMD_SIZE(otm8018b_v1_cm68));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm69,otm8018b_v1_cm70,CMD_SIZE(otm8018b_v1_cm69),CMD_SIZE(otm8018b_v1_cm70));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm71,otm8018b_v1_cm72,CMD_SIZE(otm8018b_v1_cm71),CMD_SIZE(otm8018b_v1_cm72));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm73,otm8018b_v1_cm74,CMD_SIZE(otm8018b_v1_cm73),CMD_SIZE(otm8018b_v1_cm74));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm75,otm8018b_v1_cm76,CMD_SIZE(otm8018b_v1_cm75),CMD_SIZE(otm8018b_v1_cm76));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm77,otm8018b_v1_cm78,CMD_SIZE(otm8018b_v1_cm77),CMD_SIZE(otm8018b_v1_cm78));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm79,otm8018b_v1_cm80,CMD_SIZE(otm8018b_v1_cm79),CMD_SIZE(otm8018b_v1_cm80));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm81,otm8018b_v1_cm82,CMD_SIZE(otm8018b_v1_cm81),CMD_SIZE(otm8018b_v1_cm82));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm83,otm8018b_v1_cm84,CMD_SIZE(otm8018b_v1_cm83),CMD_SIZE(otm8018b_v1_cm84));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm85,otm8018b_v1_cm86,CMD_SIZE(otm8018b_v1_cm85),CMD_SIZE(otm8018b_v1_cm86));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm87,otm8018b_v1_cm88,CMD_SIZE(otm8018b_v1_cm87),CMD_SIZE(otm8018b_v1_cm88));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_v1_cm89,otm8018b_v1_cm90,CMD_SIZE(otm8018b_v1_cm89),CMD_SIZE(otm8018b_v1_cm90));
		otm8018b_send_mipi_cmd_long2short_gamma(sender,otm8018b_v1_cm91,otm8018b_v1_cm92,CMD_SIZE(otm8018b_v1_cm92));
		otm8018b_send_mipi_cmd_long2short_gamma(sender,otm8018b_v1_cm93,otm8018b_v1_cm94,CMD_SIZE(otm8018b_v1_cm94));


		//Orise mode off ++
		otm8018b_send_mipi_cmd(sender,otm8018b_orise_off_1,CMD_SIZE(otm8018b_orise_off_1));
		otm8018b_send_mipi_cmd(sender,otm8018b_orise_off_2,CMD_SIZE(otm8018b_orise_off_2));
		otm8018b_send_mipi_cmd_long2short(sender,otm8018b_orise_off_3,otm8018b_orise_off_4,CMD_SIZE(otm8018b_orise_off_3),CMD_SIZE(otm8018b_orise_off_4));
		//Orise mode off --
		//READ for checking disable ++
		mdfld_dsi_send_gen_short_lp(sender,0x00,0xb4,2,0);

		read_ret = mdfld_dsi_read_gen_lp(sender,0xc0,0x0,1,&read_data,1);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
			read_ret = -EIO;
		printk("[A450CG] reg 0x%x = 0x%x (ret = %d)\n",0xc0,read_data,read_ret);

		if(read_data != 0x00){
			if(!retry_times) return -EIO;
			goto otm8018b_retry;
		}

		mdfld_dsi_send_gen_short_lp(sender,0x11,0x00,1,0);

		printk("TCON IC : OTM8018B\n");
		//READ for checking disable --
#if 0
		otm8018b_send_mipi_cmd(sender,otm8018b_bist1,CMD_SIZE(otm8018b_bist1));
		otm8018b_send_mipi_cmd(sender,otm8018b_bist2,CMD_SIZE(otm8018b_bist2));
		otm8018b_send_mipi_cmd(sender,otm8018b_bist3,CMD_SIZE(otm8018b_bist3));
		otm8018b_send_mipi_cmd(sender,otm8018b_bist4,CMD_SIZE(otm8018b_bist4));
		otm8018b_send_mipi_cmd(sender,otm8018b_bist5,CMD_SIZE(otm8018b_bist5));
		otm8018b_send_mipi_cmd(sender,otm8018b_bist6,CMD_SIZE(otm8018b_bist6));
#endif
	}

	return 0;

}


//change long cmd to short cmd
static int send_mipi_cmd(
				struct mdfld_dsi_pkg_sender * sender,
				u8 * data,
				u32 len){

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	switch(len){
		case 1:
			mdfld_dsi_send_mcs_short_lp(sender, data[0],0, 0, 0);
			break;
		case 2:
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[1], 1, 0);
			break;
		case 3:
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x00, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[1], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x01, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[2], 1, 0);
			break;
		case 4:
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x00, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[1], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x01, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[2], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x02, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[3], 1, 0);
			break;
		case 5:
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x00, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[1], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x01, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[2], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x02, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[3], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x03, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[4], 1, 0);
			break;
		case 6:
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x00, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[1], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x01, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[2], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x02, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[3], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x03, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[4], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x04, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[5], 1, 0);
			break;

		case 7:
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x00, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[1], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x01, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[2], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x02, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[3], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x03, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[4], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x04, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[5], 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, 0x6f, 0x05, 1, 0);
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[6], 1, 0);
			break;
		default:
			mdfld_dsi_send_mcs_long_lp(sender, data, len, 0);
			break;
	}
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL){
		printk("%s : MDFLD_DSI_CONTROL_ABNORMAL\n",__func__);
		return -EIO;
	}
	else{
		return 0;
	}
}

//normal
static int send_mipi_cmd2(
				struct mdfld_dsi_pkg_sender * sender,
				u8 * data,
				u32 len){

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	switch(len){
		case 1:
			mdfld_dsi_send_mcs_short_lp(sender, data[0],0, 0, 0);
			break;
		case 2:
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[1], 1, 0);;
			break;
		default:
			mdfld_dsi_send_mcs_long_lp(sender, data, len, 0);
			break;
	}
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL){
		printk("%s : MDFLD_DSI_CONTROL_ABNORMAL\n",__func__);
		return -EIO;
	}
	else{
		return 0;
	}

}

//compare
static int compare_mipi_reg(
				struct mdfld_dsi_pkg_sender * sender,
				u8 * data,
				u32 len,
				u8 * compare_data){
	int ret =0;
	int retry_times=LP_RETRY;
	u8 data2[60]={0};
	int i,r=0;

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	r = mdfld_dsi_read_mcs_lp(sender,data[0],data2, len);

	ret = memcmp(data2, compare_data + 1, len);
	if (!ret){
		return 0;
	}else{
		printk("%s : %d, %02x,",__func__,r,data[0]);
		for(i=0;i<len;i++){
			printk(" %02x", data2[i]);
		}
		printk(" (compare fail, retry again)\n");
		return -EIO;
	}
}

static int dds_mipi_cmd_setting(struct mdfld_dsi_config *dsi_config){
	int ret = -1;

	ret = otm8018b_mipi_cmd_setting(dsi_config);
	DRM_INFO("[DISPLAY] %s: End\n", __func__);

	return ret;
}

static void
otm8018b_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	struct drm_device *dev = dsi_config->dev;
	struct csc_setting csc = {	.pipe = 0,
								.type = CSC_REG_SETTING,
								.enable_state = true,
								.data_len = CSC_REG_COUNT,
								.data.csc_reg_data = {
									0x400, 0x0, 0x4000000, 0x0, 0x0, 0x400}
							 };
	struct gamma_setting gamma = {	.pipe = 0,
									.type = GAMMA_REG_SETTING,
									.enable_state = true,
									.data_len = GAMMA_10_BIT_TABLE_COUNT,
									.gamma_tableX100 = {
										0x000000, 0x020202, 0x040404, 0x060606,
										0x080808, 0x0A0A0A, 0x0C0C0C, 0x0E0E0E,
										0x101010, 0x121212, 0x141414, 0x161616,
										0x181818, 0x1A1A1A, 0x1C1C1C, 0x1E1E1E,
										0x202020, 0x222222, 0x242424, 0x262626,
										0x282828, 0x2A2A2A, 0x2C2C2C, 0x2E2E2E,
										0x303030, 0x323232, 0x343434, 0x363636,
										0x383838, 0x3A3A3A, 0x3C3C3C, 0x3E3E3E,
										0x404040, 0x424242, 0x444444, 0x464646,
										0x484848, 0x4A4A4A, 0x4C4C4C, 0x4E4E4E,
										0x505050, 0x525252, 0x545454, 0x565656,
										0x585858, 0x5A5A5A, 0x5C5C5C, 0x5E5E5E,
										0x606060, 0x626262, 0x646464, 0x666666,
										0x686868, 0x6A6A6A, 0x6C6C6C, 0x6E6E6E,
										0x707070, 0x727272, 0x747474, 0x767676,
										0x787878, 0x7A7A7A, 0x7C7C7C, 0x7E7E7E,
										0x808080, 0x828282, 0x848484, 0x868686,
										0x888888, 0x8A8A8A, 0x8C8C8C, 0x8E8E8E,
										0x909090, 0x929292, 0x949494, 0x969696,
										0x989898, 0x9A9A9A, 0x9C9C9C, 0x9E9E9E,
										0xA0A0A0, 0xA2A2A2, 0xA4A4A4, 0xA6A6A6,
										0xA8A8A8, 0xAAAAAA, 0xACACAC, 0xAEAEAE,
										0xB0B0B0, 0xB2B2B2, 0xB4B4B4, 0xB6B6B6,
										0xB8B8B8, 0xBABABA, 0xBCBCBC, 0xBEBEBE,
										0xC0C0C0, 0xC2C2C2, 0xC4C4C4, 0xC6C6C6,
										0xC8C8C8, 0xCACACA, 0xCCCCCC, 0xCECECE,
										0xD0D0D0, 0xD2D2D2, 0xD4D4D4, 0xD6D6D6,
										0xD8D8D8, 0xDADADA, 0xDCDCDC, 0xDEDEDE,
										0xE0E0E0, 0xE2E2E2, 0xE4E4E4, 0xE6E6E6,
										0xE8E8E8, 0xEAEAEA, 0xECECEC, 0xEEEEEE,
										0xF0F0F0, 0xF2F2F2, 0xF4F4F4, 0xF6F6F6,
										0xF8F8F8, 0xFAFAFA, 0xFCFCFC, 0xFEFEFE,
										0x010000, 0x010000, 0x010000}
								 };
	dsi_config->lane_count = 2;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_2_2;
	dsi_config->enable_gamma_csc = ENABLE_GAMMA | ENABLE_CSC;

	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;
	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;
	hw_ctx->turn_around_timeout = 0x3f;
	hw_ctx->device_reset_timer = 0xffff;
	if(asus_panel_id == 0x51 && board_hw_id == 0x6)
		hw_ctx->high_low_switch_count = 0x00000fff;
	else
		hw_ctx->high_low_switch_count = 0x00000013;
	hw_ctx->init_count = 0x7d0;
	hw_ctx->eot_disable = 0x3;
	hw_ctx->lp_byteclk = 0x3;
	hw_ctx->clk_lane_switch_time_cnt = 0x00130009;
	if(asus_panel_id == 0x51 && board_hw_id == 0x6)
		hw_ctx->dphy_param = 0x1f15340a;
	else
		hw_ctx->dphy_param = 0x150c340a;

	/* Setup video mode format */
	if(asus_panel_id == 0x51 && board_hw_id == 0x6)
		hw_ctx->video_mode_format = 0x7;
	else
		hw_ctx->video_mode_format = 0xf;

	/* Set up func_prg, RGB888(0x200) */
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);

	/* Setup mipi port configuration */
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;

	if(!splendid_init){
		if (dsi_config->enable_gamma_csc & ENABLE_CSC) {
			/* setting the tuned csc setting */
			drm_psb_enable_color_conversion = 1;
			mdfld_intel_crtc_set_color_conversion(dev, &csc);
		}

		if (dsi_config->enable_gamma_csc & ENABLE_GAMMA) {
			/* setting the tuned gamma setting */
			drm_psb_enable_gamma = 1;
			mdfld_intel_crtc_set_gamma(dev, &gamma);
		}
	}
	splendid_init = true;

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
}

static int otm8018b_vid_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val, dphy_val;
	int pipe = dsi_config->pipe;

	if (pipe == 0) {
		/*
		 * FIXME: WA to detect the panel connection status, and need to
		 * implement detection feature with get_power_mode DSI command.
		 */
		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
			OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("hw begin failed\n");
			return -EAGAIN;
		}

		dpll_val = REG_READ(regs->dpll_reg);
		device_ready_val = REG_READ(regs->device_ready_reg);
		if ((device_ready_val & DSI_DEVICE_READY) &&
		    (dpll_val & DPLL_VCO_ENABLE)) {
			dsi_config->dsi_hw_context.panel_on = true;
			psb_enable_vblank(dev, pipe);
		} else {
			dsi_config->dsi_hw_context.panel_on = false;
			DRM_INFO("%s: panel is not detected!\n", __func__);
		}

		status = MDFLD_DSI_PANEL_CONNECTED;
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return status;
}

static int otm8018b_vid_gpio_control(int on){

	int reset_inno;

	reset_inno = RESET_INNO_SR;

	if(on){
		usleep_range(1000, 2000);

		if((board_hw_id == 0x0)||(board_hw_id == 0x4)){
			intel_scu_ipc_iowrite8(LCD_LOGIC_PWR_EN,0x1);

			usleep_range(10000, 10000);

			if (gpio_direction_output(LCD_ANALONG_PWR_EN, 1))
				gpio_set_value_cansleep(LCD_ANALONG_PWR_EN, 1);
		}
		else
			intel_scu_ipc_iowrite8(LCD_ANALONG_PWR_EN_ER,LCD_ANALONG_PWR_EN_ER_AUTO);
	}else{

		if (gpio_direction_output(reset_inno, 0))
			gpio_set_value_cansleep(reset_inno, 0);

		usleep_range(1000, 2000);

		if((board_hw_id == 0x0)||(board_hw_id == 0x4)){
			if (gpio_direction_output(LCD_ANALONG_PWR_EN, 0))
				gpio_set_value_cansleep(LCD_ANALONG_PWR_EN, 0);

			intel_scu_ipc_iowrite8(LCD_LOGIC_PWR_EN,0x0);
		}
		else
			intel_scu_ipc_iowrite8(LCD_ANALONG_PWR_EN_ER,LCD_ANALONG_PWR_EN_ER_OFF);
	}

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	return 0;

}

static int otm8018b_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	if(asus_panel_id == 0x50) {
		usleep_range(120000, 120000);

		mdfld_dsi_send_gen_short_lp(sender,0x29,0x00,1,0);
	}

	/* Send TURN_ON packet */
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
				    MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}

	pwm_enable();

	/* LCD_BL_EN*/
	intel_scu_ipc_iowrite8(LCD_BL_EN,0x1);

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	return 0;
}

static int otm8018b_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;
	int reset_inno;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	pwm_disable();

	mdfld_dsi_send_gen_short_lp(sender,0x10,0x00,1,0); //sleep in

	usleep_range(120000, 120000);

	/* LCD_BL_EN*/
	intel_scu_ipc_iowrite8(LCD_BL_EN,0x0);

	mdfld_dsi_send_gen_short_lp(sender,0x28,0x00,1,0); //display off

	/* Send SHUT_DOWN packet */
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
				    MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		return err;
	}

	usleep_range(10000, 10000);

	otm8018b_vid_gpio_control(0);

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	return 0;
}

static int otm8018b_vid_reset(struct mdfld_dsi_config *dsi_config)
{
	otm8018b_vid_gpio_control(1);
	pr_debug("[DISPLAY] %s: Enter\n", __func__);

	return 0;
}

#define PWM0DUTYCYCLE 0x67
#define DUTY_VALUE_MAX	0x63

static int bl_prev_level = 0;
static int otm8018b_vid_set_brightness(struct mdfld_dsi_config *dsi_config,
					 int level)
{
	int duty_val = 0;
	int ret = 0;
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	if(board_hw_id == 0x0){
		duty_val = level==1?0:level;
	}else if(board_hw_id == 0x4){
		duty_val = level==1?0:level;
	}else if(board_hw_id == 0x2){
		duty_val = level==1?0:level;
	}else{
		duty_val = (level==1?0:level);
	}

	pwm_configure(duty_val);

	//add for debug ++
	if(!!bl_prev_level^!!duty_val)
		DRM_INFO("[DISPLAY] brightness level : %d > %d\n", bl_prev_level, duty_val);

	bl_prev_level = duty_val;
	//add for debug --

	PSB_DEBUG_ENTRY("level = %d , duty_val = %d\n", level, duty_val);

	return 0;
}

struct drm_display_mode *otm8018b_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;
	mode->hdisplay = 480;
	mode->vdisplay = 854;

	if(asus_panel_id == 0x51 && board_hw_id == 0x6) {
		mode->hsync_start = mode->hdisplay + 60;
		mode->hsync_end = mode->hsync_start + 18;
		mode->htotal = mode->hsync_end + 60;
		mode->vsync_start = mode->vdisplay + 50;
		mode->vsync_end = mode->vsync_start + 14;
		mode->vtotal = mode->vsync_end + 50;

		mode->vrefresh = 60;
		//for CTS refresh rate
		mode->clock = mode->vrefresh * (854+60+4+60) * (480+80+4+80) / 1000;
		printk("Porch : NT35510\n");
	}
	else {
		mode->hsync_start = mode->hdisplay + 50;
		mode->hsync_end = mode->hsync_start + 4;
		mode->htotal = mode->hsync_end + 48;
		mode->vsync_start = mode->vdisplay + 16;
		mode->vsync_end = mode->vsync_start + 4;
		mode->vtotal = mode->vsync_end + 15;

		mode->vrefresh = 60;
		//for CTS refresh rate
		mode->clock = mode->vrefresh * mode->vtotal * mode->htotal / 1000;
		printk("Porch : OTM8018B & NT35510S\n");
	}
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	pr_debug("[DISPLAY] %s: Enter A450\n", __func__);
	return mode;
}

static void otm8018b_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	pi->width_mm = 55;
	pi->height_mm = 99;
}

static int otm8018b_vid_gpio_init(void)
{
/*
	if( board_hw_id == HW_ID_SR1)
		gpio_request(LCM_ID_SR,"LCM_ID_SR");
	else
*/
	gpio_request(LCM_ID_SR,"LCM_ID_SR");


	gpio_request(RESET_INNO_SR,"RESET_INNO_SR");

	if((board_hw_id == 0x0)||(board_hw_id == 0x4))
		gpio_request(LCD_ANALONG_PWR_EN,"LCD_ANALONG_PWR_EN");

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return 0;
}

#define PWM0CLKDIV1 0x61
#define PWM0CLKDIV0 0x62

static int otm8018b_vid_brightness_init(void)
{
	int ret = 0;

	pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG,PWMCTRL_SIZE);
	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return ret;
}

#ifdef OTM8018B_DEBUG

static int send_mipi_ret = -1;
static int read_mipi_ret = -1;
static u8 read_mipi_data = 0;

static ssize_t send_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0, x1=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(otm8018b_dsi_config);

    sscanf(buf, "%x,%x", &x0, &x1);

    send_mipi_ret = mdfld_dsi_send_mcs_short_lp(sender,x0,x1,1,0);

	DRM_INFO("[DISPLAY] send %x,%x : ret = %d\n",x0,x1,send_mipi_ret);

    return count;
}

static ssize_t send_mipi_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",send_mipi_ret);
}


static ssize_t read_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(otm8018b_dsi_config);

    sscanf(buf, "%x", &x0);

    read_mipi_ret = mdfld_dsi_read_mcs_lp(sender,x0,&read_mipi_data,1);
    if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
        read_mipi_ret = -EIO;

	DRM_INFO("[DISPLAY] read 0x%x :ret=%d data=0x%x\n", x0, read_mipi_ret, read_mipi_data);

    return count;
}

static ssize_t read_mipi_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
	return snprintf(buf, PAGE_SIZE, "ret=%d data=0x%x\n",read_mipi_ret,read_mipi_data);
}

DEVICE_ATTR(send_mipi_otm8018b,S_IRUGO | S_IWUSR, send_mipi_show,send_mipi_store);
DEVICE_ATTR(read_mipi_otm8018b,S_IRUGO | S_IWUSR, read_mipi_show,read_mipi_store);

static struct attribute *otm8018b_attrs[] = {
        &dev_attr_send_mipi_otm8018b.attr,
        &dev_attr_read_mipi_otm8018b.attr,
        NULL
};

static struct attribute_group otm8018b_attr_group = {
        .attrs = otm8018b_attrs,
        .name = "otm8018b",
};

#endif

static int init_asus_panel_id(){
	int panel_id_value=0;

/*
	if( board_hw_id == HW_ID_SR1)
		panel_id_value = gpio_get_value(LCM_ID_SR)?0x1:0x0;
	else
*/
	panel_id_value = gpio_get_value(LCM_ID_SR)?0x1:0x0;

	asus_panel_id = A450CG_PANEL | panel_id_value;

	DRM_INFO("[DISPLAY] %s: asus_panel_id = 0x%x\n", __func__, asus_panel_id);

	return 0;
}

void otm8018b_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) dev->dev_private;
	struct mdfld_dsi_config *dsi_config = dev_priv->dsi_configs[0];

	int ret = 0;

	p_funcs->get_config_mode = otm8018b_vid_get_config_mode;
	p_funcs->get_panel_info = otm8018b_vid_get_panel_info;
	p_funcs->dsi_controller_init = otm8018b_vid_dsi_controller_init;
	p_funcs->detect = otm8018b_vid_detect;
	p_funcs->power_on = otm8018b_vid_power_on;
	p_funcs->drv_ic_init = dds_mipi_cmd_setting;
	p_funcs->power_off = otm8018b_vid_power_off;
	p_funcs->reset = otm8018b_vid_reset;
	p_funcs->set_brightness = otm8018b_vid_set_brightness;

	board_hw_id = Read_HW_ID();

	ret = otm8018b_vid_gpio_init();
	if (ret)
		DRM_ERROR("Faild to request GPIO for OTM8018B panel\n");

	ret = otm8018b_vid_brightness_init();
	if (ret)
		DRM_ERROR("Faild to initilize PWM of MSCI\n");

	init_asus_panel_id();

#ifdef OTM8018B_DEBUG

    sysfs_create_group(&dev->dev->kobj, &otm8018b_attr_group);

#endif

	otm8018b_dsi_config = dsi_config;

	DRM_INFO("[DISPLAY] %s: board_hw_id = %d\n", __func__, board_hw_id);

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

}

static int otm8018b_vid_lcd_probe(struct platform_device *pdev)
{
	int ret = 0;

	DRM_INFO("%s: otm8018b panel detected\n", __func__);
	intel_mid_panel_register(otm8018b_vid_init);


	return 0;
}

struct platform_driver a450cg_otm8018b_vid_lcd_driver = {
	.probe	= otm8018b_vid_lcd_probe,
	.driver	= {
		.name	= OTM8018B_PANEL_NAME,
		.owner	= THIS_MODULE,
	},
};
