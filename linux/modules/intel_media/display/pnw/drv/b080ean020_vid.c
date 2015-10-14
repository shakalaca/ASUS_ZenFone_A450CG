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

#include "displays/b080ean020_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include <linux/gpio.h>
#include "psb_drv.h"
#include <linux/lnw_gpio.h>
#include <asm/intel_scu_pmic.h>
#include <linux/HWVersion.h>
#include "asus_panel_id.h"

#define B080EAN020_PANEL_NAME	"B080EAN020"
//#define USE_AUO

#define LCD_BL_EN 0x7E           //PMIC:BACKLIGHT_EN
#define LCD_LOGIC_PWR_EN 0x7F    //PMIC:PANEL_EN
#define LCD_ANALONG_PWR_EN 108   //CPU:GP_CORE_012
#define RESET_INNO 116        //GP_CORE_020 = 20 + 96 =116
#define PANEL_ID 112          //GP_CORE_016 =16 + 96 =112

#define PWM_ENABLE_GPIO 49
#define PWM_BASE_UNIT 0x444 //5,000Hz

#define B080EAN020_DEBUG
//#define NT35521_BIST

extern int Read_PROJ_ID(void);
extern int Read_HW_ID(void);
static int proj_id = 0;
static int board_hw_id = 0;
static int panel_id; // 0:Innolux(NT35521), 1:AUO(S6D7AA0X04)

static struct mdfld_dsi_config *b080ean020_dsi_config;

#define CMD_SIZE(x) (sizeof((x)) / sizeof((x)[0]))

//======= S6D7AA0X04 mipi (AUO: High) ============================
static u8 s6d7aa0x04_passwd1[]={0xF0, 0x5A, 0x5A};
static u8 s6d7aa0x04_passwd1_data[]={0x5A, 0x5A};
//static u8 s6d7aa0x04_sleep_out[]={0x11};
//static u8 s6d7aa0x04_display_on[]={0x29};
static u8 s6d7aa0x04_enable_LED_IC[]={0xC3, 0x40, 0x00, 0x28};
static u8 s6d7aa0x04_enable_LED_IC_data[]={0x40, 0x00, 0x28};
//static u8 s6d7aa0x04_disable_LED_IC[]={0xC3, 0x40, 0x00, 0x20};
//static u8 s6d7aa0x04_disable_LED_IC_data[]={0x40, 0x00, 0x20};

//======= NT35521_v0.06b mipi (INX: Low) =========================
static u8 nt35521v06b_cm1[]={0xFF,0xAA,0x55,0xA5,0x80};
static u8 nt35521v06b_cm2[]={ 0x6F,0x11,0x00 };
static u8 nt35521v06b_cm3[]={ 0xF7,0x20,0x00 };
static u8 nt35521v06b_cm4[]={ 0x6F,0x06 };
static u8 nt35521v06b_cm5[]={ 0xF7,0xA0 };
static u8 data_nt35521v06b_cm5[]={ 0xA0 };

static u8 nt35521v06b_cm6[]={ 0x6F,0x19 };
static u8 nt35521v06b_cm7[]={ 0xF7,0x12 };

static u8 nt35521v06b_cm8[]={ 0x6F,0x08 };
static u8 nt35521v06b_cm9[]={ 0xFA,0x40 };
static u8 nt35521v06b_cm10[]={ 0x6F,0x11 };
static u8 nt35521v06b_cm11[]={ 0xF3,0x01 };

static u8 nt35521v06b_ths_prepare1[]={0x6F,0x01};
static u8 nt35521v06b_ths_prepare2[]={0xf7,0x03};

static u8 nt35521v06b_cm1_off[]={0xFF,0xAA,0x55,0xA5,0x00};

//========== page0 relative ==========
static u8 nt35521v06b_cm12[]={ 0xF0,0x55,0xAA,0x52,0x08,0x00 };
static u8 nt35521v06b_cm13[]={  0xC8, 0x80 };

static u8 nt35521v06b_cm14[]={ 0xB1,0x68,0x01 };

static u8 nt35521v06b_cm15[]={ 0xB6,0x08 };

static u8 nt35521v06b_cm16[]={ 0x6F,0x02 };
static u8 nt35521v06b_cm17[]={ 0xB8,0x08 };

static u8 nt35521v06b_cm18[]={ 0xBB,0x74,0x44 };

static u8 nt35521v06b_cm19[]={ 0xBC,0x00,0x00 };

static u8 nt35521v06b_cm20[]={ 0xBD, 0x02,0xB0,0x0C,0x0A,0x00 };

//========== page1 relative ==========
static u8 nt35521v06b_cm21[]={ 0xF0,0x55,0xAA,0x52,0x08,0x01 };

static u8 nt35521v06b_cm22[]={ 0xB0,0x05,0x05 };
static u8 nt35521v06b_cm23[]={ 0xB1,0x05,0x05 };

static u8 nt35521v06b_cm24[]={ 0xBC,0x90,0x01 };
static u8 nt35521v06b_cm25[]={ 0xBD,0x90,0x01 };

static u8 nt35521v06b_cm26[]={ 0xCA,0x00 };

static u8 nt35521v06b_cm27[]={ 0xC0,0x04 };

static u8 nt35521v06b_cm28[]={ 0xBE,0x29 };

static u8 nt35521v06b_cm29[]={ 0xB3,0x37,0x37 };
static u8 nt35521v06b_cm30[]={ 0xB4,0x19,0x19 };

static u8 nt35521v06b_cm31[]={ 0xB9,0x46,0x46 };
static u8 nt35521v06b_cm32[]={ 0xBA,0x24,0x24 };

//========== page2 relative ==========
static u8 nt35521v06b_cm33[]={ 0xF0,0x55,0xAA,0x52,0x08,0x02 };
static u8 nt35521v06b_cm34[]={ 0xEE,0x01 };
static u8 nt35521v06b_cm35[]={ 0xEF,0x09,0x06,0x15,0x18 };
static u8 data_nt35521v06b_cm35[]={ 0x09,0x06,0x15,0x18 };

static u8 nt35521v06b_cm36[]={ 0xB0,0x00,0x00,0x00,0x25,0x00,0x43 };
static u8 nt35521v06b_cm37[]={ 0x6F,0x06 };
static u8 nt35521v06b_cm38[]={ 0xB0,0x00,0x54,0x00,0x68,0x00,0xA0 };
static u8 nt35521v06b_cm39[]={ 0x6F,0x0C };
static u8 nt35521v06b_cm40[]={ 0xB0,0x00,0xC0,0x01,0x00 };
static u8 data_nt35521v06b_cm36_40[]={0x00,0x00,0x00,0x25,0x00,0x43,
					   0x00,0x54,0x00,0x68,0x00,0xA0,
					   0x00,0xC0,0x01,0x00};

static u8 nt35521v06b_cm41[]={ 0xB1,0x01,0x30,0x01,0x78,0x01,0xAE };
static u8 nt35521v06b_cm42[]={ 0x6F,0x06 };
static u8 nt35521v06b_cm43[]={ 0xB1,0x02,0x08,0x02,0x50,0x02,0x52 };
static u8 nt35521v06b_cm44[]={ 0x6F,0x0C };
static u8 nt35521v06b_cm45[]={ 0xB1,0x02,0x96,0x02,0xDC };
static u8 data_nt35521v06b_cm41_45[]={0x01,0x30,0x01,0x78,0x01,0xAE,
					   0x02,0x08,0x02,0x50,0x02,0x52,
					   0x02,0x96,0x02,0xDC};

static u8 nt35521v06b_cm46[]={ 0xB2,0x03,0x08,0x03,0x49,0x03,0x77 };
static u8 nt35521v06b_cm47[]={ 0x6F,0x06 };
static u8 nt35521v06b_cm48[]={ 0xB2,0x03,0xA3,0x03,0xAC,0x03,0xC2 };
static u8 nt35521v06b_cm49[]={ 0x6F,0x0C };
static u8 nt35521v06b_cm50[]={ 0xB2,0x03,0xC9,0x03,0xE3 };
static u8 data_nt35521v06b_cm46_50[]={0x03,0x08,0x03,0x49,0x03,0x77,
					   0x03,0xA3,0x03,0xAC,0x03,0xC2,
					   0x03,0xC9,0x03,0xE3};

static u8 nt35521v06b_cm51[]={ 0xB3,0x03,0xFC,0x03,0xFF };
static u8 data_nt35521v06b_cm51[]={ 0x03,0xFC,0x03,0xFF };


// PAGE6 : GOUT Mapping, VGLO select
static u8 nt35521v06b_cm52[]={  0xF0, 0x55,0xAA,0x52,0x08,0x06 };
static u8 nt35521v06b_cm53[]={  0xB0, 0x00,0x10 };
static u8 nt35521v06b_cm54[]={  0xB1, 0x12,0x14 };
static u8 nt35521v06b_cm55[]={  0xB2, 0x16,0x18 };
static u8 nt35521v06b_cm56[]={  0xB3, 0x1A,0x29 };
static u8 nt35521v06b_cm57[]={  0xB4, 0x2A,0x08 };
static u8 nt35521v06b_cm58[]={  0xB5, 0x31,0x31 };
static u8 nt35521v06b_cm59[]={  0xB6, 0x31,0x31 };
static u8 nt35521v06b_cm60[]={  0xB7, 0x31,0x31 };
static u8 nt35521v06b_cm61[]={  0xB8, 0x31,0x0A };
static u8 nt35521v06b_cm62[]={  0xB9, 0x31,0x31 };
static u8 nt35521v06b_cm63[]={  0xBA, 0x31,0x31 };
static u8 nt35521v06b_cm64[]={  0xBB, 0x0B,0x31 };
static u8 nt35521v06b_cm65[]={  0xBC, 0x31,0x31 };
static u8 nt35521v06b_cm66[]={  0xBD, 0x31,0x31 };
static u8 nt35521v06b_cm67[]={  0xBE, 0x31,0x31 };
static u8 nt35521v06b_cm68[]={  0xBF, 0x09,0x2A };
static u8 nt35521v06b_cm69[]={  0xC0, 0x29,0x1B };
static u8 nt35521v06b_cm70[]={  0xC1, 0x19,0x17 };
static u8 nt35521v06b_cm71[]={  0xC2, 0x15,0x13 };
static u8 nt35521v06b_cm72[]={  0xC3, 0x11,0x01 };
static u8 nt35521v06b_cm73[]={  0xE5, 0x31,0x31 };
static u8 nt35521v06b_cm74[]={  0xC4, 0x09,0x1B };
static u8 nt35521v06b_cm75[]={  0xC5, 0x19,0x17 };
static u8 nt35521v06b_cm76[]={  0xC6, 0x15,0x13 };
static u8 nt35521v06b_cm77[]={  0xC7, 0x11,0x29 };
static u8 nt35521v06b_cm78[]={  0xC8, 0x2A,0x01 };
static u8 nt35521v06b_cm79[]={  0xC9, 0x31,0x31 };
static u8 nt35521v06b_cm80[]={  0xCA, 0x31,0x31 };
static u8 nt35521v06b_cm81[]={  0xCB, 0x31,0x31 };
static u8 nt35521v06b_cm82[]={  0xCC, 0x31,0x0B };
static u8 nt35521v06b_cm83[]={  0xCD, 0x31,0x31 };
static u8 nt35521v06b_cm84[]={  0xCE, 0x31,0x31 };
static u8 nt35521v06b_cm85[]={  0xCF, 0x0A,0x31 };
static u8 nt35521v06b_cm86[]={  0xD0, 0x31,0x31 };
static u8 nt35521v06b_cm87[]={  0xD1, 0x31,0x31 };
static u8 nt35521v06b_cm88[]={  0xD2, 0x31,0x31 };
static u8 nt35521v06b_cm89[]={  0xD3, 0x00,0x2A };
static u8 nt35521v06b_cm90[]={  0xD4, 0x29,0x10 };
static u8 nt35521v06b_cm91[]={  0xD5, 0x12,0x14 };
static u8 nt35521v06b_cm92[]={  0xD6, 0x16,0x18 };
static u8 nt35521v06b_cm93[]={  0xD7, 0x1A,0x08 };
static u8 nt35521v06b_cm94[]={  0xE6, 0x31,0x31 };
static u8 nt35521v06b_cm95[]={  0xD8, 0x00,0x00,0x00,0x54,0x00 };
static u8 nt35521v06b_cm96[]={  0xD9, 0x00,0x15,0x00,0x00,0x00 };
static u8 nt35521v06b_cm97[]={  0xE7, 0x00 };

// PAGE3 :
static u8 nt35521v06b_cm98[]={  0xF0, 0x55,0xAA,0x52,0x08,0x03 };
static u8 nt35521v06b_cm99[]={  0xB0, 0x20,0x00 };
static u8 nt35521v06b_cm100[]={  0xB1, 0x20,0x00 };
static u8 nt35521v06b_cm101[]={  0xB2, 0x05,0x00,0x00,0x00,0x00 };

static u8 nt35521v06b_cm102[]={  0xB6, 0x05,0x00,0x00,0x00,0x00 };
static u8 nt35521v06b_cm103[]={  0xB7, 0x05,0x00,0x00,0x00,0x00 };

static u8 nt35521v06b_cm104[]={  0xBA, 0x57,0x00,0x00,0x00,0x00 };
static u8 nt35521v06b_cm105[]={  0xBB, 0x57,0x00,0x00,0x00,0x00 };

static u8 nt35521v06b_cm106[]={  0xC0, 0x00,0x00,0x00,0x00 };
static u8 nt35521v06b_cm107[]={  0xC1, 0x00,0x00,0x00,0x00 };

static u8 nt35521v06b_cm108[]={  0xC4, 0x60 };
static u8 nt35521v06b_cm109[]={  0xC5, 0x40 };

// PAGE5 :
static u8 nt35521v06b_cm110[]={  0xF0, 0x55,0xAA,0x52,0x08,0x05 };
static u8 nt35521v06b_cm111[]={  0xBD, 0x03,0x01,0x03,0x03,0x03 };
static u8 nt35521v06b_cm112[]={  0xB0, 0x17,0x06 };
static u8 nt35521v06b_cm113[]={  0xB1, 0x17,0x06 };
static u8 nt35521v06b_cm114[]={  0xB2, 0x17,0x06 };
static u8 nt35521v06b_cm115[]={  0xB3, 0x17,0x06 };
static u8 nt35521v06b_cm116[]={  0xB4, 0x17,0x06 };
static u8 nt35521v06b_cm117[]={  0xB5, 0x17,0x06 };

static u8 nt35521v06b_cm118[]={  0xB8, 0x00 };
static u8 nt35521v06b_cm119[]={  0xB9, 0x00 };
static u8 nt35521v06b_cm120[]={  0xBA, 0x00 };
static u8 nt35521v06b_cm121[]={  0xBB, 0x02 };
static u8 nt35521v06b_cm122[]={  0xBC, 0x00 };

static u8 nt35521v06b_cm123[]={  0xC0, 0x07 };

static u8 nt35521v06b_cm124[]={  0xC4, 0x81 };
static u8 nt35521v06b_cm125[]={  0xC5, 0xA3 };

static u8 nt35521v06b_cm126[]={  0xC8, 0x05,0x30 };
static u8 nt35521v06b_cm127[]={  0xC9, 0x01,0x31 };

static u8 nt35521v06b_cm128[]={  0xCC, 0x00,0x00,0x3C };
static u8 nt35521v06b_cm129[]={  0xCD, 0x00,0x00,0x3C };

static u8 nt35521v06b_cm130[]={  0xD1, 0x00,0x04,0xFD,0x07,0x10 };
static u8 nt35521v06b_cm131[]={  0xD2, 0x00,0x05,0x02,0x07,0x10 };

static u8 nt35521v06b_cm132[]={  0xE5, 0x06 };
static u8 nt35521v06b_cm133[]={  0xE6, 0x06 };
static u8 nt35521v06b_cm134[]={  0xE7, 0x06 };
static u8 nt35521v06b_cm135[]={  0xE8, 0x06 };
static u8 nt35521v06b_cm136[]={  0xE9, 0x06 };
static u8 nt35521v06b_cm137[]={  0xEA, 0x06 };

static u8 nt35521v06b_cm138[]={  0xED, 0x30 };

static u8 nt35521v06b_cm139[]={  0x6F, 0x11 };
static u8 nt35521v06b_cm140[]={  0xF3, 0x01 };

static u8 nt35521v06b_cm141[]={  0x35 };
static u8 nt35521v06b_cm142[]={  0x11 };
static u8 nt35521v06b_cm143[]={  0x29 };

#ifdef NT35521_BIST
static u8 bist_cm1[]={  0xF0, 0x55,0xAA,0x52,0x08,0x00  };
static u8 bist_cm2[]={  0xEF, 0x03,0xFF  };
static u8 bist_cm3[]={  0xEE, 0x87,0x78,0x02,0x40 };
#endif
//================================================================

union sst_pwmctrl_reg {
	struct {
		u32 pwmtd:8;
		u32 pwmbu:22;
		u32 pwmswupdate:1;
		u32 pwmenable:1;
	} part;
	u32 full;
};

static void __iomem *pwmctrl_mmio;

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
	writel(pwmctrl.full, pwmctrl_mmio);

	return 0;
}

static void pwm_enable(void) {
	union sst_pwmctrl_reg pwmctrl;

	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);
	/*Enable the PWM by setting PWM enable bit to 1 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 1;
	writel(pwmctrl.full, pwmctrl_mmio);
}

static void pwm_disable(void) {
	union sst_pwmctrl_reg pwmctrl;
	/*setting PWM enable bit to 0 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 0;
	writel(pwmctrl.full, pwmctrl_mmio);

	gpio_set_value(PWM_ENABLE_GPIO, 0);
	lnw_gpio_set_alt(PWM_ENABLE_GPIO, 0);
}

//change long cmd to short cmd
static int send_mipi_cmd(
				struct mdfld_dsi_pkg_sender * sender,
				u8 * data,
				u32 len){

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
	u8 data2[20]={0};
	int i,r=0;

	r = mdfld_dsi_read_mcs_lp(sender,data[0],data2, len);

	ret = memcmp(data2, compare_data, len);
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

#define LP_RETRY 5
static int mipi_cmd_setting(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int r = 0;
	u8 rdata;
	int retry_times;

	r = mdfld_dsi_read_mcs_lp(sender,0xa,&rdata, 1);

	//RESX : L > H > L > H
	if (panel_id) {
		// AUO(S6D7AA0X04): 16ms
		if (gpio_direction_output(RESET_INNO, 0))
			gpio_set_value_cansleep(RESET_INNO, 0);
		usleep_range(2000, 2200);             // > 1ms  L

		if (gpio_direction_output(RESET_INNO, 1))
			gpio_set_value_cansleep(RESET_INNO, 1);
		usleep_range(2000, 2200);             //        H

		if (gpio_direction_output(RESET_INNO, 0))
			gpio_set_value_cansleep(RESET_INNO, 0);
		usleep_range(2000, 2200);             // > 10us L

		if (gpio_direction_output(RESET_INNO, 1))
			gpio_set_value_cansleep(RESET_INNO, 1);
		usleep_range(10000, 11000);           // > 5ms  H

	} else {
		// Innolux(NT35521): 74ms
		if (gpio_direction_output(RESET_INNO, 0))
			gpio_set_value_cansleep(RESET_INNO, 0);
		usleep_range(40000, 45000);           // > 40ms  L

		if (gpio_direction_output(RESET_INNO, 1))
			gpio_set_value_cansleep(RESET_INNO, 1);
		usleep_range(2000, 2200);             //         H

		if (gpio_direction_output(RESET_INNO, 0))
			gpio_set_value_cansleep(RESET_INNO, 0);
		usleep_range(2000, 2200);             // > 10us  L

		if (gpio_direction_output(RESET_INNO, 1))
			gpio_set_value_cansleep(RESET_INNO, 1);
		usleep_range(30000, 35000);           // > 20ms  H
	}

	//mipi initial code
	if (panel_id) {
		// AUO(S6D7AA0X04)
		retry_times=LP_RETRY;
		do{
			retry_times--;
			send_mipi_cmd2(sender, s6d7aa0x04_passwd1, CMD_SIZE(s6d7aa0x04_passwd1));

			r = compare_mipi_reg(sender, s6d7aa0x04_passwd1,
							CMD_SIZE(s6d7aa0x04_passwd1_data), s6d7aa0x04_passwd1_data);

			if(!r) break;
			if(!retry_times) return -EIO;

		}while(retry_times);

		mdfld_dsi_send_mcs_short_lp(sender, 0x11, 0x00, 0, 0); //sleep out
		usleep_range(5000, 5500);
		mdfld_dsi_send_mcs_short_lp(sender, 0x29, 0x00, 0, 0); //display on

		retry_times=LP_RETRY;
		do{
			retry_times--;
			send_mipi_cmd2(sender, s6d7aa0x04_enable_LED_IC, CMD_SIZE(s6d7aa0x04_enable_LED_IC));

			r = compare_mipi_reg(sender, s6d7aa0x04_enable_LED_IC,
							CMD_SIZE(s6d7aa0x04_enable_LED_IC_data), s6d7aa0x04_enable_LED_IC_data);

			if(!r) break;
			if(!retry_times) return -EIO;

		}while(retry_times);

	} else {
		// Innolux(NT35521)
		retry_times=LP_RETRY;
		do{
			retry_times--;
			send_mipi_cmd2(sender,nt35521v06b_cm1,CMD_SIZE(nt35521v06b_cm1));

			//========== Internal setting ==========
			send_mipi_cmd2(sender,nt35521v06b_cm2,CMD_SIZE(nt35521v06b_cm2));
			send_mipi_cmd2(sender,nt35521v06b_cm3,CMD_SIZE(nt35521v06b_cm3));
			send_mipi_cmd2(sender,nt35521v06b_cm4,CMD_SIZE(nt35521v06b_cm4));
			send_mipi_cmd2(sender,nt35521v06b_cm5,CMD_SIZE(nt35521v06b_cm5));

			send_mipi_cmd2(sender,nt35521v06b_cm4,CMD_SIZE(nt35521v06b_cm4));
			r = compare_mipi_reg(sender,nt35521v06b_cm5,CMD_SIZE(data_nt35521v06b_cm5),data_nt35521v06b_cm5);

			if(!r) break;
			if(!retry_times) return -EIO;

		}while(retry_times);

		send_mipi_cmd2(sender,nt35521v06b_cm6,CMD_SIZE(nt35521v06b_cm6));
		send_mipi_cmd2(sender,nt35521v06b_cm7,CMD_SIZE(nt35521v06b_cm7));
		send_mipi_cmd2(sender,nt35521v06b_cm8,CMD_SIZE(nt35521v06b_cm8));
		send_mipi_cmd2(sender,nt35521v06b_cm9,CMD_SIZE(nt35521v06b_cm9));
		send_mipi_cmd2(sender,nt35521v06b_cm10,CMD_SIZE(nt35521v06b_cm10));
		send_mipi_cmd2(sender,nt35521v06b_cm11,CMD_SIZE(nt35521v06b_cm11));

		send_mipi_cmd2(sender,nt35521v06b_ths_prepare1,CMD_SIZE(nt35521v06b_ths_prepare1));
		send_mipi_cmd2(sender,nt35521v06b_ths_prepare2,CMD_SIZE(nt35521v06b_ths_prepare2));

		send_mipi_cmd2(sender,nt35521v06b_cm1_off,CMD_SIZE(nt35521v06b_cm1_off));


		//========== page0 relative ==========
		send_mipi_cmd(sender,nt35521v06b_cm12,CMD_SIZE(nt35521v06b_cm12));
		send_mipi_cmd(sender,nt35521v06b_cm13,CMD_SIZE(nt35521v06b_cm13)); //Black Frame Insertion when occur error
		send_mipi_cmd(sender,nt35521v06b_cm14,CMD_SIZE(nt35521v06b_cm14));
		send_mipi_cmd(sender,nt35521v06b_cm15,CMD_SIZE(nt35521v06b_cm15));
		send_mipi_cmd(sender,nt35521v06b_cm16,CMD_SIZE(nt35521v06b_cm16));
		send_mipi_cmd(sender,nt35521v06b_cm17,CMD_SIZE(nt35521v06b_cm17));
		send_mipi_cmd(sender,nt35521v06b_cm18,CMD_SIZE(nt35521v06b_cm18));
		send_mipi_cmd(sender,nt35521v06b_cm19,CMD_SIZE(nt35521v06b_cm19));
		send_mipi_cmd(sender,nt35521v06b_cm20,CMD_SIZE(nt35521v06b_cm20));


		//========== page1 relative ==========
		send_mipi_cmd(sender,nt35521v06b_cm21,CMD_SIZE(nt35521v06b_cm21));
		send_mipi_cmd(sender,nt35521v06b_cm22,CMD_SIZE(nt35521v06b_cm22));
		send_mipi_cmd(sender,nt35521v06b_cm23,CMD_SIZE(nt35521v06b_cm23));
		send_mipi_cmd(sender,nt35521v06b_cm24,CMD_SIZE(nt35521v06b_cm24));
		send_mipi_cmd(sender,nt35521v06b_cm25,CMD_SIZE(nt35521v06b_cm25));
		send_mipi_cmd(sender,nt35521v06b_cm26,CMD_SIZE(nt35521v06b_cm26));
		send_mipi_cmd(sender,nt35521v06b_cm27,CMD_SIZE(nt35521v06b_cm27));
		send_mipi_cmd(sender,nt35521v06b_cm28,CMD_SIZE(nt35521v06b_cm28));
		send_mipi_cmd(sender,nt35521v06b_cm29,CMD_SIZE(nt35521v06b_cm29));
		send_mipi_cmd(sender,nt35521v06b_cm30,CMD_SIZE(nt35521v06b_cm30));
		send_mipi_cmd(sender,nt35521v06b_cm31,CMD_SIZE(nt35521v06b_cm31));
		send_mipi_cmd(sender,nt35521v06b_cm32,CMD_SIZE(nt35521v06b_cm32));


		//========== page2 relative ==========
		send_mipi_cmd(sender,nt35521v06b_cm33,CMD_SIZE(nt35521v06b_cm33));
		send_mipi_cmd(sender,nt35521v06b_cm34,CMD_SIZE(nt35521v06b_cm34));

		retry_times=LP_RETRY;
		do{
			retry_times--;
			send_mipi_cmd2(sender,nt35521v06b_cm35,CMD_SIZE(nt35521v06b_cm35));
			r = compare_mipi_reg(sender,nt35521v06b_cm35,CMD_SIZE(data_nt35521v06b_cm35),data_nt35521v06b_cm35);

			if(!r) break;
			if(!retry_times) return -EIO;

		}while(retry_times);

		retry_times=LP_RETRY;
		do{
			retry_times--;
			send_mipi_cmd2(sender,nt35521v06b_cm36,CMD_SIZE(nt35521v06b_cm36));
			send_mipi_cmd2(sender,nt35521v06b_cm37,CMD_SIZE(nt35521v06b_cm37));
			send_mipi_cmd2(sender,nt35521v06b_cm38,CMD_SIZE(nt35521v06b_cm38));
			send_mipi_cmd2(sender,nt35521v06b_cm39,CMD_SIZE(nt35521v06b_cm39));
			send_mipi_cmd2(sender,nt35521v06b_cm40,CMD_SIZE(nt35521v06b_cm40));

			r = compare_mipi_reg(sender,nt35521v06b_cm36,CMD_SIZE(data_nt35521v06b_cm36_40),data_nt35521v06b_cm36_40);

			if(!r) break;
			if(!retry_times) return -EIO;

		}while(retry_times);

		retry_times=LP_RETRY;
		do{
			retry_times--;
			send_mipi_cmd2(sender,nt35521v06b_cm41,CMD_SIZE(nt35521v06b_cm41));
			send_mipi_cmd2(sender,nt35521v06b_cm42,CMD_SIZE(nt35521v06b_cm42));
			send_mipi_cmd2(sender,nt35521v06b_cm43,CMD_SIZE(nt35521v06b_cm43));
			send_mipi_cmd2(sender,nt35521v06b_cm44,CMD_SIZE(nt35521v06b_cm44));
			send_mipi_cmd2(sender,nt35521v06b_cm45,CMD_SIZE(nt35521v06b_cm45));

			r = compare_mipi_reg(sender,nt35521v06b_cm41,CMD_SIZE(data_nt35521v06b_cm41_45),data_nt35521v06b_cm41_45);

			if(!r) break;
			if(!retry_times) return -EIO;

		}while(retry_times);

		retry_times=LP_RETRY;
		do{
			retry_times--;
			send_mipi_cmd2(sender,nt35521v06b_cm46,CMD_SIZE(nt35521v06b_cm46));
			send_mipi_cmd2(sender,nt35521v06b_cm47,CMD_SIZE(nt35521v06b_cm47));
			send_mipi_cmd2(sender,nt35521v06b_cm48,CMD_SIZE(nt35521v06b_cm48));
			send_mipi_cmd2(sender,nt35521v06b_cm49,CMD_SIZE(nt35521v06b_cm49));
			send_mipi_cmd2(sender,nt35521v06b_cm50,CMD_SIZE(nt35521v06b_cm50));

			r = compare_mipi_reg(sender,nt35521v06b_cm46,CMD_SIZE(data_nt35521v06b_cm46_50),data_nt35521v06b_cm46_50);

			if(!r) break;
			if(!retry_times) return -EIO;

		}while(retry_times);

		retry_times=LP_RETRY;
		do{
			retry_times--;
			send_mipi_cmd2(sender,nt35521v06b_cm51,CMD_SIZE(nt35521v06b_cm51));
			r = compare_mipi_reg(sender,nt35521v06b_cm51,CMD_SIZE(data_nt35521v06b_cm51),data_nt35521v06b_cm51);

			if(!r) break;
			if(!retry_times) return -EIO;

		}while(retry_times);


		// PAGE6 : GOUT Mapping, VGLO select
		send_mipi_cmd(sender,nt35521v06b_cm52,CMD_SIZE(nt35521v06b_cm52));
		send_mipi_cmd(sender,nt35521v06b_cm53,CMD_SIZE(nt35521v06b_cm53));
		send_mipi_cmd(sender,nt35521v06b_cm54,CMD_SIZE(nt35521v06b_cm54));
		send_mipi_cmd(sender,nt35521v06b_cm55,CMD_SIZE(nt35521v06b_cm55));
		send_mipi_cmd(sender,nt35521v06b_cm56,CMD_SIZE(nt35521v06b_cm56));
		send_mipi_cmd(sender,nt35521v06b_cm57,CMD_SIZE(nt35521v06b_cm57));
		send_mipi_cmd(sender,nt35521v06b_cm58,CMD_SIZE(nt35521v06b_cm58));
		send_mipi_cmd(sender,nt35521v06b_cm59,CMD_SIZE(nt35521v06b_cm59));
		send_mipi_cmd(sender,nt35521v06b_cm60,CMD_SIZE(nt35521v06b_cm60));
		send_mipi_cmd(sender,nt35521v06b_cm61,CMD_SIZE(nt35521v06b_cm61));
		send_mipi_cmd(sender,nt35521v06b_cm62,CMD_SIZE(nt35521v06b_cm62));
		send_mipi_cmd(sender,nt35521v06b_cm63,CMD_SIZE(nt35521v06b_cm63));
		send_mipi_cmd(sender,nt35521v06b_cm64,CMD_SIZE(nt35521v06b_cm64));
		send_mipi_cmd(sender,nt35521v06b_cm65,CMD_SIZE(nt35521v06b_cm65));
		send_mipi_cmd(sender,nt35521v06b_cm66,CMD_SIZE(nt35521v06b_cm66));
		send_mipi_cmd(sender,nt35521v06b_cm67,CMD_SIZE(nt35521v06b_cm67));
		send_mipi_cmd(sender,nt35521v06b_cm68,CMD_SIZE(nt35521v06b_cm68));
		send_mipi_cmd(sender,nt35521v06b_cm69,CMD_SIZE(nt35521v06b_cm69));
		send_mipi_cmd(sender,nt35521v06b_cm70,CMD_SIZE(nt35521v06b_cm70));
		send_mipi_cmd(sender,nt35521v06b_cm71,CMD_SIZE(nt35521v06b_cm71));
		send_mipi_cmd(sender,nt35521v06b_cm72,CMD_SIZE(nt35521v06b_cm72));
		send_mipi_cmd(sender,nt35521v06b_cm73,CMD_SIZE(nt35521v06b_cm73));
		send_mipi_cmd(sender,nt35521v06b_cm74,CMD_SIZE(nt35521v06b_cm74));
		send_mipi_cmd(sender,nt35521v06b_cm75,CMD_SIZE(nt35521v06b_cm75));
		send_mipi_cmd(sender,nt35521v06b_cm76,CMD_SIZE(nt35521v06b_cm76));
		send_mipi_cmd(sender,nt35521v06b_cm77,CMD_SIZE(nt35521v06b_cm77));
		send_mipi_cmd(sender,nt35521v06b_cm78,CMD_SIZE(nt35521v06b_cm78));
		send_mipi_cmd(sender,nt35521v06b_cm79,CMD_SIZE(nt35521v06b_cm79));
		send_mipi_cmd(sender,nt35521v06b_cm80,CMD_SIZE(nt35521v06b_cm80));
		send_mipi_cmd(sender,nt35521v06b_cm81,CMD_SIZE(nt35521v06b_cm81));
		send_mipi_cmd(sender,nt35521v06b_cm82,CMD_SIZE(nt35521v06b_cm82));
		send_mipi_cmd(sender,nt35521v06b_cm83,CMD_SIZE(nt35521v06b_cm83));
		send_mipi_cmd(sender,nt35521v06b_cm84,CMD_SIZE(nt35521v06b_cm84));
		send_mipi_cmd(sender,nt35521v06b_cm85,CMD_SIZE(nt35521v06b_cm85));
		send_mipi_cmd(sender,nt35521v06b_cm86,CMD_SIZE(nt35521v06b_cm86));
		send_mipi_cmd(sender,nt35521v06b_cm87,CMD_SIZE(nt35521v06b_cm87));
		send_mipi_cmd(sender,nt35521v06b_cm88,CMD_SIZE(nt35521v06b_cm88));
		send_mipi_cmd(sender,nt35521v06b_cm89,CMD_SIZE(nt35521v06b_cm89));
		send_mipi_cmd(sender,nt35521v06b_cm90,CMD_SIZE(nt35521v06b_cm90));
		send_mipi_cmd(sender,nt35521v06b_cm91,CMD_SIZE(nt35521v06b_cm91));
		send_mipi_cmd(sender,nt35521v06b_cm92,CMD_SIZE(nt35521v06b_cm92));
		send_mipi_cmd(sender,nt35521v06b_cm93,CMD_SIZE(nt35521v06b_cm93));
		send_mipi_cmd(sender,nt35521v06b_cm94,CMD_SIZE(nt35521v06b_cm94));
		send_mipi_cmd(sender,nt35521v06b_cm95,CMD_SIZE(nt35521v06b_cm95));
		send_mipi_cmd(sender,nt35521v06b_cm96,CMD_SIZE(nt35521v06b_cm96));
		send_mipi_cmd(sender,nt35521v06b_cm97,CMD_SIZE(nt35521v06b_cm97));


		// PAGE3 :
		send_mipi_cmd(sender,nt35521v06b_cm98,CMD_SIZE(nt35521v06b_cm98));
		send_mipi_cmd(sender,nt35521v06b_cm99,CMD_SIZE(nt35521v06b_cm99));
		send_mipi_cmd(sender,nt35521v06b_cm100,CMD_SIZE(nt35521v06b_cm100));
		send_mipi_cmd(sender,nt35521v06b_cm101,CMD_SIZE(nt35521v06b_cm101));
		send_mipi_cmd(sender,nt35521v06b_cm102,CMD_SIZE(nt35521v06b_cm102));
		send_mipi_cmd(sender,nt35521v06b_cm103,CMD_SIZE(nt35521v06b_cm103));
		send_mipi_cmd(sender,nt35521v06b_cm104,CMD_SIZE(nt35521v06b_cm104));
		send_mipi_cmd(sender,nt35521v06b_cm105,CMD_SIZE(nt35521v06b_cm105));
		send_mipi_cmd(sender,nt35521v06b_cm106,CMD_SIZE(nt35521v06b_cm106));
		send_mipi_cmd(sender,nt35521v06b_cm107,CMD_SIZE(nt35521v06b_cm107));
		send_mipi_cmd(sender,nt35521v06b_cm108,CMD_SIZE(nt35521v06b_cm108));
		send_mipi_cmd(sender,nt35521v06b_cm109,CMD_SIZE(nt35521v06b_cm109));


		// PAGE5 :
		send_mipi_cmd(sender,nt35521v06b_cm110,CMD_SIZE(nt35521v06b_cm110));
		send_mipi_cmd(sender,nt35521v06b_cm111,CMD_SIZE(nt35521v06b_cm111));
		send_mipi_cmd(sender,nt35521v06b_cm112,CMD_SIZE(nt35521v06b_cm112));
		send_mipi_cmd(sender,nt35521v06b_cm113,CMD_SIZE(nt35521v06b_cm113));
		send_mipi_cmd(sender,nt35521v06b_cm114,CMD_SIZE(nt35521v06b_cm114));
		send_mipi_cmd(sender,nt35521v06b_cm115,CMD_SIZE(nt35521v06b_cm115));
		send_mipi_cmd(sender,nt35521v06b_cm116,CMD_SIZE(nt35521v06b_cm116));
		send_mipi_cmd(sender,nt35521v06b_cm117,CMD_SIZE(nt35521v06b_cm117));
		send_mipi_cmd(sender,nt35521v06b_cm118,CMD_SIZE(nt35521v06b_cm118));
		send_mipi_cmd(sender,nt35521v06b_cm119,CMD_SIZE(nt35521v06b_cm119));
		send_mipi_cmd(sender,nt35521v06b_cm120,CMD_SIZE(nt35521v06b_cm120));
		send_mipi_cmd(sender,nt35521v06b_cm121,CMD_SIZE(nt35521v06b_cm121));
		send_mipi_cmd(sender,nt35521v06b_cm122,CMD_SIZE(nt35521v06b_cm122));
		send_mipi_cmd(sender,nt35521v06b_cm123,CMD_SIZE(nt35521v06b_cm123));
		send_mipi_cmd(sender,nt35521v06b_cm124,CMD_SIZE(nt35521v06b_cm124));
		send_mipi_cmd(sender,nt35521v06b_cm125,CMD_SIZE(nt35521v06b_cm125));
		send_mipi_cmd(sender,nt35521v06b_cm126,CMD_SIZE(nt35521v06b_cm126));
		send_mipi_cmd(sender,nt35521v06b_cm127,CMD_SIZE(nt35521v06b_cm127));
		send_mipi_cmd(sender,nt35521v06b_cm128,CMD_SIZE(nt35521v06b_cm128));
		send_mipi_cmd(sender,nt35521v06b_cm129,CMD_SIZE(nt35521v06b_cm129));
		send_mipi_cmd(sender,nt35521v06b_cm130,CMD_SIZE(nt35521v06b_cm130));
		send_mipi_cmd(sender,nt35521v06b_cm131,CMD_SIZE(nt35521v06b_cm131));
		send_mipi_cmd(sender,nt35521v06b_cm132,CMD_SIZE(nt35521v06b_cm132));
		send_mipi_cmd(sender,nt35521v06b_cm133,CMD_SIZE(nt35521v06b_cm133));
		send_mipi_cmd(sender,nt35521v06b_cm134,CMD_SIZE(nt35521v06b_cm134));
		send_mipi_cmd(sender,nt35521v06b_cm135,CMD_SIZE(nt35521v06b_cm135));
		send_mipi_cmd(sender,nt35521v06b_cm136,CMD_SIZE(nt35521v06b_cm136));
		send_mipi_cmd(sender,nt35521v06b_cm137,CMD_SIZE(nt35521v06b_cm137));
		send_mipi_cmd(sender,nt35521v06b_cm138,CMD_SIZE(nt35521v06b_cm138));

		//reload function
		send_mipi_cmd(sender,nt35521v06b_cm139,CMD_SIZE(nt35521v06b_cm139));
		send_mipi_cmd(sender,nt35521v06b_cm140,CMD_SIZE(nt35521v06b_cm140));

#ifdef NT35521_BIST
		send_mipi_cmd(sender,bist_cm1,CMD_SIZE(bist_cm1));
		send_mipi_cmd(sender,bist_cm2,CMD_SIZE(bist_cm2));
		send_mipi_cmd(sender,bist_cm3,CMD_SIZE(bist_cm3));
#else
		// Normal Display
		send_mipi_cmd(sender,nt35521v06b_cm141,CMD_SIZE(nt35521v06b_cm141));
		send_mipi_cmd(sender,nt35521v06b_cm142,CMD_SIZE(nt35521v06b_cm142));
		send_mipi_cmd(sender,nt35521v06b_cm143,CMD_SIZE(nt35521v06b_cm143));
#endif
	}

	return 0;
}

static void b080ean020_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
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

	/* Reconfig lane configuration */
	dsi_config->lane_count = 4;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
	dsi_config->enable_gamma_csc = ENABLE_GAMMA | ENABLE_CSC;

	hw_ctx->cck_div = 0;
	hw_ctx->pll_bypass_mode = 1;
	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffff;
	hw_ctx->turn_around_timeout = 0x3f;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x18;
	hw_ctx->init_count = 0x7d0;
	hw_ctx->eot_disable = 0x3;
	hw_ctx->lp_byteclk = 0x4;
	hw_ctx->clk_lane_switch_time_cnt = 0x18000b;
	hw_ctx->dphy_param = 0x2a0f5f0c;

	/* Setup video mode format */
	hw_ctx->video_mode_format = 0xe;

	/* Set up func_prg, RGB888(0x200) */
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);

	/* Setup mipi port configuration */
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;

	b080ean020_dsi_config = dsi_config;

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

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
}

static int b080ean020_vid_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val;
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

static int b080ean020_vid_gpio_control(int on)
{
	if(on){
		intel_scu_ipc_iowrite8(LCD_LOGIC_PWR_EN,0x1);
		usleep_range(1000, 2000);

		if (gpio_direction_output(LCD_ANALONG_PWR_EN, 1))
			gpio_set_value_cansleep(LCD_ANALONG_PWR_EN, 1);
		usleep_range(1000, 2000);

	}else{

		usleep_range(2000, 3000);

		if (gpio_direction_output(RESET_INNO, 0))
			gpio_set_value_cansleep(RESET_INNO, 0);

		usleep_range(1000, 2000);

		if (gpio_direction_output(LCD_ANALONG_PWR_EN, 0))
			gpio_set_value_cansleep(LCD_ANALONG_PWR_EN, 0);

		usleep_range(5000, 6000);

		intel_scu_ipc_iowrite8(LCD_LOGIC_PWR_EN,0x0);

		usleep_range(1000, 2000);
	}

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	return 0;
}

static int b080ean020_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	usleep_range(1000, 1200);

	/* Send TURN_ON packet */
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
					    MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}

	if (panel_id)
		usleep_range(195000, 200000);
	else
		usleep_range(135000, 136000);  //8 vs

	pwm_enable();

	/* LCD_BL_EN*/
	intel_scu_ipc_iowrite8(LCD_BL_EN,0x1);

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	return 0;
}

static int b080ean020_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	pwm_disable();

	/* LCD_BL_EN*/
	intel_scu_ipc_iowrite8(LCD_BL_EN,0x0);

	if (panel_id)
		usleep_range(50000, 55000);

	mdfld_dsi_send_mcs_short_lp(sender,0x28,0x00,0,0); //display off
	mdfld_dsi_send_mcs_short_lp(sender,0x10,0x00,0,0); //sleep in

	/* Send SHUT_DOWN packet */
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
					    MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		return err;
	}

	usleep_range(100000, 101000);  //>100ms

	b080ean020_vid_gpio_control(0);

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	return 0;
}

static int b080ean020_vid_reset(struct mdfld_dsi_config *dsi_config)
{
	b080ean020_vid_gpio_control(1);

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	printk("[DISPLAY] Panel: %s\n", panel_id ? "AUO (S6D7AA0X04)" : "Innolux (NT35521)");

	return 0;
}

static int bl_prev_level = 0;
static int b080ean020_vid_set_brightness(struct mdfld_dsi_config *dsi_config,
					 int level)
{
	int duty_val = 0;
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	duty_val = level==1?0:level;
	pwm_configure(duty_val);

	//add for debug ++
	if(!!bl_prev_level^!!duty_val)
		DRM_INFO("[DISPLAY] brightness level : %d > %d\n", bl_prev_level, duty_val);

	bl_prev_level = duty_val;
	//add for debug --

	PSB_DEBUG_ENTRY("level = %d , duty_val = %d\n", level, duty_val);

	return 0;
}

struct drm_display_mode *b080ean020_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 800;
	mode->vdisplay = 1280;

	if (panel_id) {
		// AUO(S6D7AA0X04)
		mode->hsync_start = mode->hdisplay + 24;	//sync offset
		mode->hsync_end = mode->hsync_start + 4;	//pulse width
		mode->htotal = mode->hdisplay + 160;		//blank
		mode->vsync_start = mode->vdisplay + 8;
		mode->vsync_end = mode->vsync_start + 4;
		mode->vtotal = mode->vdisplay + 20;
	} else {
		// Innolux(NT35521)
		mode->hsync_start = 840;
		mode->hsync_end = 844;
		mode->htotal = 884;
		mode->vsync_start = 1300;
		mode->vsync_end = 1304;
		mode->vtotal = 1324;
	}

	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return mode;
}

static void b080ean020_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	pi->width_mm = 108;
	pi->height_mm = 172;
}

static int b080ean020_vid_gpio_init(void)
{
	gpio_request(PANEL_ID,"PANEL_ID");
	gpio_request(RESET_INNO,"RESET_INNO");
	gpio_request(LCD_ANALONG_PWR_EN,"LCD_ANALONG_PWR_EN");

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return 0;
}

#define PWMCTRL_REG 0xffae9000
#define PWMCTRL_SIZE 0x80
static int b080ean020_vid_brightness_init(void)
{
	int ret = 0;

	pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG, PWMCTRL_SIZE);
	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return ret;
}


#ifdef B080EAN020_DEBUG
static int send_mipi_ret = -1;
static int read_mipi_ret = -1;
static u8 read_mipi_data = 0;

static ssize_t send_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0, x1=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(b080ean020_dsi_config);

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
			= mdfld_dsi_get_pkg_sender(b080ean020_dsi_config);

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

DEVICE_ATTR(send_mipi_b080ean020, (S_IRUGO|S_IWUSR), send_mipi_show, send_mipi_store);
DEVICE_ATTR(read_mipi_b080ean020, (S_IRUGO|S_IWUSR), read_mipi_show, read_mipi_store);

static struct attribute *b080ean020_attrs[] = {
        &dev_attr_send_mipi_b080ean020.attr,
        &dev_attr_read_mipi_b080ean020.attr,
        NULL
};

static struct attribute_group b080ean020_attr_group = {
        .attrs = b080ean020_attrs,
        .name = "b080ean020",
};
#endif

static int init_asus_panel_id(void)
{
	panel_id = 0; //default setting

	panel_id = gpio_get_value(PANEL_ID)?0x1:0x0;

#ifdef USE_AUO
	panel_id = 0x1;
#endif

	asus_panel_id = FE380CG_PANEL | panel_id;

	DRM_INFO("[DISPLAY] %s: asus_panel_id=0x%x\n", __func__, asus_panel_id);

	return 0;
}

void b080ean020_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ret = 0;

	p_funcs->get_config_mode = b080ean020_vid_get_config_mode;
	p_funcs->get_panel_info = b080ean020_vid_get_panel_info;
	p_funcs->dsi_controller_init = b080ean020_vid_dsi_controller_init;
	p_funcs->detect = b080ean020_vid_detect;
	p_funcs->power_on = b080ean020_vid_power_on;
	p_funcs->drv_ic_init = mipi_cmd_setting;
	p_funcs->power_off = b080ean020_vid_power_off;
	p_funcs->reset = b080ean020_vid_reset;
	p_funcs->set_brightness = b080ean020_vid_set_brightness;

	proj_id = Read_PROJ_ID();
	board_hw_id = Read_HW_ID();

	ret = b080ean020_vid_gpio_init();
	if (ret)
		DRM_ERROR("Faild to request GPIO for B101UAN01.7 panel\n");

	ret = b080ean020_vid_brightness_init();
	if (ret)
		DRM_ERROR("Faild to initilize PWM of MSCI\n");

	init_asus_panel_id();

#ifdef B080EAN020_DEBUG
    sysfs_create_group(&dev->dev->kobj, &b080ean020_attr_group);
#endif

	DRM_INFO("[DISPLAY] %s: board_hw_id=%d\n", __func__, board_hw_id);

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

}

static int b080ean020_vid_lcd_probe(struct platform_device *pdev)
{
	DRM_INFO("%s: B080EAN020 panel detected\n", __func__);
	intel_mid_panel_register(b080ean020_vid_init);

	return 0;
}

struct platform_driver b080ean020_vid_lcd_driver = {
	.probe	= b080ean020_vid_lcd_probe,
	.driver	= {
		.name	= B080EAN020_PANEL_NAME,
		.owner	= THIS_MODULE,
	},
};
