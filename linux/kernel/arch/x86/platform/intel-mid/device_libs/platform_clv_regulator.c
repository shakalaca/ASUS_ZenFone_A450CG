/*
 * CLV regulator platform data
 * Copyright (c) 2012, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/lcd.h>
#include <linux/regulator/intel_pmic.h>
#include <linux/regulator/machine.h>
#include <asm/intel_scu_flis.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/driver.h>

#include <asm/intel-mid.h>

/****** Clovertrail SoC ******/

/*** Redhookbay ***/

static struct regulator_consumer_supply redhookbay_vprog1_consumer[] = {
	REGULATOR_SUPPLY("vprog1", "4-0048"), /* lm3554 */
	REGULATOR_SUPPLY("vprog1", "4-0036"), //ASUS_BSP++ ar0543_raw
	REGULATOR_SUPPLY("vprog1", "4-003C"), //ASUS_BSP++
	REGULATOR_SUPPLY("vprog1", "4-0021"), //ASUS_BSP+++
	REGULATOR_SUPPLY("vprog1", "4-001A"), //ASUS_BSP+++
	/*
	 * Begin Scaleht / VV board consumers
	 *
	 * Scaleht and VV boards have devices such as the wm5102
	 * codec, but the SPID on VV boards still indicates they're
	 * Redhookbay. Put the consumers here even if they don't
	 * actually exist on every board. Should the device not be
	 * present on the SFI tables, the consumer entries just won't
	 * get used.
	 */
	REGULATOR_SUPPLY("AVDD", "1-001a"), /* wm5102 */
	REGULATOR_SUPPLY("DBVDD1", "1-001a"), /* wm5102 */
	REGULATOR_SUPPLY("DBVDD2", "wm5102-codec"),
	REGULATOR_SUPPLY("DBVDD3", "wm5102-codec"),
	REGULATOR_SUPPLY("CPVDD", "wm5102-codec"),
	REGULATOR_SUPPLY("SPKVDDL", "wm5102-codec"),
	REGULATOR_SUPPLY("SPKVDDR", "wm5102-codec"),
	/* End Scaleht / VV board consumers */
};

static struct regulator_init_data redhookbay_vprog1_data = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2800000,
		.apply_uV		= 1,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY | REGULATOR_MODE_FAST,
		//.force_boot_off = 1, //ASUS_BSP++, add for turn off power at booting
	},
	.num_consumer_supplies	= ARRAY_SIZE(redhookbay_vprog1_consumer),
	.consumer_supplies	= redhookbay_vprog1_consumer,
};

static struct intel_pmic_info redhookbay_vprog1_info = {
	.pmic_reg   = VPROG1CNT_ADDR,
	.init_data  = &redhookbay_vprog1_data,
	.table_len  = ARRAY_SIZE(VPROG1_VSEL_table),
	.table      = VPROG1_VSEL_table,
};

static struct platform_device redhookbay_vprog1_device = {
	.name = "intel_regulator",
	.id = VPROG1,
	.dev = {
		.platform_data = &redhookbay_vprog1_info,
	},
};

static void __init atom_regulator_redhookbay_init(void)
{
	platform_device_register(&redhookbay_vprog1_device);
}

/*** Victoria bay ***/

static struct regulator_consumer_supply victoriabay_vprog1_consumer[] = {
	REGULATOR_SUPPLY("VDDD", "4-003c"), /* s5k8aayx */
};

static struct regulator_init_data victoriabay_vprog1_data = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 1200000,
		.apply_uV		= 1,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY
			| REGULATOR_MODE_FAST,
	},
	.num_consumer_supplies	= ARRAY_SIZE(victoriabay_vprog1_consumer),
	.consumer_supplies	= victoriabay_vprog1_consumer,
};

static struct intel_pmic_info victoriabay_vprog1_info = {
	.pmic_reg   = VPROG1CNT_ADDR,
	.init_data  = &victoriabay_vprog1_data,
	.table_len  = ARRAY_SIZE(VPROG1_VSEL_table),
	.table      = VPROG1_VSEL_table,
};

static struct platform_device victoriabay_vprog1_device = {
	.name = "intel_regulator",
	.id = VPROG1,
	.dev = {
		.platform_data = &victoriabay_vprog1_info,
	},
};

/*
 * Victoria bay macro add-on board does not use vemmc1 but there's no
 * harm from using it.
 */
static struct regulator_consumer_supply victoriabay_vemmc1_consumer[] = {
	REGULATOR_SUPPLY("vprog1", "4-0048"), /* lm3554 */
	REGULATOR_SUPPLY("VANA", "4-0020"), /* imx135 */
	REGULATOR_SUPPLY("VDDA", "4-003c"), /* s5k8aayx */
	REGULATOR_SUPPLY("vemmc1", "4-0010"), /* imx135, for compat */
	REGULATOR_SUPPLY("vemmc1", "4-003c"), /* s5k8aayx, for compat */
};

static struct regulator_init_data victoriabay_vemmc1_data = {
	.constraints = {
		.min_uV			= 2850000,
		.max_uV			= 2850000,
		.apply_uV		= 1,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY
			| REGULATOR_MODE_FAST,
	},
	.num_consumer_supplies	= ARRAY_SIZE(victoriabay_vemmc1_consumer),
	.consumer_supplies	= victoriabay_vemmc1_consumer,
};

static struct intel_pmic_info victoriabay_vemmc1_info = {
	.pmic_reg   = VEMMC1CNT_ADDR,
	.init_data  = &victoriabay_vemmc1_data,
	.table_len  = ARRAY_SIZE(VEMMC1_VSEL_table),
	.table      = VEMMC1_VSEL_table,
};

static struct platform_device victoriabay_vemmc1_device = {
	.name = "intel_regulator",
	.id = VEMMC1,
	.dev = {
		.platform_data = &victoriabay_vemmc1_info,
	},
};

static struct regulator_consumer_supply victoriabay_v_1p05_cam_ldo_consumer[] = {
	REGULATOR_SUPPLY("VDDL", "4-0010"), /* imx135 */
};

static struct regulator_init_data victoriabay_v_1p05_cam_ldo_data = {
	.supply_regulator		= "v_2p80_vcm_vdd",
	.constraints = {
		/*
		 * FIXME: This is really 1,05 V but let's pretend for
		 * now we provide 1,2.
		 */
		.min_uV			= 1200000,
		.max_uV			= 1200000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies	= ARRAY_SIZE(victoriabay_v_1p05_cam_ldo_consumer),
	.consumer_supplies	= victoriabay_v_1p05_cam_ldo_consumer,
};

static struct fixed_voltage_config victoriabay_v_1p05_cam_ldo_pdata = {
	.supply_name	= "v_1p05_cam_ldo",
	.microvolts	= 1200000,
	.gpio		= -1,
	.init_data	= &victoriabay_v_1p05_cam_ldo_data,
};

static struct platform_device victoriabay_v_1p05_cam_ldo_pdev = {
	.name = "reg-fixed-voltage",
	.id = -1,
	.dev = {
		.platform_data = &victoriabay_v_1p05_cam_ldo_pdata,
	},
};

static void __init atom_regulator_victoriabay_init(void)
{
	platform_device_register(&victoriabay_vprog1_device);
	platform_device_register(&victoriabay_vemmc1_device);
	platform_device_register(&victoriabay_v_1p05_cam_ldo_pdev);
}

/*** Clovertrail SoC specific regulators ***/

static struct regulator_consumer_supply vprog2_consumer[] = {
	REGULATOR_SUPPLY("vprog2", "4-0048"), //ASUS_BSP+++
	REGULATOR_SUPPLY("vprog2", "4-0036"), //ASUS_BSP+++
	REGULATOR_SUPPLY("vprog2", "4-003C"), //ASUS_BSP+++
	REGULATOR_SUPPLY("vprog2", "4-001A"), //ASUS_BSP+++
	REGULATOR_SUPPLY("vprog2", "4-0021"), //ASUS_BSP+++
};

static struct regulator_init_data vprog2_data = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2800000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY | REGULATOR_MODE_FAST,
		//.force_boot_off = 1, //ASUS_BSP++, add for turn off power at booting
	},
	.num_consumer_supplies		= ARRAY_SIZE(vprog2_consumer),
	.consumer_supplies		= vprog2_consumer,
};

static struct intel_pmic_info vprog2_info = {
	.pmic_reg   = VPROG2CNT_ADDR,
	.init_data  = &vprog2_data,
	.table_len  = ARRAY_SIZE(VPROG2_VSEL_table),
	.table      = VPROG2_VSEL_table,
};

static struct platform_device vprog2_device = {
	.name = "intel_regulator",
	.id = VPROG2,
	.dev = {
		.platform_data = &vprog2_info,
	},
};

//ASUS_BSP+++
static struct regulator_consumer_supply vemmc1_consumer[] = {
	REGULATOR_SUPPLY("vemmc1", "4-003C"),
	REGULATOR_SUPPLY("vemmc1", "4-0036"), //ASUS_BSP++ ar0543_raw
	REGULATOR_SUPPLY("vemmc1", "4-001A"),
};

static struct regulator_init_data vemmc1_data = {
	.constraints = {
		.min_uV			= 2850000,
		.max_uV			= 2850000,
		.apply_uV		= 1,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY | REGULATOR_MODE_FAST,
		//.force_boot_off = 1, //ASUS_BSP++, add for turn off power at booting
	},
	.num_consumer_supplies		= ARRAY_SIZE(vemmc1_consumer),
	.consumer_supplies		= vemmc1_consumer,
};

static struct intel_pmic_info vemmc1_info = {
	.pmic_reg   = VEMMC1CNT_ADDR,
	.init_data  = &vemmc1_data,
	.table_len  = ARRAY_SIZE(VEMMC1_VSEL_table),
	.table      = VEMMC1_VSEL_table,
};

static struct platform_device vemmc1_device = {
	.name = "intel_regulator",
	.id = VEMMC1,
	.dev = {
		.platform_data = &vemmc1_info,
	},
};
//ASUS_BSP---

/***********VEMMC2 REGUATOR platform data*************/
static struct regulator_consumer_supply vemmc2_consumer[] = {
};

static struct regulator_init_data vemmc2_data = {
	.constraints = {
		.min_uV			= 2850000,
		.max_uV			= 2850000,
		.apply_uV		= 1,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY | REGULATOR_MODE_FAST,
	},
	.num_consumer_supplies		= ARRAY_SIZE(vemmc2_consumer),
	.consumer_supplies		= vemmc2_consumer,
};

static struct regulator_consumer_supply vccsdio_consumer[] = {
	REGULATOR_SUPPLY("vmmc", "0000:00:04.0"),
};
static struct regulator_init_data vccsdio_data = {
	.constraints = {
		.min_uV			= 2850000,
		.max_uV			= 2850000,
		.valid_ops_mask	=   REGULATOR_CHANGE_STATUS |
						REGULATOR_CHANGE_MODE,
		.valid_modes_mask   =   REGULATOR_MODE_NORMAL |
						REGULATOR_MODE_STANDBY |
						REGULATOR_MODE_FAST,
#ifdef CONFIG_MMC_SD_VCCSDIO_ALWAYS_ON
		.always_on = 1,
#else
		.force_boot_off = 1,
#endif
	},
	.num_consumer_supplies	= ARRAY_SIZE(vccsdio_consumer),
	.consumer_supplies		= vccsdio_consumer,
};

static struct intel_pmic_info vemmc2_info = {
	.pmic_reg   = VEMMC2CNT_ADDR,
	.init_data  = &vemmc2_data,
	.table_len  = ARRAY_SIZE(VEMMC2_VSEL_table),
	.table      = VEMMC2_VSEL_table,
};

#define ENCTRL0_ISOLATE        0x55555557
#define ENCTRL1_ISOLATE        0x5555
#define STORAGESTIO_FLISNUM    0x8
#define ENCTRL0_OFF        0x10
#define ENCTRL1_OFF        0x11

static unsigned int enctrl0_orig;
static unsigned int enctrl1_orig;

static int _mmc1_disconnect_shim(void)
{
   int err = 0;

   err = intel_scu_ipc_read_shim(&enctrl0_orig,
           STORAGESTIO_FLISNUM, ENCTRL0_OFF);
   if (err) {
       pr_err("%s: ENCTRL0 read failed\n", __func__);
       goto out;
   }
   err = intel_scu_ipc_read_shim(&enctrl1_orig,
           STORAGESTIO_FLISNUM, ENCTRL1_OFF);
   if (err) {
       pr_err("%s: ENCTRL1 read failed\n", __func__);
       goto out;
   }

   pr_info("%s: mmc1.vmmc save original enctrl 0x%x, 0x%x\n",
           __func__, enctrl0_orig, enctrl1_orig);
   /* isolate shim */
   err = intel_scu_ipc_write_shim(ENCTRL0_ISOLATE,
           STORAGESTIO_FLISNUM, ENCTRL0_OFF);
   if (err) {
       pr_err("%s: ENCTRL0 ISOLATE failed\n",
               __func__);
       goto out;
   }

   err = intel_scu_ipc_write_shim(ENCTRL1_ISOLATE,
           STORAGESTIO_FLISNUM, ENCTRL1_OFF);
   if (err) {
       pr_err("%s: ENCTRL1 ISOLATE failed\n", __func__);
       goto out;
   }
out:
    return err;
}

/* Wrapper function, always return 0 (pass) */
static int vccsdio_before_vreg_off(struct regulator_dev *rdev)
{
    _mmc1_disconnect_shim();
    return 0;
}

static int _mmc1_reconnect_shim(void)
{
   int err = 0;

   /* reconnect shim */
   err = intel_scu_ipc_write_shim(enctrl0_orig,
           STORAGESTIO_FLISNUM, ENCTRL0_OFF);
   if (err) {
       pr_err("%s: ENCTRL0 CONNECT shim failed\n", __func__);
       goto out;
   }

   err = intel_scu_ipc_write_shim(enctrl1_orig,
           STORAGESTIO_FLISNUM, ENCTRL1_OFF);
   if (err) {
       pr_err("%s: ENCTRL1 CONNECT shim failed\n", __func__);
       goto out;
   }

   pr_info("%s: mmc1.vmmc recover original enctrl 0x%x, 0x%x\n",
           __func__, enctrl0_orig, enctrl1_orig);
out:
   return err;
}

/* Wrapper function, always return 0 (pass) */
static int vccsdio_after_vreg_on(struct regulator_dev *rdev)
{
    _mmc1_reconnect_shim();
    return 0;
}

static unsigned long vccsdio_latest_disable_time = 0;

static int vccsdio_before_vreg_on (struct regulator_dev *rdev)
{
    struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);

    if (!pmic_info || pmic_info->min_disable_interval_ms == 0 ||
        vccsdio_latest_disable_time == 0 ||
        vccsdio_latest_disable_time == INITIAL_JIFFIES)
        return 0;

    while (jiffies_to_msecs(jiffies - vccsdio_latest_disable_time) < pmic_info->min_disable_interval_ms)
        msleep(1);
    return 0;
}

static int vccsdio_after_vreg_off(struct regulator_dev *rdev)
{
    vccsdio_latest_disable_time = jiffies;
    return 0;
}

static struct intel_pmic_info vccsdio_info = {
	.pmic_reg   = VCCSDIOCNT_ADDR,
	.init_data  = &vccsdio_data,
	.table_len  = ARRAY_SIZE(VCCSDIO_VSEL_table),
	.table      = VCCSDIO_VSEL_table,
	.before_vreg_off = vccsdio_before_vreg_off,
    .after_vreg_off = vccsdio_after_vreg_off,
    .before_vreg_on = vccsdio_before_vreg_on,
	.after_vreg_on = vccsdio_after_vreg_on,
    .min_disable_interval_ms = 20,
};

static struct platform_device vemmc2_device = {
	.name = "intel_regulator",
	.id = VEMMC2,
	.dev = {
		.platform_data = &vemmc2_info,
	},
};

static struct platform_device vccsdio_device = {
	.name = "intel_regulator",
	.id = VCCSDIO,
	.dev = {
		.platform_data = &vccsdio_info,
	},
};

static int __init regulator_init(void)
{
	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_CLOVERVIEW)
		return 0;

	if (INTEL_MID_BOARD(2, PHONE, CLVTP, VB, PRO)
	    || INTEL_MID_BOARD(2, PHONE, CLVTP, VB, ENG)
	    || INTEL_MID_BOARD(3, PHONE, CLVTP, RHB, PRO, VVLITE)
	    || INTEL_MID_BOARD(3, PHONE, CLVTP, RHB, ENG, VVLITE)
	    || ((INTEL_MID_BOARD(2, PHONE, CLVTP, RHB, PRO)
		 || INTEL_MID_BOARD(2, PHONE, CLVTP, RHB, ENG))
		&& (SPID_HARDWARE_ID(CLVTP, PHONE, VB, PR1A)
		    || SPID_HARDWARE_ID(CLVTP, PHONE, VB, PR1B)
		    || SPID_HARDWARE_ID(CLVTP, PHONE, VB, PR20))))
		atom_regulator_victoriabay_init();
	else if (INTEL_MID_BOARD(2, PHONE, CLVTP, RHB, PRO)
		 || INTEL_MID_BOARD(2, PHONE, CLVTP, RHB, ENG))
		atom_regulator_redhookbay_init();

	platform_device_register(&vprog2_device);
	platform_device_register(&vemmc1_device); //ASUS_BSP+++
	platform_device_register(&vemmc2_device);
	platform_device_register(&vccsdio_device);

	return 0;
}
device_initcall(regulator_init);
