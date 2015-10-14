#ifndef __IMX219_H__
#define __IMX219_H__
#include "common.h"

/************************** settings for imx *************************/

static struct imx_reg const imx219_STILL_8M_20fps[] = {
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x0C},
	{IMX_8BIT, 0x300A, 0xFF},
	{IMX_8BIT, 0x300B, 0xFF},
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x09},

	{IMX_8BIT, 0x0114, 0x01},
	{IMX_8BIT, 0x0128, 0x00},
	{IMX_8BIT, 0x012A, 0x13},
	{IMX_8BIT, 0x012B, 0x34},
	{IMX_8BIT, 0x0160, 0x0A},
	{IMX_8BIT, 0x0161, 0x01},
	{IMX_8BIT, 0x0162, 0x0D},
	{IMX_8BIT, 0x0163, 0x78},
	{IMX_8BIT, 0x0164, 0x00},
	{IMX_8BIT, 0x0165, 0x00},
	{IMX_8BIT, 0x0166, 0x0C},
	{IMX_8BIT, 0x0167, 0xCF},
	{IMX_8BIT, 0x0168, 0x00},
	{IMX_8BIT, 0x0169, 0x00},
	{IMX_8BIT, 0x016A, 0x09},
	{IMX_8BIT, 0x016B, 0x9F},
	{IMX_8BIT, 0x016C, 0x0C},
	{IMX_8BIT, 0x016D, 0xD0},
	{IMX_8BIT, 0x016E, 0x09},
	{IMX_8BIT, 0x016F, 0xA0},
	{IMX_8BIT, 0x0170, 0x01},
	{IMX_8BIT, 0x0171, 0x01},
	{IMX_8BIT, 0x0174, 0x00},
	{IMX_8BIT, 0x0175, 0x00},
	{IMX_8BIT, 0x0176, 0x00},
	{IMX_8BIT, 0x0177, 0x00},
	{IMX_8BIT, 0x018C, 0x0A},
	{IMX_8BIT, 0x018D, 0x0A},
	{IMX_8BIT, 0x0301, 0x05},
	{IMX_8BIT, 0x0303, 0x01},
	{IMX_8BIT, 0x0304, 0x02},
	{IMX_8BIT, 0x0305, 0x02},
	{IMX_8BIT, 0x0306, 0x00},
	{IMX_8BIT, 0x0307, 0x2E},
	{IMX_8BIT, 0x0309, 0x0A},
	{IMX_8BIT, 0x030B, 0x01},
	{IMX_8BIT, 0x030C, 0x00},
	{IMX_8BIT, 0x030D, 0x5C},
	{IMX_8BIT, 0x455E, 0x00},
	{IMX_8BIT, 0x471E, 0x4B},
	{IMX_8BIT, 0x4767, 0x0F},
	{IMX_8BIT, 0x4750, 0x14},
	{IMX_8BIT, 0x4540, 0x00},
	{IMX_8BIT, 0x47B4, 0x14},
	{IMX_8BIT, 0x4713, 0x30},
	{IMX_8BIT, 0x478B, 0x10},
	{IMX_8BIT, 0x478F, 0x10},
	{IMX_8BIT, 0x4793, 0x10},
	{IMX_8BIT, 0x4797, 0x0E},
	{IMX_8BIT, 0x479B, 0x0E},

	{IMX_TOK_TERM, 0, 0}
};

static struct imx_reg const imx219_STILL_8M_18fps[] = {
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x0C},
	{IMX_8BIT, 0x300A, 0xFF},
	{IMX_8BIT, 0x300B, 0xFF},
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x09},

	{IMX_8BIT, 0x0114, 0x01},
	{IMX_8BIT, 0x0128, 0x00},
	{IMX_8BIT, 0x012A, 0x13},
	{IMX_8BIT, 0x012B, 0x34},
	{IMX_8BIT, 0x0160, 0x0A},
	{IMX_8BIT, 0x0161, 0x01},
	{IMX_8BIT, 0x0162, 0x0E},
	{IMX_8BIT, 0x0163, 0xE8},
	{IMX_8BIT, 0x0164, 0x00},
	{IMX_8BIT, 0x0165, 0x00},
	{IMX_8BIT, 0x0166, 0x0C},
	{IMX_8BIT, 0x0167, 0xCF},
	{IMX_8BIT, 0x0168, 0x00},
	{IMX_8BIT, 0x0169, 0x00},
	{IMX_8BIT, 0x016A, 0x09},
	{IMX_8BIT, 0x016B, 0x9F},
	{IMX_8BIT, 0x016C, 0x0C},
	{IMX_8BIT, 0x016D, 0xD0},
	{IMX_8BIT, 0x016E, 0x09},
	{IMX_8BIT, 0x016F, 0xA0},
	{IMX_8BIT, 0x0170, 0x01},
	{IMX_8BIT, 0x0171, 0x01},
	{IMX_8BIT, 0x0174, 0x00},
	{IMX_8BIT, 0x0175, 0x00},
	{IMX_8BIT, 0x0176, 0x00},
	{IMX_8BIT, 0x0177, 0x00},
	{IMX_8BIT, 0x018C, 0x0A},
	{IMX_8BIT, 0x018D, 0x0A},
	{IMX_8BIT, 0x0301, 0x05},
	{IMX_8BIT, 0x0303, 0x01},
	{IMX_8BIT, 0x0304, 0x02},
	{IMX_8BIT, 0x0305, 0x02},
	{IMX_8BIT, 0x0306, 0x00},
	{IMX_8BIT, 0x0307, 0x2E},
	{IMX_8BIT, 0x0309, 0x0A},
	{IMX_8BIT, 0x030B, 0x01},
	{IMX_8BIT, 0x030C, 0x00},
	{IMX_8BIT, 0x030D, 0x5C},
	{IMX_8BIT, 0x455E, 0x00},
	{IMX_8BIT, 0x471E, 0x4B},
	{IMX_8BIT, 0x4767, 0x0F},
	{IMX_8BIT, 0x4750, 0x14},
	{IMX_8BIT, 0x4540, 0x00},
	{IMX_8BIT, 0x47B4, 0x14},
	{IMX_8BIT, 0x4713, 0x30},
	{IMX_8BIT, 0x478B, 0x10},
	{IMX_8BIT, 0x478F, 0x10},
	{IMX_8BIT, 0x4793, 0x10},
	{IMX_8BIT, 0x4797, 0x0E},
	{IMX_8BIT, 0x479B, 0x0E},

	{IMX_TOK_TERM, 0, 0}
};

static struct imx_reg const imx219_STILL_6M_27fps[] = {
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x0C},
	{IMX_8BIT, 0x300A, 0xFF},
	{IMX_8BIT, 0x300B, 0xFF},
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x09},

	{IMX_8BIT, 0x0114, 0x01},
	{IMX_8BIT, 0x0128, 0x00},
	{IMX_8BIT, 0x012A, 0x13},
	{IMX_8BIT, 0x012B, 0x34},
	{IMX_8BIT, 0x0160, 0x07},
	{IMX_8BIT, 0x0161, 0x69},
	{IMX_8BIT, 0x0162, 0x0D},
	{IMX_8BIT, 0x0163, 0x78},
	{IMX_8BIT, 0x0164, 0x00},
	{IMX_8BIT, 0x0165, 0x00},
	{IMX_8BIT, 0x0166, 0x0C},
	{IMX_8BIT, 0x0167, 0xCF},
	{IMX_8BIT, 0x0168, 0x01},
	{IMX_8BIT, 0x0169, 0x32},
	{IMX_8BIT, 0x016A, 0x08},
	{IMX_8BIT, 0x016B, 0x6D},
	{IMX_8BIT, 0x016C, 0x0C},
	{IMX_8BIT, 0x016D, 0xD0},
	{IMX_8BIT, 0x016E, 0x07},
	{IMX_8BIT, 0x016F, 0x3C},
	{IMX_8BIT, 0x0170, 0x01},
	{IMX_8BIT, 0x0171, 0x01},
	{IMX_8BIT, 0x0174, 0x00},
	{IMX_8BIT, 0x0175, 0x00},
	{IMX_8BIT, 0x0176, 0x00},
	{IMX_8BIT, 0x0177, 0x00},
	{IMX_8BIT, 0x018C, 0x0A},
	{IMX_8BIT, 0x018D, 0x0A},
	{IMX_8BIT, 0x0301, 0x05},
	{IMX_8BIT, 0x0303, 0x01},
	{IMX_8BIT, 0x0304, 0x02},
	{IMX_8BIT, 0x0305, 0x02},
	{IMX_8BIT, 0x0306, 0x00},
	{IMX_8BIT, 0x0307, 0x2E},
	{IMX_8BIT, 0x0309, 0x0A},
	{IMX_8BIT, 0x030B, 0x01},
	{IMX_8BIT, 0x030C, 0x00},
	{IMX_8BIT, 0x030D, 0x5C},
	{IMX_8BIT, 0x455E, 0x00},
	{IMX_8BIT, 0x471E, 0x4B},
	{IMX_8BIT, 0x4767, 0x0F},
	{IMX_8BIT, 0x4750, 0x14},
	{IMX_8BIT, 0x4540, 0x00},
	{IMX_8BIT, 0x47B4, 0x14},
	{IMX_8BIT, 0x4713, 0x30},
	{IMX_8BIT, 0x478B, 0x10},
	{IMX_8BIT, 0x478F, 0x10},
	{IMX_8BIT, 0x4793, 0x10},
	{IMX_8BIT, 0x4797, 0x0E},
	{IMX_8BIT, 0x479B, 0x0E},

	{IMX_TOK_TERM, 0, 0}
};

static struct imx_reg const imx219_STILL_6M_25fps[] = {
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x0C},
	{IMX_8BIT, 0x300A, 0xFF},
	{IMX_8BIT, 0x300B, 0xFF},
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x09},

	{IMX_8BIT, 0x0114, 0x01},
	{IMX_8BIT, 0x0128, 0x00},
	{IMX_8BIT, 0x012A, 0x13},
	{IMX_8BIT, 0x012B, 0x34},
	{IMX_8BIT, 0x0160, 0x07},
	{IMX_8BIT, 0x0161, 0x69},
	{IMX_8BIT, 0x0162, 0x0E},
	{IMX_8BIT, 0x0163, 0x78},
	{IMX_8BIT, 0x0164, 0x00},
	{IMX_8BIT, 0x0165, 0x00},
	{IMX_8BIT, 0x0166, 0x0C},
	{IMX_8BIT, 0x0167, 0xCF},
	{IMX_8BIT, 0x0168, 0x01},
	{IMX_8BIT, 0x0169, 0x32},
	{IMX_8BIT, 0x016A, 0x08},
	{IMX_8BIT, 0x016B, 0x6D},
	{IMX_8BIT, 0x016C, 0x0C},
	{IMX_8BIT, 0x016D, 0xD0},
	{IMX_8BIT, 0x016E, 0x07},
	{IMX_8BIT, 0x016F, 0x3C},
	{IMX_8BIT, 0x0170, 0x01},
	{IMX_8BIT, 0x0171, 0x01},
	{IMX_8BIT, 0x0174, 0x00},
	{IMX_8BIT, 0x0175, 0x00},
	{IMX_8BIT, 0x0176, 0x00},
	{IMX_8BIT, 0x0177, 0x00},
	{IMX_8BIT, 0x018C, 0x0A},
	{IMX_8BIT, 0x018D, 0x0A},
	{IMX_8BIT, 0x0301, 0x05},
	{IMX_8BIT, 0x0303, 0x01},
	{IMX_8BIT, 0x0304, 0x02},
	{IMX_8BIT, 0x0305, 0x02},
	{IMX_8BIT, 0x0306, 0x00},
	{IMX_8BIT, 0x0307, 0x2E},
	{IMX_8BIT, 0x0309, 0x0A},
	{IMX_8BIT, 0x030B, 0x01},
	{IMX_8BIT, 0x030C, 0x00},
	{IMX_8BIT, 0x030D, 0x5C},
	{IMX_8BIT, 0x455E, 0x00},
	{IMX_8BIT, 0x471E, 0x4B},
	{IMX_8BIT, 0x4767, 0x0F},
	{IMX_8BIT, 0x4750, 0x14},
	{IMX_8BIT, 0x4540, 0x00},
	{IMX_8BIT, 0x47B4, 0x14},
	{IMX_8BIT, 0x4713, 0x30},
	{IMX_8BIT, 0x478B, 0x10},
	{IMX_8BIT, 0x478F, 0x10},
	{IMX_8BIT, 0x4793, 0x10},
	{IMX_8BIT, 0x4797, 0x0E},
	{IMX_8BIT, 0x479B, 0x0E},

	{IMX_TOK_TERM, 0, 0}
};

static struct imx_reg const imx219_PREVIEW_28fps[] = {
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x0C},
	{IMX_8BIT, 0x300A, 0xFF},
	{IMX_8BIT, 0x300B, 0xFF},
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x09},

	{IMX_8BIT, 0x0114, 0x01},
	{IMX_8BIT, 0x0128, 0x00},
	{IMX_8BIT, 0x012A, 0x13},
	{IMX_8BIT, 0x012B, 0x34},
	{IMX_8BIT, 0x0160, 0x07},
	{IMX_8BIT, 0x0161, 0x04},
	{IMX_8BIT, 0x0162, 0x0D},
	{IMX_8BIT, 0x0163, 0x78},
	{IMX_8BIT, 0x0164, 0x00},
	{IMX_8BIT, 0x0165, 0x00},
	{IMX_8BIT, 0x0166, 0x0C},
	{IMX_8BIT, 0x0167, 0xCF},
	{IMX_8BIT, 0x0168, 0x00},
	{IMX_8BIT, 0x0169, 0x00},
	{IMX_8BIT, 0x016A, 0x09},
	{IMX_8BIT, 0x016B, 0x9F},
	{IMX_8BIT, 0x016C, 0x06},
	{IMX_8BIT, 0x016D, 0x68},
	{IMX_8BIT, 0x016E, 0x04},
	{IMX_8BIT, 0x016F, 0xD0},
	{IMX_8BIT, 0x0170, 0x01},
	{IMX_8BIT, 0x0171, 0x01},
	{IMX_8BIT, 0x0174, 0x01},
	{IMX_8BIT, 0x0175, 0x01},
	{IMX_8BIT, 0x0176, 0x00},
	{IMX_8BIT, 0x0177, 0x00},
	{IMX_8BIT, 0x018C, 0x0A},
	{IMX_8BIT, 0x018D, 0x0A},
	{IMX_8BIT, 0x0301, 0x05},
	{IMX_8BIT, 0x0303, 0x01},
	{IMX_8BIT, 0x0304, 0x02},
	{IMX_8BIT, 0x0305, 0x02},
	{IMX_8BIT, 0x0306, 0x00},
	{IMX_8BIT, 0x0307, 0x2E},
	{IMX_8BIT, 0x0309, 0x0A},
	{IMX_8BIT, 0x030B, 0x01},
	{IMX_8BIT, 0x030C, 0x00},
	{IMX_8BIT, 0x030D, 0x5C},
	{IMX_8BIT, 0x455E, 0x00},
	{IMX_8BIT, 0x471E, 0x4B},
	{IMX_8BIT, 0x4767, 0x0F},
	{IMX_8BIT, 0x4750, 0x14},
	{IMX_8BIT, 0x4540, 0x00},
	{IMX_8BIT, 0x47B4, 0x14},
	{IMX_8BIT, 0x4713, 0x30},
	{IMX_8BIT, 0x478B, 0x10},
	{IMX_8BIT, 0x478F, 0x10},
	{IMX_8BIT, 0x4793, 0x10},
	{IMX_8BIT, 0x4797, 0x0E},
	{IMX_8BIT, 0x479B, 0x0E},

	{IMX_TOK_TERM, 0, 0}
};

static struct imx_reg const imx219_WIDE_PREVIEW_37fps[] = {
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x0C},
	{IMX_8BIT, 0x300A, 0xFF},
	{IMX_8BIT, 0x300B, 0xFF},
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x09},

	{IMX_8BIT, 0x0114, 0x01},
	{IMX_8BIT, 0x0128, 0x00},
	{IMX_8BIT, 0x012A, 0x13},
	{IMX_8BIT, 0x012B, 0x34},
	{IMX_8BIT, 0x0160, 0x05},
	{IMX_8BIT, 0x0161, 0x68},
	{IMX_8BIT, 0x0162, 0x0D},
	{IMX_8BIT, 0x0163, 0x78},
	{IMX_8BIT, 0x0164, 0x00},
	{IMX_8BIT, 0x0165, 0x00},
	{IMX_8BIT, 0x0166, 0x0C},
	{IMX_8BIT, 0x0167, 0xCF},
	{IMX_8BIT, 0x0168, 0x01},
	{IMX_8BIT, 0x0169, 0x36},
	{IMX_8BIT, 0x016A, 0x08},
	{IMX_8BIT, 0x016B, 0x69},
	{IMX_8BIT, 0x016C, 0x06},
	{IMX_8BIT, 0x016D, 0x68},
	{IMX_8BIT, 0x016E, 0x03},
	{IMX_8BIT, 0x016F, 0x9A},
	{IMX_8BIT, 0x0170, 0x01},
	{IMX_8BIT, 0x0171, 0x01},
	{IMX_8BIT, 0x0174, 0x01},
	{IMX_8BIT, 0x0175, 0x01},
	{IMX_8BIT, 0x0176, 0x00},
	{IMX_8BIT, 0x0177, 0x00},
	{IMX_8BIT, 0x018C, 0x0A},
	{IMX_8BIT, 0x018D, 0x0A},
	{IMX_8BIT, 0x0301, 0x05},
	{IMX_8BIT, 0x0303, 0x01},
	{IMX_8BIT, 0x0304, 0x02},
	{IMX_8BIT, 0x0305, 0x02},
	{IMX_8BIT, 0x0306, 0x00},
	{IMX_8BIT, 0x0307, 0x2E},
	{IMX_8BIT, 0x0309, 0x0A},
	{IMX_8BIT, 0x030B, 0x01},
	{IMX_8BIT, 0x030C, 0x00},
	{IMX_8BIT, 0x030D, 0x5C},
	{IMX_8BIT, 0x455E, 0x00},
	{IMX_8BIT, 0x471E, 0x4B},
	{IMX_8BIT, 0x4767, 0x0F},
	{IMX_8BIT, 0x4750, 0x14},
	{IMX_8BIT, 0x4540, 0x00},
	{IMX_8BIT, 0x47B4, 0x14},
	{IMX_8BIT, 0x4713, 0x30},
	{IMX_8BIT, 0x478B, 0x10},
	{IMX_8BIT, 0x478F, 0x10},
	{IMX_8BIT, 0x4793, 0x10},
	{IMX_8BIT, 0x4797, 0x0E},
	{IMX_8BIT, 0x479B, 0x0E},

	{IMX_TOK_TERM, 0, 0}
};

static struct imx_reg const imx219_VIDEO_1080P_30fps[] = {
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x0C},
	{IMX_8BIT, 0x300A, 0xFF},
	{IMX_8BIT, 0x300B, 0xFF},
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x09},

	{IMX_8BIT, 0x0114, 0x01},
	{IMX_8BIT, 0x0128, 0x00},
	{IMX_8BIT, 0x012A, 0x13},
	{IMX_8BIT, 0x012B, 0x34},
	{IMX_8BIT, 0x0160, 0x06},
	{IMX_8BIT, 0x0161, 0xAB},
	{IMX_8BIT, 0x0162, 0x0D},
	{IMX_8BIT, 0x0163, 0x78},
	{IMX_8BIT, 0x0164, 0x02},
	{IMX_8BIT, 0x0165, 0xA0},
	{IMX_8BIT, 0x0166, 0x0A},
	{IMX_8BIT, 0x0167, 0x2F},
	{IMX_8BIT, 0x0168, 0x02},
	{IMX_8BIT, 0x0169, 0xAC},
	{IMX_8BIT, 0x016A, 0x06},
	{IMX_8BIT, 0x016B, 0xF3},
	{IMX_8BIT, 0x016C, 0x07},
	{IMX_8BIT, 0x016D, 0x90},
	{IMX_8BIT, 0x016E, 0x04},
	{IMX_8BIT, 0x016F, 0x48},
	{IMX_8BIT, 0x0170, 0x01},
	{IMX_8BIT, 0x0171, 0x01},
	{IMX_8BIT, 0x0174, 0x00},
	{IMX_8BIT, 0x0175, 0x00},
	{IMX_8BIT, 0x0176, 0x00},
	{IMX_8BIT, 0x0177, 0x00},
	{IMX_8BIT, 0x018C, 0x0A},
	{IMX_8BIT, 0x018D, 0x0A},
	{IMX_8BIT, 0x0301, 0x05},
	{IMX_8BIT, 0x0303, 0x01},
	{IMX_8BIT, 0x0304, 0x02},
	{IMX_8BIT, 0x0305, 0x02},
	{IMX_8BIT, 0x0306, 0x00},
	{IMX_8BIT, 0x0307, 0x2E},
	{IMX_8BIT, 0x0309, 0x0A},
	{IMX_8BIT, 0x030B, 0x01},
	{IMX_8BIT, 0x030C, 0x00},
	{IMX_8BIT, 0x030D, 0x5C},
	{IMX_8BIT, 0x455E, 0x00},
	{IMX_8BIT, 0x471E, 0x4B},
	{IMX_8BIT, 0x4767, 0x0F},
	{IMX_8BIT, 0x4750, 0x14},
	{IMX_8BIT, 0x4540, 0x00},
	{IMX_8BIT, 0x47B4, 0x14},
	{IMX_8BIT, 0x4713, 0x30},
	{IMX_8BIT, 0x478B, 0x10},
	{IMX_8BIT, 0x478F, 0x10},
	{IMX_8BIT, 0x4793, 0x10},
	{IMX_8BIT, 0x4797, 0x0E},
	{IMX_8BIT, 0x479B, 0x0E},

	{IMX_TOK_TERM, 0, 0}
};

static struct imx_reg const imx219_VIDEO_720P_45fps[] = {
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x0C},
	{IMX_8BIT, 0x300A, 0xFF},
	{IMX_8BIT, 0x300B, 0xFF},
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x09},

	{IMX_8BIT, 0x0114, 0x01},
	{IMX_8BIT, 0x0128, 0x00},
	{IMX_8BIT, 0x012A, 0x13},
	{IMX_8BIT, 0x012B, 0x34},
	{IMX_8BIT, 0x0160, 0x04},
	{IMX_8BIT, 0x0161, 0x60},
	{IMX_8BIT, 0x0162, 0x0D},
	{IMX_8BIT, 0x0163, 0x78},
	{IMX_8BIT, 0x0164, 0x01},
	{IMX_8BIT, 0x0165, 0x58},
	{IMX_8BIT, 0x0166, 0x0B},
	{IMX_8BIT, 0x0167, 0x77},
	{IMX_8BIT, 0x0168, 0x01},
	{IMX_8BIT, 0x0169, 0xF0},
	{IMX_8BIT, 0x016A, 0x07},
	{IMX_8BIT, 0x016B, 0xAF},
	{IMX_8BIT, 0x016C, 0x05},
	{IMX_8BIT, 0x016D, 0x10},
	{IMX_8BIT, 0x016E, 0x02},
	{IMX_8BIT, 0x016F, 0xE0},
	{IMX_8BIT, 0x0170, 0x01},
	{IMX_8BIT, 0x0171, 0x01},
	{IMX_8BIT, 0x0174, 0x01},
	{IMX_8BIT, 0x0175, 0x01},
	{IMX_8BIT, 0x0176, 0x00},
	{IMX_8BIT, 0x0177, 0x00},
	{IMX_8BIT, 0x018C, 0x0A},
	{IMX_8BIT, 0x018D, 0x0A},
	{IMX_8BIT, 0x0301, 0x05},
	{IMX_8BIT, 0x0303, 0x01},
	{IMX_8BIT, 0x0304, 0x02},
	{IMX_8BIT, 0x0305, 0x02},
	{IMX_8BIT, 0x0306, 0x00},
	{IMX_8BIT, 0x0307, 0x2E},
	{IMX_8BIT, 0x0309, 0x0A},
	{IMX_8BIT, 0x030B, 0x01},
	{IMX_8BIT, 0x030C, 0x00},
	{IMX_8BIT, 0x030D, 0x5C},
	{IMX_8BIT, 0x455E, 0x00},
	{IMX_8BIT, 0x471E, 0x4B},
	{IMX_8BIT, 0x4767, 0x0F},
	{IMX_8BIT, 0x4750, 0x14},
	{IMX_8BIT, 0x4540, 0x00},
	{IMX_8BIT, 0x47B4, 0x14},
	{IMX_8BIT, 0x4713, 0x30},
	{IMX_8BIT, 0x478B, 0x10},
	{IMX_8BIT, 0x478F, 0x10},
	{IMX_8BIT, 0x4793, 0x10},
	{IMX_8BIT, 0x4797, 0x0E},
	{IMX_8BIT, 0x479B, 0x0E},

	{IMX_TOK_TERM, 0, 0}
};

static struct imx_reg const imx219_VIDEO_480P_57fps[] = {
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x0C},
	{IMX_8BIT, 0x300A, 0xFF},
	{IMX_8BIT, 0x300B, 0xFF},
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x09},

	{IMX_8BIT, 0x0114, 0x01},
	{IMX_8BIT, 0x0128, 0x00},
	{IMX_8BIT, 0x012A, 0x13},
	{IMX_8BIT, 0x012B, 0x34},
	{IMX_8BIT, 0x0160, 0x03},
	{IMX_8BIT, 0x0161, 0x80},
	{IMX_8BIT, 0x0162, 0x0D},
	{IMX_8BIT, 0x0163, 0x78},
	{IMX_8BIT, 0x0164, 0x01},
	{IMX_8BIT, 0x0165, 0x48},
	{IMX_8BIT, 0x0166, 0x0B},
	{IMX_8BIT, 0x0167, 0x87},
	{IMX_8BIT, 0x0168, 0x00},
	{IMX_8BIT, 0x0169, 0xF0},
	{IMX_8BIT, 0x016A, 0x08},
	{IMX_8BIT, 0x016B, 0xAF},
	{IMX_8BIT, 0x016C, 0x02},
	{IMX_8BIT, 0x016D, 0x90},
	{IMX_8BIT, 0x016E, 0x01},
	{IMX_8BIT, 0x016F, 0xF0},
	{IMX_8BIT, 0x0170, 0x01},
	{IMX_8BIT, 0x0171, 0x01},
	{IMX_8BIT, 0x0174, 0x02},
	{IMX_8BIT, 0x0175, 0x02},
	{IMX_8BIT, 0x0176, 0x00},
	{IMX_8BIT, 0x0177, 0x00},
	{IMX_8BIT, 0x018C, 0x0A},
	{IMX_8BIT, 0x018D, 0x0A},
	{IMX_8BIT, 0x0301, 0x05},
	{IMX_8BIT, 0x0303, 0x01},
	{IMX_8BIT, 0x0304, 0x02},
	{IMX_8BIT, 0x0305, 0x02},
	{IMX_8BIT, 0x0306, 0x00},
	{IMX_8BIT, 0x0307, 0x2E},
	{IMX_8BIT, 0x0309, 0x0A},
	{IMX_8BIT, 0x030B, 0x01},
	{IMX_8BIT, 0x030C, 0x00},
	{IMX_8BIT, 0x030D, 0x5C},
	{IMX_8BIT, 0x455E, 0x00},
	{IMX_8BIT, 0x471E, 0x4B},
	{IMX_8BIT, 0x4767, 0x0F},
	{IMX_8BIT, 0x4750, 0x14},
	{IMX_8BIT, 0x4540, 0x00},
	{IMX_8BIT, 0x47B4, 0x14},
	{IMX_8BIT, 0x4713, 0x30},
	{IMX_8BIT, 0x478B, 0x10},
	{IMX_8BIT, 0x478F, 0x10},
	{IMX_8BIT, 0x4793, 0x10},
	{IMX_8BIT, 0x4797, 0x0E},
	{IMX_8BIT, 0x479B, 0x0E},

	{IMX_TOK_TERM, 0, 0}
};

static struct imx_reg const imx219_VIDEO_CIF_80fps[] = {
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x0C},
	{IMX_8BIT, 0x300A, 0xFF},
	{IMX_8BIT, 0x300B, 0xFF},
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x09},

	{IMX_8BIT, 0x0114, 0x01},
	{IMX_8BIT, 0x0128, 0x00},
	{IMX_8BIT, 0x012A, 0x13},
	{IMX_8BIT, 0x012B, 0x34},
	{IMX_8BIT, 0x0160, 0x02},
	{IMX_8BIT, 0x0161, 0x80},
	{IMX_8BIT, 0x0162, 0x0D},
	{IMX_8BIT, 0x0163, 0x78},
	{IMX_8BIT, 0x0164, 0x03},
	{IMX_8BIT, 0x0165, 0x88},
	{IMX_8BIT, 0x0166, 0x09},
	{IMX_8BIT, 0x0167, 0x47},
	{IMX_8BIT, 0x0168, 0x02},
	{IMX_8BIT, 0x0169, 0x70},
	{IMX_8BIT, 0x016A, 0x07},
	{IMX_8BIT, 0x016B, 0x2F},
	{IMX_8BIT, 0x016C, 0x01},
	{IMX_8BIT, 0x016D, 0x70},
	{IMX_8BIT, 0x016E, 0x01},
	{IMX_8BIT, 0x016F, 0x30},
	{IMX_8BIT, 0x0170, 0x01},
	{IMX_8BIT, 0x0171, 0x01},
	{IMX_8BIT, 0x0174, 0x02},
	{IMX_8BIT, 0x0175, 0x02},
	{IMX_8BIT, 0x0176, 0x00},
	{IMX_8BIT, 0x0177, 0x00},
	{IMX_8BIT, 0x018C, 0x0A},
	{IMX_8BIT, 0x018D, 0x0A},
	{IMX_8BIT, 0x0301, 0x05},
	{IMX_8BIT, 0x0303, 0x01},
	{IMX_8BIT, 0x0304, 0x02},
	{IMX_8BIT, 0x0305, 0x02},
	{IMX_8BIT, 0x0306, 0x00},
	{IMX_8BIT, 0x0307, 0x2E},
	{IMX_8BIT, 0x0309, 0x0A},
	{IMX_8BIT, 0x030B, 0x01},
	{IMX_8BIT, 0x030C, 0x00},
	{IMX_8BIT, 0x030D, 0x5C},
	{IMX_8BIT, 0x455E, 0x00},
	{IMX_8BIT, 0x471E, 0x4B},
	{IMX_8BIT, 0x4767, 0x0F},
	{IMX_8BIT, 0x4750, 0x14},
	{IMX_8BIT, 0x4540, 0x00},
	{IMX_8BIT, 0x47B4, 0x14},
	{IMX_8BIT, 0x4713, 0x30},
	{IMX_8BIT, 0x478B, 0x10},
	{IMX_8BIT, 0x478F, 0x10},
	{IMX_8BIT, 0x4793, 0x10},
	{IMX_8BIT, 0x4797, 0x0E},
	{IMX_8BIT, 0x479B, 0x0E},

	{IMX_TOK_TERM, 0, 0}
};

static struct imx_reg const imx219_VIDEO_QCIF_80fps[] = {
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x0C},
	{IMX_8BIT, 0x300A, 0xFF},
	{IMX_8BIT, 0x300B, 0xFF},
	{IMX_8BIT, 0x30EB, 0x05},
	{IMX_8BIT, 0x30EB, 0x09},

	{IMX_8BIT, 0x0114, 0x01},
	{IMX_8BIT, 0x0128, 0x00},
	{IMX_8BIT, 0x012A, 0x13},
	{IMX_8BIT, 0x012B, 0x34},
	{IMX_8BIT, 0x0160, 0x02},
	{IMX_8BIT, 0x0161, 0x80},
	{IMX_8BIT, 0x0162, 0x0D},
	{IMX_8BIT, 0x0163, 0x78},
	{IMX_8BIT, 0x0164, 0x04},
	{IMX_8BIT, 0x0165, 0xE8},
	{IMX_8BIT, 0x0166, 0x07},
	{IMX_8BIT, 0x0167, 0xE7},
	{IMX_8BIT, 0x0168, 0x03},
	{IMX_8BIT, 0x0169, 0x90},
	{IMX_8BIT, 0x016A, 0x06},
	{IMX_8BIT, 0x016B, 0x0F},
	{IMX_8BIT, 0x016C, 0x00},
	{IMX_8BIT, 0x016D, 0xC0},
	{IMX_8BIT, 0x016E, 0x00},
	{IMX_8BIT, 0x016F, 0xA0},
	{IMX_8BIT, 0x0170, 0x01},
	{IMX_8BIT, 0x0171, 0x01},
	{IMX_8BIT, 0x0174, 0x02},
	{IMX_8BIT, 0x0175, 0x02},
	{IMX_8BIT, 0x0176, 0x00},
	{IMX_8BIT, 0x0177, 0x00},
	{IMX_8BIT, 0x018C, 0x0A},
	{IMX_8BIT, 0x018D, 0x0A},
	{IMX_8BIT, 0x0301, 0x05},
	{IMX_8BIT, 0x0303, 0x01},
	{IMX_8BIT, 0x0304, 0x02},
	{IMX_8BIT, 0x0305, 0x02},
	{IMX_8BIT, 0x0306, 0x00},
	{IMX_8BIT, 0x0307, 0x2E},
	{IMX_8BIT, 0x0309, 0x0A},
	{IMX_8BIT, 0x030B, 0x01},
	{IMX_8BIT, 0x030C, 0x00},
	{IMX_8BIT, 0x030D, 0x5C},
	{IMX_8BIT, 0x455E, 0x00},
	{IMX_8BIT, 0x471E, 0x4B},
	{IMX_8BIT, 0x4767, 0x0F},
	{IMX_8BIT, 0x4750, 0x14},
	{IMX_8BIT, 0x4540, 0x00},
	{IMX_8BIT, 0x47B4, 0x14},
	{IMX_8BIT, 0x4713, 0x30},
	{IMX_8BIT, 0x478B, 0x10},
	{IMX_8BIT, 0x478F, 0x10},
	{IMX_8BIT, 0x4793, 0x10},
	{IMX_8BIT, 0x4797, 0x0E},
	{IMX_8BIT, 0x479B, 0x0E},

	{IMX_TOK_TERM, 0, 0}
};


static struct imx_reg const imx219_init_settings[] = {
	{IMX_TOK_TERM, 0, 0}
};

/* TODO settings of preview/still/video will be updated with new use case */
struct imx_resolution imx219_res_preview[] = {
/*
	{
		.desc = "PREVIEW_CIF_80fps",
		.regs = imx219_VIDEO_CIF_80fps,
		.width = 368,
		.height = 304,
		.bin_factor_x = 4,
		.bin_factor_y = 4,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
			{
				.fps = 80,
				.pixels_per_line = 0x0D78,
				.lines_per_frame = 0x0280,
			},
			{
			}
		},
	},
*/
	{
		.desc = "PREVIEW_6M_27fps",
		.regs = imx219_STILL_6M_27fps,
		.width = 3280,
		.height = 1852,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
	 		{
				.fps = 27,
				.pixels_per_line = 0x0D78,
				.lines_per_frame = 0x0769,
			},
			{
			}
		},
	},
	{
		.desc = "PREVIEW_8M_20fps",
		.regs = imx219_STILL_8M_20fps,
		.width = 3280,
		.height = 2464,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
	 		{
				.fps = 20,
				.pixels_per_line = 0x0D78,
				.lines_per_frame = 0x0A01,
			},
			{
			}
		},
	},
	{
		.desc = "VIDEO_1080P_30fps",
		.regs = imx219_VIDEO_1080P_30fps,
		.width = 1936,
		.height = 1096,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
	 		{
				.fps = 30,
				.pixels_per_line = 0x0D78,
				.lines_per_frame = 0x06AB,
			},
			{
			}
		},
	},
	{
		.desc = "WIDE_PREVIEW_37fps",
		.regs = imx219_WIDE_PREVIEW_37fps,
		.width = 1640,
		.height = 922,
		.bin_factor_x = 2,
		.bin_factor_y = 2,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
	 		{
				.fps = 37,
				.pixels_per_line = 0x0D78,
				.lines_per_frame = 0x0568,
			},
			{
			}
		},
	},
	{
		.desc = "PREVIEW_28fps",
		.regs = imx219_PREVIEW_28fps,
		.width = 1640,
		.height = 1232,
		.bin_factor_x = 2,
		.bin_factor_y = 2,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
	 		{
				.fps = 28,
				.pixels_per_line = 0x0D78,
				.lines_per_frame = 0x0704,
			},
			{
			}
		},
	},
};

struct imx_resolution imx219_res_still[] = {
	{
		.desc = "WIDE_STILL_37fps",
		.regs = imx219_WIDE_PREVIEW_37fps,
		.width = 1640,
		.height = 922,
		.bin_factor_x = 2,
		.bin_factor_y = 2,
		.used = 0,
		.skip_frames = 1,
		.fps_options = {
	 		{
				.fps = 37,
				.pixels_per_line = 0x0D78,
				.lines_per_frame = 0x0568,
			},
			{
			}
		},
	},
	{
		.desc = "STILL_28fps",
		.regs = imx219_PREVIEW_28fps,
		.width = 1640,
		.height = 1232,
		.bin_factor_x = 2,
		.bin_factor_y = 2,
		.used = 0,
		.skip_frames = 1,
		.fps_options = {
	 		{
				.fps = 28,
				.pixels_per_line = 0x0D78,
				.lines_per_frame = 0x0704,
			},
			{
			}
		},
	},
	{
		.desc = "VIDEO_1080P_30fps",
		.regs = imx219_VIDEO_1080P_30fps,
		.width = 1936,
		.height = 1096,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
	 		{
				.fps = 30,
				.pixels_per_line = 0x0D78,
				.lines_per_frame = 0x06AB,
			},
			{
			}
		},
	},
	{
		.desc = "STILL_6M_25fps",
		.regs = imx219_STILL_6M_25fps,
		.width = 3280,
		.height = 1852,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.used = 0,
		.skip_frames = 1,
		.fps_options = {
	 		{
				.fps = 25,
				.pixels_per_line = 0x0E78,
				.lines_per_frame = 0x0769,
			},
			{
			}
		},
	},
	{
		.desc = "STILL_8M_18fps",
		.regs = imx219_STILL_8M_18fps,
		.width = 3280,
		.height = 2464,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.used = 0,
		.skip_frames = 1,
		.fps_options = {
	 		{
				.fps = 18,
				.pixels_per_line = 0x0EE8,
				.lines_per_frame = 0x0A01,
			},
			{
			}
		},
	},
};

struct imx_resolution imx219_res_video[] = {
	{
		.desc = "VIDEO_QCIF_80fps",
		.regs = imx219_VIDEO_QCIF_80fps,
		.width = 192,
		.height = 160,
		.bin_factor_x = 4,
		.bin_factor_y = 4,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
	 		{
				.fps = 80,
				.pixels_per_line = 0x0D78,
				.lines_per_frame = 0x0280,
			},
			{
			}
		},
	},
	{
		.desc = "VIDEO_CIF_80fps",
		.regs = imx219_VIDEO_CIF_80fps,
		.width = 368,
		.height = 304,
		.bin_factor_x = 4,
		.bin_factor_y = 4,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
	 		{
				.fps = 80,
				.pixels_per_line = 0x0D78,
				.lines_per_frame = 0x0280,
			},
			{
			}
		},
	},
	{
		.desc = "VIDEO_480P_57fps",
		.regs = imx219_VIDEO_480P_57fps,
		.width = 656,
		.height = 496,
		.bin_factor_x = 4,
		.bin_factor_y = 4,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
	 		{
				.fps = 57,
				.pixels_per_line = 0x0D78,
				.lines_per_frame = 0x0380,
			},
			{
			}
		},
	},
	{
		.desc = "VIDEO_720P_45fps",
		.regs = imx219_VIDEO_720P_45fps,
		.width = 1296,
		.height = 736,
		.bin_factor_x = 2,
		.bin_factor_y = 2,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
	 		{
				.fps = 45,
				.pixels_per_line = 0x0D78,
				.lines_per_frame = 0x0460,
			},
			{
			}
		},
	},
	{
		.desc = "VIDEO_1080P_30fps",
		.regs = imx219_VIDEO_1080P_30fps,
		.width = 1936,
		.height = 1096,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
	 		{
				.fps = 30,
				.pixels_per_line = 0x0D78,
				.lines_per_frame = 0x06AB,
			},
			{
			}
		},
	},
};

#endif
