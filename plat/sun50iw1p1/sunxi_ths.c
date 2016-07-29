/*
 * Copyright (c) 2016, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ccmu.h>
#include <debug.h>
#include <delay_timer.h>
#include <plat_config.h>
#include <mmio.h>
#include <sys/errno.h>
#include "sunxi_def.h"
#include "sunxi_private.h"

#define SENS0_ENABLE_BIT	(0x1<<0)
#define SENS1_ENABLE_BIT	(0x1<<1)
#define SENS2_ENABLE_BIT	(0x1<<2)

#define THS_CTRL0_REG           (0x00)
#define THS_CTRL1_REG           (0x04)
#define ADC_CDAT_REG            (0x14)
#define THS_CTRL2_REG           (0x40)
#define THS_INT_CTRL_REG        (0x44)
#define THS_INT_STA_REG         (0x48)
#define THS_INT_ALM_TH_REG0     (0x50)
#define THS_INT_ALM_TH_REG1     (0x54)
#define THS_INT_ALM_TH_REG2     (0x58)
#define THS_INT_SHUT_TH_REG0    (0x60)
#define THS_INT_SHUT_TH_REG1    (0x64)
#define THS_INT_SHUT_TH_REG2    (0x68)
#define THS_FILT_CTRL_REG       (0x70)
#define THS_0_1_CDATA_REG       (0x74)
#define THS_2_CDATA_REG         (0x78)
#define THS_DATA_REG0           (0x80)
#define THS_DATA_REG1           (0x84)
#define THS_DATA_REG2           (0x88)

#define THS_INT_ALM_TH_VALUE0   (0x50)
#define THS_INT_ALM_TH_VALUE1   (0x54)
#define THS_INT_ALM_TH_VALUE2   (0x58)
#define THS_INT_SHUT_TH_VALUE0  (0x60)
#define THS_INT_SHUT_TH_VALUE1  (0x64)
#define THS_INT_SHUT_TH_VALUE2  (0x68)

#define THS_CTRL0_VALUE		(0x190)
#define THS_CTRL1_VALUE		(0x1<<17)
#define THS_CTRL2_VALUE		(0x01900001)
#define THS_INT_CTRL_VALUE	(0x18070)
#define THS_CLEAR_INT_STA	(0x777)
#define THS_FILT_CTRL_VALUE	(0x06)

/* temperature = ( MINUPA - reg * MULPA) / DIVPA */
#define MULPA			(1000)
#define DIVPA			(8560)
#define MINUPA			(2170000)

static int sun50_th_temp_to_reg(long temp)
{
	int reg;

	reg = (MINUPA - temp * DIVPA);
	reg = reg / MULPA;

	return (uint32_t)reg;
}

static long sun50_th_reg_to_temp(int reg_data)
{
	int t;

	t = (MINUPA - reg_data * MULPA);
	t = t / DIVPA;

	return (long)t;
}

static int init_ths(void)
{
	uint32_t reg;
	uint32_t shut_tmp = 120;

	reg = mmio_read_32(CCMU_BUS_CLK_GATING_REG2);
	reg |=  (0x1 << 8);
	mmio_write_32(CCMU_BUS_CLK_GATING_REG2, reg);

	reg = mmio_read_32(CCMU_THS_CLK_REG);
	reg |=  ((0x1 << 31 | 0x1 << 0));
	mmio_write_32(CCMU_THS_CLK_REG, reg);

	reg = mmio_read_32(CCMU_BUS_SOFT_RST_REG3);
	reg |=  (0x1 << 8);
	mmio_write_32(CCMU_BUS_SOFT_RST_REG3, reg);

	mmio_write_32(SUNXI_THS_BASE + THS_CTRL1_REG, THS_CTRL1_VALUE);
	mmio_write_32(SUNXI_THS_BASE + THS_CTRL0_REG, THS_CTRL0_VALUE);
	mmio_write_32(SUNXI_THS_BASE + THS_CTRL2_REG, THS_CTRL2_VALUE);

	udelay(400);

	mmio_write_32(SUNXI_THS_BASE + THS_INT_CTRL_REG, THS_INT_CTRL_VALUE);
	mmio_write_32(SUNXI_THS_BASE + THS_INT_STA_REG, 0x777);
	mmio_write_32(SUNXI_THS_BASE + THS_FILT_CTRL_REG, 0x06);

	reg = sun50_th_temp_to_reg(shut_tmp);
	reg = (reg<<16);

	mmio_write_32(SUNXI_THS_BASE + THS_INT_SHUT_TH_REG0, reg);

	reg = mmio_read_32(SUNXI_THS_BASE + THS_CTRL2_REG);
	reg |= SENS0_ENABLE_BIT | SENS1_ENABLE_BIT | SENS2_ENABLE_BIT;
	mmio_write_32(SUNXI_THS_BASE + THS_CTRL2_REG, reg);

	return 0;
}

static int sun50i_th_get_temp()
{
	int reg_data;
	long t = 0;

	reg_data = mmio_read_32(SUNXI_THS_BASE + (THS_DATA_REG0));
	t = sun50_th_reg_to_temp(reg_data);
	NOTICE("Temp is %d\n", t);

	return 0;
}

int sunxi_ths_setup(void)
{
	int ret;

	NOTICE("Configuring THS\n");
	ret = init_ths();
	sun50i_th_get_temp();

	return ret;
}
