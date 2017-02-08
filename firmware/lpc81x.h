/*
 * Copyright (c) 2017, Guillermo A. Amaral B. <g@maral.me>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
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

#ifndef LPC81X_H
#define LPC81X_H 1

#define GPIO0_BASE  ((void *)(0xA0000000UL))
#define I2C_BASE    ((void *)(0x40050000UL))
#define IOCON_BASE  ((void *)(0x40044000UL))
#define NVIC_BASE   ((void *)(0xE000E100UL))
#define PMU_BASE    ((void *)(0x40020000UL))
#define SCB_BASE    ((void *)(0xE000ED00UL))
#define SWM_BASE    ((void *)(0x4000C000UL))
#define SYSCON_BASE ((void *)(0x40048000UL))

#define __RW volatile
#define __WO volatile
#define __RO const volatile
#define __RV const

struct i2c_t
{
	__RW uint32_t CFG;
	__RW uint32_t STAT;
	__RW uint32_t INTENSET;
	__WO uint32_t INTENCLR;
	__RW uint32_t TIMEOUT;
	__RW uint32_t CLKDIV;
	__RO uint32_t INTSTAT;
	__RV uint32_t _RESERVED0[1];
	__RW uint32_t MSTCTL;
	__RW uint32_t MSTTIME;
	__RW uint32_t MSTDAT;
	__RV uint32_t _RESERVED1[5];
	__RW uint32_t SLVCTL;
	__RW uint32_t SLVDAT;
	__RW uint32_t SLVADR[4];
	__RW uint32_t SLVQUAL;
	__RV uint32_t _RESERVED2[9];
	__RO uint32_t MONRXDAT;
};

struct iocon_t
{
	__RW uint32_t PIO0_17;
	__RW uint32_t PIO0_13;
	__RW uint32_t PIO0_12;
	__RW uint32_t PIO0_5;
	__RW uint32_t PIO0_4;
	__RW uint32_t PIO0_3;
	__RW uint32_t PIO0_2;
	__RW uint32_t PIO0_11;
	__RW uint32_t PIO0_10;
	__RW uint32_t PIO0_16;
	__RW uint32_t PIO0_15;
	__RW uint32_t PIO0_1;
	__RV uint32_t _RESERVED[1];
	__RW uint32_t PIO0_9;
	__RW uint32_t PIO0_8;
	__RW uint32_t PIO0_7;
	__RW uint32_t PIO0_6;
	__RW uint32_t PIO0_0;
	__RW uint32_t PIO0_14;
};

struct gpio_t
{
	__RW uint8_t B[18];
	__RV uint8_t _RESERVED1[4078];
	__RW uint32_t W[18];
	__RV uint32_t _RESERVED2[1006];
	__RW uint32_t DIR0;
	__RV uint32_t _RESERVED3[31];
	__RW uint32_t MASK0;
	__RV uint32_t _RESERVED4[31];
	__RW uint32_t PIN0;
	__RV uint32_t _RESERVED5[31];
	__RW uint32_t MPIN0;
	__RV uint32_t _RESERVED6[31];
	__RW uint32_t SET0;
	__RV uint32_t _RESERVED7[31];
	__WO uint32_t CLR0;
	__RV uint32_t _RESERVED8[31];
	__WO uint32_t NOT0;
};

struct nvic_t
{
	__RW uint32_t ISER[1];
	__RV uint32_t _RESERVED0[31];
	__RW uint32_t ICER[1];
	__RV uint32_t _RESERVED1[31];
	__RW uint32_t ISPR[1];
	__RV uint32_t _RESERVED2[31];
	__RW uint32_t ICPR[1];
	__RV uint32_t _RESERVED3[31];
	__RV uint32_t _RESERVED4[64];
	__RW uint32_t IP[8];
};

struct pmu_t
{
	__RW uint32_t PCON;
	__RW uint32_t GPREG[4];
	__RW uint32_t DPDCTRL;
};

struct scb_t
{
	__RO uint32_t CPUID;
	__RW uint32_t ICSR;
	__RW uint32_t VTOR;
	__RW uint32_t AIRCR;
	__RW uint32_t SCR;
	__RW uint32_t CCR;
	__RV uint32_t _RESERVED[1];
	__RW uint32_t SHP[2];
	__RW uint32_t SHCSR;
};

struct switchmatrix_t
{
	__RW uint32_t PINASSIGN[9];
	__RV uint32_t _RESERVED[103];
	__RW uint32_t PINENABLE0;
};

struct syscon_t
{
	__RW uint32_t SYSMEMREMAP;
	__RW uint32_t PRESETCTRL;
	__RW uint32_t SYSPLLCTRL;
	__RO uint32_t SYSPLLSTAT;
	__RV uint32_t _RESERVED0[4];
	__RW uint32_t SYSOSCCTRL;
	__RW uint32_t WDTOSCCTRL;
	__RV uint32_t _RESERVED1[2];
	__RW uint32_t SYSRSTSTAT;
	__RV uint32_t _RESERVED2[3];
	__RW uint32_t SYSPLLCLKSEL;
	__RW uint32_t SYSPLLCLKUEN;
	__RV uint32_t _RESERVED3[10];
	__RW uint32_t MAINCLKSEL;
	__RW uint32_t MAINCLKUEN;
	__RW uint32_t SYSAHBCLKDIV;
	__RV uint32_t _RESERVED4[1];
	__RW uint32_t SYSAHBCLKCTRL;
	__RV uint32_t _RESERVED5[4];
	__RW uint32_t UARTCLKDIV;
	__RV uint32_t _RESERVED6[18];
	__RW uint32_t CLKOUTSEL;
	__RW uint32_t CLKOUTUEN;
	__RW uint32_t CLKOUTDIV;
	__RV uint32_t _RESERVED7[1];
	__RW uint32_t UARTFRGDIV;
	__RW uint32_t UARTFRGMULT;
	__RV uint32_t _RESERVED8[1];
	__RW uint32_t EXTTRACECMD;
	__RO uint32_t PIOPORCAP0;
	__RV uint32_t _RESERVED9[12];
	__RW uint32_t IOCONCLKDIV[7];
	__RW uint32_t BODCTRL;
	__RW uint32_t SYSTCKCAL;
	__RV uint32_t _RESERVED10[6];
	__RW uint32_t IRQLATENCY;
	__RW uint32_t NMISRC;
	__RW uint32_t PINTSEL[8];
	__RV uint32_t _RESERVED11[27];
	__RW uint32_t STARTERP0;
	__RV uint32_t _RESERVED12[3];
	__RW uint32_t STARTERP1;
	__RV uint32_t _RESERVED13[6];
	__RW uint32_t PDSLEEPCFG;
	__RW uint32_t PDAWAKECFG;
	__RW uint32_t PDRUNCFG;
	__RV uint32_t _RESERVED14[111];
	__RO uint32_t DEVICE_ID;
};

enum i2c_cfg_sym_t
{
	I2C_CFG_MSTEN     = (1 << 0),
	I2C_CFG_SLVEN     = (1 << 1),
	I2C_CFG_MONEN     = (1 << 2),
	I2C_CFG_TIMEOUTEN = (1 << 3),
	I2C_CFG_MONCLKSTR = (1 << 4)
};

enum i2c_slvctl_sym_t
{
	I2C_SLVCTL_SLVCONTINUE = (1 << 0),
	I2C_SLVCTL_SLVNACK     = (1 << 1)
};

enum i2c_stat_sym_t
{
	I2C_STAT_MSTPENDING   = (1 << 0),
	I2C_STAT_MSTARBLOSS   = (1 << 4),
	I2C_STAT_MSTSTSTPERR  = (1 << 6),
	I2C_STAT_SLVPENDING   = (1 << 8),
	I2C_STAT_SLVNOTSTR    = (1 << 11),
	I2C_STAT_SLVSEL       = (1 << 14),
	I2C_STAT_SLVDESEL     = (1 << 15),
	I2C_STAT_MONRDY       = (1 << 16),
	I2C_STAT_MONOV        = (1 << 17),
	I2C_STAT_MONACTIVE    = (1 << 18),
	I2C_STAT_MONIDLE      = (1 << 19),
	I2C_STAT_EVENTTIMEOUT = (1 << 24),
	I2C_STAT_SCLTIMEOUT   = (1 << 25),

	I2C_STAT_SLVSTATE      = (0b11 << 9),
	I2C_STAT_SLVSTATE_ADDR = (0b00 << 9),
	I2C_STAT_SLVSTATE_RX   = (0b01 << 9),
	I2C_STAT_SLVSTATE_TX   = (0b10 << 9),

	I2C_STAT_MSTSTATE      = (0b111 << 1),

	I2C_STAT_SLVIDX        = (0b11 << 12),
	I2C_STAT_SLVIDX_SLV_0  = (0b00 << 12),
	I2C_STAT_SLVIDX_SLV_1  = (0b01 << 12),
	I2C_STAT_SLVIDX_SLV_2  = (0b10 << 12),
	I2C_STAT_SLVIDX_SLV_3  = (0b11 << 12)
};

enum i2c_intenset_sym_t
{
	I2C_INTENSET_SLVPENDINGEN = (1 << 8),
	I2C_INTENSET_SLVSELEN     = (1 << 14),
	I2C_INTENSET_SLVDESELEN   = (1 << 15)
};

enum nvic_iser0_sym_t
{
	NVIC_ISER0_ISE_I2C = (1 << 8)
};

enum scb_sym_t
{
	SCB_SCR_SLEEPDEEP = (1 << 2)
};

enum syscon_presetctrl_sym_t
{
	SYSCON_PRESETCTRL_I2C_RST_N = (1 << 6),
	SYSCON_PRESETCTRL_GPIO_RST_N = (1 << 10)
};

enum syscon_starterp1_sym_t
{
	SYSCON_STARTERP1_I2C = (1 << 8)
};

enum syscon_sysahbclkctrl_sym_t
{
	SYSCON_SYSAHBCLKCTRL_SYS      = (1 << 0),
	SYSCON_SYSAHBCLKCTRL_ROM      = (1 << 1),
	SYSCON_SYSAHBCLKCTRL_RAM      = (1 << 2),
	SYSCON_SYSAHBCLKCTRL_FLASHREG = (1 << 3),
	SYSCON_SYSAHBCLKCTRL_FLASH    = (1 << 4),
	SYSCON_SYSAHBCLKCTRL_I2C      = (1 << 5),
	SYSCON_SYSAHBCLKCTRL_GPIO     = (1 << 6),
	SYSCON_SYSAHBCLKCTRL_SWM      = (1 << 7),
	SYSCON_SYSAHBCLKCTRL_SCT      = (1 << 8),
	SYSCON_SYSAHBCLKCTRL_WKT      = (1 << 9),
	SYSCON_SYSAHBCLKCTRL_MRT      = (1 << 10),
	SYSCON_SYSAHBCLKCTRL_SPI0     = (1 << 11),
	SYSCON_SYSAHBCLKCTRL_SPI1     = (1 << 12),
	SYSCON_SYSAHBCLKCTRL_CRC      = (1 << 13),
	SYSCON_SYSAHBCLKCTRL_UART0    = (1 << 14),
	SYSCON_SYSAHBCLKCTRL_UART1    = (1 << 15),
	SYSCON_SYSAHBCLKCTRL_UART2    = (1 << 16),
	SYSCON_SYSAHBCLKCTRL_WWDT     = (1 << 17),
	SYSCON_SYSAHBCLKCTRL_IOCON    = (1 << 18),
	SYSCON_SYSAHBCLKCTRL_ACMP     = (1 << 19)
};

#endif
