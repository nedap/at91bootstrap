/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "common.h"
#include "hardware.h"
#include "arch/at91_ccfg.h"
#include "arch/at91_rstc.h"
#include "arch/at91_pmc/pmc.h"
#include "arch/at91_smc.h"
#include "arch/at91_pio.h"
#include "arch/at91_ddrsdrc.h"
#include "gpio.h"
#include "pmc.h"
#include "usart.h"
#include "debug.h"
#include "ddramc.h"
#include "timer.h"
#include "watchdog.h"
#include "string.h"
#include "nedap9g45.h"
#include "arch/at91_spi.h"

static void at91_dbgu_hw_init(void)
{
	/* Configure DBGU pin */
	const struct pio_desc dbgu_pins[] = {
		{"RXD", AT91C_PIN_PB(12), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"TXD", AT91C_PIN_PB(13), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	/* Configure the dbgu pins */
	pmc_enable_periph_clock(AT91C_ID_PIOB, PMC_PERIPH_CLK_DIVIDER_NA);
	pio_configure(dbgu_pins);
}

static void initialize_dbgu(void)
{
	at91_dbgu_hw_init();
	usart_init(BAUDRATE(MASTER_CLOCK, 115200));
}

static void internal_pullup(void)
{
	/* Configure PD17 with internal pullup */
	const struct pio_desc pd17_pin[] = {
		{"PD17", AT91C_PIN_PD(17), 0, PIO_PULLUP, PIO_INPUT},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};
	pmc_enable_periph_clock(AT91C_ID_PIOD_E, PMC_PERIPH_CLK_DIVIDER_NA);
	pio_configure(pd17_pin);
}

#ifdef CONFIG_DDR2
static void ddramc_reg_config(struct ddramc_register *ddramc_config)
{
	ddramc_config->mdr = (AT91C_DDRC2_DBW_16_BITS
			| AT91C_DDRC2_MD_DDR2_SDRAM);

	ddramc_config->cr = (AT91C_DDRC2_NC_DDR10_SDR9	/* 10 column bits */
			| AT91C_DDRC2_NR_14		/* 14 row bits(8K)*/
			| AT91C_DDRC2_CAS_3		/* CAS Latency 3 */
			| AT91C_DDRC2_EBISHARE		/* DQM is shared with other controller */
			| AT91C_DDRC2_DISABLE_RESET_DLL);

	ddramc_config->rtr = 0x24B;

	ddramc_config->t0pr = (AT91C_DDRC2_TRAS_(6)	/* 6 * 7.5 = 45 ns */
			| AT91C_DDRC2_TRCD_(2)		/* 2 * 7.5 = 22.5 ns */
			| AT91C_DDRC2_TWR_(2)		/* 2 * 7.5 = 15 ns */
			| AT91C_DDRC2_TRC_(8)		/* 8 * 7.5 = 75 ns */
			| AT91C_DDRC2_TRP_(2)		/* 2 * 7.5 = 22.5 ns */
			| AT91C_DDRC2_TRRD_(1)		/* 1 * 7.5 = 7.5 ns */
			| AT91C_DDRC2_TWTR_(1)		/* 1 clock cycle */
			| AT91C_DDRC2_TMRD_(2));	/* 2 clock cycles */

	ddramc_config->t1pr = (AT91C_DDRC2_TXP_(2)	/* 2 * 7.5 = 15 ns */
			| AT91C_DDRC2_TXSRD_(200)	/* 200 clock cycles */
			| AT91C_DDRC2_TXSNR_(16)	/* 16 * 7.5 = 120 ns */
			| AT91C_DDRC2_TRFC_(14));	/* 14 * 7.5 = 142 ns */

	ddramc_config->t2pr = (AT91C_DDRC2_TRTP_(1)	/* 1 * 7.5 = 7.5 ns */
			| AT91C_DDRC2_TRPA_(0)		/* 0 * 7.5 = 0 ns */
			| AT91C_DDRC2_TXARDS_(7)	/* 7 clock cycles */
			| AT91C_DDRC2_TXARD_(2));	/* 2 clock cycles */
}

static void ddramc_init(void)
{
	unsigned long csa;
	struct ddramc_register  ddramc_reg;

	ddramc_reg_config(&ddramc_reg);

	/* ENABLE DDR2 clock */ 
	pmc_enable_system_clock(AT91C_PMC_DDR);

	/* Chip select 1 is for DDR2/SDRAM */
	csa = readl(AT91C_BASE_CCFG + CCFG_EBICSA);
	csa |= AT91C_EBI_CS1A_SDRAMC;
	csa &= ~AT91C_VDDIOM_SEL_33V;
	writel(csa, AT91C_BASE_CCFG + CCFG_EBICSA);

	/* DDRAM2 Controller initialize */
	ddram_initialize(AT91C_BASE_DDRSDRC, AT91C_DDRAM_BASE_ADDR, &ddramc_reg);
}
#endif /* #ifdef CONFIG_DDR2 */

#if defined(CONFIG_NANDFLASH_RECOVERY) || defined(CONFIG_DATAFLASH_RECOVERY)
static void recovery_buttons_hw_init(void)
{
	/* Configure recovery button PINs */
	const struct pio_desc recovery_button_pins[] = {
		{"RECOVERY_BUTTON", CONFIG_SYS_RECOVERY_BUTTON_PIN, 0, PIO_PULLUP, PIO_INPUT},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pmc_enable_periph_clock(AT91C_ID_PIOB, PMC_PERIPH_CLK_DIVIDER_NA);
	pio_configure(recovery_button_pins);
}
#endif /* #if defined(CONFIG_NANDFLASH_RECOVERY) || defined(CONFIG_DATAFLASH_RECOVERY) */

/*
 * Enable ARM instruction cache for faster boot
 */
static void nedap_enable_icache(void)
{
	unsigned int reg;

	asm volatile ("mrc p15, 0, %0, c1, c0, 0" : "=r" (reg));
	reg |= (1 << 12);	/* I-cache enable */
	asm volatile ("mcr p15, 0, %0, c1, c0, 0" : : "r" (reg));
}

/*
 * Cycle USB PLL (UTMI) to ensure known state after power-on.
 */
static void nedap_usb_fix(void)
{
	unsigned int lock = 1000000;

	writel(readl(AT91C_BASE_PMC + PMC_UCKR) | AT91C_CKGR_UPLLEN_ENABLED,
		AT91C_BASE_PMC + PMC_UCKR);
	while ((lock--) && !(readl(AT91C_BASE_PMC + PMC_SR) & AT91C_PMC_LOCKU))
		;
	writel(readl(AT91C_BASE_PMC + PMC_UCKR) & ~AT91C_CKGR_UPLLEN_ENABLED,
		AT91C_BASE_PMC + PMC_UCKR);
}

/*
 * Initialize Ethernet switch via SPI1 CS2.
 */
static void nedap_switch_init(void)
{
	#define AT91C_SPI_CLK		5000000
	#define CLOCK			AT91C_SPI_SCBR(MASTER_CLOCK / AT91C_SPI_CLK)
	#define SPI_CSR_SETTINGS	(AT91C_SPI_DLYBCT(0) | AT91C_SPI_DLYBS(0) | CLOCK | AT91C_SPI_CPOL)
	#define SPI_MR_SETTINGS		(AT91C_SPI_DLYBCS(0) | AT91C_SPI_PCS(0x0B) | AT91C_SPI_MODFDIS | AT91C_SPI_MSTR)
	#define NCHARS 3
	unsigned int tx[1] = { 2 | (198 << 8) | (10 << 16) };
	unsigned int rx[1];

	const struct pio_desc switch_pins[] = {
		{"MISO",  AT91C_PIN_PB(14), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"MOSI",  AT91C_PIN_PB(15), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SPCK",  AT91C_PIN_PB(16), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"NPCS2", AT91C_PIN_PD(18), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{(char *) 0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pmc_enable_periph_clock(AT91C_ID_PIOB, PMC_PERIPH_CLK_DIVIDER_NA);
	pmc_enable_periph_clock(AT91C_ID_PIOD_E, PMC_PERIPH_CLK_DIVIDER_NA);
	pio_configure(switch_pins);
	pmc_enable_periph_clock(AT91C_ID_SPI1, PMC_PERIPH_CLK_DIVIDER_NA);


	writel(AT91C_SPI_SWRST | AT91C_SPI_SPIDIS, AT91C_BASE_SPI1 + SPI_CR);
	writel(AT91C_SPI_SWRST | AT91C_SPI_SPIDIS, AT91C_BASE_SPI1 + SPI_CR);

	writel(SPI_CSR_SETTINGS, AT91C_BASE_SPI1 + SPI_CSR(2));
	writel(SPI_MR_SETTINGS, AT91C_BASE_SPI1 + SPI_MR);

	writel(AT91C_SPI_SPIEN, AT91C_BASE_SPI1 + SPI_CR);

	writel((unsigned int)rx, AT91C_BASE_SPI1 + SPI_RPR);
	writel((unsigned int)tx, AT91C_BASE_SPI1 + SPI_TPR);

	writel(NCHARS, AT91C_BASE_SPI1 + SPI_RCR);
	writel(NCHARS, AT91C_BASE_SPI1 + SPI_TCR);

	writel(AT91C_SPI_RXTEN | AT91C_SPI_TXTEN, AT91C_BASE_SPI1 + SPI_PTCR);

	while (!(readl(AT91C_BASE_SPI1 + SPI_SR) & AT91C_SPI_RXBUFF)) ;
}

#ifdef CONFIG_HW_INIT
void hw_init(void)
{
	/* Disable watchdog */
	at91_disable_wdt();

	/*
	 * At this stage the main oscillator
	 * is supposed to be enabled PCK = MCK = MOSC
	 */
	pmc_init_pll(0);

	/* Configure PLLA = MOSC * (PLL_MULA + 1) / PLL_DIVA */
	pmc_cfg_plla(PLLA_SETTINGS);

	/* Switch PCK/MCK on Main clock output */
	pmc_mck_cfg_set(0, BOARD_PRESCALER_MAIN_CLOCK,
			AT91C_PMC_PLLADIV2 | AT91C_PMC_MDIV | AT91C_PMC_CSS);

	/* Switch PCK/MCK on PLLA output */
	pmc_mck_cfg_set(0, BOARD_PRESCALER_PLLA,
			AT91C_PMC_PLLADIV2 | AT91C_PMC_MDIV | AT91C_PMC_CSS);

	/* Enable External Reset */
	writel(AT91C_RSTC_KEY_UNLOCK | AT91C_RSTC_URSTEN, AT91C_BASE_RSTC + RSTC_RMR);

	/* Init timer */
	timer_init();

	/* Initialize dbgu */
	initialize_dbgu();

	/* Enable internal pull-up on PD17 to avoid floating input */
	internal_pullup();

#ifdef CONFIG_DDR2
	/* Initialize DDRAM Controller */
	ddramc_init();
#endif

#if defined(CONFIG_NANDFLASH_RECOVERY) || defined(CONFIG_DATAFLASH_RECOVERY)
	/* Init the recovery buttons pins */
	recovery_buttons_hw_init();
#endif
#ifdef CONFIG_USER_HW_INIT
	nedap_enable_icache();
	nedap_usb_fix();
	nedap_switch_init();
#endif
}
#endif /* #ifdef CONFIG_HW_INIT */

#ifdef CONFIG_DATAFLASH
void at91_spi0_hw_init(void)
{
	/* Configure spi0 PINs */
	const struct pio_desc spi0_pins[] = {
		{"MISO", 	AT91C_PIN_PB(0),	0, PIO_DEFAULT, PIO_PERIPH_A},
		{"MOSI", 	AT91C_PIN_PB(1),	0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SPCK", 	AT91C_PIN_PB(2),	0, PIO_DEFAULT, PIO_PERIPH_A},
		{"NPCS",	CONFIG_SYS_SPI_PCS,	1, PIO_DEFAULT, PIO_OUTPUT},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pmc_enable_periph_clock(AT91C_ID_PIOB, PMC_PERIPH_CLK_DIVIDER_NA);
	pio_configure(spi0_pins);

	pmc_enable_periph_clock(AT91C_ID_SPI0, PMC_PERIPH_CLK_DIVIDER_NA);
}
#endif /* #ifdef CONFIG_DATAFLASH */

#ifdef CONFIG_SDCARD
#ifdef CONFIG_OF_LIBFDT
void at91_board_set_dtb_name(char *of_name)
{
	strcpy(of_name, "nedap9g45.dtb");
}
#endif

void at91_mci0_hw_init(void)
{
	const struct pio_desc mci_pins[] = {
		{"MCCK", 	AT91C_PIN_PA(0), 0, PIO_PULLUP, PIO_PERIPH_A},
		{"MCCDA",	AT91C_PIN_PA(1), 0, PIO_PULLUP, PIO_PERIPH_A},
		{"MCDA0",	AT91C_PIN_PA(2), 0, PIO_PULLUP, PIO_PERIPH_A},
		{"MCDA1",	AT91C_PIN_PA(3), 0, PIO_PULLUP, PIO_PERIPH_A},
		{"MCDA2",	AT91C_PIN_PA(4), 0, PIO_PULLUP, PIO_PERIPH_A},
		{"MCDA3",	AT91C_PIN_PA(5), 0, PIO_PULLUP, PIO_PERIPH_A},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	/* Configure the PIO controller */
	pmc_enable_periph_clock(AT91C_ID_PIOA, PMC_PERIPH_CLK_DIVIDER_NA);
	pio_configure(mci_pins);

	/* Enable the clock */
	pmc_enable_periph_clock(AT91C_ID_MCI0, PMC_PERIPH_CLK_DIVIDER_NA);
}
#endif /* #ifdef CONFIG_SDCARD */

#ifdef CONFIG_NANDFLASH
void nandflash_hw_init(void)
{
	unsigned int reg;

	/* Configure PIOs */
	const struct pio_desc nand_pins[] = {
		{"NANDCS",	CONFIG_SYS_NAND_ENABLE_PIN,	1, PIO_PULLUP, PIO_OUTPUT},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	/* Configure the PIO controll */
	pmc_enable_periph_clock(AT91C_ID_PIOC, PMC_PERIPH_CLK_DIVIDER_NA);
	pio_configure(nand_pins);

	/* Setup Smart Media, first enable the address range of CS3
	 * in HMATRIX user interface
	* EBI IO in 1.8V mode */
	reg = readl(AT91C_BASE_CCFG + CCFG_EBICSA);
	reg |= AT91C_EBI_CS3A_SM;
	reg &= ~AT91C_VDDIOM_SEL_33V;
	writel(reg, AT91C_BASE_CCFG + CCFG_EBICSA);

	/* Configure SMC CS3 */
	writel((AT91C_SMC_NWESETUP_(2)
		| AT91C_SMC_NCS_WRSETUP_(0)
		| AT91C_SMC_NRDSETUP_(2)
		| AT91C_SMC_NCS_RDSETUP_(0)),
		AT91C_BASE_SMC + SMC_SETUP3);

	writel((AT91C_SMC_NWEPULSE_(4)
		| AT91C_SMC_NCS_WRPULSE_(4)
		| AT91C_SMC_NRDPULSE_(4)
		| AT91C_SMC_NCS_RDPULSE_(4)),
		AT91C_BASE_SMC + SMC_PULSE3);

	writel((AT91C_SMC_NWECYCLE_(7)
		|  AT91C_SMC_NRDCYCLE_(7)),
		AT91C_BASE_SMC + SMC_CYCLE3);

	writel((AT91C_SMC_READMODE
		| AT91C_SMC_WRITEMODE
		| AT91C_SMC_NWAITM_NWAIT_DISABLE
		| AT91C_SMC_DBW_WIDTH_BITS_8
		| AT91_SMC_TDF_(3)),
		AT91C_BASE_SMC + SMC_CTRL3);
}
#endif /* #ifdef CONFIG_NANDFLASH */
