/*
 * si5351.c - Si5351 library for avr-gcc
 *
 * Copyright (C) 2014 Jason Milldrum <milldrum@gmail.com>
 *
 * Some tuning algorithms derived from clk-si5351.c in the Linux kernel.
 * Sebastian Hesselbarth <sebastian.hesselbarth@gmail.com>
 * Rabeeh Khoury <rabeeh@solid-run.com>
 *
 * rational_best_approximation() derived from lib/rational.c in
 * the Linux kernel.
 * Copyright (C) 2009 emlix GmbH, Oskar Schirmer <oskar@scara.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <util/twi.h>
#include <avr/eeprom.h>

#include "si5351.h"
#include "i2c.h"

uint32_t EEMEM ee_ref_correction = 0;

int32_t ref_correction = 0;
uint32_t plla_freq = 0;
uint32_t pllb_freq = 0;

struct Si5351Status dev_status;
struct Si5351IntStatus dev_int_status;

/******************************/
/* Suggested public functions */
/******************************/

/*
 * si5351_init(void)
 *
 * Call this to initialize I2C communications and get the
 * Si5351 ready for use.
 */
void si5351_init(void)
{
	i2c_init();

	/* Set crystal load capacitance */
	si5351_write(SI5351_CRYSTAL_LOAD, SI5351_CRYSTAL_LOAD_8PF);

	/* Get the correction factor from EEPROM */
	ref_correction = eeprom_read_dword(&ee_ref_correction);
}

static void
si5351_set_multisynth(struct Si5351RegSet *msreg, int clock)
{
	int addr;
	char regbuf[8];
    
	/* Registers 42-43 */
	regbuf[0] = (uint8_t) (msreg->p3 >> 8);
	regbuf[1] = (uint8_t) msreg->p3;

	/* Register 44 */
	/* TODO: add code for output divider */
	regbuf[2] = (uint8_t) ((msreg->p1 >> 16) & 0x03);

	/* Registers 45-46 */
	regbuf[3] = (uint8_t) (msreg->p1 >> 8);
	regbuf[4] = (uint8_t) msreg->p1;

	/* Register 47 */
	regbuf[5] = (uint8_t) ((msreg->p3 >> 12) & 0xF0) |
	    (uint8_t) ((msreg->p2 >> 16) & 0x0F);

	/* Registers 48-49 */
	regbuf[6] = (uint8_t) (msreg->p2 >> 8);
	regbuf[7] = (uint8_t) msreg->p2;
	
	/* XXX: I suspect there's a better way than this */
	switch (clock) {
	case SI5351_CLK0:
		addr = SI5351_CLK0_PARAMETERS;
		break;
	case SI5351_CLK1:
		addr = SI5351_CLK1_PARAMETERS;
		break;
	case SI5351_CLK2:
		addr = SI5351_CLK2_PARAMETERS;
		break;
	case SI5351_CLK3:
		addr = SI5351_CLK3_PARAMETERS;
		break;
	case SI5351_CLK4:
		addr = SI5351_CLK4_PARAMETERS;
		break;
	case SI5351_CLK5:
		addr = SI5351_CLK5_PARAMETERS;
		break;
	case SI5351_CLK6:
		addr = SI5351_CLK6_PARAMETERS;
		break;
	case SI5351_CLK7:
		addr = SI5351_CLK7_PARAMETERS;
		break;
	}

	si5351_write_bulk(addr, 8, regbuf);
	si5351_set_ms_source(clk, target_pll);
}

/*
 * si5351_set_freq(uint32_t freq, uint32_t pll_freq, enum si5351_clock output)
 *
 * Sets the clock frequency of the specified CLK output
 *
 * freq - Output frequency in Hz
 * pll_freq - Frequency of the PLL driving the Multisynth
 *   Use a 0 to have the function choose a PLL frequency
 * clk - Clock output
 *   (use the si5351_clock enum)
 */
uint32_t
si5351_set_freq(uint32_t freq, uint32_t pll_freq, enum si5351_clock clk)
{
	struct Si5351RegSet msreg;
	char regbuf[8];
	int pll_prog, addr;
	int program_pll = 0;
	
	/*
	 * if pll_freq == 0, let multisynth_calc() pick the PLL frequency
	 */
	 if (pll_freq == 0) {
		pll_freq = multisynth_calc(freq, &msreg);
	 } else {
		/* TODO: bounds checking */
		multisynth_recalc(freq, pll_freq, &msreg);
	 }
	 
	 /*
	  * select PLL
	  */
	switch (clock) {
	case SI5351_CLK0:
		pll_prog = SI5351_PLLA;
		program_pll = (plla_freq != pll_freq);
		break;
	/* CLK1 and CLK2 share PLLB; first programmed sets PLL freq */
	case SI5351_CLK1:
	case SI5351_CLK2:
		pll_prog = SI5351_PLLB;
		if (pllb_freq != 0) {
			multisynth_recalc(freq, pllb_freq, &msreg);
		} else {
			program_pll = 1;
		}
		break;
	default:
		/* XXX: need to handle other clock outputs */
		break;
	}

	if (program_pll)
		si5351_set_pll(pll_freq, pll_prog);

	si5351_set_multisynth(&msreg, clock);
	return (pll_freq);
}

/*
 * si5351_set_pll(uint32_t pll_freq, enum si5351_pll target_pll)
 *
 * Set the specified PLL to a specific oscillation frequency
 *
 * pll_freq - Desired PLL frequency
 * target_pll - Which PLL to set
 *     (use the si5351_pll enum)
 */
void
si5351_set_pll(uint32_t pll_freq, enum si5351_pll pll_prog)
{
	struct Si5351RegSet pllreg;
	int addr;
	char regbuf[8];
    
	/* keep track of PLL programming */
	switch (pll_prog) {
	case SI5351_PLLA:
		plla_freq = pll_freq;
		addr = SI5351_PLLA_PARAMETERS;
		break;
	case SI5351_PLLB:
		pllb_freq = pll_freq;
		addr = SI5351_PLLB_PARAMETERS;
		break;
	}
    
	pll_calc(pll_freq, &pllreg, ref_correction);
	
	/* Registers 26-27 */
	regbuf[0] = (uint8_t) (pllreg.p3 >> 8);
	regbuf[1] = (uint8_t) pllreg.p3;

	/* Register 28 */
	regbuf[2] = (uint8_t) ((pllreg.p1 >> 16) & 0x03);

	/* Registers 29-30 */
	regbuf[3] = (uint8_t) (pllreg.p1 >> 8);
	regbuf[4] = (uint8_t) pllreg.p1;

	/* Register 31 */
	regbuf[5] = ((uint8_t) ((pllreg.p3 >> 12) & 0xF0)) |
	    ((uint8_t) ((pllreg.p2 >> 16) & 0x0F));

	/* Registers 32-33 */
	regbuf[6] = (uint8_t) (pllreg.p2 >> 8);
	regbuf[7] = (uint8_t) pllreg.p2;

	/* Write the parameters */
	si5351_write_bulk(addr, 8, regbuf);
}

/*
 * si5351_clock_enable(enum si5351_clock clk, uint8_t enable)
 *
 * Enable or disable a chosen clock
 * clk - Clock output
 *   (use the si5351_clock enum)
 * enable - Set to 1 to enable, 0 to disable
 */
void si5351_clock_enable(enum si5351_clock clk, uint8_t enable)
{
	uint8_t reg_val;

	if(si5351_read(SI5351_OUTPUT_ENABLE_CTRL, &reg_val) != 0) {
		return;
	}

	if (enable) {
		reg_val &= ~(1<<(uint8_t)clk);
	} else {
		reg_val |= (1<<(uint8_t)clk);
	}

	si5351_write(SI5351_OUTPUT_ENABLE_CTRL, reg_val);
}

/*
 * si5351_drive_strength(enum si5351_clock clk, enum si5351_drive drive)
 *
 * Sets the drive strength of the specified clock output
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * drive - Desired drive level
 *   (use the si5351_drive enum)
 */
void si5351_drive_strength(enum si5351_clock clk, enum si5351_drive drive)
{
	uint8_t reg_val;
	const uint8_t mask = 0x03;

	if(si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk, &reg_val) != 0) {
		return;
	}

	switch(drive) {
	case SI5351_DRIVE_2MA:
		reg_val &= ~(mask);
		reg_val |= 0x00;
		break;
	case SI5351_DRIVE_4MA:
		reg_val &= ~(mask);
		reg_val |= 0x01;
		break;
	case SI5351_DRIVE_6MA:
		reg_val &= ~(mask);
		reg_val |= 0x02;
		break;
	case SI5351_DRIVE_8MA:
		reg_val &= ~(mask);
		reg_val |= 0x03;
		break;
	default:
		break;
	}

	si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

/*
 * si5351_update_status(void)
 *
 * Call this to update the status structs, then access them
 * via the dev_status and dev_int_status global variables.
 *
 * See the header file for the struct definitions. These
 * correspond to the flag names for registers 0 and 1 in
 * the Si5351 datasheet.
 */
void si5351_update_status(void)
{
	si5351_update_sys_status(&dev_status);
	si5351_update_int_status(&dev_int_status);
}

/*
 * si5351_set_correction(int32_t corr)
 *
 * Use this to set the oscillator correction factor to
 * EEPROM. This value is a signed 32-bit integer of the
 * parts-per-10 million value that the actual oscillation
 * frequency deviates from the specified frequency.
 *
 * The frequency calibration is done as a one-time procedure.
 * Any desired test frequency within the normal range of the
 * Si5351 should be set, then the actual output frequency
 * should be measured as accurately as possible. The
 * difference between the measured and specified frequencies
 * should be calculated in Hertz, then multiplied by 10 in
 * order to get the parts-per-10 million value.
 *
 * Since the Si5351 itself has an intrinsic 0 PPM error, this
 * correction factor is good across the entire tuning range of
 * the Si5351. Once this calibration is done accurately, it
 * should not have to be done again for the same Si5351 and
 * crystal. The library will read the correction factor from
 * EEPROM during initialization for use by the tuning
 * algorithms.
 */
void si5351_set_correction(int32_t corr)
{
	eeprom_write_dword(&ee_ref_correction, corr);
	ref_correction = corr;
}

/*
 * si5351_get_correction(void)
 *
 * Returns the oscillator correction factor stored
 * in EEPROM.
 */
int32_t si5351_get_correction(void)
{
	return eeprom_read_dword(&ee_ref_correction);
}

/*******************************/
/* Suggested private functions */
/*******************************/

/*
 * Calculate best rational approximation for a given fraction
 * taking into account restricted register size, e.g. to find
 * appropriate values for a pll with 5 bit denominator and
 * 8 bit numerator register fields, trying to set up with a
 * frequency ratio of 3.1415, one would say:
 *
 * rational_best_approximation(31415, 10000,
 *              (1 << 8) - 1, (1 << 5) - 1, &n, &d);
 *
 * you may look at given_numerator as a fixed point number,
 * with the fractional part size described in given_denominator.
 *
 * for theoretical background, see:
 * http://en.wikipedia.org/wiki/Continued_fraction
 */

void rational_best_approximation(
        unsigned long given_numerator, unsigned long given_denominator,
        unsigned long max_numerator, unsigned long max_denominator,
        unsigned long *best_numerator, unsigned long *best_denominator)
{
	unsigned long n, d, n0, d0, n1, d1;
	n = given_numerator;
	d = given_denominator;
	n0 = d1 = 0;
	n1 = d0 = 1;
	for (;;) {
		unsigned long t, a;
		if ((n1 > max_numerator) || (d1 > max_denominator)) {
			n1 = n0;
			d1 = d0;
			break;
		}
		if (d == 0)
			break;
		t = d;
		a = n / d;
		d = n % d;
		n = t;
		t = n0 + a * n1;
		n0 = n1;
		n1 = t;
		t = d0 + a * d1;
		d0 = d1;
		d1 = t;
	}
	*best_numerator = n1;
	*best_denominator = d1;
}

static uint32_t
pll_calc(uint32_t freq, struct Si5351RegSet *reg, int32_t correction)
{
	uint32_t ref_freq = SI5351_XTAL_FREQ;
	uint32_t rfrac, denom, a, b, c, p1, p2, p3;

	/* Factor calibration value into nominal crystal frequency */
	/* Measured in parts-per-ten million */
	ref_freq += (int32_t) ((((((int64_t)correction) << 31) /
	    10000000LL) * ref_freq) >> 31);

	/* PLL bounds checking */
	if (freq < SI5351_PLL_VCO_MIN) {
		freq = SI5351_PLL_VCO_MIN;
	} else if (freq > SI5351_PLL_VCO_MAX) {
		freq = SI5351_PLL_VCO_MAX;
	}

	/* Determine integer part of feedback equation */
	a = freq / ref_freq;

	if (a < SI5351_PLL_A_MIN) {
		freq = ref_freq * SI5351_PLL_A_MIN;
	} else if (a > SI5351_PLL_A_MAX) {
		freq = ref_freq * SI5351_PLL_A_MAX;
	}

	/* find best approximation for b/c = fVCO mod fIN */
	b = 0;
	c = 1;
	denom = 1000000L;
	rfrac = (((uint64_t)(freq % ref_freq)) * denom) / ref_freq;
	if (rfrac)
		rational_best_approximation(rfrac, denom,
		    SI5351_PLL_B_MAX, SI5351_PLL_C_MAX, &b, &c);

	/* calculate parameters */
	p3  = c;
	p2  = (128 * b) % c;
	p1  = 128 * a;
	p1 += (128 * b / c);
	p1 -= 512;

	/* recalculate rate by fIN * (a + b/c) */
	freq = (uint32_t) ((((uint64_t) ref_freq) * ((uint64_t) b)) / c) +
	    ref_freq * a;

	reg->p1 = p1;
	reg->p2 = p2;
	reg->p3 = p3;

	return freq;
}

static uint32_t
multisynth_calc(uint32_t freq, struct Si5351RegSet *reg)
{
	uint32_t pll_freq;
	uint32_t a, b, c, p1, p2, p3;
	uint8_t divby4;

	/* Multisynth bounds checking */
	if (freq > SI5351_MULTISYNTH_MAX_FREQ) {
		freq = SI5351_MULTISYNTH_MAX_FREQ;
	} else if (freq < SI5351_MULTISYNTH_MIN_FREQ) {
		freq = SI5351_MULTISYNTH_MIN_FREQ;
	}

	divby4 = freq > SI5351_MULTISYNTH_DIVBY4_FREQ;

	/* Find largest integer divider for max */
	/* VCO frequency and given target frequency */
	a = divby4 ? 4 : (uint32_t) (((uint64_t) SI5351_PLL_VCO_MAX) / freq);

	b = 0;
	c = 1;
	pll_freq = a * freq;

	/* Recalculate output frequency by fOUT = fIN / (a + b/c) */
	freq = (uint32_t) ((((uint64_t)pll_freq) * c) / (a * c + b));
    
	/* Calculate parameters */
	if (divby4) {
		p3 = 1;
		p2 = 0;
		p1 = 0;
	} else {
		p3  = c;
		p2  = (128 * b) % c;
		p1  = 128 * a;
		p1 += (128 * b / c);
		p1 -= 512;
	}

	reg->p1 = p1;
	reg->p2 = p2;
	reg->p3 = p3;

	return pll_freq;
}

static uint32_t
multisynth_recalc(uint32_t freq, uint32_t pll_freq, struct Si5351RegSet *reg)
{
	uint32_t rfrac, denom, a, b, c, p1, p2, p3;
	uint8_t divby4;

	/* Multisynth bounds checking */
	if (freq > SI5351_MULTISYNTH_MAX_FREQ) {
		freq = SI5351_MULTISYNTH_MAX_FREQ;
	} else if (freq < SI5351_MULTISYNTH_MIN_FREQ) {
		freq = SI5351_MULTISYNTH_MIN_FREQ;
	}

	divby4 = freq > SI5351_MULTISYNTH_DIVBY4_FREQ;

	/* Determine integer part of feedback equation */
	a = pll_freq / freq;

	/* TODO: not sure this is correct */
	if (a < SI5351_MULTISYNTH_A_MIN) {
		freq = pll_freq / SI5351_MULTISYNTH_A_MIN;
	} else if (a > SI5351_MULTISYNTH_A_MAX) {
		freq = pll_freq / SI5351_MULTISYNTH_A_MAX;
	}

	/* find best approximation for b/c */
	b = 0;
	c = 1;
	denom = 1000000L;
	rfrac = (uint32_t) ((((uint64_t) (pll_freq % freq)) * denom) / freq);
	if (rfrac)
		rational_best_approximation(rfrac, denom,
		    SI5351_MULTISYNTH_B_MAX, SI5351_MULTISYNTH_C_MAX, &b, &c);

	/* Recalculate output frequency by fOUT = fIN / (a + b/c) */
	freq = (uint32_t) ((((uint64_t)pll_freq) * c) / (a * c + b));

	/* Calculate parameters */
	if (divby4) {
		p3 = 1;
		p2 = 0;
		p1 = 0;
	} else {
		p3  = c;
		p2  = (128 * b) % c;
		p1  = 128 * a;
		p1 += (128 * b / c);
		p1 -= 512;
	}

	reg->p1 = p1;
	reg->p2 = p2;
	reg->p3 = p3;

	return freq;
}

uint8_t si5351_write_bulk(uint8_t addr, uint8_t bytes, uint8_t *data)
{
	int i;

	i2c_start();
	if(i2c_status() != TW_START)
	{
		i2c_stop();
		return 1;
	}
	i2c_write(SI5351_BUS_BASE_ADDR);
	if(i2c_status() != TW_MT_SLA_ACK)
	{
		i2c_stop();
		return 1;
	}
	i2c_write(addr);
	if(i2c_status() != TW_MT_DATA_ACK)
	{
		i2c_stop();
		return 1;
	}

	for(i = 0; i < bytes; i++)
	{
		i2c_write(data[i]);
		if(i2c_status() != TW_MT_DATA_ACK)
		{
			i2c_stop();
			return 1;
		}
	}

	i2c_stop();
	return 0;
}

uint8_t si5351_write(uint8_t addr, uint8_t data)
{
	i2c_start();
	if(i2c_status() != TW_START)
	{
		i2c_stop();
		return 1;
	}
	i2c_write(SI5351_BUS_BASE_ADDR);
	if(i2c_status() != TW_MT_SLA_ACK)
	{
		i2c_stop();
		return 1;
	}
	i2c_write(addr);
	if(i2c_status() != TW_MT_DATA_ACK)
	{
		i2c_stop();
		return 1;
	}
	i2c_write(data);
	if(i2c_status() != TW_MT_DATA_ACK)
	{
		i2c_stop();
		return 1;
	}
	i2c_stop();
	return 0;
}

uint8_t si5351_read(uint8_t addr, uint8_t *data)
{
	i2c_start();
	if(i2c_status() != TW_START)
	{
		i2c_stop();
		return 1;
	}
	i2c_write(SI5351_BUS_BASE_ADDR);
	if(i2c_status() != TW_MT_SLA_ACK)
	{
		i2c_stop();
		return 1;
	}
	i2c_write(addr);
	if(i2c_status() != TW_MT_DATA_ACK)
	{
		i2c_stop();
		return 1;
	}

	i2c_start();
	if(i2c_status() != TW_REP_START)
	{
		i2c_stop();
		return 1;
	}
	i2c_write(SI5351_BUS_BASE_ADDR | TW_READ);
	if(i2c_status() != TW_MR_SLA_ACK)
	{
		i2c_stop();
		return 1;
	}
	*data = i2c_read_nack();
	if(i2c_status() != TW_MR_DATA_NACK)
	{
		i2c_stop();
		return 1;
	}
	i2c_stop();
	return 0;
}


void si5351_update_sys_status(struct Si5351Status *status)
{
	uint8_t reg_val = 0;

	if(si5351_read(SI5351_DEVICE_STATUS, &reg_val) != 0)
	{
		return;
	}

	/* Parse the register */
	status->SYS_INIT = (reg_val >> 7) & 0x01;
	status->LOL_B = (reg_val >> 6) & 0x01;
	status->LOL_A = (reg_val >> 5) & 0x01;
	status->LOS = (reg_val >> 4) & 0x01;
	status->REVID = reg_val & 0x03;
}

void si5351_update_int_status(struct Si5351IntStatus *int_status)
{
	uint8_t reg_val = 0;

	if(si5351_read(SI5351_DEVICE_STATUS, &reg_val) != 0)
	{
		return;
	}

	/* Parse the register */
	int_status->SYS_INIT_STKY = (reg_val >> 7) & 0x01;
	int_status->LOL_B_STKY = (reg_val >> 6) & 0x01;
	int_status->LOL_A_STKY = (reg_val >> 5) & 0x01;
	int_status->LOS_STKY = (reg_val >> 4) & 0x01;
}

void si5351_set_ms_source(enum si5351_clock clk, enum si5351_pll pll)
{
	uint8_t reg_val = 0x0c;
	uint8_t reg_val2;

	if(si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk, &reg_val2) != 0)
	{
		return;
	}

	if(pll == SI5351_PLLA)
	{
		reg_val &= ~(SI5351_CLK_PLL_SELECT);
	}
	else if(pll == SI5351_PLLB)
	{
		reg_val |= SI5351_CLK_PLL_SELECT;
	}
	si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}
