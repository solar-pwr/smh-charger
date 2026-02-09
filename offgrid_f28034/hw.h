//    Custom firmware for EASUN ISolar-SMH-II-7K solar inverter charger
//    Copyright (C) 2025-2026 Jakub Strnad
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef HW_H
#define HW_H

#define GLOBAL_Q 16

#include "DSP28x_Project.h"
#include "IQmathLib.h"

#include <stdint.h>

#define NOP __asm(" NOP")
#define DSP_CLK 60000000.0 // 60 Mhz

#define ADC_TBOOST AdcResult.ADCRESULT0
#define ADC_TMOSFET AdcResult.ADCRESULT1
#define ADC_TINV AdcResult.ADCRESULT2
#define ADC_FANS AdcResult.ADCRESULT3
#define ADC_VINVDC AdcResult.ADCRESULT4
#define ADC_VBAT AdcResult.ADCRESULT5
#define ADC_ICTH AdcResult.ADCRESULT6
#define ADC_VGRID AdcResult.ADCRESULT7
#define ADC_VINV AdcResult.ADCRESULT8
#define ADC_IINV AdcResult.ADCRESULT9
#define ADC_VBUS AdcResult.ADCRESULT10
#define ADC_IDCDC AdcResult.ADCRESULT11
#define ADC_ICTL AdcResult.ADCRESULT12
#define ADC_KEY AdcResult.ADCRESULT13
#define ADC_VPV AdcResult.ADCRESULT14
#define ADC_IPC AdcResult.ADCRESULT15

#define PERIOD 1200 // orig: 1562
#define FAN_PERIOD 2999

const _iq dcdcRatio = _IQ(7.5);
const _iq vInvFactor = _IQ((665.0 * 4 + 10.0) / 8.66 * 3.3 / 4096.0); // 0.2484 (orig: 2.4897 / 10)
const _iq iInvFactor = _IQ(12.5 * 66.7 / 15.0 * 3.3 / 4096.0); // 0.04478
const _iq vGridFactor = _IQ((665.0 * 4 + 10.0) / 8.66 * 3.3 / 4096.0); // 0.2484 (orig: 2.4897 / 10)
const _iq vBatFactor = _IQ((1000.0 * 4 + 15.0) / 100.0 * 3.3 / 4096.0); // 0.03235
const _iq vBusFactor = _IQ((1000.0 * 3 + 15.0) / 15.0 * 3.3 / 4096.0); // 0.1619
const _iq iDcdcFactor = _IQ(20.0 / 4.0 * (20.0 + 20.0) / 20.0 * 2 * 3.3 / 4096.0); // 0.01611 (orig: 1.5708 / 100)
const _iq vPvFactor = _IQ((1000.0 * 3 + 15.0) / 15.0 * 3.3 / 4096.0);
const _iq iPvFactor = _IQ(30.0 / 2.0 * 10.0 / 15.0 * 3.3 / 4096.0);
const _iq vInvDcFactor = _IQ(1.0 / 7.8 * 3.3 / 4096.0); // quite inaccurate in IQ16 but no problem
//const _iq ivComp = _IQ(0.00002 * 50.0 * PERIOD / 3); // output capacitor power factor compensation

const _iq iMaxDcdcShutdown = _IQ(0.5); // maximum current allowed when shutting down the DC/DC converter
//const int16_t iInvRawLimit = 500; // 22A - PWM will turn off if this is exceeded
const int16_t iInvRawLimit = 800; // 36A - PWM will turn off if this is exceeded
//const int16_t iInvRawLimit = 1005; // 45A - PWM will turn off if this is exceeded
const _iq iDcdcLimit = _IQ(20.0); // 20A

void delay(uint32_t cycles) {
	uint32_t start = CpuTimer0Regs.TIM.all;
	while (start - CpuTimer0Regs.TIM.all < cycles);
}

inline void smpsOn() {
	GpioDataRegs.GPBSET.bit.GPIO33 = 1;
}

inline void smpsOff() {
	GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;
}

inline void softStartOn() {
	GpioDataRegs.GPBSET.bit.GPIO39 = 1;
}

inline void softStartOff() {
	GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;
}

inline void dcdcOn() {
	GpioDataRegs.GPASET.bit.GPIO6 = 1;
}

inline void dcdcOff() {
	GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
}

inline void gridRelayOn() {
	GpioDataRegs.GPASET.bit.GPIO19 = 1;
}

inline void gridRelayOff() {
	GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
}

inline uint16_t gridRelayGpioSts() {
	return GpioDataRegs.GPADAT.bit.GPIO19;
}

inline void outRelayOn() {
	GpioDataRegs.GPASET.bit.GPIO12 = 1;
}

inline void outRelayOff() {
	GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;
}

inline uint16_t outRelayGpioSts() {
	return GpioDataRegs.GPADAT.bit.GPIO12;
}

inline uint16_t swOnSts() {
	return GpioDataRegs.GPADAT.bit.GPIO5;
}

// FANS
inline void fan1on() {
	GpioDataRegs.GPASET.bit.GPIO25 = 1;
}

inline void fan1off() {
	GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
}

inline void fan2on() {
	GpioDataRegs.GPBSET.bit.GPIO44 = 1;
}

inline void fan2off() {
	GpioDataRegs.GPBCLEAR.bit.GPIO44 = 1;
}

inline void fan3on() {
	GpioDataRegs.GPASET.bit.GPIO16 = 1;
}

inline void fan3off() {
	GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
}

inline void setFanPwm(uint16_t percent) {
	EPwm7Regs.CMPB = FAN_PERIOD - percent * (FAN_PERIOD / 100);
}

void gpioSetup() {
	EALLOW;

	smpsOn();
	fan1on();
	fan2on();
	fan3on();
	GpioDataRegs.GPASET.bit.GPIO15 = 1; // backlight ON

	// GPIO6 = SG3525 on
	// GPIO12 = output relay
	// GPIO13 = display data
	// GPIO15 = display backlight
	// GPIO16 = fan 3
	// GPIO19 = grid relays
	// GPIO20 = display clock
	// GPIO25 = fan 1
	// --
	// GPIO33 = SMPS on
	// GPIO34 = display chip select
	// GPIO39 = softstart on
	// GPIO44 = fan 2
	GpioCtrlRegs.GPADIR.all |= 0x0219b040;
	GpioCtrlRegs.GPBDIR.all |= 0x00001086;
}

void comSetBaud(uint32_t baud) {
	uint32_t tmp;
	uint16_t swnrst;

	swnrst = LinaRegs.SCIGCR1.bit.SWnRST;
	LinaRegs.SCIGCR1.bit.SWnRST = 0;
	tmp = DSP_CLK / 2;
	tmp = tmp / baud - 16;
	LinaRegs.BRSR.bit.SCI_LIN_PSH = (tmp >> 20) & 0xff;
	LinaRegs.BRSR.bit.SCI_LIN_PSL = (tmp >> 4) & 0xffff;
	LinaRegs.BRSR.bit.M = tmp & 0xf;
	LinaRegs.SCIGCR1.bit.SWnRST = swnrst;
}

void comSetup() {
	EALLOW;

	LinaRegs.SCIGCR0.bit.RESET = 0;		// Into reset
	LinaRegs.SCIGCR0.bit.RESET = 1;		// Out of reset

	LinaRegs.SCIGCR1.bit.SWnRST = 0;	// Into software reset

	LinaRegs.SCIGCR1.bit.COMMMODE = 0;	// Idle-Line Mode
	LinaRegs.SCIGCR1.bit.TIMINGMODE = 1;	// Asynchronous Timing
	LinaRegs.SCIGCR1.bit.PARITYENA = 0;	// No Parity Check
	LinaRegs.SCIGCR1.bit.PARITY = 0;	// Odd Parity
	LinaRegs.SCIGCR1.bit.STOP = 0;		// One Stop Bit
	LinaRegs.SCIGCR1.bit.CLK_MASTER = 1;	// Enable SCI Clock
	LinaRegs.SCIGCR1.bit.LINMODE = 0;	// SCI Mode
	LinaRegs.SCIGCR1.bit.SLEEP = 0;		// Ensure Out of Sleep
	LinaRegs.SCIGCR1.bit.MBUFMODE = 0;	// No Buffers Mode
	LinaRegs.SCIGCR1.bit.LOOPBACK = 0;	// External Loopback
	LinaRegs.SCIGCR1.bit.CONT = 1;		// Continue on Suspend
	LinaRegs.SCIGCR1.bit.RXENA = 1;		// Enable RX
	LinaRegs.SCIGCR1.bit.TXENA = 1;		// Enable TX

	// Ensure IODFT is disabled
	LinaRegs.IODFTCTRL.bit.IODFTENA = 0x0;

	// Set transmission length
	LinaRegs.SCIFORMAT.bit.CHAR = 7;	// Eight bits
	LinaRegs.SCIFORMAT.bit.LENGTH = 0;	// One byte

	comSetBaud(9600);

	LinaRegs.SCIGCR1.bit.SWnRST = 1;  // bring out of software reset

	EDIS;
}

void epwmSetup() {
	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;

	// TBCTL:
	// bit 15-14: 01 => emu stop at 0; this means, at zero,
	//                  all transistor driving PWMs should be OFF !
	//                  (at least in PFC charger mode)
	//            10 => free run
	// bit 13: PHSDIR direction after SYNC; set 1 for up-down, 0 for down count
	// bit 12-10: CLKDIV = /1
	// bit 9-7: HSPCLKDIV = /2 (default, not used)
	// bit 6: SWFSYNC = 0
	// bit 5-4: SYNCOSEL; 01 = EPWMxSYNCO at zero, 11 = disable EPWMxSYNCO
	// bit 3: PRDLD = 0; use shadow register
	// bit 2: PHSEN; 1 = use EPWMxSYNCI, load from TBCTR from TBPHS
	// bit 0-1: CTRMODE; 00 = up, 01 = down, 11 = up-down

	// inv L
	EPwm1Regs.TBCTL.all = 0x6012; // L: up-down count, EPWM1SYNCO
	EPwm1Regs.TBCTR = 0;
	//EPwm1Regs.TBPHS.all = 0;
	EPwm1Regs.TBPRD = PERIOD;
	EPwm1Regs.ETSEL.all = 0x0900; // zero => ADCSOCA
	EPwm1Regs.ETPS.all = 0x0100; // pulse on each event
	EPwm1Regs.AQCTLA.all = 0x0061;
	EPwm1Regs.AQCTLB.all = 0x0601;
	EPwm1Regs.CMPA.half.CMPA = PERIOD;
	EPwm1Regs.CMPB = PERIOD;
	EPwm1Regs.AQSFRC.all = 0x002d; // clear output A & B

	// inv N
	EPwm2Regs.TBCTL.all = 0x60a6; // N: up-down count, PHSEN (from EPWM1SYNCO)
	EPwm2Regs.TBCTR = 0;
	EPwm2Regs.TBPHS.all = 0;
	EPwm2Regs.TBPRD = PERIOD;
	EPwm2Regs.AQCTLA.all = 0x0061;
	EPwm2Regs.AQCTLB.all = 0x0601;
	EPwm2Regs.CMPA.half.CMPA = PERIOD;
	EPwm2Regs.CMPB = PERIOD;
	EPwm2Regs.AQSFRC.all = 0x002d; // clear output A & B

	// charging buck
	EPwm3Regs.TBCTL.all = 0x40a1; // buck - down count
	EPwm3Regs.TBCTR = PERIOD;
	EPwm3Regs.TBPHS.all = 0;
	EPwm3Regs.TBPRD = PERIOD;
	EPwm3Regs.AQCTLA.all = 0x0049; // zero -> clear, period -> set, CMPA(decr.) -> clear
	EPwm3Regs.CMPA.half.CMPA = PERIOD;
	EPwm3Regs.AQSFRC.all = 0x002d; // clear output A & B

	// EPWM5A = MPPT boost
	EPwm5Regs.TBCTL.all = 0x60a2; // up-down count
	EPwm5Regs.TBCTR = 0;
	EPwm5Regs.TBPHS.all = 0;
	EPwm5Regs.TBPRD = PERIOD;
	EPwm5Regs.AQCTLA.all = 0x0061;
	EPwm5Regs.CMPA.half.CMPA = PERIOD;
	EPwm5Regs.AQSFRC.all = 0x002d; // clear output A & B

	// EPWM6B = beep

	// EPWM7B = fan
	EPwm7Regs.TBCTL.all = 0x40a1;
	EPwm7Regs.TBCTR = 0;
	EPwm7Regs.TBPRD = FAN_PERIOD;
	EPwm7Regs.AQCTLB.all = 0x0409;
	EPwm7Regs.CMPB = FAN_PERIOD;
	EPwm7Regs.AQSFRC.all = 0x002d; // clear output A & B

	EPwm1Regs.CMPCTL.all = 0x0000; // shadow reload on zero
	EPwm2Regs.CMPCTL.all = 0x0000; // shadow reload on zero
	EPwm3Regs.CMPCTL.all = 0x0000; // shadow reload on zero
	EPwm5Regs.CMPCTL.all = 0x0000; // shadow reload on zero
	EPwm6Regs.CMPCTL.all = 0x0000; // shadow reload on zero
	EPwm7Regs.CMPCTL.all = 0x0000; // shadow reload on zero

	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;

	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1; // EPWM1A
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1; // EPWM1B
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1; // EPWM2A
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1; // EPWM2B
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1; // EPWM3A - buck
	GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1; // EPWM5A - MPPT boost
	GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 1; // EPWM7B - fan
	EDIS;
}

void comTx(uint16_t data)
{
	while(LinaRegs.SCIFLR.bit.TXRDY == 0);
	LinaRegs.SCITD = data;
}

uint16_t comRx() {
	while(LinaRegs.SCIFLR.bit.RXRDY == 0);
		return LinaRegs.SCIRD;
}

uint16_t comRxRdy() {
	return LinaRegs.SCIFLR.bit.RXRDY;
}

void adcSetup() {
	const uint16_t adcsocctl = 0x280a;
	
	EALLOW;
	
	AdcRegs.ADCCTL1.all = 0x0000;
	asm (" NOP");
	asm (" NOP");
	AdcRegs.ADCCTL1.all = 0x40e4; // power on, INPUTPULSEPOS = 1
	AdcRegs.ADCCTL2.all = 0x0002; // ADCCLK = SYSCLK, ADCNONOVERLAP = 1;
//	AdcRegs.SOCPRICTL.all = 0x0010; // reset
//	AdcRegs.SOCPRICTL.all = 0x0400; // reset, RRPOINTER = 0x20
	AdcRegs.ADCINTSOCSEL1.all = 0x0000;
	AdcRegs.ADCINTSOCSEL2.all = 0x0000;

	// sample 11 cycles, trigger by EPWM1->ADCSOCA
	AdcRegs.ADCSOC0CTL.all = adcsocctl | (0 << 6);
	AdcRegs.ADCSOC1CTL.all = adcsocctl | (1 << 6);
	AdcRegs.ADCSOC2CTL.all = adcsocctl | (2 << 6);
	AdcRegs.ADCSOC3CTL.all = adcsocctl | (3 << 6);
	AdcRegs.ADCSOC4CTL.all = adcsocctl | (4 << 6);
	AdcRegs.ADCSOC5CTL.all = adcsocctl | (5 << 6);
	AdcRegs.ADCSOC6CTL.all = adcsocctl | (6 << 6);
	AdcRegs.ADCSOC7CTL.all = adcsocctl | (7 << 6);
	AdcRegs.ADCSOC8CTL.all = adcsocctl | (8 << 6);
	AdcRegs.ADCSOC9CTL.all = adcsocctl | (9 << 6);
	AdcRegs.ADCSOC10CTL.all = adcsocctl | (10 << 6);
	AdcRegs.ADCSOC11CTL.all = adcsocctl | (11 << 6);
	AdcRegs.ADCSOC12CTL.all = adcsocctl | (12 << 6);
	AdcRegs.ADCSOC13CTL.all = adcsocctl | (13 << 6);
	AdcRegs.ADCSOC14CTL.all = adcsocctl | (14 << 6);
	AdcRegs.ADCSOC15CTL.all = adcsocctl | (15 << 6);

	AdcRegs.INTSEL1N2.all = 0x006f; // INT1 on EOC15, continuous

	AdcRegs.ADCINTFLGCLR.all |= 0x01ff;
	EDIS;
}

inline void buckSetPwm(int16_t pwm) {
	EPwm3Regs.CMPA.half.CMPA = PERIOD - pwm;
}

inline void buckPwmOff() {
	EPwm3Regs.AQSFRC.all = 0x0005; // clear output A
	EPwm3Regs.CMPA.half.CMPA = PERIOD;
	EPwm3Regs.AQSFRC.all = 0x0005; // clear output A
}

inline void pfcSetPwm(int16_t invPwm) {
	if (invPwm >= 0) {
		EPwm1Regs.CMPB = PERIOD;
		EPwm1Regs.CMPA.half.CMPA = PERIOD - invPwm;

		// To switch N or not to switch N ? Not sure if there's
		// any difference. Current goes through the diode anyways.
		EPwm2Regs.CMPB = PERIOD;
		EPwm2Regs.CMPA.half.CMPA = PERIOD - invPwm;
	} else {
		EPwm1Regs.CMPA.half.CMPA = PERIOD;
		EPwm1Regs.CMPB = PERIOD + invPwm;

		EPwm2Regs.CMPB = PERIOD;
		EPwm2Regs.CMPA.half.CMPA = PERIOD - invPwm;
	}
}

inline void pfcPwmOff() {
	EPwm1Regs.AQSFRC.all = 0x002d; // clear output A & B
	EPwm2Regs.AQSFRC.all = 0x002d; // clear output A & B

	EPwm1Regs.CMPA.half.CMPA = PERIOD;
	EPwm1Regs.CMPB = PERIOD;
	EPwm2Regs.CMPA.half.CMPA = PERIOD;
	EPwm2Regs.CMPB = PERIOD;
	
	EPwm1Regs.AQSFRC.all = 0x002d; // clear output A & B
	EPwm2Regs.AQSFRC.all = 0x002d; // clear output A & B
}

void lcdCsOn() {
	GpioDataRegs.GPASET.bit.GPIO20 = 1; // set CLK
	GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1; // clear CS
}

void lcdCsOff() {
	GpioDataRegs.GPBSET.bit.GPIO34 = 1; // set CS
	GpioDataRegs.GPASET.bit.GPIO20 = 1; // set CLK
}

void lcdSend(uint16_t bits, uint16_t data) {
	lcdCsOn();
	delay(60);
	do {
		GpioDataRegs.GPACLEAR.bit.GPIO20 = 1; // set CLK
		if (data & (1 << (bits - 1)))
			GpioDataRegs.GPASET.bit.GPIO13 = 1;
		else
			GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;
		delay(20);
		GpioDataRegs.GPASET.bit.GPIO20 = 1; // set CLK
		delay(30);
	} while (--bits);
	delay(120);
	lcdCsOff();
}

#endif
