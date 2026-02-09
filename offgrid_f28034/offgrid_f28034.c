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

#define GLOBAL_Q 16 // 1 bit sign, 15 bits integer, 16 bits decimal (-32768..+32767, 0.000015 resolution)

#include "DSP28x_Project.h"
#include "IQmathLib.h"
#include "hw.h"
#include "sevenseg.h"

#include <stdint.h>
#include <string.h>


#define PI 3.141592653589
#define DEG90 _IQ28(PI / 2)
#define DEG180 _IQ28(PI)
#define DEG270 _IQ28(PI * 3 / 2)
#define DEG360 _IQ28(PI * 2)

#define MAX_ZERO_OFFSET 50

#define I2C_TIMEOUT 10000

#pragma CODE_SECTION(adcIsr, "ramfuncs");

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadSize;
extern Uint16 RamfuncsRunStart;

const uint16_t lcdSegMap[8][7] = {
	{ 15, 14, 13, 8, 9, 11, 10 },
	{ 23, 22, 21, 16, 17, 19, 18 },
	{ 31, 30, 29, 24, 25, 27, 26 },
	{ 47, 46, 45, 40, 41, 43, 42 },
	{ 55, 54, 53, 48, 49, 51, 50 },
	{ 71, 70, 69, 64, 65, 67, 66 },
	{ 79, 78, 77, 72, 73, 75, 74 },
	{ 87, 86, 85, 80, 81, 83, 82 }
};

enum { MODE_STARTUP, MODE_ACCHARGE, MODE_INVERTER };
enum { DCDC_OFF, DCDC_SOFTSTART, DCDC_ON };

enum { KEY_INV, KEY_ESC, KEY_UP, KEY_DOWN, KEY_ENTER };
enum { KEYSTATE_NONE, KEYSTATE_FIRST, KEYSTATE_REP, KEYSTATE_INVALID };

enum { LCDLEFT_VBAT, LCDLEFT_VGRID, LCDLEFT_VINV, LCDLEFT_FINV, LCDLEFT_VBUS,
	LCDLEFT_IBAT, LCDLEFT_TINV, LCDLEFT_TMOSFET, LCDLEFT_TBOOST, LCDLEFT_AC_START,
	LCDLEFT_AC_STOP, LCDLEFT_ISR_MAX, LCDLEFT_PFC_KP, LCDLEFT_PFC_KI,
	LCDLEFT_PFC_KD, LCDLEFT_CNT };

uint16_t keyNow = 0, keyPrev = 0, keyLast = 0, keyState = 0;
uint16_t lcdLeftSel = 0;
uint16_t lcdSetMode = 0;
uint16_t lcdBatLevel = 0;

uint16_t mode = MODE_STARTUP;
uint16_t startupCnt = 0;

volatile _iq28 angle = 0; // 0 .. 2xPI (IQ28 range -8..+7)

// angle increment in one inverter PWM cycle 
// = 19.2kHz for PERIOD=1562 (default)
// = 25kHz for PERIOD=1200
volatile _iq28 angleStep = _IQ28(2.0 * PI / (DSP_CLK / PERIOD / 2 / 50));
const _iq28 angleStepDefault = _IQ28(2.0 * PI / (DSP_CLK / PERIOD / 2 / 50));

const uint16_t angleStepPeriodShift = 5; // minimum 0.002914Hz increment
const uint16_t angleStepPhaseShift = 5;

const uint16_t gridPeriodMin = DSP_CLK / PERIOD / 2 / (50.0 * 1.1);
const uint16_t gridPeriodMax = DSP_CLK / PERIOD / 2 / (50.0 * 0.9);
const _iq pfcPwmMax = _IQ(0.96 * PERIOD);

volatile uint16_t timInv = 0; // inv. interrupt counter (0-65535)
volatile uint16_t invIdx = 0; // inv. wave index
volatile uint16_t waveCnt = 0; // counter of AC sinewaves (0-65536)
volatile int16_t invPeriod = DSP_CLK / PERIOD / 2 / 50;

volatile int16_t gridPeriod = 0;
volatile uint16_t gridPol = 0;
volatile uint16_t gridPolCnt = 0;
volatile uint16_t gridLastZeroCross = 0;
volatile int16_t gridPhase = 0, gridPhaseTmp = 0, gridPhaseLast = 0;
volatile uint16_t gridRevPolTmp = 0;
uint16_t gridRevPolSum;
_iq gridRevPolEma;

volatile uint16_t faultCode = 0;

volatile uint16_t gridVoltOk = 0;
volatile uint16_t gridVoltOkCnt = 0;
volatile uint16_t gridPllOk = 0;
volatile uint16_t gridPllOkCnt = 0;
uint16_t swOn = 1;
volatile uint16_t pfcRun = 0;
volatile _iq ivRatio = 0;
volatile _iq iInvReq = 0;
volatile _iq iErr0 = 0;
volatile _iq iErr1 = 0;
volatile _iq iErr2 = 0;
volatile _iq pfcPwm;
volatile _iq pfcTune = 0;
volatile uint16_t pfcPause = 0;
volatile uint16_t sc = 0;
volatile uint16_t scTot = 0;
volatile _iq pfcKp = _IQ(4.0);
volatile _iq pfcKi = _IQ(2.5);
volatile _iq pfcKd = _IQ(5.0);

volatile uint16_t dcdcSts = DCDC_OFF;
volatile uint16_t dcdcReq = 0;

volatile _iq vInv = 0;
volatile _iq vInvLast = 0;
volatile _iq vInvDelta = 0;

volatile int16_t vInvRaw = 0;
volatile int16_t vInvOffset = -2048;
volatile uint32_t vInvSqTmp = 0;
uint32_t vInvSqSum;
_iq vInvRms;

volatile int16_t vGridRaw = 0;
volatile int16_t vGridOffset = -2048;
volatile int16_t vGridAmplTmp = 0;
int16_t vGridAmplRaw;
volatile uint32_t vGridSqTmp = 0;
uint32_t vGridSqSum;
_iq vGridAmpl;
_iq vGridAmplEma;
_iq vGridRms;
const _iq vGridRmsMin = _IQ(230.0 * 0.8);
const _iq vGridRmsMax = _IQ(230.0 * 1.2);

_iq iChargeReq = 0;
_iq iChargeErr0 = 0;
_iq iChargeErr1 = 0;
_iq iChargeErr2 = 0;
_iq iChargeSet = _IQ(50.0);
_iq vBatAcStart = _IQ(53.0);
_iq vBatAcStop = _IQ(55.5);
_iq acChargeKp = 30; // 30/65536
_iq acChargeKi = 30; // 30/65536
_iq acChargeKd = 15; // 15/65536
uint16_t acCharge = 1;

volatile int16_t iInvRaw = 0;
volatile int16_t iInvOffset = -2048;
volatile _iq iInv = 0;

volatile int16_t iCthRaw = 0;
volatile int16_t iCthOffset = -2048;
volatile int16_t iCtlRaw = 0;
volatile int16_t iCtlOffset = -2048;

volatile int16_t iDcdcRaw = 0;
volatile int16_t iDcdcOffset = -2048;
volatile int32_t iDcdcTmp = 0;
int32_t iDcdcSum;
volatile _iq iDcdc = 0;
volatile _iq iDcdcEma = 0;
_iq iDcdcAvg;
_iq iBat;

volatile int32_t vBatTmp = 0;
int32_t vBatSum;
_iq vBat;

volatile _iq vBus = 0;
volatile _iq vBusEma = 0;
volatile int32_t vBusTmp = 0;
int32_t vBusSum;
_iq vBusAvg;

volatile uint16_t newWaveData = 0;
volatile int16_t newInvPeriod = 0;

// temperature measurements
uint16_t tSampleCnt = 0;
int32_t tInvTmp = 0;
int32_t tMosfetTmp = 0;
int32_t tBoostTmp = 0;
_iq tInv = 0;
_iq tMosfet = 0;
_iq tBoost = 0;

uint16_t timNow = 0;
uint16_t timKey = 0;
uint16_t timLcd = 0;
uint16_t timWave = 0; // 50 Hz
uint16_t timTempCheck = 0;
uint16_t gridRelayWait = 0;
uint16_t gridRelaySts = 0;
uint16_t gridRelayReq = 0;

uint16_t acStartCnt = 0, acStopCnt = 0;

uint16_t buckPwm = 0;

uint16_t vram[8];

// DEBUG
volatile int16_t snapReady, snapIdx, snapVInv, snapIInv, snapIInv1, snapVInv1, snapICth, snapICtl;
volatile int16_t snapIDcdc, snapPwm, snapVBus, snapIvRatio, snapVBat, snapIInvReq, snapPfcTune;

volatile _iq faultDcdcEma, faultIInv, faultIvRatio;
volatile uint16_t faultPwm;

volatile uint16_t start, end, maxEnd = 0;

__interrupt void adcIsr() {
	start = CpuTimer0Regs.TIM.all;

	// check for inverter overcurrent
	iInvRaw = ADC_IINV + iInvOffset;
	if (abs(iInvRaw) > iInvRawLimit) {
		if (pfcRun) {
			pfcPwmOff();
			pfcPause = 2;
			scTot++;
			if (sc > 1000) {
				pfcRun = 0;
				faultCode = 51; // overload/surge
				faultDcdcEma = iDcdcEma;
				faultIInv = iInv;
				faultPwm = _IQint(pfcPwm);
				faultIvRatio = ivRatio;
			} else {
				sc += 100;
			}
		}
	}

	// check for bus overvoltage
	vBus = _IQmpyI32(vBusFactor, ADC_VBUS);
	if (vBus > _IQ(490.0)) {
		if (pfcRun) {
			pfcPwmOff();
			pfcPause = 2;
		}

		if (vBusEma > _IQ(495.0)) {
			softStartOff();
			dcdcSts = DCDC_OFF;
			dcdcReq = 0;
			pfcRun = 0;
			faultCode = 8; // bus voltage too high
		}
	}

	// check for DC/DC bus side overcurrent
	iDcdcRaw = ADC_IDCDC + iDcdcOffset;
	iDcdcTmp += iDcdcRaw;
	iDcdc = _IQmpyI32(iDcdcFactor, iDcdcRaw);
	iDcdcEma = _IQdiv16(iDcdcEma * 15 + iDcdc);
	if (_IQabs(iDcdcEma) > iDcdcLimit) {
		if (pfcRun) {
			pfcPwmOff();
			buckPwmOff();
			pfcRun = 0;
			buckPwm = 0;
			faultCode = 12; // DC/DC overcurrent
			faultDcdcEma = iDcdcEma;
			faultIInv = iInv;
			faultPwm = _IQint(pfcPwm);
		}
	}

	timInv++;

	// other ADC measurements
	vInvRaw = ADC_VINV + vInvOffset;
	vInv = _IQmpyI32(vInvFactor, vInvRaw);
	vInvSqTmp += (int32_t) vInvRaw * vInvRaw;
	vInvDelta = vInv - vInvLast;
	vInvLast = vInv;

	vGridRaw = ADC_VGRID + vGridOffset;
	vGridAmplTmp = __max(abs(vGridRaw), vGridAmplTmp);
	vGridSqTmp += (int32_t) vGridRaw * vGridRaw;

	iInv = _IQmpyI32(iInvFactor, iInvRaw);

	vBatTmp += ADC_VBAT;

	vBusTmp += ADC_VBUS;
	vBusEma = _IQdiv16(vBusEma * 15 + vBus);

	iCthRaw = ADC_ICTH + iCthOffset;
	iCtlRaw = ADC_ICTL + iCtlOffset;

	// grid period measurement
        if ((vGridRaw < 0 && gridPol) || (vGridRaw >= 0 && !gridPol)) {
                gridPolCnt++;
                if (gridPolCnt > 10) {
                        gridPol = vGridRaw >= 0;
                        if (gridPol) {
                                gridPeriod = timInv - gridLastZeroCross;
                                gridLastZeroCross = timInv;
                        }
                }
        } else {
                gridPolCnt = 0;
        }
	
	// grid phase measurement
	if (vGridRaw < 0 && angle < DEG180) {
		gridRevPolTmp++;
		if (angle < DEG90)
			gridPhaseTmp++;
		else
			gridPhaseTmp--;
	}

	if (vGridRaw > 0 && angle >= DEG180) {
		gridRevPolTmp++;
		if (angle < DEG270)
			gridPhaseTmp++;
		else
			gridPhaseTmp--;
	}
	
	// PFC switching control
	if (
	  pfcRun &&
	  !pfcPause &&
	  vBus != 0
	  ) {
		iInvReq = - _IQmpy(vInv, ivRatio);
		iErr2 = iErr1;
		iErr1 = iErr0;
		iErr0 = _IQabs(iInvReq) - _IQabs(iInv);
		
		pfcTune += (
			_IQmpy(iErr0 - iErr1, pfcKp) + 
			_IQmpy(iErr0, pfcKi) + 
			_IQmpy(iErr0 - (iErr1 << 1) + iErr2, pfcKd)) >> 8;

		pfcTune = _IQsat(pfcTune, _IQ(1.5), 0);

		// PERIOD * (1.02 - vInv / vBus)
		// Don't ask me why 1.02. It just shows better results than just 1 :)
		pfcPwm = _IQmpy(_IQ(PERIOD), (_IQ(1.02) - _IQdiv(_IQabs(vInv), vBus)));
		pfcPwm = _IQsat(_IQmpy(pfcPwm, pfcTune), pfcPwmMax, 0);

		if (vInv < 0)
			pfcSetPwm(_IQint(pfcPwm));
		else
			pfcSetPwm(-_IQint(pfcPwm));
	} else {
		pfcPwm = 0;
		pfcPwmOff();
	}

	if (!snapReady && snapIdx == invIdx) {
		snapVInv = vInvRaw;
		snapIInv = iInvRaw;
		snapICth = iCthRaw;
		snapICtl = iCtlRaw;
		snapPwm = _IQint(pfcPwm);
		snapVBus = ADC_VBUS;
		snapIvRatio = ivRatio;
		snapIDcdc = iDcdcRaw;
		snapVBat = ADC_VBAT;
		snapIInvReq = iInvReq >> 8;
		snapPfcTune = pfcTune >> 8;
		snapReady = 1;
	}

	angle += angleStep;
	invIdx++;
	if (angle > DEG360) {
		angle -= DEG360;

		invPeriod = invIdx;
		invIdx = 0;

		waveCnt++;

		// PLL
		gridPhaseLast = gridPhase;
		gridPhase = gridPhaseTmp;
		gridPhaseTmp = -5;

		if (gridVoltOk) {
			if (gridPeriod > gridPeriodMin && gridPeriod < gridPeriodMax)
				angleStep += (int32_t) (invPeriod - gridPeriod) << angleStepPeriodShift;

			if (abs(gridPhase) > abs(gridPhaseLast))
				angleStep -= (int32_t) gridPhase << angleStepPhaseShift;
		} else {
			if (angleStep < angleStepDefault) angleStep++;
			else if (angleStep > angleStepDefault) angleStep--;
		}

		// new values to be processed outside of interrupt
		if (!newWaveData) {
			vGridAmplRaw = vGridAmplTmp;
			vInvSqSum = vInvSqTmp;
			vGridSqSum = vGridSqTmp;
			iDcdcSum = iDcdcTmp;
			vBusSum = vBusTmp;
			vBatSum = vBatTmp;
			gridRevPolSum = gridRevPolTmp;
			newInvPeriod = invPeriod;
			newWaveData = 1;
		}
		vGridAmplTmp = 0;
		vInvSqTmp = 0;
		vGridSqTmp = 0;
		iDcdcTmp = 0;
		vBusTmp = 0;
		vBatTmp = 0;
		gridRevPolTmp = 0;

		if (sc && pfcRun) sc--;
	}

	end = CpuTimer0Regs.TIM.all;
	if (start - end > maxEnd) maxEnd = start - end;

	AdcRegs.ADCINTFLGCLR.all = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

uint16_t intToHex(uint16_t value) {
	value &= 0xf;
	if (value <= 9)
		return value + '0';
	else
		return value + 'A' - 10;
}

void comPrintInt(int16_t word) {
	if (word < 0) {
		comTx('-');
		word = -word;
	}
	comTx('0' + (word / 1000 % 10));
	comTx('0' + (word / 100 % 10));
	comTx('0' + (word / 10 % 10));
	comTx('0' + (word % 10));
}

void comPrintWord(uint16_t word) {
	comTx(intToHex(word >> 12));
	comTx(intToHex(word >> 8));
	comTx(intToHex(word >> 4));
	comTx(intToHex(word));
}

void comPrintIQ(char* format, _iq num) {
	char buf[32];
	uint16_t i;

	_IQtoa(buf, format, num);
	for (i = 0; i < 32; i++) {
		if (buf[i] == 0) break;
		comTx(buf[i]);
	}
}

_iq ntc15kh4150(_iq adcVal) {
	_iq tmp;

	// calculate R/R0
	tmp = _IQdiv(_IQ(5.0 * 0.75 / 3.3 / 15.0 * 4096.0), adcVal) - _IQ(0.75 / 15.0);

	// T = T0 * B / (T0 * log(R/R0) + B) - TK;
	tmp = _IQlog(tmp); // log(R/R0)
	tmp = _IQ(4150.0) + _IQmpy(tmp, _IQ(298.15)); // B + T0*log(R/R0)
	tmp = _IQdiv(_IQ(298.15), tmp); // T0/tmp
	tmp = _IQmpy(_IQ(4150.0), tmp); // tmp*B
	return tmp - _IQ(273.15); // tmp - TK
}

void calcWaveData() {
	if (!newWaveData) return;

	// grid voltage amplitude
	vGridAmpl = _IQmpyI32(vGridFactor, vGridAmplRaw);
	vGridAmplEma = _IQdiv8(vGridAmplEma * 7 + vGridAmpl);

	// inverter RMS voltage (not needed in PFC mode)
	vInvRms = _IQmpy(_IQ8sqrt((vInvSqSum / newInvPeriod) << 8) << 8, vInvFactor);

	// grid RMS voltage
	vGridRms = _IQmpy(_IQ8sqrt((vGridSqSum / newInvPeriod) << 8) << 8, vGridFactor);

	// DC/DC current
	iDcdcAvg = _IQmpyI32(iDcdcFactor, iDcdcSum / newInvPeriod);
	iBat = _IQmpy(iDcdcAvg, dcdcRatio);

	// bus voltage (not needed)
	vBusAvg = _IQmpyI32(vBusFactor, vBusSum / newInvPeriod);

	// battery voltage
	vBat = _IQmpyI32(vBatFactor, vBatSum / newInvPeriod);

	// number of reverse polarity samples on grid voltage for PLL lock detection
	gridRevPolEma = _IQdiv16(gridRevPolEma * 15 + ((uint32_t) gridRevPolSum << 16));

	newWaveData = 0;
}

void dcdcTask() {
	switch (dcdcSts) {
	case DCDC_OFF:
		if (dcdcReq && !faultCode) {
			softStartOn();
			dcdcSts = DCDC_SOFTSTART;
			break;
		}
	case DCDC_SOFTSTART:
		if (!dcdcReq) {
			softStartOff();
			dcdcSts = DCDC_OFF;
			break;
		}
		if (vBusEma >= _IQmpy(vBat, dcdcRatio)) {
			softStartOff();
			dcdcOn();
			dcdcSts = DCDC_ON;
		}
		break;
	case DCDC_ON:
		// wait for current to drop before turning DC/DC off
		if (!dcdcReq && _IQabs(iDcdcAvg) < iMaxDcdcShutdown) {
			dcdcOff();
			dcdcSts = DCDC_OFF;
		}
		break;
	}
}

uint16_t getKey() {
	uint16_t adc;

	adc = ADC_KEY;
	timNow = timInv;

	keyNow = KEY_INV;
	if (adc > 3686) keyNow = KEY_ESC;
	else if (adc > 2954 && adc < 3610) keyNow = KEY_UP;
	else if (adc > 2005 && adc < 2451) keyNow = KEY_DOWN;
	else if (adc > 959 && adc < 1173) keyNow = KEY_ENTER;

	if (keyNow != keyPrev) {
		timKey = timNow;
		keyPrev = keyNow;
	} else if (keyNow == KEY_INV && timNow - timKey > 2000) { // no key pressed - back to normal state
		keyLast = KEY_INV;
		keyState = KEYSTATE_NONE;
		timKey = timNow;
	} else if (keyNow != KEY_INV && keyState == KEYSTATE_NONE && timNow - timKey > 300) { // key pressed
		keyLast = keyNow;
		keyState = KEYSTATE_FIRST;
		timKey = timNow;
		return keyNow;
	} else if (keyNow == keyLast && keyState == KEYSTATE_FIRST && timNow - timKey > 20000) { // key hold - repeat delay
		keyState = KEYSTATE_REP;
		timKey = timNow;
		return keyNow;
	} else if (keyNow == keyLast && keyState == KEYSTATE_REP && timNow - timKey > 8000) { // key hold - repeat rate
		timKey = timNow;
		return keyNow;
	} else {
		if (keyNow != keyLast && keyState != KEYSTATE_NONE) // change of pressed keys - ignore (is this even reachable?)
			keyState = KEYSTATE_INVALID;
	}
	return KEY_INV;
}


inline void setVramBit(uint16_t bitIdx, uint16_t value) {
	value = value ? 1 : 0;
	vram[bitIdx >> 4] &= ~(1 << (bitIdx & 0xf));
	vram[bitIdx >> 4] |= value << (bitIdx & 0xf);
}

inline void lcdAscii(uint16_t idx, uint16_t ascii) {
	uint16_t i, bitIdx;

	for (i = 0; i < 7; i++) {
		bitIdx = lcdSegMap[idx][i];
		setVramBit(bitIdx, (sevenSegAscii[ascii - 0x20] >> i) & 1);
	}
}

inline void iqTo2dig(uint16_t idx, _iq value) {
	uint16_t intVal;

	value = _IQabs(value);
	if (value >= _IQ(100.0)) {
		lcdAscii(idx, 'H');
		lcdAscii(idx + 1, 'H');
	} else {
		intVal = _IQint(value);
		lcdAscii(idx + 1, '0' + intVal % 10);
		intVal /= 10;
		lcdAscii(idx, '0' + intVal % 10);
	}
}

inline void iqTo3dig(uint16_t idx, _iq value) {
	uint16_t intVal;
	uint16_t dot1, dot2;

	value = _IQabs(value);

	dot1 = idx == 0 ? 12 : 68;
	dot2 = idx == 0 ? 20 : 76;

	if (value >= _IQ(1000.0)) {
		lcdAscii(idx, 'H');
		lcdAscii(idx + 1, 'H');
		lcdAscii(idx + 2, 'H');
		setVramBit(dot1, 0);
		setVramBit(dot2, 0);
	} else {
		if (value < _IQ(10.0)) {
			intVal = _IQint(_IQmpyI32(value, 100.0));
			setVramBit(dot1, 1);
			setVramBit(dot2, 0);
		} else if (value < _IQ(100.0)) {
			intVal = _IQint(_IQmpyI32(value, 10));
			setVramBit(dot1, 0);
			setVramBit(dot2, 1);
		} else {
			intVal = _IQint(value);
			setVramBit(dot1, 0);
			setVramBit(dot2, 0);
		}
		lcdAscii(idx + 2, '0' + intVal % 10);
		intVal /= 10;
		lcdAscii(idx + 1, '0' + intVal % 10);
		intVal /= 10;
		lcdAscii(idx, '0' + intVal % 10);
	}
}

void vramTask() {
	uint16_t key;

	key = getKey();
	if (lcdSetMode) {
		switch (lcdLeftSel) {
		case LCDLEFT_AC_START:
			if (key == KEY_DOWN) vBatAcStart -= _IQ(0.1);
			if (key == KEY_UP) vBatAcStart += _IQ(0.1);
			break;
		case LCDLEFT_AC_STOP:
			if (key == KEY_DOWN) vBatAcStop -= _IQ(0.1);
			if (key == KEY_UP) vBatAcStop += _IQ(0.1);
			break;
		case LCDLEFT_IBAT:
			if (key == KEY_DOWN) iChargeSet -= _IQ(1.0);
			if (key == KEY_UP) iChargeSet += _IQ(1.0);
			break;
		case LCDLEFT_PFC_KP:
			if (key == KEY_DOWN) pfcKp -= _IQ(0.2);
			if (key == KEY_UP) pfcKp += _IQ(0.2);
//			if (key == KEY_DOWN) acChargeKp--;
//			if (key == KEY_UP) acChargeKp++;
			break;
		case LCDLEFT_PFC_KI:
			if (key == KEY_DOWN) pfcKi -= _IQ(0.2);
			if (key == KEY_UP) pfcKi += _IQ(0.2);
//			if (key == KEY_DOWN) acChargeKi--;
//			if (key == KEY_UP) acChargeKi++;
			break;
		case LCDLEFT_PFC_KD:
			if (key == KEY_DOWN) pfcKd -= _IQ(0.5);
			if (key == KEY_UP) pfcKd += _IQ(0.5);
//			if (key == KEY_DOWN) acChargeKd--;
//			if (key == KEY_UP) acChargeKd++;
			break;
		}
	} else {
		if (key == KEY_DOWN) {
			lcdLeftSel--;
			if (lcdLeftSel > LCDLEFT_CNT)
				lcdLeftSel = LCDLEFT_CNT - 1;
		}
		if (key == KEY_UP) {
			lcdLeftSel++;
			if (lcdLeftSel == LCDLEFT_CNT)
				lcdLeftSel = 0;
		}
	}

	if (key == KEY_ENTER && (
		lcdLeftSel == LCDLEFT_AC_START ||
		lcdLeftSel == LCDLEFT_AC_STOP ||
		lcdLeftSel == LCDLEFT_IBAT ||
		lcdLeftSel == LCDLEFT_PFC_KP ||
		lcdLeftSel == LCDLEFT_PFC_KI ||
		lcdLeftSel == LCDLEFT_PFC_KD
		)) {
		lcdSetMode = !lcdSetMode;
	}

	timNow = timInv;
	if (timNow - timLcd < 9600 && !key) return;

	timLcd = timNow;

	setVramBit(52, 1); // center border
	setVramBit(58, faultCode); // ERROR
	setVramBit(92, 1); // underline
	setVramBit(110, pfcRun); // charger -> battery arrow
	setVramBit(113, pfcRun); // charger
	setVramBit(114, 1); // battery
	setVramBit(121, pfcRun); // grid -> charger arrow
	setVramBit(123, gridVoltOk); // grid

	// battery charging symbol
	if (!pfcRun) {
		lcdBatLevel = 0;
	} else {
		lcdBatLevel++;
		if (lcdBatLevel > 4) lcdBatLevel = 1;
	}
	setVramBit(119 , lcdBatLevel >= 1);
	setVramBit(118 , lcdBatLevel >= 2);
	setVramBit(117 , lcdBatLevel >= 3);
	setVramBit(116 , lcdBatLevel >= 4);

	switch (lcdLeftSel) {
	case LCDLEFT_VBAT:
		iqTo3dig(0, vBat);
		break;
	case LCDLEFT_VGRID:
		iqTo3dig(0, vGridRms);
		break;
	case LCDLEFT_VINV:
		iqTo3dig(0, vInvRms);
		break;
	case LCDLEFT_FINV:
		iqTo3dig(0, _IQmpy(angleStep >> 12, _IQ(60000000.0 / PI / 4.0 / PERIOD)));
		break;
	case LCDLEFT_VBUS:
		iqTo3dig(0, vBusEma);
		break;
	case LCDLEFT_IBAT:
		if (!lcdSetMode)
			iqTo3dig(0, iBat);
		else
			iqTo3dig(0, iChargeSet);
		break;
	case LCDLEFT_TINV:
		iqTo3dig(0, tInv);
		break;
	case LCDLEFT_TMOSFET:
		iqTo3dig(0, tMosfet);
		break;
	case LCDLEFT_TBOOST:
		iqTo3dig(0, tBoost);
		break;
	case LCDLEFT_AC_START:
		iqTo3dig(0, vBatAcStart);
		break;
	case LCDLEFT_AC_STOP:
		iqTo3dig(0, vBatAcStop);
		break;
	case LCDLEFT_ISR_MAX:
		iqTo3dig(0, ((int32_t) maxEnd) * 6554);
		break;
	case LCDLEFT_PFC_KP:
		iqTo3dig(0, pfcKp);
//		iqTo3dig(0, acChargeKp << 16);
		break;
	case LCDLEFT_PFC_KI:
		iqTo3dig(0, pfcKi);
//		iqTo3dig(0, acChargeKi << 16);
		break;
	case LCDLEFT_PFC_KD:
		iqTo3dig(0, pfcKd);
//		iqTo3dig(0, acChargeKd << 16);
		break;
	}

	setVramBit(0, lcdLeftSel == LCDLEFT_VINV || lcdLeftSel == LCDLEFT_FINV); // INV
	setVramBit(2, lcdLeftSel == LCDLEFT_VINV || lcdLeftSel == LCDLEFT_VGRID || lcdLeftSel == LCDLEFT_FINV); // AC
	setVramBit(6, lcdLeftSel == LCDLEFT_VBAT || lcdLeftSel == LCDLEFT_IBAT || lcdLeftSel == LCDLEFT_AC_START || lcdLeftSel == LCDLEFT_AC_STOP); // BATT
	setVramBit(7, lcdLeftSel == LCDLEFT_VGRID); // INPUT
	setVramBit(28, lcdLeftSel == LCDLEFT_FINV); // Hz
	setVramBit(33,
		lcdLeftSel == LCDLEFT_VGRID ||
		lcdLeftSel == LCDLEFT_VINV ||
		lcdLeftSel == LCDLEFT_VBAT ||
		lcdLeftSel == LCDLEFT_VBUS ||
		lcdLeftSel == LCDLEFT_AC_START ||
		lcdLeftSel == LCDLEFT_AC_STOP); // V
	setVramBit(34, lcdLeftSel == LCDLEFT_IBAT); // A
	setVramBit(36, lcdLeftSel == LCDLEFT_TINV || lcdLeftSel == LCDLEFT_TMOSFET || lcdLeftSel == LCDLEFT_TBOOST); // C
	setVramBit(37, lcdLeftSel == LCDLEFT_TINV || lcdLeftSel == LCDLEFT_TMOSFET || lcdLeftSel == LCDLEFT_TBOOST); // deg symbol
	setVramBit(57, lcdSetMode); // wrench

	if (faultCode) {
		lcdAscii(4, '0' + faultCode % 10);
		lcdAscii(3, '0' + faultCode / 10 % 10);
	} else {
		switch (lcdLeftSel) {
		case LCDLEFT_VBUS: lcdAscii(3, 'b'); lcdAscii(4, 'u'); break;
		case LCDLEFT_TINV: lcdAscii(3, 't'); lcdAscii(4, '1'); break;
		case LCDLEFT_TMOSFET: lcdAscii(3, 't'); lcdAscii(4, '2'); break;
		case LCDLEFT_TBOOST: lcdAscii(3, 't'); lcdAscii(4, '3'); break;
		case LCDLEFT_AC_START: lcdAscii(3, 'b'); lcdAscii(4, '-'); break;
		case LCDLEFT_AC_STOP: lcdAscii(3, 'b'); lcdAscii(4, '+'); break;
		case LCDLEFT_ISR_MAX: lcdAscii(3, 'i'); lcdAscii(4, 't'); break;
		case LCDLEFT_PFC_KP: lcdAscii(3, 'k'); lcdAscii(4, 'p'); break;
		case LCDLEFT_PFC_KI: lcdAscii(3, 'k'); lcdAscii(4, 'i'); break;
		case LCDLEFT_PFC_KD: lcdAscii(3, 'k'); lcdAscii(4, 'd'); break;
		default: iqTo2dig(3, (int32_t) scTot << 16); break;
		}
	}
	iqTo3dig(5, iBat);
	setVramBit(62, 1); // right BATT
	setVramBit(90, 1); // right A
	setVramBit(107, 1); // Li
}

void lcdTask() {
	uint16_t i;

	lcdSend(12, 0x852); // bias & commons
	lcdSend(12, 0x830); // rc oscillator
	lcdSend(12, 0x80a); // wdt disable
	lcdSend(12, 0x80c); // 0x08? timer enable/disable
	lcdSend(12, 0x802); // turn on system oscillator
	lcdSend(12, 0x806); // LCD on
	for (i = 0; i < 32; i++) {
		lcdSend(13, 0x1400 | (i << 4) | ((vram[i >> 2] >> ((i & 3) << 2) & 0xf)));
	}
}

void gridCheck() {
	if (!gridVoltOk) {
		if (vGridRms >= vGridRmsMin && vGridRms <= vGridRmsMax) {
			if (gridVoltOkCnt < 10) {
				gridVoltOkCnt++;
			} else {
				gridVoltOk = 1;
				gridVoltOkCnt = 0;
			}
		} else {
			gridVoltOkCnt = 0;
		}
	} else {
		if (vGridRms < vGridRmsMin || vGridRms > vGridRmsMax) {
			if (gridVoltOkCnt < 3) {
				gridVoltOkCnt++;
			} else {
				gridVoltOk = 0;
				gridPllOk = 0;
				gridVoltOkCnt = 0;
			}
		} else {
			gridVoltOkCnt = 0;
		}
	}

	if (!gridPllOk) {
		if (gridVoltOk && gridRevPolEma < _IQ(50.0) && abs(gridPhase) < 40) {
			if (gridPllOkCnt < 50) {
				gridPllOkCnt++;
			} else {
				gridPllOk = 1;
				gridPllOkCnt = 0;
			}
		} else {
			gridPllOkCnt = 0;
		}
	} else {
		if (!gridVoltOk || gridRevPolEma > _IQ(50.0) || abs(gridPhase) > 40) {
			if (gridPllOkCnt < 3) {
				gridPllOkCnt++;
			} else {
				gridPllOk = 0;
				gridPllOkCnt = 0;
			}
		} else {
			gridPllOkCnt = 0;
		}
	}
}

void pfcTask() {
	if (acCharge &&
	  gridPllOk &&
	  gridVoltOk &&
	  !faultCode
	) {
		dcdcReq = 1;
		gridRelayReq = 1;
		if (dcdcSts && gridRelaySts)
			pfcRun = 1;
	} else {
		pfcRun = 0;
		ivRatio = 0;
		pfcTune = 0;
		dcdcReq = 0;
		gridRelayReq = 0;
	}
}

void gridRelayTask() {
	if (gridRelayWait == 0 || gridRelayWait > 10) {
		gridRelayWait = 0;
		gridRelaySts = gridRelayGpioSts();
	} else if (gridRelayWait > 0) {
		gridRelayWait++;
	}
	
	if (gridRelayReq &&
	  !gridRelayGpioSts() &&
	  !outRelayGpioSts() &&
	  !pfcRun &&
	  dcdcSts == DCDC_ON &&
	  vGridAmplEma + _IQ(20.0) < _IQmpy(vBat, dcdcRatio)
	) {
		gridRelayOn();
		gridRelayWait = 1;
	} else if (
	  !gridRelayReq &&
	  gridRelayGpioSts()
	) {
		gridRelayOff();
		gridRelayWait = 1;
	}
}

void acChargeTask() {
	_iq ivRatioTmp, ivRatioAdj;

	if (!pfcRun || faultCode) {
		ivRatio = 0;
		return;
	}

	buckPwm += 50;
	buckPwm = __min(buckPwm, PERIOD - 1);
	buckSetPwm(buckPwm);
	if (buckPwm != PERIOD - 1) {
		ivRatio = 0;
		return;
	}

	iChargeReq = _IQrsmpy(vBatAcStop - vBat, iChargeSet);
	iChargeReq = _IQrsmpy(iChargeReq, _IQ(10.0)); // current slope starts 0.1V below charging voltage
	iChargeReq = _IQsat(iChargeReq, iChargeSet, _IQdiv4(iChargeSet)); // minimum quarter charging current

	iChargeErr2 = iChargeErr1;
	iChargeErr1 = iChargeErr0;
	iChargeErr0 = iChargeReq - iBat;
	ivRatioAdj =
		_IQmpy(iChargeErr0 - iChargeErr1, acChargeKp) +
		_IQmpy(iChargeErr0, acChargeKi) +
		_IQmpy(iChargeErr0 - 2 * iChargeErr1 + iChargeErr2, acChargeKd);
	ivRatioAdj = _IQsat(ivRatioAdj, 32, -64); // 32/65536, -64/63356
	ivRatioTmp = _IQsat(ivRatio + ivRatioAdj, _IQ(0.13), _IQ(0));

	DINT;
	if (!pfcPause) {
		ivRatio = ivRatioTmp;
	} else {
		ivRatio = 0;
		pfcTune = 0;
		pfcPause--;
	}
	EINT;
}

void zeroOffsetCal() {
	uint32_t iCthZero = 0;
	uint32_t iCtlZero = 0;
	uint32_t vInvZero = 0;
	uint32_t iInvZero = 0;
	uint32_t iDcdcZero = 0;
	uint32_t i, timZero;

	timZero = timInv;
	for (i = 0; i < 19200; i++) {
		while (timZero == timInv);
		timZero = timInv;
		iCthZero += ADC_ICTH;
		iCtlZero += ADC_ICTL;
		vInvZero += ADC_VINV;
		iInvZero += ADC_IINV;
		iDcdcZero += ADC_IDCDC;
	}
//	vInvOffset = - (int16_t) (vInvZero / 19200);
	iInvOffset = - (int16_t) (iInvZero / 19200);
	iDcdcOffset = - (int16_t) (iDcdcZero / 19200);
//	if (vInvOffset < (-2048 - MAX_ZERO_OFFSET) || vInvOffset > (-2048 + MAX_ZERO_OFFSET)) faultCode = 57;
	if (iInvOffset < (-2048 - MAX_ZERO_OFFSET) || iInvOffset > (-2048 + MAX_ZERO_OFFSET)) faultCode = 57;
	if (iDcdcOffset < (-2048 - MAX_ZERO_OFFSET) || iDcdcOffset > (-2048 + MAX_ZERO_OFFSET)) faultCode = 57;
}

void powerSwTask() {
	uint16_t timShutdown;

	swOn = swOnSts();
	if (!swOn) {
		if (vGridRms < _IQ(20.0)) {
			timShutdown = timInv;
			pfcRun = 0;
			pfcPwmOff();
			while (timInv - timShutdown < 38400);
			smpsOff();
		}
	}
}

void acChargeSwitch() {
	if (faultCode) {
		acCharge = 0;
		return;
	}

	if (!acCharge) {
		if (vBat < vBatAcStart && swOn) {
			if (acStartCnt < 250) {
				acStartCnt++;
			} else {
				if (!faultCode) acCharge = 1;
				acStartCnt = 0;
			}
		} else {
			acStartCnt = 0;
		}
	} else {
		if (vBat > vBatAcStop + _IQ(0.2) || !swOn) {
			acCharge = 0;
			acStopCnt = 0;
		} else if (vBat > vBatAcStop) {
			if (acStopCnt < 3000) {
				acStopCnt++;
			} else {
				acCharge = 0;
				acStopCnt = 0;
			}
		} else {
			acStopCnt = 0;
		}
	}
}

void tempCheck() {
	int32_t fanPwm;

	// only when we have new samples
	timNow = timInv;
	if (timNow == timTempCheck) return;
	timTempCheck = timNow;

	tInvTmp += ADC_TINV;
	tMosfetTmp += ADC_TMOSFET;
	tBoostTmp += ADC_TBOOST;
	tSampleCnt++;
	if (tSampleCnt != 1 << 8) return;

	// may be inaccurate but I don't have exact specs of these NTCs
	tInv = ntc15kh4150(tInvTmp << 8);
	tMosfet = ntc15kh4150(tMosfetTmp << 8);
	tBoost = ntc15kh4150(tBoostTmp << 8);

	tSampleCnt = 0;
	tInvTmp = 0;
	tMosfetTmp = 0;
	tBoostTmp = 0;
	
	if (tInv > _IQ(70.0) ||	tMosfet > _IQ(70.0) || tBoost > _IQ(70.0)) {
		faultCode = 2;
		pfcRun = 0;
		acCharge = 0;
	}

	fanPwm = __max(8, (_IQint(tInv) - 40) * 3);
	fanPwm = __max(fanPwm, (_IQint(tMosfet) - 40) * 3);
	fanPwm = __max(fanPwm, (_IQint(tBoost) - 40) * 3);
	fanPwm = __min(fanPwm, 100);
	setFanPwm(fanPwm);
}

/*
void initI2c() {
	EALLOW;
	I2caRegs.I2CMDR.all = 0;
	SysCtrlRegs.PCLKCR0.bit.I2CAENCLK = 1;

	GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3;  // asynchronous
	GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;

	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1; // SDAA
	GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1; // SCLA

	I2caRegs.I2CSAR = 0x0050; // I2C eeprom address
	I2caRegs.I2CPSC.all = 6;
	I2caRegs.I2CCLKH = 5;
	I2caRegs.I2CCLKL = 10;
	I2caRegs.I2CMDR.all = 0x4420; // turn off reset, master mode
	I2caRegs.I2CFFTX.all = 0x6000;
	I2caRegs.I2CFFRX.all = 0x2000;
}

uint16_t i2cReadBlock(uint16_t memAddr, uint16_t cntWords, uint16_t *bin) {
	uint16_t i;
	uint32_t wait;

	for (wait = 0; wait < I2C_TIMEOUT; wait++) {
		if (!I2caRegs.I2CSTR.bit.BB) break;
	}
	if (wait == I2C_TIMEOUT) return 0;
	I2caRegs.I2CSAR = 0x50 | (memAddr >> 8 & 7);
	I2caRegs.I2CCNT = 1;
	I2caRegs.I2CDXR = memAddr & 0xff;
	I2caRegs.I2CMDR.all = 0x6620; // master, start, transmit
	for (wait = 0; wait < I2C_TIMEOUT; wait++) {
		if (!I2caRegs.I2CFFTX.bit.TXFFST) break;
	}
	if (wait == I2C_TIMEOUT) return 0;

	I2caRegs.I2CCNT = cntWords;
	I2caRegs.I2CMDR.all = 0x6c20; // master, start, (stop), receive
	i = 0;
	for (wait = 0; wait < I2C_TIMEOUT; wait++) {
		if (I2caRegs.I2CFFRX.bit.RXFFST > 0) {
			bin[i++] = I2caRegs.I2CDRR & 0xff;
			wait = 0;
			if (i == cntWords) break;
		}
	}
	if (i != cntWords) return 0;
	return 1;
}
*/

void startupMode() {
	timNow = waveCnt;

	// once every full wave
	if (timNow != timWave) {
		timWave = timNow;
		calcWaveData();
		gridCheck();
		startupCnt++;
		if (startupCnt > 50) mode = MODE_ACCHARGE;
	}
	vramTask();
	lcdTask();
	tempCheck();
}

void acChargeMode() {
	timNow = waveCnt;

	// once every full wave
	if (timNow != timWave) {
		timWave = timNow;
		calcWaveData();
		powerSwTask();
		gridCheck();
		acChargeSwitch();
		pfcTask();
		acChargeTask();
		gridRelayTask();
	}

	vramTask();
	lcdTask();
	dcdcTask();
	tempCheck();
	
	if (snapReady) {
		comPrintInt(snapIdx);
		comTx(';');
		comPrintInt(snapVInv);
		comTx(';');
		comPrintInt(snapIInv);
		comTx(';');
		comPrintInt(snapPwm);
		comTx(';');
		comPrintInt(snapIInvReq);
		comTx(';');
		comPrintInt(snapPfcTune);
		comTx(13);
		comTx(10);
		DINT;
		snapIdx++;
		if (snapIdx >= invPeriod) snapIdx = 0;
		snapReady = 0;
		EINT;
	}
}

void main() {
	// C2000Ware device support functions BEGIN
	memcpy(	(uint16_t *)&RamfuncsRunStart,
		(uint16_t *)&RamfuncsLoadStart,
		(unsigned long)&RamfuncsLoadSize);
	InitSysCtrl();
	CsmUnlock();
	InitFlash();

	DINT;
	InitPieCtrl();
	IER = 0x0000;
	IFR = 0x0000;
	InitPieVectTable();
	InitLinaGpio();
	InitAdc();
	AdcOffsetSelfCal();
	// C2000Ware device support functions END

	comSetup();
	adcSetup();
	gpioSetup();

	EALLOW;
	PieVectTable.ADCINT1 = &adcIsr;
	PieCtrlRegs.PIEIFR1.bit.INTx1 = 0;
	PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
	IER |= M_INT1;
	EINT;
	ERTM;
	EDIS;

	epwmSetup();

	memset(vram, 0, sizeof(vram));

	EALLOW;
	CpuTimer0Regs.PRD.all = 0xffffffff; // used for delay
	CpuTimer0Regs.TIM.all = 0xffffffff;
	CpuTimer0Regs.TCR.all = 0x4000;
	EDIS;

	delay(DSP_CLK / 2);
	zeroOffsetCal();

	DINT;
	snapIdx = 0;
	snapReady = 0;
	EINT;
	while (1) {
		switch (mode) {
		case MODE_STARTUP:
			startupMode();
			break;
		case MODE_ACCHARGE:
			acChargeMode();
			break;
		}
	}
}
