/*
 * =============================================================================
 *
 *       Filename:  mpguino.cpp
 *
 *    Description:  MPGuino - open source fuel consumption tracking system
 *
 *        Version:  1.0
 *        Created:  12/27/2014 09:43:50 PM
 *       Compiler:  gcc
 *
 *        License:  gpl 2.0
 *      Additions:  mass production use rights reserved by opengauge.org
 *                  personal use is perfectly fine
 *                  no warranties expressed or implied
 *         Thanks:  Special thanks to the good folks at ecomodder.com, 
 *                  ardunio.cc, avrfreaks.net, cadsoft.de, atmel.com, and all 
 *                  the folks who donate their time and resources and share 
 *                  their experiences freely
 *
 * =============================================================================
 */

/* Program overview
 set up timer hardware
 set up interrupts
 set up system constants
 load system settings from EEPROM
 set up LCD hardware
 (if configured) set up serial UART output

 create accumulators for raw speed/injector data

 mainloop{
 update instantaneous trip, current trip, tank trip, any other setup trip accumulators with raw data accumulators
 reset raw data accumulators
 (if configured) transmit instantaneous trip accumulators
 display computations
 scan for key presses and perform their function (change screen, reset a trip, goto setup, edit screen, restore trips, etc)
 pause for remainder of 1/2 second
 }

*/

#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include "configure.h"

#ifdef ArduinoMega2560
extern "C" {
	void __vector_32()
	{
	}
}
#endif

typedef void (*pFunc)(void); // type for display function pointers
#ifdef useBuffering
typedef void (*qFunc)(uint8_t); // type for buffer function pointers
#endif

const uint8_t loopsPerSecond = 2; // how many times will we try and loop in a second
const uint8_t samplesPerSecond = 2; // how many times will we try to sample the ADC output in a second

#ifdef use20MHz
const uint8_t processorSpeed = 20; // processor speed in megahertz
#ifdef useLegacyLCD
const uint8_t lcdDelayTable[4] PROGMEM = { 1, 2, 51, 185 }; // LCD delay values, using ADC freewheeling and a divider of 128
#endif
#else
const uint8_t processorSpeed = 16; // processor speed in megahertz
#ifdef useLegacyLCD
const uint8_t lcdDelayTable[4] PROGMEM = { 1, 1, 41, 148 }; // LCD delay values, using ADC freewheeling and a divider of 128
#endif
#endif

const unsigned long t2CyclesPerSecond = (unsigned long)(processorSpeed * 15625ul); // (processorSpeed * 1000000 / (timer 2 prescaler))
const unsigned long loopSystemLength = (t2CyclesPerSecond / (loopsPerSecond * 10)); // divided by 10 to keep cpu loading value from overflowing
const unsigned int loopTickLength = (unsigned int)(t2CyclesPerSecond / (loopsPerSecond * 256ul));
const unsigned int sampleTickLength  = (unsigned int)(t2CyclesPerSecond / (samplesPerSecond * 256ul));
const unsigned int myubbr = (unsigned int)(processorSpeed * 625ul / 96ul - 1);
const unsigned int keyDelay = (unsigned int)(t2CyclesPerSecond / 256ul);
const unsigned int keyShortDelay = keyDelay - (5 * keyDelay / 100); // wait 5/100 of a second before accepting button presses
const unsigned int vssResetDelay = loopTickLength; // VSS pulse timeout is the same as the loop system length

const uint8_t holdDelay = loopsPerSecond * 2 - 1;

const unsigned int delay1500ms = (int)(1500ul * t2CyclesPerSecond / 256000ul);
const unsigned int delay0005ms = (int)(5ul * t2CyclesPerSecond / 256000ul);

union union_16
{
	unsigned int ui;
	uint8_t u8[2];
};

union union_64
{
	unsigned long long ull;
	unsigned long ul[2];
	unsigned int ui[4];
	uint8_t u8[8];
};

#ifdef useChryslerMAPCorrection
void readMAP(void);
#endif
#ifdef useIsqrt
unsigned int iSqrt(unsigned int n);
#endif
void updateVSS(unsigned long cycle);
void initStatusLine(void);
void execStatusLine(void);
void clrEOL(void);
void gotoXY(uint8_t x, uint8_t y);
void blinkFlash(const char * str, uint8_t condition);
const char * findStr(const char * str, uint8_t strIdx);
void printStr(const char * str, uint8_t strIdx);
void printFlash(const char * str);
void print(char * str);
void charOut(uint8_t chr);
void loadCGRAM(const char * c);
#ifdef useBigNumberDisplay
void displayBigStatus(uint8_t dIdx, const char * str);
uint8_t fedSelect(uint8_t dIdx);
void displayBigNumber(char * str);
#endif
#ifdef useBigTimeDisplay
void displayBigTime(char * val, uint8_t b);
#endif
#ifdef useSavedTrips
unsigned int getBaseTripPointer(uint8_t tripPos);
#endif
unsigned long SWEET64(const uint8_t * sched, uint8_t tripIdx);
#ifdef useSerialDebugOutput
void pushHexNybble(uint8_t val);
void pushHexByte(uint8_t val);
void pushHexWord(unsigned int val);
void pushHexDWord(unsigned long val);
#endif
void copy64(union union_64 * an, union union_64 * ann);
void tripVarLoad64(union union_64 * an, uint8_t tripIdx, uint8_t dataIdx);
void EEPROMsave64(union union_64 * an, uint8_t dataIdx);
void init64(union union_64 * an, unsigned long dWordL);
void swap64(union union_64 * an, union union_64 * ann);
void shr64(union union_64 * an);
void shl64(union union_64 * an);
void add64(union union_64 * an, union union_64 * ann, uint8_t mode);
#ifndef useSWEET64multDiv
void mul64(union union_64 * an, union union_64 * ann);
void div64(union union_64 * an, union union_64 * ann);
#endif
uint8_t zeroTest64(union union_64 * an);
uint8_t ltOrEtest64(union union_64 * an, union union_64 * ann);
uint8_t lsbTest64(union union_64 * an);
uint8_t msbTest64(union union_64 * an);
char * doFormat(uint8_t tripIdx, uint8_t dispPos);
unsigned long doCalculate(uint8_t calcIdx, uint8_t tripIdx);
char * doFormat(uint8_t tripIdx, uint8_t calcIdx, uint8_t dispPos);
char * format(unsigned long num, uint8_t ndp);
char * format64(const uint8_t * prgmPtr, unsigned long num, char * str, uint8_t ndp);
unsigned long rformat(void);
unsigned long convertTime(unsigned long * an);
#ifdef useWindowFilter
void resetWindowFilter(void);
#endif
void initGuino(void);
void delay2(unsigned int ms);
#ifdef useSerialPortDataLogging
void doOutputDataLog(void);
void simpletx(char * str);
#endif
#ifdef useSerialPort
void pushSerialCharacter(uint8_t chr);
#ifdef useBufferedSerialPort
void serialTransmitEnable(void);
void serialTransmitDisable(void);
void serialTransmitByte(uint8_t s);
#endif
#endif
void doCursorMoveRelative(uint8_t i, uint8_t j);
void doCursorMoveAbsolute(uint8_t i, uint8_t j);
void doRefreshDisplay(void);
void doNothing(void);
void doNothing2(uint8_t s);
void noSupport(void);
void displayMainScreenFunction(uint8_t readingIdx, uint8_t k, uint8_t functBlink, uint8_t tripBlink);
void writeCGRAMlabelChar(uint8_t cgChar, uint8_t functIdx, uint8_t tripIdx, uint8_t functBlink, uint8_t tripBlink);
void doCursorUpdateMain(void);
void doMainScreenDisplay(void);
void doNextBright(void);
void doLongGoLeft(void);
void doLongGoRight(void);
void doTripResetTank(void);
void doTripResetCurrent(void);
#ifdef useBarGraph
void clearBGplot(uint8_t mode);
uint8_t bgPlotConvert(uint8_t coord);
void bgPlot(uint8_t idx, uint8_t lowerPoint, uint8_t upperPoint, uint8_t mode);
void bgOutputPlot(uint8_t idx, uint8_t yIdx);
uint8_t bgConvert(unsigned long v, unsigned long ll, unsigned long d);
void formatBarGraph(uint8_t bgSize, uint8_t slotIdx, unsigned long centerVal, unsigned long topLimit);
void displayBarGraphLine(uint8_t lineNum, uint8_t tripIdx, uint8_t tripCalcIdx);
void displayBarGraph(uint8_t trip1idx, uint8_t trip1CalcIdx, uint8_t trip2idx, uint8_t trip2CalcIdx);
#endif
#ifdef useBarFuelEconVsSpeed
void doCursorUpdateBarFEvS(void);
void doBarFEvSdisplay(void);
void doResetBarFEvS(void);
#endif
#ifdef useBarFuelEconVsTime
void doResetBarFEvT(void);
void doCursorUpdateBarFEvT(void);
void doBarFEvTdisplay(void);
#endif
#ifdef useCPUreading
void doDisplaySystemInfo(void);
void displayCPUutil(void);
void doShowCPU(void);
#endif
#ifdef useBenchMark
void doBenchMark(void);
#endif
void doCursorUpdateSetting(void);
void doSettingEditDisplay(void);
void doGoSettingsEdit(void);
void doReturnToMain(void);
void doParamEditDisplay(void);
void doParamExit(void);
void doParamSave(void);
void doGoParamEdit(void);
void generalMenuLevelReturn(const char * s, uint8_t newMenuLevel);
void printStatusMessage(const char * s);
void doParamFindLeft(void);
void doParamFindRight(void);
void doParamStoreMax(void);
void doParamStoreMin(void);
void doParamRevert(void);
void doParamStoreNumber(unsigned long v);
void doParamReformat(void);
void doParamChangeDigit(void);
#ifdef useEEPROMviewer
void doEEPROMviewDisplay(void);
void goEEPROMview(void);
#endif
#ifdef useBigFE
void doCursorUpdateBigFEscreen(void);
void doBigFEdisplay(void);
#endif
#ifdef useBigDTE
void doCursorUpdateBigDTEscreen(void);
void doBigDTEdisplay(void);
#endif
#ifdef useBigTTE
void doCursorUpdateBigTTEscreen(void);
void doBigTTEdisplay(void);
#endif
#ifdef useClock // Clock support section
void doCursorUpdateSystemTimeScreen(void);
void doDisplaySystemTime(void);
void doGoEditSystemTime(void);
void doEditSystemTimeDisplay(void);
void doEditSystemTimeChangeDigit(void);
void doEditSystemTimeSave(void);
void doEditSystemTimeCancel(void);
#endif
#ifdef useSavedTrips // Trip save/restore/raw data view support section
void doCursorUpdateTripShow(void);
void doTripSaveDisplay(void);
void doTripShowDisplay(void);
void doGoTripTank(void);
void doGoTripCurrent(void);
void goSavedTrip(uint8_t tripSlot);
void doTripSelect(void);
void doTripLongSelect(void);
void goTripSelect(uint8_t pressFlag);
void doTripSave(uint8_t tripIdx);
void doTripLoad(uint8_t tripIdx);
uint8_t doTripAutoAction(uint8_t taaMode);
void doTripReset(uint8_t tripIdx);
void doTripPrintType(uint8_t tripIdx);
void doTripBumpSlot(void);
void doTripShowCancel(void);
#endif
#ifdef useScreenEditor // Programmable main display screen edit support section
void doCursorUpdateScreenEdit(void);
void doScreenEditDisplay(void);
void doGoScreenEdit(void);
void doScreenEditReturnToMain(void);
void doScreenEditRevert(void);
void doScreenEditBump(void);
void doSaveScreen(void);
#endif
uint8_t loadParams(void);
uint8_t eepromWriteVal(unsigned int eePtr, unsigned long val);
unsigned long eepromReadVal(unsigned int eePtr);
unsigned int eepromGetAddress(unsigned int eePtr);
void callFuncPointer(const uint8_t * funcIdx);
unsigned long cycles2(void);
unsigned long findCycleLength(unsigned long lastCycle, unsigned long thisCycle);
int main(void);

const uint8_t idxDoNothing =				0;
const uint8_t idxNoSupport =				idxDoNothing + 1;
const uint8_t idxDoCursorUpdateMain =			idxNoSupport + 1;
const uint8_t idxDoCursorUpdateSetting =		idxDoCursorUpdateMain + 1;
const uint8_t idxDoMainScreenDisplay =			idxDoCursorUpdateSetting + 1;
const uint8_t idxDoSettingEditDisplay =			idxDoMainScreenDisplay + 1;
const uint8_t idxDoParamEditDisplay =			idxDoSettingEditDisplay + 1;
const uint8_t idxDoGoSettingsEdit = 			idxDoParamEditDisplay + 1;
const uint8_t idxDoNextBright =				idxDoGoSettingsEdit + 1;
const uint8_t idxDoTripResetCurrent =			idxDoNextBright + 1;
const uint8_t idxDoLongGoRight =			idxDoTripResetCurrent + 1;
const uint8_t idxDoTripResetTank =			idxDoLongGoRight + 1;
const uint8_t idxDoLongGoLeft =				idxDoTripResetTank + 1;
const uint8_t idxDoReturnToMain =			idxDoLongGoLeft + 1;
const uint8_t idxDoGoParamEdit =			idxDoReturnToMain + 1;
const uint8_t idxDoParamFindRight =			idxDoGoParamEdit + 1;
const uint8_t idxDoParamExit =				idxDoParamFindRight + 1;
const uint8_t idxDoParamFindLeft =			idxDoParamExit + 1;
const uint8_t idxDoParamChangeDigit =			idxDoParamFindLeft + 1;
const uint8_t idxDoParamSave =				idxDoParamChangeDigit + 1;
const uint8_t idxDoParamStoreMin =			idxDoParamSave + 1;
const uint8_t idxDoParamStoreMax =			idxDoParamStoreMin + 1;
const uint8_t idxDoParamRevert =			idxDoParamStoreMax + 1;
#define nextAllowedValue idxDoParamRevert
#ifdef useCPUreading
const uint8_t idxDoDisplaySystemInfo =			nextAllowedValue + 1;
const uint8_t idxDoShowCPU =				idxDoDisplaySystemInfo + 1;
#define nextAllowedValue idxDoShowCPU
#endif
#ifdef useBigFE
const uint8_t idxDoCursorUpdateBigFEscreen =		nextAllowedValue + 1;
const uint8_t idxDoBigFEdisplay =			idxDoCursorUpdateBigFEscreen + 1;
#define nextAllowedValue idxDoBigFEdisplay
#endif
#ifdef useBigDTE
const uint8_t idxDoCursorUpdateBigDTEscreen =		nextAllowedValue + 1;
const uint8_t idxDoBigDTEdisplay =			idxDoCursorUpdateBigDTEscreen + 1;
#define nextAllowedValue idxDoBigDTEdisplay
#endif
#ifdef useBigTTE
const uint8_t idxDoCursorUpdateBigTTEscreen =		nextAllowedValue + 1;
const uint8_t idxDoBigTTEdisplay =			idxDoCursorUpdateBigTTEscreen + 1;
#define nextAllowedValue idxDoBigTTEdisplay
#endif
#ifdef useClock
const uint8_t idxDoCursorUpdateSystemTimeScreen =	nextAllowedValue + 1;
const uint8_t idxDoDisplaySystemTime =			idxDoCursorUpdateSystemTimeScreen + 1;
const uint8_t idxDoGoEditSystemTime =			idxDoDisplaySystemTime + 1;
const uint8_t idxDoEditSystemTimeDisplay =		idxDoGoEditSystemTime + 1;
const uint8_t idxDoEditSystemTimeCancel =		idxDoEditSystemTimeDisplay + 1;
const uint8_t idxDoEditSystemTimeChangeDigit =		idxDoEditSystemTimeCancel + 1;
const uint8_t idxDoEditSystemTimeSave =			idxDoEditSystemTimeChangeDigit + 1;
#define nextAllowedValue idxDoEditSystemTimeSave
#endif
#ifdef useSavedTrips
const uint8_t idxDoCursorUpdateTripShow =		nextAllowedValue + 1;
const uint8_t idxDoTripSaveDisplay =			idxDoCursorUpdateTripShow + 1;
const uint8_t idxDoTripShowDisplay =			idxDoTripSaveDisplay + 1;
const uint8_t idxDoGoTripCurrent =			idxDoTripShowDisplay + 1;
const uint8_t idxDoGoTripTank =				idxDoGoTripCurrent + 1;
const uint8_t idxDoTripBumpSlot =			idxDoGoTripTank + 1;
const uint8_t idxDoTripSelect =				idxDoTripBumpSlot + 1;
const uint8_t idxDoTripLongSelect =			idxDoTripSelect + 1;
const uint8_t idxDoTripShowCancel =			idxDoTripLongSelect + 1;
#define nextAllowedValue idxDoTripShowCancel
#endif
#ifdef useScreenEditor
const uint8_t idxDoScreenEditDisplay =			nextAllowedValue + 1;
const uint8_t idxDoGoScreenEdit =			idxDoScreenEditDisplay + 1;
const uint8_t idxDoScreenEditReturnToMain =		idxDoGoScreenEdit + 1;
const uint8_t idxDoScreenEditRevert =			idxDoScreenEditReturnToMain + 1;
const uint8_t idxDoSaveScreen =				idxDoScreenEditRevert + 1;
const uint8_t idxDoScreenEditBump =			idxDoSaveScreen + 1;
const uint8_t idxDoCursorUpdateScreenEdit =		idxDoScreenEditBump + 1;
#define nextAllowedValue idxDoCursorUpdateScreenEdit
#endif
#ifdef useBarFuelEconVsTime
const uint8_t idxDoCursorUpdateBarFEvT =		nextAllowedValue + 1;
const uint8_t idxDoBarFEvTdisplay =			idxDoCursorUpdateBarFEvT + 1;
#define nextAllowedValue idxDoBarFEvTdisplay
#endif
#ifdef useBarFuelEconVsSpeed
const uint8_t idxDoCursorUpdateBarFEvS =		nextAllowedValue + 1;
const uint8_t idxDoBarFEvSdisplay =			idxDoCursorUpdateBarFEvS + 1;
const uint8_t idxDoResetBarFEvS =			idxDoBarFEvSdisplay + 1;
#define nextAllowedValue idxDoResetBarFEvS
#endif
#ifdef useBenchMark
const uint8_t idxDoBenchMark =				nextAllowedValue + 1;
#define nextAllowedValue idxDoBenchMark
#endif
#ifdef useEEPROMviewer
const uint8_t idxDoEEPROMviewDisplay =			nextAllowedValue + 1;
const uint8_t idxGoEEPROMview =				idxDoEEPROMviewDisplay + 1;
#define nextAllowedValue idxGoEEPROMview
#endif

const uint8_t rvLength = 8;

const uint8_t rvVSSpulseIdx = 0; 		// from the speedo
const uint8_t rvInjPulseIdx = 1; 		// rpm
const uint8_t rvVSScycleIdx = 2; 		// time that the vehicle has spent moving
const uint8_t rvInjCycleIdx = 4; 		// engine run time since this class was reset
const uint8_t rvInjOpenCycleIdx = 6; 		// time that the fuel injector has been open

const uint8_t tFuelUsed =		0;
const uint8_t tFuelRate =		tFuelUsed + 1;
const uint8_t tEngineRunTime =		tFuelRate + 1;
const uint8_t tTimeToEmpty =		tEngineRunTime + 1;
const uint8_t tDistance =		tTimeToEmpty + 1;
const uint8_t tSpeed =			tDistance + 1;
const uint8_t tMotionTime =		tSpeed + 1;
const uint8_t tFuelEcon =		tMotionTime + 1;
const uint8_t tRemainingFuel =		tFuelEcon + 1;
const uint8_t tDistanceToEmpty =	tRemainingFuel + 1;
const uint8_t tEngineSpeed =		tDistanceToEmpty + 1;
const uint8_t tInjectorOpenTime =	tEngineSpeed + 1;
const uint8_t tInjectorTotalTime =	tInjectorOpenTime + 1;
const uint8_t tVSStotalTime =		tInjectorTotalTime + 1;
const uint8_t tInjectorPulseCount =	tVSStotalTime + 1;
const uint8_t tVSSpulseCount =		tInjectorPulseCount + 1;
#define nextAllowedValue tVSSpulseCount
#ifdef useFuelCost
const uint8_t tFuelCost =		nextAllowedValue + 1;
const uint8_t tFuelRateCost =		tFuelCost + 1;
const uint8_t tFuelCostPerDistance =	tFuelRateCost + 1;
const uint8_t tDistancePerFuelCost =	tFuelCostPerDistance + 1;
const uint8_t tFuelCostRemaining =	tDistancePerFuelCost + 1;
#define nextAllowedValue tFuelCostRemaining
#endif
const uint8_t dfMaxValCount =		nextAllowedValue + 1;
#ifdef useAnalogRead
#ifdef TinkerkitLCDmodule
const uint8_t tAnalogChannel0 =		nextAllowedValue + 1;
const uint8_t tAnalogChannel1 =		tAnalogChannel0 + 1;
const uint8_t tAnalogChannel2 =		tAnalogChannel1 + 1;
#define nextAllowedValue tAnalogChannel2
#else
const uint8_t tAnalogChannel0 =		nextAllowedValue + 1;
const uint8_t tAnalogChannel1 =		tAnalogChannel0 + 1;
#define nextAllowedValue tAnalogChannel1
#ifdef useAnalogButtons
const uint8_t tAnalogChannel2 =		nextAllowedValue + 1;
const uint8_t tAnalogChannel3 =		tAnalogChannel2 + 1;
const uint8_t tAnalogChannel4 =		tAnalogChannel3 + 1;
#define nextAllowedValue tAnalogChannel4
#endif
#endif
const uint8_t dfMaxValAnalogCount = nextAllowedValue + 1;
#endif
#ifdef useChryslerMAPCorrection
const uint8_t tPressureChannel0 =	nextAllowedValue + 1;
const uint8_t tPressureChannel1 =	tPressureChannel0 + 1;
const uint8_t tPressureChannel2 =	tPressureChannel1 + 1;
const uint8_t tPressureChannel3 =	tPressureChannel2 + 1;
const uint8_t tCorrectionFactor =	tPressureChannel3 + 1;
#define nextAllowedValue tCorrectionFactor
const uint8_t dfMaxValMAPCount = nextAllowedValue + 1;
#endif

const uint8_t dfBitShift = 5;
const uint8_t dfTripMask = 0xE0;
const uint8_t dfValMask = 0x1F;
const uint8_t dfMaxTripCount = 6;

const uint8_t calcDecimalPoints[] PROGMEM = {
	2,	// fuel used
	2,	// fuel rate
	0,	// engine run time
	0,	// time to empty
	1,	// distance travelled
	1,	// average speed
	0,	// time in motion
	2,	// fuel economy
	2,	// remaining fuel
	1,	// remaining distance
	0,	// engine speed
	0,	// fuel used, in microseconds
	0,	// engine run time, in microseconds
	0,	// time in motion, in microseconds
	0,	// injector pulses
	0,	// VSS pulses
#ifdef useFuelCost
	2,	// fuel cost
	2,	// fuel cost rate
	2,	// fuel cost per unit distance
	1,	// distance per unit fuel cost
	2,	// fuel cost remaining
#endif
#ifdef useAnalogRead
#ifdef TinkerkitLCDmodule
	3,	// voltage
	3,	// voltage
	3,	// voltage
#else
	3,	// voltage
	3,	// voltage
#ifdef useAnalogButtons
	3,	// voltage
	3,	// voltage
	3,	// voltage
#endif
#endif
#endif
#ifdef useChryslerMAPCorrection
	2,	// pressure
	2,	// pressure
	2,	// pressure
	2,	// pressure
	3,	// correction factor
#endif
};

const uint8_t dfMaxValDisplayCount = (sizeof(calcDecimalPoints) / sizeof(uint8_t));

#ifdef useSavedTrips
const uint8_t tripSaveSlotCount = 10;
const uint8_t tripListLength = rvLength;
const uint8_t tripListSize = tripListLength + 2; // (uint8_t signature) plus (uint32_t timestamp)
const uint8_t tripListSigPointer = tripListSize - 1;
const uint8_t eepromTripListSize = tripListLength * 4 + 5; // trip list length plus (uint8_t signature) plus (uint32_t timestamp)
#endif

#ifdef useBarGraph
const uint8_t bgDataSize = 15;
#endif

const uint8_t rawIdx = 				0;
const uint8_t instantIdx = 			rawIdx + 1;
const uint8_t currentIdx = 			instantIdx + 1;
const uint8_t tankIdx = 			currentIdx + 1;
#define nextAllowedValue tankIdx
#ifdef trackIdleEOCdata
const uint8_t rawIdleIdx = 			nextAllowedValue + 1;
const uint8_t eocIdleInstantIdx = 		rawIdleIdx + 1;
const uint8_t eocIdleCurrentIdx = 		eocIdleInstantIdx + 1;
const uint8_t eocIdleTankIdx = 			eocIdleCurrentIdx + 1;
#define nextAllowedValue eocIdleTankIdx
#endif
#ifdef useBarFuelEconVsTime
const uint8_t periodIdx = 			nextAllowedValue + 1;
#define nextAllowedValue periodIdx
#endif
#ifdef useBarFuelEconVsSpeed
const uint8_t FEvsSpeedIdx = 			nextAllowedValue + 1;
#define nextAllowedValue (FEvsSpeedIdx + bgDataSize - 1)
#endif
#ifdef useCoastDownCalculator
const uint8_t thisCoastDownIdx = 		nextAllowedValue + 1;
const uint8_t lastCoastDownIdx = 		thisCoastDownIdx + 1;
#define nextAllowedValue lastCoastDownIdx
#endif
#ifdef useWindowFilter
const uint8_t windowFilterSize = 4;
const uint8_t windowFilterElemIdx = 		nextAllowedValue + 1;
const uint8_t windowFilterSumIdx = 		windowFilterElemIdx + windowFilterSize;
#define nextAllowedValue windowFilterSumIdx
#endif
const uint8_t tripSlotCount = 			nextAllowedValue + 1;

const uint8_t displayPageCount = 9		// count of base number of data screens
#ifdef trackIdleEOCdata
	+ 3									// count of Idle/EOC tracking data screens
#endif
#ifdef useAnalogRead
	+ 1									// count of analog voltage data screen
#endif
#ifdef useChryslerMAPCorrection
	+ 1									// count of Chrysler MAP-specific data screen
#endif
;
const uint8_t displayFormatSize = displayPageCount * 4;

const uint8_t tripScreenIdxBase = 6
#ifdef useChryslerMAPCorrection
	+ 1
#endif
#ifdef useAnalogRead
	+ 1
#endif
#ifdef trackIdleEOCdata
	+ 1
#endif
;

#ifdef useLegacyLCD
#ifdef useLegacyLCDinvertedBrightness
const uint8_t brightness[] PROGMEM = { 255, 170, 85, 0 }; //middle button cycles through these brightness settings
#else
const uint8_t brightness[] PROGMEM = { 0, 41, 84, 128 }; //middle button cycles through these brightness settings
#endif
const char brightString[] PROGMEM = {
	" OFF\0"
	" LOW\0"
	" MED\0"
	"HIGH\0"
};
const uint8_t brightnessLength = (sizeof(brightness) / sizeof(uint8_t) ); // size of brightness table
#endif
#ifdef useParallaxLCD
const uint8_t brightnessLength = 2;
const char brightString[] PROGMEM = {
	" OFF\0"
	"  ON\0"
};
#endif

#ifdef TinkerkitLCDmodule
const uint8_t vssBit = 			(1 << PINB1);
#else
const uint8_t vssBit = 			(1 << PINC0);
#endif

#ifdef useLegacyButtons
#ifdef ArduinoMega2560
const uint8_t lbuttonBit = 		(1 << PINK3);
const uint8_t mbuttonBit = 		(1 << PINK4);
const uint8_t rbuttonBit = 		(1 << PINK5);
const uint8_t longButtonBit = 		(1 << PINK6); // PINK6 isn't being used for anything right now
#else
const uint8_t lbuttonBit = 		(1 << PINC3);
const uint8_t mbuttonBit = 		(1 << PINC4);
const uint8_t rbuttonBit = 		(1 << PINC5);
const uint8_t longButtonBit = 		(1 << PINC6); // PINC6 is used as the RESET pin, so this value is safe to use for long-press signalling
#endif

const uint8_t buttonsUp = 		rbuttonBit | mbuttonBit | lbuttonBit;

const uint8_t btnShortPressL = 		rbuttonBit | mbuttonBit;
const uint8_t btnShortPressC = 		rbuttonBit | lbuttonBit;
const uint8_t btnShortPressCL = 	rbuttonBit;
const uint8_t btnShortPressR = 		mbuttonBit | lbuttonBit;
const uint8_t btnShortPressRL = 	mbuttonBit;
const uint8_t btnShortPressRC = 	lbuttonBit;
const uint8_t btnShortPressRCL = 	0;
#endif

#ifdef useAnalogButtons
const uint8_t longButtonBit = 		0b10000000;

const uint8_t buttonsUp = 		0;
const uint8_t btnShortPressL = 		buttonsUp + 1;
const uint8_t btnShortPressC = 		btnShortPressL + 1;
const uint8_t btnShortPressR = 		btnShortPressC + 2;
const uint8_t btnShortPress1 = 		btnShortPressR + 4;
const uint8_t btnShortPress1L = 	btnShortPress1 + 1;
const uint8_t btnShortPress1C = 	btnShortPress1L + 1;
const uint8_t btnShortPress1CL = 	btnShortPress1C + 1;
const uint8_t btnShortPress1R = 	btnShortPress1CL + 1;
const uint8_t btnShortPress1RL = 	btnShortPress1R + 1;
const uint8_t btnShortPress1RC = 	btnShortPress1RL + 1;
const uint8_t btnShortPress1RCL = 	btnShortPress1RC + 1;
const uint8_t btnShortPress2 = 		btnShortPress1RCL + 1;
const uint8_t btnShortPress2L = 	btnShortPress2 + 1;
const uint8_t btnShortPress2C = 	btnShortPress2L + 1;
const uint8_t btnShortPress2CL = 	btnShortPress2C + 1;
const uint8_t btnShortPress2R = 	btnShortPress2CL + 1;
const uint8_t btnShortPress2RL = 	btnShortPress2R + 1;
const uint8_t btnShortPress2RC = 	btnShortPress2RL + 1;
const uint8_t btnShortPress2RCL = 	btnShortPress2RC + 1;
const uint8_t btnShortPress21 = 	btnShortPress2RCL + 1;
const uint8_t btnShortPress21L = 	btnShortPress21 + 1;
const uint8_t btnShortPress21C = 	btnShortPress21L + 1;
const uint8_t btnShortPress21CL = 	btnShortPress21C + 1;
const uint8_t btnShortPress21R = 	btnShortPress21CL + 1;
const uint8_t btnShortPress21RL = 	btnShortPress21R + 1;
const uint8_t btnShortPress21RC = 	btnShortPress21RL + 1;
const uint8_t btnShortPress21RCL = 	btnShortPress21RC + 1;

#ifdef useAnalogMuxButtons
const uint8_t btnShortPressCL = 	btnShortPressC + 1;
const uint8_t btnShortPressRL = 	btnShortPressR + 1;
const uint8_t btnShortPressRC = 	btnShortPressRL + 1;
const uint8_t btnShortPressRCL = 	btnShortPressRC + 1;
#endif

#ifdef useParallax5PositionSwitch
const uint8_t btnShortPressCL = 	btnShortPress1L;
const uint8_t btnShortPressRL = 	btnShortPress1;
const uint8_t btnShortPressRC = 	btnShortPress1R;
const uint8_t btnShortPressRCL = 	btnShortPress2;
#endif
#endif

const uint8_t btnLongPressL = 		longButtonBit | btnShortPressL;
const uint8_t btnLongPressC = 		longButtonBit | btnShortPressC;
const uint8_t btnLongPressCL = 		longButtonBit | btnShortPressCL;
const uint8_t btnLongPressR = 		longButtonBit | btnShortPressR;
const uint8_t btnLongPressRL = 		longButtonBit | btnShortPressRL;
const uint8_t btnLongPressRC = 		longButtonBit | btnShortPressRC;
const uint8_t btnLongPressRCL = 	longButtonBit | btnShortPressRCL;

#ifdef useAnalogButtons
const uint8_t btnLongPress1 = 		longButtonBit | btnShortPress1;
const uint8_t btnLongPress1L = 		longButtonBit | btnShortPress1L;
const uint8_t btnLongPress1C = 		longButtonBit | btnShortPress1C;
const uint8_t btnLongPress1CL = 	longButtonBit | btnShortPress1CL;
const uint8_t btnLongPress1R = 		longButtonBit | btnShortPress1R;
const uint8_t btnLongPress1RL = 	longButtonBit | btnShortPress1RL;
const uint8_t btnLongPress1RC = 	longButtonBit | btnShortPress1RC;
const uint8_t btnLongPress1RCL = 	longButtonBit | btnShortPress1RCL;
const uint8_t btnLongPress2 = 		longButtonBit | btnShortPress2;
const uint8_t btnLongPress2L = 		longButtonBit | btnShortPress2L;
const uint8_t btnLongPress2C = 		longButtonBit | btnShortPress2C;
const uint8_t btnLongPress2CL = 	longButtonBit | btnShortPress2CL;
const uint8_t btnLongPress2R = 		longButtonBit | btnShortPress2R;
const uint8_t btnLongPress2RL = 	longButtonBit | btnShortPress2RL;
const uint8_t btnLongPress2RC = 	longButtonBit | btnShortPress2RC;
const uint8_t btnLongPress2RCL = 	longButtonBit | btnShortPress2RCL;
const uint8_t btnLongPress21 = 		longButtonBit | btnShortPress21;
const uint8_t btnLongPress21L = 	longButtonBit | btnShortPress21L;
const uint8_t btnLongPress21C = 	longButtonBit | btnShortPress21C;
const uint8_t btnLongPress21CL = 	longButtonBit | btnShortPress21CL;
const uint8_t btnLongPress21R = 	longButtonBit | btnShortPress21R;
const uint8_t btnLongPress21RL = 	longButtonBit | btnShortPress21RL;
const uint8_t btnLongPress21RC = 	longButtonBit | btnShortPress21RC;
const uint8_t btnLongPress21RCL = 	longButtonBit | btnShortPress21RCL;
#endif

const uint8_t dispRaw = 		0b10000000;
const uint8_t dispFE = 			0b01000000;
const uint8_t dispDTE = 		0b00100000;

const uint8_t guinosig = 		0b10110111;

const uint8_t dirtySysTick = 		0b00001000;
const uint8_t dirtyInjOpenRead = 	0b00000100;
const uint8_t dirtyGoodInj = 		0b00000010;
const uint8_t dirtyGoodVSS = 		0b00000001;

const uint8_t tcDoDelay = 		0b10000000;
const uint8_t tcStartLoop = 		0b01000000;
#ifdef useAnalogRead
const uint8_t tcResetADC = 		0b00100000;
#endif
const uint8_t tcFallAsleep = 		0b00010000;
const uint8_t tcWakeUp = 		0b00001000;
const uint8_t tcLCDdelay = 		0b00000100;
const uint8_t tcDisplayDelay = 		0b00000010;

const uint8_t tsLoopExec = 		0b10000000;
const uint8_t tsButtonsUp = 		0b01000000;
const uint8_t tsAwake = 		0b00100000;
const uint8_t tsFellAsleep = 		0b00010000;
const uint8_t tsMarkLoop = 		0b00001000;
const uint8_t tsButtonRead = 		0b00000100;
const uint8_t tsDisplayDelay = 		0b00000010;
#ifdef useAnalogRead
const uint8_t tsADCnormal = 		0b00000001;
#endif

#ifdef useLegacyLCD
#ifdef ArduinoMega2560
const uint8_t lcdData = 		(1 << PORTA4); // on PORTA
const uint8_t lcdEnable = 		(1 << PORTA5); // on PORTA
const uint8_t lcdBit3 = 		(1 << PORTA0); // on PORTA
const uint8_t lcdBit2 = 		(1 << PORTA1); // on PORTA
const uint8_t lcdBit1 = 		(1 << PORTA2); // on PORTA
const uint8_t lcdBit0 = 		(1 << PORTA3); // on PORTA
const uint8_t lcdBrightness = 		(1 << DDB5); // on PORTB
const uint8_t lcdContrast = 		(1 << DDB7); // on PORTB
#else
#ifdef TinkerkitLCDmodule
const uint8_t lcdDirection = 		(1 << PORTF0); // on PORTF
const uint8_t lcdData = 		(1 << PORTF1); // on PORTF
const uint8_t lcdEnable = 		(1 << PORTE6); // on PORTE
const uint8_t lcdBit3 = 		(1 << PORTB4); // on PORTB
const uint8_t lcdBit2 = 		(1 << PORTD6); // on PORTD
const uint8_t lcdBit1 = 		(1 << PORTD4); // on PORTD
const uint8_t lcdBit0 = 		(1 << PORTF4); // on PORTF
const uint8_t lcdBrightness = 		(1 << DDB6); // on PORTB
const uint8_t lcdContrast = 		(1 << DDB5); // on PORTB
#else
const uint8_t lcdData = 		(1 << PORTD4); // on PORTD
const uint8_t lcdEnable = 		(1 << PORTD5); // on PORTD
const uint8_t lcdBit3 = 		(1 << PORTB5); // on PORTB
const uint8_t lcdBit2 = 		(1 << PORTB4); // on PORTB
const uint8_t lcdBit1 = 		(1 << PORTB0); // on PORTB
const uint8_t lcdBit0 = 		(1 << PORTD7); // on PORTD
const uint8_t lcdBrightness = 		(1 << DDB1); // on PORTB
const uint8_t lcdContrast = 		(1 << DDD6); // on PORTD
#endif
#endif

const uint8_t lcdDataByte = 		0b00001000;
const uint8_t lcdCommandByte = 		0b00000000;
const uint8_t lcdSendByte = 		0b00000100;
const uint8_t lcdDelay0015ms = 		0x03;
const uint8_t lcdDelay4100us = 		0x02;
const uint8_t lcdDelay0100us = 		0x01;
const uint8_t lcdDelay0080us = 		0x00;
#endif

const uint8_t cgramBigNum = 1;

#define EuB0 (1 << 0)
#define EuB1 (1 << 1)
#define EuB2 (1 << 2)
#define EuB3 (1 << 3)
#define EuB4 (1 << 4)
#define EuB5 (1 << 5)
#define EuB6 (1 << 6)
#define EuB7 (1 << 7)

#ifdef useSavedTrips
#define EuB7 0
#ifdef trackIdleEOCdata
#define EuB6 0
#endif
#endif

#ifdef useScreenEditor
#define EuB5 0
#endif

const uint8_t EEPROMusage = EuB7 | EuB6 | EuB5 | EuB4 | EuB3 | EuB2 | EuB1 | EuB0;

const uint8_t eePtrSignature = 0;
const unsigned int eePtrSettingsStart = 1;

const uint8_t eeAdrSignature = 0;
const unsigned int eeAdrSettingsStart = 3;

// start of remarkably long EEPROM stored settings section

#define nextAllowedValue (uint8_t)(eePtrSettingsStart - 1)
#ifdef useLegacyLCD
const uint8_t pContrastIdx =			nextAllowedValue + 1;
#define nextAllowedValue pContrastIdx
#endif
const uint8_t pMetricFlagIdx =			nextAllowedValue + 1;
const uint8_t pInjEdgeTriggerIdx =		pMetricFlagIdx + 1;
#define nextAllowedValue pInjEdgeTriggerIdx
#ifdef useIsqrt
const uint8_t pSysFuelPressureIdx =		nextAllowedValue + 1;
#define nextAllowedValue pSysFuelPressureIdx
#endif
#ifdef useCalculatedFuelFactor
const uint8_t pRefFuelPressureIdx =		nextAllowedValue + 1;
const uint8_t pInjectorCountIdx =		pRefFuelPressureIdx + 1;
const uint8_t pInjectorSizeIdx =		pInjectorCountIdx + 1;
#define nextAllowedValue pInjectorSizeIdx
#endif
const uint8_t pMicroSecondsPerQuantityIdx =	nextAllowedValue + 1;
const uint8_t pInjectorSettleTimeIdx =		pMicroSecondsPerQuantityIdx + 1;
const uint8_t pPulsesPerDistanceIdx =		pInjectorSettleTimeIdx + 1;
const uint8_t pVSSpauseIdx =			pPulsesPerDistanceIdx + 1;
const uint8_t pCrankRevPerInjIdx =		pVSSpauseIdx + 1;
const uint8_t pMinGoodRPMidx =			pCrankRevPerInjIdx + 1;
const uint8_t pTankSizeIdx =			pMinGoodRPMidx + 1;
#define nextAllowedValue pTankSizeIdx
#ifdef useChryslerMAPCorrection
const uint8_t pMAPsensorFloorIdx =		nextAllowedValue + 1;
const uint8_t pBaroSensorFloorIdx =		pMAPsensorFloorIdx + 1;
const uint8_t pMAPsensorCeilingIdx =		pBaroSensorFloorIdx + 1;
const uint8_t pBaroSensorCeilingIdx =		pMAPsensorCeilingIdx + 1;
const uint8_t pMAPsensorRangeIdx =		pBaroSensorCeilingIdx + 1;
const uint8_t pBaroSensorRangeIdx =		pMAPsensorRangeIdx + 1;
const uint8_t pMAPsensorOffsetIdx =		pBaroSensorRangeIdx + 1;
const uint8_t pBaroSensorOffsetIdx =		pMAPsensorOffsetIdx + 1;
#define nextAllowedValue pBaroSensorOffsetIdx
#endif
#ifdef useVehicleMass
const uint8_t pVehicleMassIdx =			nextAllowedValue + 1;
#define nextAllowedValue pVehicleMassIdx
#endif
#ifdef useCoastDownCalculator
const uint8_t pVehicleFrontalAreaIdx =		nextAllowedValue + 1;
const uint8_t pLocustDensityIdx =		pVehicleFrontalAreaIdx + 1;
const uint8_t pCoefficientDidx =		pLocustDensityIdx + 1;
const uint8_t pCoefficientVidx =		pCoefficientDidx + 1;
const uint8_t pCoefficientRRidx =		pCoefficientVidx + 1;
#define nextAllowedValue pCoefficientRRidx
#endif
const uint8_t pActivityTimeoutIdx =		nextAllowedValue + 1;
const uint8_t pWakupResetCurrentIdx =		pActivityTimeoutIdx + 1;
#define nextAllowedValue pWakupResetCurrentIdx
#ifdef useSerialPortDataLogging
const uint8_t pSerialDataLoggingIdx =		nextAllowedValue + 1;
#define nextAllowedValue pSerialDataLoggingIdx
#endif
#ifdef useWindowFilter
const uint8_t pWindowFilterIdx =		nextAllowedValue + 1;
#define nextAllowedValue pWindowFilterIdx
#endif
#ifdef useBarFuelEconVsTime
const uint8_t pFEvsTimeIdx =			nextAllowedValue + 1;
#define nextAllowedValue pFEvsTimeIdx
#endif
#ifdef useSavedTrips
const uint8_t pAutoSaveActiveIdx =		nextAllowedValue + 1;
#define nextAllowedValue pAutoSaveActiveIdx
#ifdef trackIdleEOCdata
const uint8_t pAutoSaveIdleIdx =		nextAllowedValue + 1;
#define nextAllowedValue pAutoSaveIdleIdx
#endif
#endif
#ifdef useBarFuelEconVsSpeed
const uint8_t pBarLowSpeedCutoffIdx =		nextAllowedValue + 1;
const uint8_t pBarSpeedQuantumIdx =		pBarLowSpeedCutoffIdx + 1;
#define nextAllowedValue pBarSpeedQuantumIdx
#endif
#ifdef useFuelCost
const uint8_t pCostPerQuantity =		nextAllowedValue + 1;
#define nextAllowedValue pCostPerQuantity
#endif
const uint8_t pScratchpadIdx = 			nextAllowedValue + 1;

const char parmLabels[] PROGMEM = {
#ifdef useLegacyLCD
	"Contrast\0"
#endif
	"Metric 1-Yes\0"
	"InjTrg 0-Dn 1-Up\0"
#ifdef useIsqrt
	"P(Fuel) {psi\\kPa}*1000\0"
#endif
#ifdef useCalculatedFuelFactor
	"P(Ref) {psi\\kPa}*1000\0"
	"Injector Count\0"
	"InjSize mL/min\0"
#endif
	"Microsec/{Gallon\\L}\0"
	"Inj Delay (uS)\0"
	"VSS Pulses/{Mile\\km}\0"
	"VSS Delay (ms)\0"
	"Revs/Inj Pulse\0"
	"Min Good RPM\0"
	"Tank ({Gal\\L})*1000\0"
#ifdef useChryslerMAPCorrection
	"MAPfloor (mV)\0"
	"BaroFloor (mV)\0"
	"MAPceiling (mV)\0"
	"BaroCeiling (mV)\0"
	"MAPrnge {psi\\kPa}*1000\0"
	"BaroRng {psi\\kPa}*1000\0"
	"MAPofst {psi\\kPa}*1000\0"
	"BroOfst {psi\\kPa}*1000\0"
#endif
#ifdef useVehicleMass
	"{Weight\\Mass} ({lbs\\kg})\0"
#endif
#ifdef useCoastDownCalculator
	"FrArea*1000 {ft\\m}^2\0"
	"rho*1000 {lb/yd\\kg/m}^3\0"
	"C(d) * 1000\0"
	"C(v) * 1000\0"
	"C(rr) * 1000\0"
#endif
	"Timeout (s)\0"
	"WakeupReset CURR\0"
#ifdef useSerialPortDataLogging
	"DLogSerial 1-Yes\0"
#endif
#ifdef useWindowFilter
	"WindowFilter 1-Y\0"
#endif
#ifdef useBarFuelEconVsTime
	"FE/Time Period s\0"
#endif
#ifdef useSavedTrips
	"AutoSaveTrip 1-Y\0"
#ifdef trackIdleEOCdata
	"AutoSaveIdle 1-Y\0"
#endif
#endif
#ifdef useBarFuelEconVsSpeed
	"bgLower*1000 {MPH\\kph}\0"
	"bgSize*1000 {MPH\\kph}\0"
#endif
#ifdef useFuelCost
	"Fuel Price*1000\0"
#endif
	"Scratchpad(odo?)\0"
};

#ifdef useLegacyLCD
const uint8_t pSizeContrast = 			8;
#endif
const uint8_t pSizeMetricFlag = 		1;
const uint8_t pSizeInjEdgeTrigger = 		1;
#ifdef useIsqrt
const uint8_t pSizeSysFuelPressure = 		32;
#endif
#ifdef useCalculatedFuelFactor
const uint8_t pSizeRefFuelPressure = 		32;
const uint8_t pSizeInjectorCount = 		8;
const uint8_t pSizeInjectorSize = 		16;
#endif
const uint8_t pSizeMicroSecondsPerQuantity =	32;
const uint8_t pSizeInjectorSettleTime =		16;
const uint8_t pSizePulsesPerDistance =		16;
const uint8_t pSizeVSSpause =			8;
const uint8_t pSizeCrankRevPerInj =		8;
const uint8_t pSizeMinGoodRPM =			16;
const uint8_t pSizeTankSize =			24;
#ifdef useChryslerMAPCorrection
const uint8_t pSizeMAPsensorFloor =		16;
const uint8_t pSizeBaroSensorFloor =		16;
const uint8_t pSizeMAPsensorCeiling =		16;
const uint8_t pSizeBaroSensorCeiling =		16;
const uint8_t pSizeMAPsensorRange =		32;
const uint8_t pSizeBaroSensorRange =		32;
const uint8_t pSizeMAPsensorOffset =		32;
const uint8_t pSizeBaroSensorOffset =		32;
#endif
#ifdef useVehicleMass
const uint8_t pSizeVehicleMass =		16;
#endif
#ifdef useCoastDownCalculator
const uint8_t pSizeVehicleFrontalArea =		16;
const uint8_t pSizeLocustDensity =		16;
const uint8_t pSizeCoefficientD =		16;
const uint8_t pSizeCoefficientV =		16;
const uint8_t pSizeCoefficientRR =		16;
#endif
const uint8_t pSizeActivityTimeout =		16;
const uint8_t pSizeWakupResetCurrent =		1;
#ifdef useSerialPortDataLogging
const uint8_t pSizeSerialDataLogging =		1;
#endif
#ifdef useWindowFilter
const uint8_t pSizeWindowFilter =		1;
#endif
#ifdef useBarFuelEconVsTime
const uint8_t pSizeFEvsTime =			16;
#endif
#ifdef useSavedTrips
const uint8_t pSizeAutoSaveActive =		1;
#ifdef trackIdleEOCdata
const uint8_t pSizeAutoSaveIdle =		1;
#endif
#endif
#ifdef useBarFuelEconVsSpeed
const uint8_t pSizeBarLowSpeedCutoff =		24;
const uint8_t pSizeBarSpeedQuantumIdx =		24;
#endif
#ifdef useFuelCost
const uint8_t pSizeFuelUnitCost =		16;
#endif
const uint8_t pSizeScratchpad =			32;

const uint8_t paramsLength[] PROGMEM = {
#ifdef useLegacyLCD
	pSizeContrast,					// LCD Contrast
#endif
	pSizeMetricFlag,				// Display Mode (0 - US Display, 1 - Metric Display)
	pSizeInjEdgeTrigger,				// Fuel Injector Edge Trigger (0 - Falling Edge, 1 - Rising Edge)
#ifdef useIsqrt
	pSizeSysFuelPressure,				// Fuel System Pressure * 1000 (psig or Pa)
#endif
#ifdef useCalculatedFuelFactor
	pSizeRefFuelPressure,				// Reference Fuel Injector Rated Pressure * 1000 (psig or Pa)
	pSizeInjectorCount,				// Fuel Injector Count
	pSizeInjectorSize,				// Fuel Injector Rated Capacity in mL/min
#endif
	pSizeMicroSecondsPerQuantity,			// Microseconds per (gal or L)
	pSizeInjectorSettleTime,			// Fuel Injector Response Delay Time (us)
	pSizePulsesPerDistance,				// VSS Pulses (per mile or per km)
	pSizeVSSpause,					// VSS Pause Debounce Count (ms)
	pSizeCrankRevPerInj,				// Crankshaft Revolutions per Fuel Injector Event
	pSizeMinGoodRPM,				// Minimum Engine Speed For Engine On (RPM)
	pSizeTankSize,					// Tank Capacity * 1000 (gal or L)
#ifdef useChryslerMAPCorrection
	pSizeMAPsensorFloor,				// MAP Sensor Floor * 1000 (mV)
	pSizeBaroSensorFloor,				// Barometric Sensor Floor * 1000 (mV)
	pSizeMAPsensorCeiling,				// MAP Sensor Ceiling * 1000 (mV)
	pSizeBaroSensorCeiling,				// Barometric Sensor Ceiling * 1000 (mV)
	pSizeMAPsensorRange,				// MAP Sensor Range * 1000 (psig or kPa)
	pSizeBaroSensorRange,				// Barometric Sensor Range * 1000 (psig or kPa)
	pSizeMAPsensorOffset,				// MAP Sensor Offset * 1000 (psig or kPa)
	pSizeBaroSensorOffset,				// Barometric Sensor Offset * 1000 (psig or kPa)
#endif
#ifdef useVehicleMass
	pSizeVehicleMass,				// Vehicle Weight/Mass (lbs or kg)
#endif
#ifdef useCoastDownCalculator
	pSizeVehicleFrontalArea,			// Vehicle Frontal Area * 1000 (ft^2 or m^2)
	pSizeLocustDensity,				// Air density (lb/yd^3 or kg/m^3)
	pSizeCoefficientD,				// Vehicle C(d) * 1000
	pSizeCoefficientV,				// Vehicle C(v) * 1000
	pSizeCoefficientRR,				// Vehicle C(rr) * 1000
#endif
	pSizeActivityTimeout,				// Activity Timeout (s)
	pSizeWakupResetCurrent,				// Enable current trip reset upon wakeup
#ifdef useSerialPortDataLogging
	pSizeSerialDataLogging,				// Serial Data Logging Enable
#endif
#ifdef useWindowFilter
	pSizeWindowFilter,				// Window Filter Enable
#endif
#ifdef useBarFuelEconVsTime
	pSizeFEvsTime,					// Period Of FE over Time BarGraph Bar (s)
#endif
#ifdef useSavedTrips
	pSizeAutoSaveActive,				// Autosave Active Trip Data Enable
#ifdef trackIdleEOCdata
	pSizeAutoSaveIdle,				// Autosave Idle Trip Data Enable
#endif
#endif
#ifdef useBarFuelEconVsSpeed
	pSizeBarLowSpeedCutoff,				// FE vs Speed Bargraph lower speed
	pSizeBarSpeedQuantumIdx,			// FE vs Speed Bargraph speed bar size
#endif
#ifdef useFuelCost
	pSizeFuelUnitCost,				// Price per unit volume of fuel
#endif
	pSizeScratchpad,				// Scratchpad Memory
};

#define byteSize(bitLength) ((((bitLength & 0x07) != 0)? 1 : 0) + (bitLength / 8))

#define nextAllowedValue 0
#ifdef useLegacyLCD
const uint8_t pOffsetContrast = 		nextAllowedValue;
#define nextAllowedValue pOffsetContrast + byteSize(pSizeContrast)
#endif
const uint8_t pOffsetMetricFlag = 		nextAllowedValue;
const uint8_t pOffsetInjEdgeTrigger = 		pOffsetMetricFlag + byteSize(pSizeMetricFlag);
#define nextAllowedValue pOffsetInjEdgeTrigger + byteSize(pSizeInjEdgeTrigger)
#ifdef useIsqrt
const uint8_t pOffsetSysFuelPressure = 		nextAllowedValue;
#define nextAllowedValue pOffsetSysFuelPressure + byteSize(pSizeSysFuelPressure)
#endif
#ifdef useCalculatedFuelFactor
const uint8_t pOffsetRefFuelPressure = 		nextAllowedValue;
const uint8_t pOffsetInjectorCount = 		pOffsetRefFuelPressure + byteSize(pSizeRefFuelPressure);
const uint8_t pOffsetInjectorSize = 		pOffsetInjectorCount + byteSize(pSizeInjectorCount);
#define nextAllowedValue pOffsetInjectorSize + byteSize(pSizeInjectorSize)
#endif
const uint8_t pOffsetMicroSecondsPerQuantity =	nextAllowedValue;
const uint8_t pOffsetInjectorSettleTime =	pOffsetMicroSecondsPerQuantity + byteSize(pSizeMicroSecondsPerQuantity);
const uint8_t pOffsetPulsesPerDistance =	pOffsetInjectorSettleTime + byteSize(pSizeInjectorSettleTime);
const uint8_t pOffsetVSSpause =			pOffsetPulsesPerDistance + byteSize(pSizePulsesPerDistance);
const uint8_t pOffsetCrankRevPerInj =		pOffsetVSSpause + byteSize(pSizeVSSpause);
const uint8_t pOffsetMinGoodRPM =		pOffsetCrankRevPerInj + byteSize(pSizeCrankRevPerInj);
const uint8_t pOffsetTankSize =			pOffsetMinGoodRPM + byteSize(pSizeMinGoodRPM);
#define nextAllowedValue pOffsetTankSize + byteSize(pSizeTankSize)
#ifdef useChryslerMAPCorrection
const uint8_t pOffsetMAPsensorFloor =		nextAllowedValue;
const uint8_t pOffsetBaroSensorFloor =		pOffsetMAPsensorFloor + byteSize(pSizeMAPsensorFloor);
const uint8_t pOffsetMAPsensorCeiling =		pOffsetBaroSensorFloor + byteSize(pSizeBaroSensorFloor);
const uint8_t pOffsetBaroSensorCeiling =	pOffsetMAPsensorCeiling + byteSize(pSizeMAPsensorCeiling);
const uint8_t pOffsetMAPsensorRange =		pOffsetBaroSensorCeiling + byteSize(pSizeBaroSensorCeiling);
const uint8_t pOffsetBaroSensorRange =		pOffsetMAPsensorRange + byteSize(pSizeMAPsensorRange);
const uint8_t pOffsetMAPsensorOffset =		pOffsetBaroSensorRange + byteSize(pSizeBaroSensorRange);
const uint8_t pOffsetBaroSensorOffset =		pOffsetMAPsensorOffset + byteSize(pSizeMAPsensorOffset);
#define nextAllowedValue pOffsetBaroSensorOffset + byteSize(pSizeBaroSensorOffset)
#endif
#ifdef useVehicleMass
const uint8_t pOffsetVehicleMass =		nextAllowedValue;
#define nextAllowedValue pOffsetVehicleMass + byteSize(pSizeVehicleMass)
#endif
#ifdef useCoastDownCalculator
const uint8_t pOffsetVehicleFrontalArea =	nextAllowedValue;
const uint8_t pOffsetLocustDensity =		pOffsetVehicleFrontalArea + byteSize(pSizeVehicleFrontalArea);
const uint8_t pOffsetCoefficientD =		pOffsetLocustDensity + byteSize(pSizeLocustDensity);
const uint8_t pOffsetCoefficientV =		pOffsetCoefficientD + byteSize(pSizeCoefficientD);
const uint8_t pOffsetCoefficientRR =		pOffsetCoefficientV + byteSize(pSizeCoefficientV);
#define nextAllowedValue pOffsetCoefficientRR + byteSize(pSizeCoefficientRR)
#endif
const uint8_t pOffsetActivityTimeout =		nextAllowedValue;
const uint8_t pOffsetWakupResetCurrent =	pOffsetActivityTimeout + byteSize(pSizeActivityTimeout);
#define nextAllowedValue pOffsetWakupResetCurrent + byteSize(pSizeWakupResetCurrent)
#ifdef useSerialPortDataLogging
const uint8_t pOffsetSerialDataLogging =	nextAllowedValue;
#define nextAllowedValue pOffsetSerialDataLogging + byteSize(pSizeSerialDataLogging)
#endif
#ifdef useWindowFilter
const uint8_t pOffsetWindowFilter =		nextAllowedValue;
#define nextAllowedValue pOffsetWindowFilter + byteSize(pSizeWindowFilter)
#endif
#ifdef useBarFuelEconVsTime
const uint8_t pOffsetFEvsTime =			nextAllowedValue;
#define nextAllowedValue pOffsetFEvsTime + byteSize(pSizeFEvsTime)
#endif
#ifdef useSavedTrips
const uint8_t pOffsetAutoSaveActive =		nextAllowedValue;
#define nextAllowedValue pOffsetAutoSaveActive + byteSize(pSizeAutoSaveActive)
#ifdef trackIdleEOCdata
const uint8_t pOffsetAutoSaveIdle =		nextAllowedValue;
#define nextAllowedValue pOffsetAutoSaveIdle + byteSize(pSizeAutoSaveIdle)
#endif
#endif
#ifdef useBarFuelEconVsSpeed
const uint8_t pOffsetBarLowSpeedCutoff =	nextAllowedValue;
const uint8_t pOffsetBarSpeedQuantumIdx =	pOffsetBarLowSpeedCutoff + byteSize(pSizeBarLowSpeedCutoff);
#define nextAllowedValue pOffsetBarSpeedQuantumIdx + byteSize(pSizeBarSpeedQuantumIdx)
#endif
#ifdef useFuelCost
const uint8_t pOffsetFuelUnitCost =		nextAllowedValue;
#define nextAllowedValue pOffsetFuelUnitCost + byteSize(pSizeFuelUnitCost)
#endif
const uint8_t pOffsetScratchpad =		nextAllowedValue;
const uint8_t pOffsetZZ =			pOffsetScratchpad + byteSize(pSizeScratchpad);

const uint8_t paramAddrs[] PROGMEM = {
#ifdef useLegacyLCD
	(uint8_t)(eeAdrSettingsStart) + pOffsetContrast,			// LCD Contrast
#endif
	(uint8_t)(eeAdrSettingsStart) + pOffsetMetricFlag,			// Display Mode (0 - US Display, 1 - Metric Display)
	(uint8_t)(eeAdrSettingsStart) + pOffsetInjEdgeTrigger,		// Fuel Injector Edge Trigger (0 - Falling Edge, 1 - Rising Edge)
#ifdef useIsqrt
	(uint8_t)(eeAdrSettingsStart) + pOffsetSysFuelPressure,		// Fuel System Pressure * 1000 (psig or Pa)
#endif
#ifdef useCalculatedFuelFactor
	(uint8_t)(eeAdrSettingsStart) + pOffsetRefFuelPressure,		// Reference Fuel Injector Rated Pressure * 1000 (psig or Pa)
	(uint8_t)(eeAdrSettingsStart) + pOffsetInjectorCount,		// Fuel Injector Count
	(uint8_t)(eeAdrSettingsStart) + pOffsetInjectorSize,		// Fuel Injector Rated Capacity in mL/min
#endif
	(uint8_t)(eeAdrSettingsStart) + pOffsetMicroSecondsPerQuantity,	// Microseconds per (gal or L)
	(uint8_t)(eeAdrSettingsStart) + pOffsetInjectorSettleTime,		// Fuel Injector Response Delay Time (us)
	(uint8_t)(eeAdrSettingsStart) + pOffsetPulsesPerDistance,		// VSS Pulses (per mile or per km)
	(uint8_t)(eeAdrSettingsStart) + pOffsetVSSpause,			// VSS Pause Debounce Count (ms)
	(uint8_t)(eeAdrSettingsStart) + pOffsetCrankRevPerInj,		// Crankshaft Revolutions per Fuel Injector Event
	(uint8_t)(eeAdrSettingsStart) + pOffsetMinGoodRPM,			// Minimum Engine Speed For Engine On (RPM)
	(uint8_t)(eeAdrSettingsStart) + pOffsetTankSize,			// Tank Capacity * 1000 (gal or L)
#ifdef useChryslerMAPCorrection
	(uint8_t)(eeAdrSettingsStart) + pOffsetMAPsensorFloor,		// MAP Sensor Floor * 1000 (mV)
	(uint8_t)(eeAdrSettingsStart) + pOffsetBaroSensorFloor,		// Barometric Sensor Floor * 1000 (mV)
	(uint8_t)(eeAdrSettingsStart) + pOffsetMAPsensorCeiling,		// MAP Sensor Ceiling * 1000 (mV)
	(uint8_t)(eeAdrSettingsStart) + pOffsetBaroSensorCeiling,		// Barometric Sensor Ceiling * 1000 (mV)
	(uint8_t)(eeAdrSettingsStart) + pOffsetMAPsensorRange,		// MAP Sensor Range * 1000 (psig or kPa)
	(uint8_t)(eeAdrSettingsStart) + pOffsetBaroSensorRange,		// Barometric Sensor Range * 1000 (psig or kPa)
	(uint8_t)(eeAdrSettingsStart) + pOffsetMAPsensorOffset,		// MAP Sensor Offset * 1000 (psig or kPa)
	(uint8_t)(eeAdrSettingsStart) + pOffsetBaroSensorOffset,		// Barometric Sensor Offset * 1000 (psig or kPa)
#endif
#ifdef useVehicleMass
	(uint8_t)(eeAdrSettingsStart) + pOffsetVehicleMass,		// Vehicle Weight/Mass (lbs or kg)
#endif
#ifdef useCoastDownCalculator
	(uint8_t)(eeAdrSettingsStart) + pOffsetVehicleFrontalArea,		// Vehicle Frontal Area * 1000 (ft^2 or m^2)
	(uint8_t)(eeAdrSettingsStart) + pOffsetLocustDensity,		// Air density (lb/yd^3 or kg/m^3)
	(uint8_t)(eeAdrSettingsStart) + pOffsetCoefficientD,		// Vehicle C(d) * 1000
	(uint8_t)(eeAdrSettingsStart) + pOffsetCoefficientV,		// Vehicle C(v) * 1000
	(uint8_t)(eeAdrSettingsStart) + pOffsetCoefficientRR,		// Vehicle C(rr) * 1000
#endif
	(uint8_t)(eeAdrSettingsStart) + pOffsetActivityTimeout,		// Activity Timeout (s)
	(uint8_t)(eeAdrSettingsStart) + pOffsetWakupResetCurrent,		// Enable current trip reset upon wakeup
#ifdef useSerialPortDataLogging
	(uint8_t)(eeAdrSettingsStart) + pOffsetSerialDataLogging,		// Serial Data Logging Enable
#endif
#ifdef useWindowFilter
	(uint8_t)(eeAdrSettingsStart) + pOffsetWindowFilter,		// Window Filter Enable
#endif
#ifdef useBarFuelEconVsTime
	(uint8_t)(eeAdrSettingsStart) + pOffsetFEvsTime,			// Period Of FE over Time Bar Graph Bar (s)
#endif
#ifdef useSavedTrips
	(uint8_t)(eeAdrSettingsStart) + pOffsetAutoSaveActive,		// Autosave Active Trip Data Enable
#ifdef trackIdleEOCdata
	(uint8_t)(eeAdrSettingsStart) + pOffsetAutoSaveIdle,		// Autosave Idle Trip Data Enable
#endif
#endif
#ifdef useBarFuelEconVsSpeed
	(uint8_t)(eeAdrSettingsStart) + pOffsetBarLowSpeedCutoff,		// FE vs Speed Bargraph lower speed
	(uint8_t)(eeAdrSettingsStart) + pOffsetBarSpeedQuantumIdx,		// FE vs Speed Bargraph speed bar size
#endif
#ifdef useFuelCost
	(uint8_t)(eeAdrSettingsStart) + pOffsetFuelUnitCost,		// Price per unit volume of fuel
#endif
	(uint8_t)(eeAdrSettingsStart) + pOffsetScratchpad,			// Scratchpad Memory
	(uint8_t)(eeAdrSettingsStart) + pOffsetZZ,				// Start address of next EEPROM logical block
};

const uint32_t params[] PROGMEM = {
#ifdef useLegacyLCD
	55,							// LCD Contrast
#endif
	0,							// Display Mode (0 - US Display, 1 - Metric Display)
	0,							// Fuel Injector Edge Trigger (0 - Falling Edge, 1 - Rising Edge)
#ifdef useIsqrt
	58015,							// Fuel System Pressure * 1000 (psig or Pa)
#endif
#ifdef useCalculatedFuelFactor
	58015,							// Reference Fuel Injector Rated Pressure * 1000 (psig or Pa)
	6,							// Fuel Injector Count
	284,							// Fuel Injector Rated Capacity in mL/min
#endif
	133262651,						// Microseconds per (gal or L)
	550,							// Fuel Injector Response Delay Time (us)
	10000,							// VSS Pulses (per mile or per km)
	2,							// VSS Pause Debounce Count (ms)
	2,							// Crankshaft Revolutions per Fuel Injector Event
	100,							// Minimum Engine Speed For Engine On (RPM)
	18000,							// Tank Capacity * 1000 (gal or L)
#ifdef useChryslerMAPCorrection
	0,							// MAP Sensor Floor * 1000 (mV)
	0,							// Barometric Sensor Floor * 1000 (mV)
	4500,							// MAP Sensor Ceiling * 1000 (mV)
	4500,							// Barometric Sensor Ceiling * 1000 (mV)
	14270,							// MAP Sensor Range * 1000 (psig or kPa)
	0,							// Barometric Sensor Range * 1000 (psig or kPa)
	551,							// MAP Sensor Offset * 1000 (psig or kPa)
	14696,							// Barometric Sensor Offset * 1000 (psig or kPa)
#endif
#ifdef useVehicleMass
	4000,							// Vehicle Weight/Mass (lbs or kg)
#endif
#ifdef useCoastDownCalculator
	25400,							// Vehicle Frontal Area (ft^2 or m^2)
	2065,							// Air density (lb/yd^3 or kg/m^3)
	346,							// C(d)
	1,							// C(v)
	8,							// C(rr)
#endif
	120,							// Activity Timeout (s)
	1,							// Enable current trip reset upon wakeup
#ifdef useSerialPortDataLogging
	1,							// Serial Data Logging Enable
#endif
#ifdef useWindowFilter
	1,							// Window Filter Enable
#endif
#ifdef useBarFuelEconVsTime
	5,							// Length Of BarGraph Bar (s)
#endif
#ifdef useSavedTrips
	1,							// Autosave Active Trip Data Enable
#ifdef trackIdleEOCdata
	1,							// Autosave Idle Trip Data Enable
#endif
#endif
#ifdef useBarFuelEconVsSpeed
	25000,							// FE vs Speed Bargraph lower speed
	5000,							// FE vs Speed Bargraph speed bar size
#endif
#ifdef useFuelCost
	3799,							// Price per unit volume of fuel
#endif
	0,							// Scratchpad Memory
};

const uint8_t settingsSize = (sizeof(params) / sizeof(uint32_t));

const unsigned int eePtrSettingsEnd = eePtrSettingsStart + (unsigned int)(settingsSize);
const unsigned int eeAdrSettingsEnd = eeAdrSettingsStart + (unsigned int)(pOffsetZZ);

// end of remarkably long EEPROM stored settings section

const unsigned long newEEPROMsignature = ((unsigned long)(guinosig) << 16) + ((unsigned long)(settingsSize) << 8) + (unsigned long)(EEPROMusage);

#define nextAllowedValue eePtrSettingsEnd
#define nextAllowedValue2 eeAdrSettingsEnd
#ifdef useScreenEditor
const unsigned int eePtrScreensStart = nextAllowedValue;
const unsigned int eeAdrScreensStart = nextAllowedValue2;
const unsigned int eePtrScreensEnd = eePtrScreensStart + (unsigned int)(displayFormatSize);
const unsigned int eeAdrScreensEnd = eeAdrScreensStart + (unsigned int)(displayFormatSize);
#define nextAllowedValue eePtrScreensEnd
#define nextAllowedValue2 eeAdrScreensEnd
#endif
#ifdef useSavedTrips
const unsigned int eePtrSavedTripsStart = nextAllowedValue;
const unsigned int eeAdrSavedTripsStart = nextAllowedValue2;
const unsigned int eeAdrSavedTripsTemp1 = (unsigned int)(E2END) - eeAdrSavedTripsStart + 1;
const uint8_t eeAdrSavedTripsTemp2 = (uint8_t)(eeAdrSavedTripsTemp1 / (unsigned int)(eepromTripListSize));
const uint8_t eeAdrSavedTripsTemp3 = ((tripSaveSlotCount > eeAdrSavedTripsTemp2) ? eeAdrSavedTripsTemp2 : tripSaveSlotCount);
const unsigned int eePtrSavedTripsEnd = eePtrSavedTripsStart + (unsigned int)(tripListSize) * (unsigned int)(eeAdrSavedTripsTemp3);
const unsigned int eeAdrSavedTripsEnd = eeAdrSavedTripsStart + (unsigned int)(eepromTripListSize) * (unsigned int)(eeAdrSavedTripsTemp3);
#define nextAllowedValue eePtrSavedTripsEnd
#define nextAllowedValue2 eeAdrSavedTripsEnd
#endif
const uint8_t eePtrEnd = nextAllowedValue;

#ifdef useSavedTrips
const uint8_t tripSelectList[] PROGMEM = {
	tankIdx,
	currentIdx,
#ifdef trackIdleEOCdata
	eocIdleTankIdx,
	eocIdleCurrentIdx,
#endif
};

const uint8_t tslSize = (sizeof(tripSelectList) / sizeof(uint8_t));
const uint8_t tslSubSize = 4;
const uint8_t tslCount = tslSize * tslSubSize;
const uint8_t tripMenuSize = tslCount + 1; // size of trip menu
const uint8_t tripValueLabelSize = 7;
const uint8_t tripValueSize = tripValueLabelSize * 2 - 4;

const char ertvNames[] PROGMEM = {
	"Ident\0"
	"Timestamp\0"
	"Inj Pulse\0"
	"VSS Pulse\0"
	"Inj Cyc\0"
	"Inj OpenCyc\0"
	"VSS Cyc\0"
};

const char tripNames[] PROGMEM = {
	"View Active \0"
	"Save \0"
	"Load \0"
	"Reset \0"
	"View Saved\0"
};

#endif

const char overFlowStr[] PROGMEM = " ---- ";

const char bigFEDispChars[] PROGMEM = {
	"RAW \0"
	"INST\0"
	"CURR\0"
	"TANK\0"
#ifdef trackIdleEOCdata
	"rC/I\0"
	"iC/I\0"
	"cC/I\0"
	"tC/I\0"
#endif
#ifdef useBarFuelEconVsTime
	"FE/T\0"
#endif
};

const char paramButtonChars[] PROGMEM = {
	" OK\0"
	" XX\0"
};

#ifdef useBigNumberDisplay
#ifdef useSpiffyBigChars
#ifdef useParallaxLCD
const uint8_t decimalPtChar = 46;
#define allTurnedOn 0x0C
#else
const uint8_t decimalPtChar = 0x0C;
#define allTurnedOn 255
#endif
const char bigNumChars1[] PROGMEM = {
	0x0E, 0x08, 0x0F, 0,
	0x08, allTurnedOn, 32, 0,
	0x0A, 0x0A, 0x0F, 0,
	0x08, 0x0A, 0x0F, 0,
	allTurnedOn, 0x09, allTurnedOn, 0,
	allTurnedOn, 0x0A, 0x0A, 0,
	0x0E, 0x0A, 0x0A, 0,
	0x08, 0x08, 0x0D, 0,
	0x0E, 0x0A, 0x0F, 0,
	0x0E, 0x0A, 0x0F, 0,
	32, 32, 32, 0,
	0x09, 0x09, 0x09, 0,
};

const char bigNumChars2[] PROGMEM = {
	0x0B, 0x09, 0x0D, 0,
	32, allTurnedOn, 32, 0,
	allTurnedOn, 0x09, 0x09, 0,
	0x09, 0x09, 0x0D, 0,
	32, 32, allTurnedOn, 0,
	0x09, 0x09, 0x0D, 0,
	0x0B, 0x09, 0x0D, 0,
	32, 0x0E, 32, 0,
	0x0B, 0x09, 0x0D, 0,
	0x09, 0x09, 0x0D, 0,
	32, 32, 32, 0,
	32, 32, 32, 0,
};

const char bigNumFont[] PROGMEM = {
	cgramBigNum, // font code
	8, // number of characters in font

	0b00011111, // char 0x08
	0b00011111,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,

	0b00000000, // char 0x09
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00011111,
	0b00011111,

	0b00011111, // char 0x0A
	0b00011111,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00011111,
	0b00011111,

	0b00011111, // char 0x0B
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00001111,
	0b00000111,

#ifdef useParallaxLCD
	0b00011111, // char 0x0C
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
#else
	0b00000000, // char 0x0C
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00001110,
	0b00001110,
	0b00001110,
#endif

	0b00011111, // char 0x0D
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011110,
	0b00011100,

	0b00000111, // char 0x0E
	0b00001111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,

	0b00011100, // char 0x0F
	0b00011110,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
};
#else
const uint8_t decimalPtChar = 0x0C;
const char bigNumChars1[] PROGMEM = {
	0x0B, 0x08, 0x0B, 0,
	0x08, 0x0B, 32, 0,
	0x0A, 0x0A, 0x0B, 0,
	0x08, 0x0A, 0x0B, 0,
	0x0B, 0x09, 0x0B, 0,
	0x0B, 0x0A, 0x0A, 0,
	0x0B, 0x0A, 0x0A, 0,
	0x08, 0x08, 0x0B, 0,
	0x0B, 0x0A, 0x0B, 0,
	0x0B, 0x0A, 0x0B, 0,
	32, 32, 32, 0,
	0x09, 0x09, 0x09, 0,
};

const char bigNumChars2[] PROGMEM = {
	0x0B, 0x09, 0x0B, 0,
	0x09, 0x0B, 0x09, 0,
	0x0B, 0x09, 0x09, 0,
	0x09, 0x09, 0x0B, 0,
	32, 32, 0x0B, 0,
	0x09, 0x09, 0x0B, 0,
	0x0B, 0x09, 0x0B, 0,
	32, 0x0B, 32, 0,
	0x0B, 0x09, 0x0B, 0,
	0x09, 0x09, 0x0B, 0,
	32, 32, 32, 0,
	32, 32, 32, 0,
};

const char bigNumFont[] PROGMEM = {
	cgramBigNum, // font code
	5, // number of characters in font

	0b00011111, // char 0x08
	0b00011111,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,

	0b00000000, // char 0x09
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00011111,
	0b00011111,

	0b00011111, // char 0x0A
	0b00011111,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00011111,
	0b00011111,

	0b00011111, // char 0x0B
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,

	0b00000000, // char 0x0C
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00001110,
	0b00001110,
	0b00001110
};
#endif
#endif

const uint8_t tripUpdateSrcList[] PROGMEM = {
	rawIdx | 0x80,						// transfer raw trip data to instant (disable interrupts)
	instantIdx,						// update tank trip with instant - this must be here, or 'remaining' calculations will be off
	instantIdx,						// update current trip with instant
#ifdef trackIdleEOCdata
	rawIdleIdx | 0x80,					// transfer raw idle/EOC trip data to idle/EOC instant (disable interrupts)
	eocIdleInstantIdx,					// update idle tank trip with idle instant
	eocIdleInstantIdx,					// update idle current trip with idle instant
#endif
#ifdef useBarFuelEconVsTime
	instantIdx,						// update bargraph periodic trip with instant
#endif
#ifdef useCoastDownCalculator
	thisCoastDownIdx,					// transfer last loop's coastdown trip data to last coastdown trip
	instantIdx,						// update this loop's coastdown trip with instant
#endif
};

const uint8_t tripUpdateDestList[] PROGMEM = {
	instantIdx | 0x80,					// transfer raw trip data to instant
	tankIdx, 						// update tank trip with instant - this must be here, or 'remaining' calculations will be off
	currentIdx, 						// update current trip with instant
#ifdef trackIdleEOCdata
	eocIdleInstantIdx | 0x80,				// transfer raw idle/EOC trip data to idle/EOC instant
	eocIdleTankIdx, 					// update idle tank trip with idle instant
	eocIdleCurrentIdx, 					// update idle current trip with idle instant
#endif
#ifdef useBarFuelEconVsTime
	periodIdx, 						// update bargraph periodic trip with instant
#endif
#ifdef useCoastDownCalculator
	lastCoastDownIdx | 0x80,				// transfer last loop's coastdown trip data to last coastdown trip
	thisCoastDownIdx,					// update this loop's coastdown trip with instant
#endif
#ifdef useBarFuelEconVsSpeed
	FEvsSpeedIdx,						// ensure this matches the value in bgDataSize
	FEvsSpeedIdx + 1,
	FEvsSpeedIdx + 2,
	FEvsSpeedIdx + 3,
	FEvsSpeedIdx + 4,
	FEvsSpeedIdx + 5,
	FEvsSpeedIdx + 6,
	FEvsSpeedIdx + 7,
	FEvsSpeedIdx + 8,
	FEvsSpeedIdx + 9,
	FEvsSpeedIdx + 10,
	FEvsSpeedIdx + 11,
	FEvsSpeedIdx + 12,
	FEvsSpeedIdx + 13,
	FEvsSpeedIdx + 14,
#endif
};

const uint8_t tUScount = (sizeof(tripUpdateSrcList) / sizeof(uint8_t));
const uint8_t tUDcount = (sizeof(tripUpdateDestList) / sizeof(uint8_t));

const uint8_t convIdx[] PROGMEM = {
	pPulsesPerDistanceIdx,
	pMicroSecondsPerQuantityIdx,
	pTankSizeIdx,
#ifdef useVehicleMass
	pVehicleMassIdx,
#endif
#ifdef useCoastDownCalculator
	pVehicleFrontalAreaIdx,
	pLocustDensityIdx,
#endif
#ifdef useCalculatedFuelFactor
	pSysFuelPressureIdx,
	pRefFuelPressureIdx,
#endif
#ifdef useChryslerMAPCorrection
	pMAPsensorRangeIdx,
	pMAPsensorOffsetIdx,
	pBaroSensorRangeIdx,
	pBaroSensorOffsetIdx,
#endif
#ifdef useBarFuelEconVsSpeed
	pBarLowSpeedCutoffIdx,
	pBarSpeedQuantumIdx,
#endif
#ifdef useFuelCost
	pCostPerQuantity,
#endif
};

const uint8_t convSize = (sizeof(convIdx) / sizeof(uint8_t));

const uint8_t idxNumerDistance = 0;
const uint8_t idxDenomDistance = idxNumerDistance + 1;
const uint8_t idxNumerVolume = idxDenomDistance + 1;
const uint8_t idxDenomVolume = idxNumerVolume + 1;
const uint8_t idxCyclesPerSecond = idxDenomVolume + 1;
const uint8_t idxMicroSecondsPerSecond = idxCyclesPerSecond + 1;
const uint8_t idxDecimalPoint = idxMicroSecondsPerSecond + 1;
const uint8_t idxMetricFE = idxDecimalPoint + 1;
const uint8_t idxSecondsPerHour = idxMetricFE + 1;
const uint8_t idxBiggestNumber = idxSecondsPerHour + 1;
const uint8_t idxNumber7nines = idxBiggestNumber + 1;
const uint8_t idxNumber6nines = idxNumber7nines + 1;
const uint8_t idxNumber5nines = idxNumber6nines + 1;
const uint8_t idxNumber500 = idxNumber5nines + 1;
const uint8_t idxNumber50 = idxNumber500 + 1;
const uint8_t idxNumber5 = idxNumber50 + 1;
#define nextAllowedValue idxNumber5
#ifdef useCPUreading
const uint8_t idxNumerCPUutil = nextAllowedValue + 1;
const uint8_t idxDenomCPUutil = idxNumerCPUutil + 1;
#define nextAllowedValue idxDenomCPUutil
#endif
#ifdef useClock
const uint8_t idxSecondsPerDay = nextAllowedValue + 1;
#define nextAllowedValue idxSecondsPerDay
#endif
#ifdef useIsqrt
const uint8_t idxNumerPressure = nextAllowedValue + 1;
const uint8_t idxDenomPressure = idxNumerPressure + 1;
const uint8_t idxCorrFactor = idxDenomPressure + 1;
#define nextAllowedValue idxCorrFactor
#endif
#ifdef useAnalogRead
const uint8_t idxNumerVoltage = nextAllowedValue + 1;
const uint8_t idxDenomVoltage = idxNumerVoltage + 1;
#define nextAllowedValue idxDenomVoltage
#endif
#ifdef useVehicleMass
const uint8_t idxNumerMass = nextAllowedValue + 1;
const uint8_t idxDenomMass = idxNumerMass + 1;
#define nextAllowedValue idxDenomMass
#endif
#ifdef useCoastDownCalculator
const uint8_t idxNumerArea = nextAllowedValue + 1;
const uint8_t idxDenomArea = idxNumerArea + 1;
const uint8_t idxNumerDensity = idxDenomArea + 1;
const uint8_t idxDenomDensity = idxNumerDensity + 1;
#define nextAllowedValue idxDenomDensity
#endif

const uint32_t convNumbers[] PROGMEM = {
	1000000ul,
	1609344ul,
	1000000000ul,
	3785411784ul,
	t2CyclesPerSecond,
	1000000ul,
	1000ul,
	100000ul,
	3600,
	0xFFFFFFFE,
	10000000,
	1000000,
	100000,
	500,
	50,
	5,
#ifdef useCPUreading
	10000ul,
	loopSystemLength,
#endif
#ifdef useClock
	86400,
#endif
#ifdef useIsqrt
	68947573ul,
	10000000ul,
	4096,
#endif
#ifdef useAnalogRead
	1024ul,
	5000ul,
#endif
#ifdef useVehicleMass
	1000000000ul,
	2204622621ul,
#endif
#ifdef useCoastDownCalculator
	9290304,
	100000000,
	100000,
	168555,
#endif
};

const uint8_t DNUISinstrDone = 				0;
const uint8_t DNUISinstrTraceOn = 			DNUISinstrDone + 1;
const uint8_t DNUISinstrTraceOff = 			DNUISinstrTraceOn + 1;
const uint8_t DNUISinstrSkipIfMetricMode = 		DNUISinstrTraceOff + 1;
const uint8_t DNUISinstrSkipIfZero = 			DNUISinstrSkipIfMetricMode + 1;
const uint8_t DNUISinstrSkipIfLTorE = 			DNUISinstrSkipIfZero + 1;
const uint8_t DNUISinstrSkipIfLSBset = 			DNUISinstrSkipIfLTorE + 1;
const uint8_t DNUISinstrSkipIfMSBset = 			DNUISinstrSkipIfLSBset + 1;
const uint8_t DNUISinstrSkipIfIndexBelow = 		DNUISinstrSkipIfMSBset + 1;
const uint8_t DNUISinstrSkip = 				DNUISinstrSkipIfIndexBelow + 1;
const uint8_t DNUISinstrLd = 				DNUISinstrSkip + 1;
const uint8_t DNUISinstrLdByte = 			DNUISinstrLd + 1;
const uint8_t DNUISinstrLdByteFromYindexed = 		DNUISinstrLdByte + 1;
const uint8_t DNUISinstrLdTripVar = 			DNUISinstrLdByteFromYindexed + 1;
const uint8_t DNUISinstrLdTtlFuelUsed = 		DNUISinstrLdTripVar + 1;
const uint8_t DNUISinstrLdConst = 			DNUISinstrLdTtlFuelUsed + 1;
const uint8_t DNUISinstrLdEEPROM = 			DNUISinstrLdConst + 1;
const uint8_t DNUISinstrStByteToYindexed = 		DNUISinstrLdEEPROM + 1;
const uint8_t DNUISinstrStEEPROM = 			DNUISinstrStByteToYindexed + 1;
const uint8_t DNUISinstrLdEEPROMindexed = 		DNUISinstrStEEPROM + 1;
const uint8_t DNUISinstrLdEEPROMindirect = 		DNUISinstrLdEEPROMindexed + 1;
const uint8_t DNUISinstrStEEPROMindirect = 		DNUISinstrLdEEPROMindirect + 1;
const uint8_t DNUISinstrLdIndex = 			DNUISinstrStEEPROMindirect + 1;
const uint8_t DNUISinstrLdNumer = 			DNUISinstrLdIndex + 1;
const uint8_t DNUISinstrLdDenom = 			DNUISinstrLdNumer + 1;
const uint8_t DNUISinstrCall = 				DNUISinstrLdDenom + 1;
const uint8_t DNUISinstrJump = 				DNUISinstrCall + 1;
const uint8_t DNUISinstrSwap = 				DNUISinstrJump + 1;
const uint8_t DNUISinstrSubYfromX = 			DNUISinstrSwap + 1;
const uint8_t DNUISinstrAddYtoX = 			DNUISinstrSubYfromX + 1;
#define nextAllowedValue DNUISinstrAddYtoX
#ifndef useSWEET64multDiv
const uint8_t DNUISinstrMulXbyY = 			DNUISinstrAddYtoX + 1;
const uint8_t DNUISinstrDivXbyY = 			DNUISinstrMulXbyY + 1;
#define nextAllowedValue DNUISinstrDivXbyY
#endif
const uint8_t DNUISinstrShiftLeft = 			nextAllowedValue + 1;
const uint8_t DNUISinstrShiftRight = 			DNUISinstrShiftLeft + 1;
const uint8_t DNUISinstrAddToIndex = 			DNUISinstrShiftRight + 1;
#define nextAllowedValue DNUISinstrAddToIndex
#ifdef useIsqrt
const uint8_t DNUISinstrIsqrt = 			nextAllowedValue + 1;
#define nextAllowedValue DNUISinstrIsqrt
#endif
#ifdef useAnalogRead
const uint8_t DNUISinstrLdVoltage = 			nextAllowedValue + 1;
#define nextAllowedValue DNUISinstrLdVoltage
#endif
#ifdef useChryslerMAPCorrection
const uint8_t DNUISinstrLdPressure = 			nextAllowedValue + 1;
#define nextAllowedValue DNUISinstrLdPressure
#endif

#define instrDone			DNUISinstrDone
#define instrTraceOn			DNUISinstrTraceOn
#define instrTraceOff			DNUISinstrTraceOff
#define instrSkipIfMetricMode		(DNUISinstrSkipIfMetricMode | 0x80)
#define instrSkipIfZero			(DNUISinstrSkipIfZero | 0x80 | 0x40)
#define instrSkipIfLTorE		(DNUISinstrSkipIfLTorE | 0x80 | 0x40)
#define instrSkipIfLSBset		(DNUISinstrSkipIfLSBset | 0x80 | 0x40)
#define instrSkipIfMSBset		(DNUISinstrSkipIfMSBset | 0x80 | 0x40)
#define instrSkipIfIndexBelow		(DNUISinstrSkipIfIndexBelow | 0x80)
#define instrSkip			(DNUISinstrSkip | 0x80)
#define instrLd				(DNUISinstrLd | 0x40)
#define instrLdByte			(DNUISinstrLdByte | 0x80 | 0x40)
#define instrLdByteFromYindexed		(DNUISinstrLdByteFromYindexed | 0x40)
#define instrLdTripVar			(DNUISinstrLdTripVar | 0x80 | 0x40)
#define instrLdTtlFuelUsed		(DNUISinstrLdTtlFuelUsed | 0x40)
#define instrLdConst			(DNUISinstrLdConst | 0x80 | 0x40)
#define instrLdEEPROM			(DNUISinstrLdEEPROM | 0x80 | 0x40)
#define instrStByteToYindexed		(DNUISinstrStByteToYindexed | 0x40)
#define instrStEEPROM			(DNUISinstrStEEPROM | 0x80 | 0x40)
#define instrLdEEPROMindexed		(DNUISinstrLdEEPROMindexed | 0x80 | 0x40)
#define instrLdEEPROMindirect		(DNUISinstrLdEEPROMindirect | 0x40)
#define instrStEEPROMindirect		(DNUISinstrStEEPROMindirect | 0x40)
#define instrLdIndex			(DNUISinstrLdIndex | 0x80)
#define instrLdNumer			(DNUISinstrLdNumer | 0x40)
#define instrLdDenom			(DNUISinstrLdDenom | 0x40)
#define instrCall			(DNUISinstrCall | 0x80)
#define instrJump			(DNUISinstrJump | 0x80)
#define instrSwap			(DNUISinstrSwap | 0x40)
#define instrSubYfromX			(DNUISinstrSubYfromX | 0x40)
#define instrAddYtoX			(DNUISinstrAddYtoX | 0x40)
#ifndef useSWEET64multDiv
#define instrMulXbyY			(DNUISinstrMulXbyY | 0x40)
#define instrDivXbyY			(DNUISinstrDivXbyY | 0x40)
#endif
#define instrShiftLeft			(DNUISinstrShiftLeft | 0x40)
#define instrShiftRight			(DNUISinstrShiftRight | 0x40)
#define instrAddToIndex			(DNUISinstrAddToIndex | 0x80)
#ifdef useAnalogRead
#define instrLdVoltage			(DNUISinstrLdVoltage | 0x40)
#endif
#ifdef useChryslerMAPCorrection
#define instrLdPressure			(DNUISinstrLdPressure | 0x40)
#endif
#ifdef useIsqrt
#define instrIsqrt			(DNUISinstrIsqrt | 0x40)
#endif

const uint8_t idxS64findRemainingFuel = dfMaxValDisplayCount;
const uint8_t idxS64doMultiply = idxS64findRemainingFuel + 1;
const uint8_t idxS64doDivide = idxS64doMultiply + 1;
const uint8_t idxS64findCyclesPerQuantity = idxS64doDivide + 1;
const uint8_t idxS64doConvertToMicroSeconds = idxS64findCyclesPerQuantity + 1;
const uint8_t idxS64doAdjust = idxS64doConvertToMicroSeconds + 1;
const uint8_t idxS64doNumber = idxS64doAdjust + 1;

const uint8_t prgmEngineSpeed[] PROGMEM = {
	instrLdTripVar, 0x02, rvInjPulseIdx,
	instrLdConst, 0x01, idxCyclesPerSecond,
	instrCall, idxS64doMultiply,
	instrLdByte, 0x01, 60,					// load seconds per minute into register 1
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxDecimalPoint,
	instrCall, idxS64doMultiply,
	instrLdEEPROM, 0x01, pCrankRevPerInjIdx,
	instrCall, idxS64doMultiply,
	instrLdTripVar, 0x01, rvInjCycleIdx,
	instrJump, idxS64doDivide,
};

const uint8_t prgmMotionTime[] PROGMEM = {
	instrLdTripVar, 0x02, rvVSScycleIdx,
	instrLdConst, 0x01, idxCyclesPerSecond,
	instrJump, idxS64doDivide,
};

const uint8_t prgmDistance[] PROGMEM = {
	instrLdTripVar, 0x02, rvVSSpulseIdx,
	instrLdConst, 0x01, idxDecimalPoint,
	instrCall, idxS64doMultiply,
	instrLdEEPROM, 0x01, pPulsesPerDistanceIdx,
	instrJump, idxS64doDivide,
};

const uint8_t prgmSpeed[] PROGMEM = {
	instrLdTripVar, 0x02, rvVSScycleIdx,
	instrSkipIfZero, 0x02, 29,

	instrLdEEPROM, 0x01, pPulsesPerDistanceIdx,
	instrCall, idxS64doMultiply,
	instrSwap, 0x23,
	instrLdTripVar, 0x02, rvVSSpulseIdx,
	instrLdConst, 0x01, idxDecimalPoint,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxCyclesPerSecond,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxSecondsPerHour,
	instrCall, idxS64doMultiply,
	instrSwap, 0x13,
	instrJump, idxS64doDivide,

	instrDone
};

#ifdef useBarFuelEconVsSpeed
const uint8_t prgmFEvsSpeed[] PROGMEM = {
	instrLdEEPROM, 0x01, pBarLowSpeedCutoffIdx,		// convert stored distance per hour to pulses per hour
	instrLdEEPROM, 0x02, pPulsesPerDistanceIdx,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxDecimalPoint,
	instrCall, idxS64doDivide,
	instrSwap, 0x23,

	instrLdTripVar, 0x02, rvVSSpulseIdx,			// generate speed in pulses per hour
	instrLdConst, 0x01, idxCyclesPerSecond,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxSecondsPerHour,
	instrCall, idxS64doMultiply,
	instrLdTripVar, 0x01, rvVSScycleIdx,
	instrCall, idxS64doDivide,

	instrSkipIfLTorE, 0x32, 4,				// if (converted value <= generated value), skip to next section
	instrLdByte, 0x02, 0xFF,				// load a 255 into register 2
	instrDone,						// exit to caller

	instrSubYfromX, 0x23,					// subtract converted value from generated value
	instrSwap, 0x23,

	instrLdEEPROM, 0x01, pBarSpeedQuantumIdx,		// convert stored distance per hour to pulses per hour
	instrLdEEPROM, 0x02, pPulsesPerDistanceIdx,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxDecimalPoint,
	instrCall, idxS64doDivide,
	instrSkipIfZero, 0x02, 232,				// if (bargraph size == 0), load an 0xFF into register 2 and exit to caller
	instrSwap, 0x21,					// move to denominator position

	instrSwap, 0x23,					// get numerator (generated - converted value)
	instrCall, idxS64doDivide,				// perform division

	instrLdByte, 0x01, bgDataSize,				// compare to bargraph data size
	instrSkipIfLTorE, 0x12, 220,				// if (bargraph size <= calculated value), load an 0xFF into register 2 and exit to caller
	instrLdByte, 0x01, FEvsSpeedIdx,			// convert calculated value into a trip index
	instrAddYtoX, 0x21,
	instrDone
};
#endif

const uint8_t prgmFuelUsed[] PROGMEM = {
	instrLdTripVar, 0x02, rvInjOpenCycleIdx,
	instrSkipIfZero, 0x02, 9,

	instrLdConst, 0x01, idxDecimalPoint,
	instrCall, idxS64doMultiply,
	instrCall, idxS64findCyclesPerQuantity,
	instrJump, idxS64doDivide,

	instrDone,
};

#ifdef useFuelCost
const uint8_t prgmFuelCost[] PROGMEM = {
	instrLdTripVar, 0x02, rvInjOpenCycleIdx,
	instrSkipIfZero, 0x02, 9,

	instrLdEEPROM, 0x01, pCostPerQuantity,
	instrCall, idxS64doMultiply,
	instrCall, idxS64findCyclesPerQuantity,
	instrJump, idxS64doDivide,

	instrDone,
};

const uint8_t prgmFuelRateCost[] PROGMEM = {
	instrLdTripVar, 0x02, rvInjOpenCycleIdx,
	instrSkipIfZero, 0x02, 25,

	instrLdEEPROM, 0x01, pCostPerQuantity,
	instrCall, idxS64doMultiply,
	instrLdTripVar, 0x01, rvInjCycleIdx,
	instrCall, idxS64doDivide,
	instrLdConst, 0x01, idxMicroSecondsPerSecond,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxSecondsPerHour,
	instrCall, idxS64doMultiply,
	instrLdEEPROM, 0x01, pMicroSecondsPerQuantityIdx,
	instrJump, idxS64doDivide,

	instrDone
};

const uint8_t prgmFuelCostPerDistance[] PROGMEM = {
	instrLdTripVar, 0x02, rvVSSpulseIdx,			// fetch the accumulated number of VSS pulses counted
	instrCall, idxS64findCyclesPerQuantity,			// calculate the cycles per unit fuel quantity factor
	instrCall, idxS64doMultiply,				// multiply the two numbers to get the denominator for fuel cost per distance
	instrSwap, 0x23,					// save it for later

	instrLdTripVar, 0x02, rvInjOpenCycleIdx,		// fetch the accumulated fuel injector open cycle measurement
	instrLdEEPROM, 0x01, pPulsesPerDistanceIdx,		// fetch the pulses per unit distance factor
	instrCall, idxS64doMultiply,				// multiply the two numbers to get the numerator for fuel cost per distance
	instrLdEEPROM, 0x01, pCostPerQuantity,			// load fuel cost per unit quantity into register 1
	instrCall, idxS64doMultiply,				// multiply the numerator by the formatting term

	instrSwap, 0x13,					// move the denominator term into position
	instrJump, idxS64doDivide,				// divide the numerator by the denominator, then exit to caller
};

const uint8_t prgmDistancePerFuelCost[] PROGMEM = {
	instrLdTripVar, 0x02, rvVSSpulseIdx,			// fetch the accumulated number of VSS pulses counted
	instrCall, idxS64findCyclesPerQuantity,			// calculate the cycles per unit fuel quantity factor
	instrCall, idxS64doMultiply,				// multiply the two numbers to get the numerator for distance per fuel cost
	instrLdConst, 0x01, idxDecimalPoint,			// load the decimal point constant used for output formatting
	instrCall, idxS64doMultiply,				// multiply the numerator by the formatting term
	instrLdConst, 0x01, idxDecimalPoint,			// load the decimal point constant used for output formatting
	instrCall, idxS64doMultiply,				// multiply the numerator by the formatting term
	instrSwap, 0x23,					// save it for later

	instrLdTripVar, 0x02, rvInjOpenCycleIdx,		// fetch the accumulated fuel injector open cycle measurement
	instrLdEEPROM, 0x01, pPulsesPerDistanceIdx,		// fetch the pulses per unit distance factor
	instrCall, idxS64doMultiply,				// multiply the two numbers to get the denominator for distance per fuel cost
	instrLdEEPROM, 0x01, pCostPerQuantity,			// load fuel cost per unit quantity into register 1
	instrCall, idxS64doMultiply,				// multiply the numerator by the formatting term
	instrSwap, 0x23,					// swap the numerator and denominator terms around

	instrSwap, 0x13,					// move the denominator term into position
	instrJump, idxS64doDivide,				// divide the numerator by the denominator, then exit to caller
};

const uint8_t prgmRemainingFuelCost[] PROGMEM = {
	instrCall, idxS64findRemainingFuel,
	instrSkipIfZero, 0x02, 20,

	instrLdEEPROM, 0x01, pCostPerQuantity,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxMicroSecondsPerSecond,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxCyclesPerSecond,
	instrCall, idxS64doDivide,
	instrLdEEPROM, 0x01, pMicroSecondsPerQuantityIdx,
	instrJump, idxS64doDivide,

	instrDone
};
#endif

const uint8_t prgmEngineRunTime[] PROGMEM = {
	instrLdTripVar, 0x02, rvInjCycleIdx,
	instrLdConst, 0x01, idxCyclesPerSecond,
	instrJump, idxS64doDivide,
};

const uint8_t prgmFuelRate[] PROGMEM = {
	instrLdTripVar, 0x02, rvInjOpenCycleIdx,
	instrSkipIfZero, 0x02, 25,

	instrLdConst, 0x01, idxDecimalPoint,
	instrCall, idxS64doMultiply,
	instrLdTripVar, 0x01, rvInjCycleIdx,
	instrCall, idxS64doDivide,
	instrLdConst, 0x01, idxMicroSecondsPerSecond,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxSecondsPerHour,
	instrCall, idxS64doMultiply,
	instrLdEEPROM, 0x01, pMicroSecondsPerQuantityIdx,
	instrJump, idxS64doDivide,

	instrDone
};

const uint8_t prgmFuelEcon[] PROGMEM = {
	instrLdTripVar, 0x02, rvVSSpulseIdx,			// fetch the accumulated number of VSS pulses counted
	instrCall, idxS64findCyclesPerQuantity,			// calculate the cycles per unit fuel quantity factor
	instrCall, idxS64doMultiply,				// multiply the two numbers to get the denominator for fuel economy
	instrSwap, 0x23,					// save it for later

	instrLdTripVar, 0x02, rvInjOpenCycleIdx,		// fetch the accumulated fuel injector open cycle measurement
	instrLdEEPROM, 0x01, pPulsesPerDistanceIdx,		// fetch the pulses per unit distance factor
	instrCall, idxS64doMultiply,				// multiply the two numbers to get the numerator for fuel economy

	instrSkipIfMetricMode, 7,				// if metric mode set, skip ahead
	instrSwap, 0x23,					// swap the numerator and denominator terms around
	instrLdConst, 0x01, idxDecimalPoint,			// load the decimal point constant used for output formatting
	instrSkip, 3,						// go skip ahead

	instrLdConst, 0x01, idxMetricFE,			// load the output formatting decimal point constant, multiplied by 100 (for 100km/L)

	instrSkipIfZero, 0x02, 6,				// if the numerator term is zero, go exit

	instrCall, idxS64doMultiply,				// multiply the numerator by the formatting term
	instrSwap, 0x13,					// move the denominator term into position
	instrJump, idxS64doDivide,				// divide the numerator by the denominator, then exit to caller

	instrDone						// exit to caller
};

const uint8_t prgmFindRemainingFuel[] PROGMEM = {
	instrLdEEPROM, 0x02, pTankSizeIdx,
	instrLdEEPROM, 0x01, pMicroSecondsPerQuantityIdx,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxCyclesPerSecond,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxMicroSecondsPerSecond,
	instrCall, idxS64doDivide,
	instrLdConst, 0x01, idxDecimalPoint,
	instrCall, idxS64doDivide,
	instrLdTtlFuelUsed, 0x01,

	instrSkipIfLTorE, 0x12, 4,

	instrLdByte, 0x02, 0,
	instrDone,

	instrSubYfromX, 0x21,
	instrDone
};

const uint8_t prgmRemainingFuel[] PROGMEM = {
	instrCall, idxS64findRemainingFuel,
	instrSkipIfZero, 0x02, 20,

	instrLdConst, 0x01, idxDecimalPoint,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxMicroSecondsPerSecond,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxCyclesPerSecond,
	instrCall, idxS64doDivide,
	instrLdEEPROM, 0x01, pMicroSecondsPerQuantityIdx,
	instrJump, idxS64doDivide,

	instrDone
};

const uint8_t prgmDistanceToEmpty[] PROGMEM = {
	instrCall, idxS64findRemainingFuel,
	instrSkipIfZero, 0x02, 22,

	instrLdConst, 0x01, idxDecimalPoint,
	instrCall, idxS64doMultiply,
	instrLdTripVar, 0x01, rvInjOpenCycleIdx,
	instrCall, idxS64doDivide,
	instrLdTripVar, 0x01, rvVSSpulseIdx,
	instrCall, idxS64doMultiply,
	instrLdEEPROM, 0x01, pPulsesPerDistanceIdx,
	instrCall, idxS64doDivide,
	instrJump, idxS64doAdjust,

	instrDone
};

const uint8_t prgmTimeToEmpty[] PROGMEM = {
	instrLdConst, 0x01, idxCyclesPerSecond,
	instrLdConst, 0x02, idxMicroSecondsPerSecond,
	instrCall, idxS64doMultiply,
	instrSwap, 0x23,

	instrCall, idxS64findRemainingFuel,
	instrSkipIfZero, 0x02, 19,

	instrLdConst, 0x01, idxMicroSecondsPerSecond,
	instrCall, idxS64doMultiply,
	instrLdTripVar, 0x01, rvInjOpenCycleIdx,
	instrCall, idxS64doDivide,
	instrLdTripVar, 0x01, rvInjCycleIdx,
	instrCall, idxS64doMultiply,
	instrSwap, 0x13,
	instrJump, idxS64doDivide,

	instrDone
};

const uint8_t prgmInjectorOpenTime[] PROGMEM = {
	instrLdTripVar, 0x02, rvInjOpenCycleIdx,
	instrJump, idxS64doConvertToMicroSeconds,
};

const uint8_t prgmInjectorTotalTime[] PROGMEM = {
	instrLdTripVar, 0x02, rvInjCycleIdx,
	instrJump, idxS64doConvertToMicroSeconds,
};

const uint8_t prgmVSStotalTime[] PROGMEM = {
	instrLdTripVar, 0x02, rvVSScycleIdx,
	instrJump, idxS64doConvertToMicroSeconds,
};

const uint8_t prgmVSSpulseCount[] PROGMEM = {
	instrLdTripVar, 0x02, rvVSSpulseIdx,
	instrDone
};

const uint8_t prgmInjectorPulseCount[] PROGMEM = {
	instrLdTripVar, 0x02, rvInjPulseIdx,
	instrDone
};
#ifdef useAnalogRead

const uint8_t prgmVoltage[] PROGMEM = {
	instrLdConst, 0x02, idxDenomVoltage,
	instrLdVoltage, 0x01,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxNumerVoltage,
	instrJump, idxS64doDivide,
};
#endif
#ifdef useChryslerMAPCorrection

const uint8_t prgmPressure[] PROGMEM = {
	instrLdPressure, 0x02,
	instrDone
};

const uint8_t prgmCorrF[] PROGMEM = {
	instrLdConst, 0x02, idxDecimalPoint,
	instrLdPressure, 0x01,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxCorrFactor,
	instrJump, idxS64doDivide,
};
#endif

const uint8_t prgmConvertToMicroSeconds[] PROGMEM = {
	instrLdConst, 0x01, idxMicroSecondsPerSecond,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxCyclesPerSecond,
	instrJump, idxS64doDivide,
};

const uint8_t prgmDoMultiply[] PROGMEM = {
#ifdef useSWEET64multDiv
	instrLd, 0x41,						// load multiplier into register 4
	instrLd, 0x52,						// load multiplicand into register 5
	instrLdByte, 0x02, 0,					// zero out result (register 2)

	instrSkipIfZero, 0x04, 13,				// if multiplier is zero, exit
	instrSkipIfLSBset, 0x04, 2,				// if the low bit of multiplier is zet, skip ahead

	instrSkip, 2,						// jump to multiplicand/multiplier adjust

	instrAddYtoX, 0x25,					// add multiplicand to result

	instrShiftLeft, 0x05,					// shift multiplicand left by one bit
	instrShiftRight, 0x04,					// shift multiplier right by one bit
	instrSkip, 240,						// jump back to multiplier tests
#else
	instrMulXbyY, 0x21,
#endif
	instrDone						// exit to caller
};

const uint8_t prgmDoDivide[] PROGMEM = {
#ifdef useSWEET64multDiv
	instrSkipIfZero, 0x02, 13,				// exit if dividend is zero
	instrSkipIfZero, 0x01, 2,				// skip if divisor is zero
	instrSkip, 11,						// skip ahead
	instrLdByte, 0x02, 0,					// zero out result (register 2)
	instrLdByte, 0x05, 1,					// load 1 into register 5
	instrSubYfromX, 0x25,					// set overflow value in result
	instrLd, 0x12,						// set overflow (or zero numerator) value in remainder
	instrDone,						// exit to caller

	instrLd, 0x41,						// load register 4 with divisor
	instrLd, 0x12,						// load register 1 with dividend
	instrLdByte, 0x05, 1,					// load register 5 with quotient bitmask
	instrLdByte, 0x02, 0,					// load result (register 2) with initialized quotient

	instrSkipIfMSBset, 0x04, 6,				// if divisor has been leftshifted to MSB, skip ahead
	instrShiftLeft, 0x04,					// shift divisor left one bit
	instrShiftLeft, 0x05,					// shift quotient bitmask left one bit
	instrSkip, 247,						// skip back to divisor test

	instrSkipIfZero, 0x05, 233,				// if quotient bitmask is zero, exit
	instrSkipIfLTorE, 0x41, 2,				// if divisor is less than or equal to dividend, skip ahead
	instrSkip, 4,						// skip to divisor/quotient bitmask adjustment routine
	instrSubYfromX, 0x14,					// subtract divisor from dividend
	instrAddYtoX, 0x25,	       				// add quotient bitmask to quotient
	instrShiftRight, 0x04,					// shift divisor right by one bit
	instrShiftRight, 0x05,					// shift quotient bitmask right by one bit
	instrSkip, 238,						// go back to quotient bitmask test
#else
	instrDivXbyY, 0x21,
	instrDone						// exit to caller
#endif
};

const uint8_t prgmDoAdjust[] PROGMEM = {
	instrSkipIfLTorE, 0x14, 1,				// if (divisor / 2 <= dividend), skip to next section
	instrDone,

	instrLdByte, 0x05, 1,					// load a 1 into the old quotient bitmask register
	instrAddYtoX, 0x25,					// bump up quotient by one
	instrDone						// exit to caller
};

const uint8_t prgmRoundOffNumber[] PROGMEM = {
	instrLdConst, 0x01, idxNumber7nines,			// if number is greater than 9999, round off to nearest 1
	instrSkipIfLTorE, 0x12, 25,
	instrLdConst, 0x01, idxNumber6nines,			// if number is greater than 999, round off to nearest 1/10th
	instrSkipIfLTorE, 0x12, 24,
	instrLdConst, 0x01, idxNumber5nines,			// if number is greater than 99, round off to nearest 1/100th
	instrSkipIfLTorE, 0x12, 23,

	instrAddToIndex, 253,					// determine whether a number of right-hand digits were specified
	instrSkipIfIndexBelow, 23, 253,				// skip if there were no specified number of of right-hand digits
	instrSkipIfIndexBelow, 5, 254,				// skip if 0 right-hand digits were specified
	instrSkipIfIndexBelow, 7, 255,				// skip if 1 right-hand digit was specified
	instrSkip, 10,						// skip if 2 right-hand digits were specified

	instrLdConst, 0x01, idxNumber500,
	instrSkip, 8,
	instrLdConst, 0x01, idxNumber50,
	instrSkip, 3,
	instrLdConst, 0x01, idxNumber5,

	instrAddYtoX, 0x21,

	instrLdConst, 0x01, idxBiggestNumber,			// if number is less than biggest number, go perform round-off
	instrSkipIfLTorE, 0x21, 8,

	instrLdIndex, 6,
	instrLdByte, 0x01, 255,					// signal overflow by loading 255 into length byte
	instrStByteToYindexed, 0x13,				// store total length into byte 6 of register 3
	instrDone,

	instrJump, idxS64doNumber,
};

const uint8_t prgmFormatToNumber[] PROGMEM = {
	instrLdIndex, 4,					// load 5 into index
	instrLdByte, 0x01, 100,					// load 100 into register 1
	instrCall, idxS64doDivide,				// perform division - quotient remains in register 2, and remainder goes into register 1
	instrStByteToYindexed, 0x13,				// store remainder into indexed byte of register 3
	instrAddToIndex, 255,					// update index
	instrSkipIfIndexBelow, 244, 255,			// continue if index is greater than or equal to 0

	instrLdIndex, 7,
	instrLdByte, 0x01, 32,					// load leading zero character into register 1
	instrStByteToYindexed, 0x13,				// store leading zero character into byte 7 of register 3
	instrLdIndex, 6,
	instrLdByte, 0x01, 5,					// load total length into register 1
	instrStByteToYindexed, 0x13,				// store total length into byte 6 of register 3
	instrDone,
};

const uint8_t prgmFindCyclesPerQuantity[] PROGMEM = {
	instrSwap, 0x23,
	instrLdConst, 0x01, idxCyclesPerSecond,
	instrLdEEPROM, 0x02, pMicroSecondsPerQuantityIdx,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxMicroSecondsPerSecond,
	instrCall, idxS64doDivide,
	instrLd, 0x12,
	instrSwap, 0x23,
	instrDone
};

const uint8_t prgmFormatToTime[] PROGMEM = {
	instrLdIndex, 2,
	instrLdByte, 0x01, 60,					// load seconds per minute into register 1
	instrCall, idxS64doDivide,
	instrStByteToYindexed, 0x13,
	instrLdIndex, 1,
	instrLdByte, 0x01, 60,					// load minutes per hour into register 1
	instrCall, idxS64doDivide,
	instrStByteToYindexed, 0x13,
	instrLdIndex, 0,
	instrLdByte, 0x01, 24,					// load hours per day into register 1
	instrCall, idxS64doDivide,
	instrStByteToYindexed, 0x13,
	instrLdIndex, 7,
	instrLdByte, 0x01, 48,					// load leading zero character into register 1
	instrStByteToYindexed, 0x13,				// store leading zero character into byte 7 of register 3
	instrLdIndex, 6,
	instrLdByte, 0x01, 3,					// load total length into register 1
	instrStByteToYindexed, 0x13,				// store total length into byte 6 of register 3
	instrDone
};

const uint8_t convNumerIdx[] PROGMEM = {
	idxNumerDistance,
	idxNumerVolume,
	idxDenomVolume,
#ifdef useVehicleMass
	idxNumerMass,
#endif
#ifdef useCoastDownCalculator
	idxNumerArea,
	idxNumerDensity,
#endif
#ifdef useCalculatedFuelFactor
	idxNumerPressure,
	idxNumerPressure,
#endif
#ifdef useChryslerMAPCorrection
	idxNumerPressure,
	idxNumerPressure,
	idxNumerPressure,
	idxNumerPressure,
#endif
#ifdef useBarFuelEconVsSpeed
	idxNumerDistance,
	idxNumerDistance,
#endif
#ifdef useFuelCost
	idxNumerVolume,
#endif
};

#ifdef useScreenEditor
uint8_t displayFormats[(unsigned int)(displayFormatSize)] = {
#else
const uint8_t displayFormats[(unsigned int)(displayFormatSize)] PROGMEM = {
#endif
	(instantIdx << dfBitShift) | tSpeed,			(instantIdx << dfBitShift) | tEngineSpeed,		(instantIdx << dfBitShift) | tFuelRate,			(instantIdx << dfBitShift) | tFuelEcon,
	(instantIdx << dfBitShift) | tFuelEcon,			(instantIdx << dfBitShift) | tSpeed,			(instantIdx << dfBitShift) | tFuelRate,			(currentIdx << dfBitShift) | tFuelEcon,
#ifdef useChryslerMAPCorrection
	(instantIdx << dfBitShift) | tPressureChannel0,		(instantIdx << dfBitShift) | tPressureChannel1,		(instantIdx << dfBitShift) | tPressureChannel3,		(instantIdx << dfBitShift) | tCorrectionFactor,
#endif
#ifdef useAnalogRead
	(instantIdx << dfBitShift) | tAnalogChannel0,		(instantIdx << dfBitShift) | tAnalogChannel1,		(instantIdx << dfBitShift) | tAnalogChannel0,		(instantIdx << dfBitShift) | tAnalogChannel1,
#endif
	(instantIdx << dfBitShift) | tFuelEcon,			(instantIdx << dfBitShift) | tSpeed,			(currentIdx << dfBitShift) | tFuelEcon,			(currentIdx << dfBitShift) | tDistance,
	(instantIdx << dfBitShift) | tFuelEcon,			(instantIdx << dfBitShift) | tSpeed,			(tankIdx << dfBitShift) | tFuelEcon,			(tankIdx << dfBitShift) | tDistance,
	(currentIdx << dfBitShift) | tSpeed,			(currentIdx << dfBitShift) | tFuelEcon,			(currentIdx << dfBitShift) | tDistance,			(currentIdx << dfBitShift) | tFuelUsed,
	(tankIdx << dfBitShift) | tSpeed,			(tankIdx << dfBitShift) | tFuelEcon,			(tankIdx << dfBitShift) | tDistance,			(tankIdx << dfBitShift) | tFuelUsed,
#ifdef trackIdleEOCdata
	(eocIdleCurrentIdx << dfBitShift) | tDistance,		(eocIdleCurrentIdx << dfBitShift) | tFuelUsed,		(eocIdleTankIdx << dfBitShift) | tDistance,		(eocIdleTankIdx << dfBitShift) | tFuelUsed,
#endif
	(tankIdx << dfBitShift) | tEngineRunTime,		(tankIdx << dfBitShift) | tFuelUsed,			(tankIdx << dfBitShift) | tMotionTime,			(tankIdx << dfBitShift) | tDistance,
	(currentIdx << dfBitShift) | tEngineRunTime,		(currentIdx << dfBitShift) | tFuelUsed,			(currentIdx << dfBitShift) | tMotionTime,		(currentIdx << dfBitShift) | tDistance,
#ifdef trackIdleEOCdata
	(eocIdleTankIdx << dfBitShift) | tEngineRunTime,	(eocIdleTankIdx << dfBitShift) | tFuelUsed,		(eocIdleTankIdx << dfBitShift) | tMotionTime,		(eocIdleTankIdx << dfBitShift) | tDistance,
	(eocIdleCurrentIdx << dfBitShift) | tEngineRunTime,	(eocIdleCurrentIdx << dfBitShift) | tFuelUsed,		(eocIdleCurrentIdx << dfBitShift) | tMotionTime,	(eocIdleCurrentIdx << dfBitShift) | tDistance,
#endif
	(tankIdx << dfBitShift) | tFuelUsed,			(tankIdx << dfBitShift) | tRemainingFuel,		(tankIdx << dfBitShift) | tTimeToEmpty,			(tankIdx << dfBitShift) | tDistanceToEmpty
};

const uint16_t funcPointers[] PROGMEM = {
	(uint16_t)doNothing,
	(uint16_t)noSupport,
	(uint16_t)doCursorUpdateMain,
	(uint16_t)doCursorUpdateSetting,
	(uint16_t)doMainScreenDisplay,
	(uint16_t)doSettingEditDisplay,
	(uint16_t)doParamEditDisplay,
	(uint16_t)doGoSettingsEdit,
	(uint16_t)doNextBright,
	(uint16_t)doTripResetCurrent,
	(uint16_t)doLongGoRight,
	(uint16_t)doTripResetTank,
	(uint16_t)doLongGoLeft,
	(uint16_t)doReturnToMain,
	(uint16_t)doGoParamEdit,
	(uint16_t)doParamFindRight,
	(uint16_t)doParamExit,
	(uint16_t)doParamFindLeft,
	(uint16_t)doParamChangeDigit,
	(uint16_t)doParamSave,
	(uint16_t)doParamStoreMin,
	(uint16_t)doParamStoreMax,
	(uint16_t)doParamRevert,
#ifdef useCPUreading
	(uint16_t)doDisplaySystemInfo,
	(uint16_t)doShowCPU,
#endif
#ifdef useBigFE
	(uint16_t)doCursorUpdateBigFEscreen,
	(uint16_t)doBigFEdisplay,
#endif
#ifdef useBigDTE
	(uint16_t)doCursorUpdateBigDTEscreen,
	(uint16_t)doBigDTEdisplay,
#endif
#ifdef useBigTTE
	(uint16_t)doCursorUpdateBigTTEscreen,
	(uint16_t)doBigTTEdisplay,
#endif
#ifdef useClock
	(uint16_t)doCursorUpdateSystemTimeScreen,
	(uint16_t)doDisplaySystemTime,
	(uint16_t)doGoEditSystemTime,
	(uint16_t)doEditSystemTimeDisplay,
	(uint16_t)doEditSystemTimeCancel,
	(uint16_t)doEditSystemTimeChangeDigit,
	(uint16_t)doEditSystemTimeSave,
#endif
#ifdef useSavedTrips
	(uint16_t)doCursorUpdateTripShow,
	(uint16_t)doTripSaveDisplay,
	(uint16_t)doTripShowDisplay,
	(uint16_t)doGoTripCurrent,
	(uint16_t)doGoTripTank,
	(uint16_t)doTripBumpSlot,
	(uint16_t)doTripSelect,
	(uint16_t)doTripLongSelect,
	(uint16_t)doTripShowCancel,
#endif
#ifdef useScreenEditor
	(uint16_t)doScreenEditDisplay,
	(uint16_t)doGoScreenEdit,
	(uint16_t)doScreenEditReturnToMain,
	(uint16_t)doScreenEditRevert,
	(uint16_t)doSaveScreen,
	(uint16_t)doScreenEditBump,
	(uint16_t)doCursorUpdateScreenEdit,
#endif
#ifdef useBarFuelEconVsTime
	(uint16_t)doCursorUpdateBarFEvT,
	(uint16_t)doBarFEvTdisplay,
#endif
#ifdef useBarFuelEconVsSpeed
	(uint16_t)doCursorUpdateBarFEvS,
	(uint16_t)doBarFEvSdisplay,
	(uint16_t)doResetBarFEvS,
#endif
#ifdef useBenchMark
	(uint16_t)doBenchMark,
#endif
#ifdef useEEPROMviewer
	(uint16_t)doEEPROMviewDisplay,
	(uint16_t)goEEPROMview,
#endif
};

// Button Press variable section

const uint8_t bpListMain[] PROGMEM = {
	btnShortPressRL, idxDoGoSettingsEdit,
	btnShortPressC, idxDoNextBright,
	btnLongPressRC, idxDoTripResetCurrent,
	btnLongPressCL, idxDoTripResetTank,
	btnLongPressR, idxDoLongGoRight,
	btnLongPressL, idxDoLongGoLeft,
#ifdef useCPUreading
	btnLongPressC, idxDoShowCPU,
#endif
#ifdef useSavedTrips
	btnShortPressRC, idxDoGoTripCurrent,
	btnShortPressCL, idxDoGoTripTank,
#endif
#ifdef useScreenEditor
	btnLongPressRL, idxDoGoScreenEdit,
#endif
#ifdef useEEPROMviewer
	btnShortPressRCL, idxGoEEPROMview,
#endif
	buttonsUp, idxNoSupport,
};

const uint8_t bpListSetting[] PROGMEM = {
	btnShortPressRL, idxDoReturnToMain,
	btnShortPressC, idxDoGoParamEdit,
	btnLongPressRL, idxDoReturnToMain,
#ifdef useCPUreading
	btnLongPressC, idxDoShowCPU,
#endif
	buttonsUp, idxDoNothing,
};

const uint8_t bpListParam[] PROGMEM = {
	btnShortPressRC, idxDoParamFindRight,
	btnShortPressRL, idxDoParamExit,
	btnShortPressCL, idxDoParamFindLeft,
	btnShortPressC, idxDoParamChangeDigit,
	btnLongPressRC, idxDoParamStoreMin,
	btnLongPressRL, idxDoParamRevert,
	btnLongPressCL, idxDoParamStoreMax,
	btnLongPressC, idxDoParamSave,
	buttonsUp, idxDoNothing,
};

#ifdef useCPUreading
const uint8_t bpListCPUmonitor[] PROGMEM = {
	btnShortPressRL, idxDoGoSettingsEdit,
	btnShortPressC, idxDoNextBright,
	btnLongPressRC, idxDoTripResetCurrent,
	btnLongPressCL, idxDoTripResetTank,
	btnLongPressR, idxDoLongGoRight,
	btnLongPressL, idxDoLongGoLeft,
	btnLongPressC, idxDoShowCPU,
#ifdef useSavedTrips
	btnShortPressRC, idxDoGoTripCurrent,
	btnShortPressCL, idxDoGoTripTank,
#endif
#ifdef useBenchMark
	btnLongPressRCL, idxDoBenchMark,
#endif
	buttonsUp, idxDoNothing,
};

#endif
#ifdef useBigNumberDisplay
const uint8_t bpListBigNum[] PROGMEM = {
	btnShortPressRL, idxDoGoSettingsEdit,
	btnShortPressC, idxDoNextBright,
	btnLongPressRC, idxDoTripResetCurrent,
	btnLongPressCL, idxDoTripResetTank,
	btnLongPressR, idxDoLongGoRight,
	btnLongPressL, idxDoLongGoLeft,
#ifdef useCPUreading
	btnLongPressC, idxDoShowCPU,
#endif
#ifdef useSavedTrips
	btnShortPressRC, idxDoGoTripCurrent,
	btnShortPressCL, idxDoGoTripTank,
#endif
	buttonsUp, idxDoNothing,
};
#endif

#ifdef useBarFuelEconVsTime
const uint8_t bpListBFET[] PROGMEM = {
	btnShortPressRL, idxDoGoSettingsEdit,
	btnShortPressC, idxDoNextBright,
	btnLongPressRC, idxDoTripResetCurrent,
	btnLongPressCL, idxDoTripResetTank,
	btnLongPressR, idxDoLongGoRight,
	btnLongPressL, idxDoLongGoLeft,
#ifdef useCPUreading
	btnLongPressC, idxDoShowCPU,
#endif
#ifdef useSavedTrips
	btnShortPressRC, idxDoGoTripCurrent,
	btnShortPressCL, idxDoGoTripTank,
#endif
	buttonsUp, idxDoNothing,
};
#endif

#ifdef useBarFuelEconVsSpeed
const uint8_t bpListBFES[] PROGMEM = {
	btnShortPressRL, idxDoGoSettingsEdit,
	btnShortPressC, idxDoNextBright,
	btnLongPressRC, idxDoTripResetCurrent,
	btnLongPressCL, idxDoTripResetTank,
	btnLongPressR, idxDoLongGoRight,
	btnLongPressL, idxDoLongGoLeft,
#ifdef useCPUreading
	btnLongPressC, idxDoShowCPU,
#endif
	btnLongPressRCL, idxDoResetBarFEvS,
#ifdef useSavedTrips
	btnShortPressRC, idxDoGoTripCurrent,
	btnShortPressCL, idxDoGoTripTank,
#endif
	buttonsUp, idxDoNothing,
};
#endif

#ifdef useClock
const uint8_t bpListTime[] PROGMEM = {
	btnShortPressRL, idxDoGoEditSystemTime,
	btnShortPressC, idxDoNextBright,
	btnLongPressRC, idxDoTripResetCurrent,
	btnLongPressCL, idxDoTripResetTank,
	btnLongPressR, idxDoLongGoRight,
	btnLongPressL, idxDoLongGoLeft,
#ifdef useCPUreading
	btnLongPressC, idxDoShowCPU,
#endif
#ifdef useSavedTrips
	btnShortPressRC, idxDoGoTripCurrent,
	btnShortPressCL, idxDoGoTripTank,
#endif
	buttonsUp, idxDoNothing,
};

const uint8_t bpListClockEdit[] PROGMEM = {
	btnShortPressRL, idxDoEditSystemTimeCancel,
	btnShortPressC, idxDoEditSystemTimeChangeDigit,
	btnLongPressRL, idxDoEditSystemTimeCancel,
	btnLongPressC, idxDoEditSystemTimeSave,
	buttonsUp, idxDoNothing,
};
#endif

#ifdef useSavedTrips
const uint8_t bpListTripSave[] PROGMEM = {
	btnShortPressRL, idxDoReturnToMain,
	btnShortPressC, idxDoTripSelect,
	btnLongPressRC, idxDoTripResetCurrent,
	btnLongPressRL, idxDoReturnToMain,
	btnLongPressCL, idxDoTripResetTank,
	btnLongPressC, idxDoTripLongSelect,
	buttonsUp, idxDoNothing,
};

const uint8_t bpListTripView[] PROGMEM = {
	btnShortPressRL, idxDoTripShowCancel,
	btnShortPressC, idxDoTripBumpSlot,
	btnLongPressRL, idxDoReturnToMain,
	buttonsUp, idxDoNothing,
};
#endif

#ifdef useScreenEditor
const uint8_t bpListScreenEdit[] PROGMEM = {
	btnShortPressRL, idxDoScreenEditReturnToMain,
	btnShortPressC, idxDoScreenEditBump,
	btnLongPressRL, idxDoScreenEditRevert,
	btnLongPressC, idxDoSaveScreen,
	buttonsUp, idxDoNothing,
};
#endif

#ifdef useEEPROMviewer
const uint8_t bpListEEPROMview[] PROGMEM = {
	btnShortPressRL, idxDoReturnToMain,
	btnLongPressRL, idxDoReturnToMain,
	buttonsUp, idxDoNothing,
};
#endif

const uint8_t bpIdxMain =	0;
const uint8_t bpIdxSetting =	bpIdxMain + 1;
const uint8_t bpIdxParam =	bpIdxSetting + 1;
#define nextAllowedValue bpIdxParam
#ifdef useCPUreading
const uint8_t bpIdxCPUmonitor =	nextAllowedValue + 1;
#define nextAllowedValue bpIdxCPUmonitor
#endif
#ifdef useBigNumberDisplay
const uint8_t bpIdxBigNum =	nextAllowedValue + 1;
#define nextAllowedValue bpIdxBigNum
#endif
#ifdef useBarFuelEconVsTime
const uint8_t bpIdxBFET =	nextAllowedValue + 1;
#define nextAllowedValue bpIdxBFET
#endif
#ifdef useBarFuelEconVsSpeed
const uint8_t bpIdxBFES =	nextAllowedValue + 1;
#define nextAllowedValue bpIdxBFES
#endif
#ifdef useClock
const uint8_t bpIdxTime =	nextAllowedValue + 1;
const uint8_t bpIdxClockEdit =	bpIdxTime + 1;
#define nextAllowedValue bpIdxClockEdit
#endif
#ifdef useSavedTrips
const uint8_t bpIdxTripSave =	nextAllowedValue + 1;
const uint8_t bpIdxTripView =	bpIdxTripSave + 1;
#define nextAllowedValue bpIdxTripView
#endif
#ifdef useScreenEditor
const uint8_t bpIdxScreenEdit =	nextAllowedValue + 1;
#define nextAllowedValue bpIdxScreenEdit
#endif
#ifdef useEEPROMviewer
const uint8_t bpIdxEEPROMview =	nextAllowedValue + 1;
#define nextAllowedValue bpIdxEEPROMview
#endif
const uint8_t bpIdxSize =	nextAllowedValue + 1;

const uint8_t * const buttonPressAdrList[(unsigned int)(bpIdxSize)] PROGMEM = {
	bpListMain,
	bpListSetting,
	bpListParam,
#ifdef useCPUreading
	bpListCPUmonitor,
#endif
#ifdef useBigNumberDisplay
	bpListBigNum,
#endif
#ifdef useBarFuelEconVsTime
	bpListBFET,
#endif
#ifdef useBarFuelEconVsSpeed
	bpListBFES,
#endif
#ifdef useClock
	bpListTime,
	bpListClockEdit,
#endif
#ifdef useSavedTrips
	bpListTripSave,
	bpListTripView,
#endif
#ifdef useScreenEditor
	bpListScreenEdit,
#endif
#ifdef useEEPROMviewer
	bpListEEPROMview,
#endif
};

// Display screen variable section

const uint8_t mainScreenSize = 1
#ifdef useCPUreading
	+ 1
#endif
#ifdef useBarFuelEconVsTime
	+ 1
#endif
#ifdef useBarFuelEconVsSpeed
	+ 1
#endif
#ifdef useBigFE
	+ 1
#endif
#ifdef useBigDTE
	+ 1
#endif
#ifdef useBigTTE
	+ 1
#endif
#ifdef useClock
	+ 1
#endif
;

const uint8_t screenSize = mainScreenSize + 2
#ifdef useClock
	+ 1
#endif
#ifdef useSavedTrips
	+ 2
#endif
#ifdef useScreenEditor
	+ 1
#endif
#ifdef useEEPROMviewer
	+ 1
#endif
;

const uint8_t mainScreenIdx =			0;
#define nextAllowedValue mainScreenIdx
#ifdef useBigFE
const uint8_t bigFEscreenIdx =			nextAllowedValue + 1;
#define nextAllowedValue bigFEscreenIdx
#endif
#ifdef useCPUreading
const uint8_t CPUmonScreenIdx =			nextAllowedValue + 1;
#define nextAllowedValue CPUmonScreenIdx
#endif
#ifdef useBarFuelEconVsTime
const uint8_t barFEvTscreenIdx =		nextAllowedValue + 1;
#define nextAllowedValue barFEvTscreenIdx
#endif
#ifdef useBarFuelEconVsSpeed
const uint8_t barFEvSscreenIdx =		nextAllowedValue + 1;
#define nextAllowedValue barFEvSscreenIdx
#endif
#ifdef useBigDTE
const uint8_t bigDTEscreenIdx =			nextAllowedValue + 1;
#define nextAllowedValue bigDTEscreenIdx
#endif
#ifdef useBigTTE
const uint8_t bigTTEscreenIdx =			nextAllowedValue + 1;
#define nextAllowedValue bigTTEscreenIdx
#endif
#ifdef useClock
const uint8_t systemTimeDisplayScreenIdx =	nextAllowedValue + 1;
#define nextAllowedValue systemTimeDisplayScreenIdx
#endif
const uint8_t settingScreenIdx =		nextAllowedValue + 1;
const uint8_t paramScreenIdx =			settingScreenIdx + 1;
#define nextAllowedValue paramScreenIdx
#ifdef useClock
const uint8_t systemTimeEditScreenIdx =		nextAllowedValue + 1;
#define nextAllowedValue systemTimeEditScreenIdx
#endif
#ifdef useSavedTrips
const uint8_t tripSaveScreenIdx =		nextAllowedValue + 1;
const uint8_t tripShowScreenIdx =		tripSaveScreenIdx + 1;
#define nextAllowedValue tripShowScreenIdx
#endif
#ifdef useScreenEditor
const uint8_t screenEditIdx =			nextAllowedValue + 1;
#define nextAllowedValue screenEditIdx
#endif
#ifdef useEEPROMviewer
const uint8_t eepromViewIdx =			nextAllowedValue + 1;
#define nextAllowedValue eepromViewIdx
#endif

const uint8_t screenParameters[(unsigned int)(screenSize)][6] PROGMEM = {
	mainScreenIdx,			mainScreenSize,		displayPageCount,		idxDoMainScreenDisplay,		idxDoCursorUpdateMain,			bpIdxMain,
#ifdef useBigFE
	mainScreenIdx,			mainScreenSize,		3,				idxDoBigFEdisplay,		idxDoCursorUpdateBigFEscreen,		bpIdxBigNum,
#endif
#ifdef useCPUreading
	mainScreenIdx,			mainScreenSize,		1,				idxDoDisplaySystemInfo,		idxDoNothing,				bpIdxCPUmonitor,
#endif
#ifdef useBarFuelEconVsTime
	mainScreenIdx,			mainScreenSize,		2,				idxDoBarFEvTdisplay,		idxDoCursorUpdateBarFEvT,		bpIdxBFET,
#endif
#ifdef useBarFuelEconVsSpeed
	mainScreenIdx,			mainScreenSize,		4,				idxDoBarFEvSdisplay,		idxDoCursorUpdateBarFEvS,		bpIdxBFES,
#endif
#ifdef useBigDTE
	mainScreenIdx,			mainScreenSize,		3,				idxDoBigDTEdisplay,		idxDoCursorUpdateBigDTEscreen,		bpIdxBigNum,
#endif
#ifdef useBigTTE
	mainScreenIdx,			mainScreenSize,		3,				idxDoBigTTEdisplay,		idxDoCursorUpdateBigTTEscreen,		bpIdxBigNum,
#endif
#ifdef useClock
	mainScreenIdx,			mainScreenSize,		1,				idxDoDisplaySystemTime,		idxDoCursorUpdateSystemTimeScreen,	bpIdxTime,
#endif
	settingScreenIdx,		1,			settingsSize,			idxDoSettingEditDisplay,	idxDoCursorUpdateSetting,		bpIdxSetting,
	paramScreenIdx,			1,			12,				idxDoParamEditDisplay,		idxDoNothing,				bpIdxParam,
#ifdef useClock
	systemTimeEditScreenIdx,	1,			4,				idxDoEditSystemTimeDisplay,	idxDoNothing,				bpIdxClockEdit,
#endif
#ifdef useSavedTrips
	tripSaveScreenIdx,		1,			tripMenuSize,			idxDoTripSaveDisplay,		idxDoNothing,				bpIdxTripSave,
	tripShowScreenIdx,		1,			tripValueSize,			idxDoTripShowDisplay,		idxDoCursorUpdateTripShow,		bpIdxTripView,
#endif
#ifdef useScreenEditor
	screenEditIdx,			1,			displayFormatSize * 2,		idxDoScreenEditDisplay,		idxDoCursorUpdateScreenEdit,		bpIdxScreenEdit,
#endif
#ifdef useEEPROMviewer
	eepromViewIdx,			1,			eePtrEnd,			idxDoEEPROMviewDisplay,		idxDoNothing,				bpIdxEEPROMview,
#endif
};

uint8_t screenCursor[(unsigned int)(screenSize)] = {
	0,
	0,
	0,
#ifdef useCPUreading
	0,
#endif
#ifdef useBigFE
	0,
#endif
#ifdef useBigDTE
	0,
#endif
#ifdef useBigTTE
	0,
#endif
#ifdef useBarFuelEconVsTime
	0,
#endif
#ifdef useBarFuelEconVsSpeed
	0,
#endif
#ifdef useClock
	0,
	0,
#endif
#ifdef useSavedTrips
	0,
	0,
#endif
#ifdef useScreenEditor
	0,
#endif
#ifdef useEEPROMviewer
	0
#endif
};

#ifdef useCoastDownCalculator
long long matrix_x[3][3];
long long matrix_r[3][3];
long long matrix_y[3];
long long matrix_z[3];
#endif

#ifdef useBarFuelEconVsTime
const char barFEvTfuncNames[] PROGMEM = {
	"DiffFE / Time\0"
	"FE / Time\0"
};
#endif

#ifdef useBarFuelEconVsSpeed
const uint8_t barFEvSdisplayFuncs[] PROGMEM = {
	tFuelEcon,
	tFuelUsed,
	tMotionTime,
	tDistance,
};

const char barFEvSfuncNames[] PROGMEM = {
	"FE / Speed\0"
	"Fuel Used/Speed\0"
	"RunTime / Speed\0"
	"Distance / Speed\0"
};
#endif

#ifdef useBuffering
const uint8_t bufferSize = 32;
const uint8_t bufferIsFull = 	0b10000000;
const uint8_t bufferIsEmpty = 	0b01000000;

class Buffer // Buffer prototype
{

public:
	volatile uint8_t storage[(unsigned int)(bufferSize)];
	volatile uint8_t bufferStart;
	volatile uint8_t bufferEnd;
	volatile uint8_t bufferStatus;

	pFunc onEmpty;
	pFunc onNoLongerEmpty;
	pFunc onNoLongerFull;
	pFunc onFull;

	qFunc process;

	void init(void);
	void push(uint8_t value);
	void pull(void);
	uint8_t updatePointer(volatile uint8_t * pointer, uint8_t clearFlag, uint8_t setFlag);
};

#endif
class Trip // Trip prototype
{

public:
	unsigned long collectedData[rvLength];

	void reset(void); // reset Trip instance
	void transfer(Trip t);
	void update(Trip t); // update with results of another Trip instance
	void add64s(uint8_t calcIdx, unsigned long v);
	void add32(uint8_t calcIdx, unsigned long v);
#ifdef useWindowFilter
	void subtract(Trip t);
	void sub32(uint8_t calcIdx, unsigned long v);
#endif
#ifdef useSavedTrips
	uint8_t load(uint8_t tripSlotIdx);
	uint8_t save(uint8_t tripSlotIdx);
#endif

};

namespace LCD // LCD prototype
{

	void init(void);
	void gotoXY(uint8_t x, uint8_t y);
	void loadCGRAMcharacter(uint8_t chr, const char * chrData, uint8_t mode);
	void setBright(uint8_t idx);
	void writeData(uint8_t value);
#ifdef useLegacyLCD
	void setContrast(uint8_t idx);
	void writeCommand(uint8_t value);
	uint8_t writeNybble(uint8_t value, uint8_t flags);
	void outputNybble(uint8_t s);
	void startOutput(void);
#endif
};

#ifdef useLegacyLCD
volatile uint8_t lcdDelayCount;
#ifdef useLegacyLCDbuffered
Buffer lcdBuffer;
#endif
#endif

#ifdef useBufferedSerialPort
Buffer serialBuffer;
#endif

#ifdef useChryslerMAPCorrection
const uint8_t pressureSize = 5;
const uint8_t MAPpressureIdx = 0;
const uint8_t baroPressureIdx = 1;
const uint8_t fuelPressureIdx = 2;
const uint8_t injPressureIdx = 3;
const uint8_t injCorrectionIdx = 4;

unsigned long pressure[(unsigned int)(pressureSize)] = { 0, 0, 0, 0, 0 };
unsigned long analogFloor[2];
unsigned long analogSlope[2];
unsigned long analogOffset[2];
volatile unsigned int sampleCount = 0;
#endif

#ifdef useAnalogRead
const uint8_t ADCfilterBitSize = 4;
const uint8_t ADCfilterSize = (1 << ADCfilterBitSize);
const uint8_t ADCfilterMask = (0xFF >> (8 - ADCfilterBitSize));
const uint8_t ADCchannelCount = 2
#ifdef useAnalogButtons
	+ 3
#endif
;

volatile unsigned int analogValue[(unsigned int)(ADCchannelCount)];
volatile uint8_t analogChannelValue[(unsigned int)(ADCchannelCount)] = { // points to the next channel to be read
#ifdef TinkerkitLCDmodule
	(1 << REFS0)|	(1 << MUX2)|			(1 << MUX0),	// analog channel 1
	(1 << REFS0)|	(1 << MUX2)|	(1 << MUX1),			// analog channel 2
	(1 << REFS0)|	(1 << MUX2)|	(1 << MUX1)|	(1 << MUX0),	// analog channel 3
#else
	(1 << REFS0)|					(1 << MUX0),	// analog channel 1
	(1 << REFS0)|			(1 << MUX1),			// analog channel 2
#ifdef useAnalogButtons
	(1 << REFS0)|			(1 << MUX1)|	(1 << MUX0),	// analog channel 3
	(1 << REFS0)|	(1 << MUX2),					// analog channel 4
	(1 << REFS0)|	(1 << MUX2)|			(1 << MUX0),	// analog channel 5
#endif
#endif
};
volatile uint8_t analogChannelIdx = 0;
#endif

volatile unsigned long sleepTicks;
volatile unsigned long timer2_overflow_count;
volatile unsigned long systemCycles[2] = { 0, 0 };
#ifdef useClock
volatile unsigned long clockCycles[2] = { 0, 0 };
#endif
volatile unsigned long injSettleCycles;
volatile unsigned long minGoodRPMcycles;
volatile unsigned long maxGoodInjCycles;

volatile unsigned int injResetCount;
volatile unsigned int vssResetCount;
volatile unsigned int buttonCount;
volatile unsigned int timerDelayCount;
volatile unsigned int injResetDelay;

volatile uint8_t vssPause;
volatile uint8_t buttonState;
volatile uint8_t VSSCount;
volatile uint8_t dirty = 0;
volatile uint8_t timerStatus = 0;
volatile uint8_t timerHeartBeat = 0;
volatile uint8_t timerCommand = 0;
volatile uint8_t holdDisplay = 0;

#ifdef ArduinoMega2560
volatile uint8_t lastPINKstate;
#else
#ifdef TinkerkitLCDmodule
volatile uint8_t lastPINBstate;
#else
volatile uint8_t lastPINCstate;
#endif
#endif

extern int __bss_end;
extern int *__brkval;

Trip tripArray[tripSlotCount]; // main objects we will be working with

#ifdef useBarFuelEconVsTime
unsigned long barFEvsTimeData[bgDataSize];
#endif

#ifdef useClock
unsigned long outputCycles[2];
#endif

unsigned long paramMaxValue;
unsigned long timerLoopStart;
unsigned long timerLoopLength;

#ifdef useBarFuelEconVsTime
unsigned int bFEvTperiod;
unsigned int bFEvTcount;

uint8_t bFEvTstartIDx;
uint8_t bFEvTsize;
#endif

#ifdef useSavedTrips
uint8_t tripShowSlot;
#endif

uint8_t menuLevel = 0;
uint8_t prevMenuLevel = 0;
uint8_t brightnessIdx = 1;
uint8_t metricFlag;
uint8_t paramLength = 0;
uint8_t paramPtr = 0;
uint8_t hPos;
uint8_t vPos;
uint8_t ignoreChar;
uint8_t printChar;
uint8_t cgramMode = 0;

#ifdef useAnalogButtons
volatile uint8_t thisAnalogKeyPressed = buttonsUp;
#endif

char mBuff1[17]; // used by format(), doFormat()
char mBuff2[17]; // used by editParm(), bar graph routines
char pBuff[12]; // used by editParm(), editClock()

// this ISR gets called every time timer 2 overflows.
// timer 2 prescaler is set at 64, and it's an 8 bit counter
// so this ISR gets called every 256 * 64 / (system clock) seconds (for 20 MHz clock, that is every 0.8192 ms)

#ifdef TinkerkitLCDmodule
ISR( TIMER0_OVF_vect ) // system timer interrupt handler
#else
ISR( TIMER2_OVF_vect ) // system timer interrupt handler
#endif
{

	static uint8_t lastKeyPressed = 0;
	static uint8_t thisKeyPressed;
	static unsigned long lastTime;
	static unsigned long timerSleep = 0;
	static unsigned long wp;
	static unsigned int timerLoopCount = 0;

	unsigned long thisTime;
	unsigned long cycleLength;

	timer2_overflow_count += 256; // update TOV count
#ifdef TinkerkitLCDmodule
	thisTime = timer2_overflow_count | TCNT0; // calculate current cycle count
#else
	thisTime = timer2_overflow_count | TCNT2; // calculate current cycle count
#endif

	if (dirty & dirtySysTick)
	{

		cycleLength = findCycleLength(lastTime, thisTime);

		systemCycles[0] += cycleLength;
		if (systemCycles[0] < cycleLength) systemCycles[1]++;

#ifdef useClock
		clockCycles[0] += cycleLength;
		if (clockCycles[0] < cycleLength) clockCycles[1]++;
#endif

	}

	if (injResetCount)
	{

		injResetCount--;
		 // if timeout is complete, cancel any pending injector pulse read and signal that no injector pulse has been read in a while
		if (injResetCount == 0) dirty &= ~(dirtyGoodInj | dirtyInjOpenRead);

	}

	if (vssResetCount)
	{

		vssResetCount--;
		if (vssResetCount == 0) dirty &= ~dirtyGoodVSS;

	}

	if (VSSCount) // if there is a VSS debounce countdown in progress
	{

		VSSCount--; // bump down the VSS count
		if (VSSCount == 0) updateVSS(thisTime); // if count has reached zero,

	}

	if (buttonCount) // if there is a button press debounce countdown in progress
	{

		buttonCount--; // bump down the button press count by one

		if (buttonCount == 0) lastKeyPressed |= longButtonBit; // signal that a "long" keypress has been detected

		if (buttonCount == keyShortDelay) // if button debounce countdown reaches this point
		{

			// figure out what buttons are being pressed
#ifdef useLegacyButtons
#ifdef ArduinoMega2560
			thisKeyPressed = buttonsUp & lastPINKstate;
#else
			thisKeyPressed = buttonsUp & lastPINCstate;
#endif
#endif

#ifdef useAnalogButtons
			thisKeyPressed = thisAnalogKeyPressed;
#endif

			if (thisKeyPressed != buttonsUp) // if any buttons are pressed
			{

				lastKeyPressed = thisKeyPressed; // remember the button press status for later
				timerStatus |= tsButtonRead; // signal that a button has been read in

			}
			else buttonCount = 0; // reset button press debounce countdown to zero

		}

		if (buttonCount == 0) // if a button has been read, go pass it on to the main program
		{

			timerCommand |= tcWakeUp; // tell system timer to wake up the main program

			// if a valid button press was read in, and main program is not asleep
			if ((timerStatus & tsButtonRead) && !(timerStatus & tsFellAsleep))
			{

				// pass off the remembered button press status to the main program
				buttonState = lastKeyPressed;
				// signal main program that a key press was detected
				timerStatus &= ~(tsButtonsUp | tsButtonRead);
				if (buttonState != buttonsUp) timerStatus &= ~tsDisplayDelay;

			}

		}

	}

#ifdef useChryslerMAPCorrection
	if (sampleCount) sampleCount--;
	else readMAP();
#endif

	if (timerStatus & tsLoopExec) // if a loop execution is in progress
	{

		if (timerLoopCount) timerLoopCount--; // if the loop countdown is in progress, bump loop count up by one tick
		else
		{

			timerStatus &= ~tsLoopExec; // stop the loop timer and signal loop finished to main program
			if (timerStatus & tsButtonsUp) // if no keypress,
			{

				timerHeartBeat <<= 1; // cycle the heartbeat bit
				if (timerHeartBeat == 0) timerHeartBeat = 1;

			}

		}

	}

	if (timerSleep) timerSleep--;
	else if (timerStatus & tsAwake) timerCommand |= tcFallAsleep;

	if (timerCommand & tcDoDelay) // if main program has requested a delay
	{

		if (timerDelayCount) timerDelayCount--; // bump timer delay value down by one tick
		else timerCommand &= ~tcDoDelay; // signal to main program that delay timer has completed main program request

	}

	if (timerCommand & tcDisplayDelay) // if main program has requested to delay status line (top line)
	{

		timerCommand &= ~tcDisplayDelay; // signal to main program that status line delay request is acknowledged
		timerStatus |= tsDisplayDelay; // signal that status line delay request is active
		holdDisplay = holdDelay; // start hold delay countdown

	}

	if (timerCommand & tcStartLoop) // if main program has requested to start cycle
	{

		timerCommand &= ~tcStartLoop; // signal to main program that loop timer has acknowledged main program request
		timerStatus |= (tsLoopExec | tsMarkLoop); // signal to main program that loop timer is in progress
		timerLoopCount = loopTickLength; // initialize loop count
		if (timerStatus & tsDisplayDelay)
		{

			if (holdDisplay) holdDisplay--;
			else timerStatus &= ~tsDisplayDelay;

		}

	}

	if (timerCommand & tcWakeUp)
	{

		// clear wakeup command and any pending sleep command
		timerCommand &= ~(tcWakeUp | tcFallAsleep);
		timerStatus |= tsAwake; // set awake status
		timerSleep = sleepTicks; // reset sleep counter

	}

	if (timerCommand & tcFallAsleep)
	{

		timerCommand &= ~tcFallAsleep; // clear sleep command
		timerStatus &= ~tsAwake; // clear awake status

	}

	dirty |= dirtySysTick;

	lastTime = thisTime; // save cycle count

}

volatile unsigned long lastInjOpenStart;
volatile unsigned long thisInjOpenStart;
volatile unsigned long totalInjCycleLength;
volatile unsigned long maximumInjOpenCycleLength;

#ifdef ArduinoMega2560
ISR( INT4_vect ) // injector opening event handler
#else
ISR( INT0_vect ) // injector opening event handler
#endif
{

	lastInjOpenStart = thisInjOpenStart;
	thisInjOpenStart = cycles2();

	if (dirty & dirtyGoodInj)
	{

		// calculate fuel injector length between pulse starts
		totalInjCycleLength = findCycleLength(lastInjOpenStart, thisInjOpenStart);

		if (totalInjCycleLength < minGoodRPMcycles)
		{

			maximumInjOpenCycleLength = 819 * totalInjCycleLength; // to determine instantaneous maximum injector on-time
			maximumInjOpenCycleLength >>= 10; // and multiply it by 0.8 (or something reasonably close) for injector duty cycle
			timerCommand |= tcWakeUp; // tell timer to wake up main program

		}
		else
		{

			totalInjCycleLength = 0;
			dirty &= ~dirtyGoodInj; // signal that no injector pulse has been read for a while

		}

	}

	if (!(dirty & dirtyGoodInj)) maximumInjOpenCycleLength = maxGoodInjCycles; // seed working maxGoodInjCycles with default value

	dirty |= dirtyInjOpenRead; // signal that injector pulse read is in progress
	injResetCount = injResetDelay; // reset injector validity monitor

}

#ifdef ArduinoMega2560
ISR( INT5_vect ) // injector closing event handler
#else
ISR( INT1_vect ) // injector closing event handler
#endif
{

	unsigned long thisTime = cycles2();
	unsigned long injOpenCycleLength = 0;
	uint8_t i = rawIdx;
#ifdef trackIdleEOCdata
	uint8_t x = 1;

	if (!(dirty & dirtyGoodVSS)) x++; // if no valid VSS pulse has been read, then vehicle is idling
#endif

	if (dirty & dirtyInjOpenRead)
	{

		// calculate fuel injector pulse length
		injOpenCycleLength = findCycleLength(thisInjOpenStart, thisTime) - injSettleCycles;

		if (injOpenCycleLength < maximumInjOpenCycleLength) // perform rationality test on injector open cycle pulse length
		{

#ifdef useChryslerMAPCorrection
			readMAP(); // calculate correction factor for differential pressure across the fuel injector
			injOpenCycleLength *= pressure[(unsigned int)(injCorrectionIdx)]; // multiply by correction factor
			injOpenCycleLength >>= 12; // divide by denominator factor
#endif

			dirty |= dirtyGoodInj; // signal that a valid fuel injector pulse has just been read
			timerCommand |= tcWakeUp; // tell timer to wake up main program

		}
		else
		{

			injOpenCycleLength = 0;
			dirty &= ~dirtyGoodInj; // signal that no injector pulse has been read
			injResetCount = 0; // stop injector validity monitor

		}

		dirty &= ~dirtyInjOpenRead; // signal that the injector pulse has been read

	}

#ifdef trackIdleEOCdata
	for (uint8_t y = 0; y < x; y++)
	{
#endif

		if (injOpenCycleLength)
		{

			tripArray[(unsigned int)(i)].collectedData[(unsigned int)(rvInjPulseIdx)]++; // update the injector pulse count
			tripArray[(unsigned int)(i)].add64s(rvInjOpenCycleIdx, injOpenCycleLength); // add to fuel injector open cycle accumulator

		}

		tripArray[(unsigned int)(i)].add64s(rvInjCycleIdx, totalInjCycleLength); // add to fuel injector total cycle accumulator

#ifdef trackIdleEOCdata
		i ^= (rawIdx ^ rawIdleIdx);

	}
#endif

	totalInjCycleLength = 0;

}

#ifdef ArduinoMega2560
ISR( PCINT2_vect )
#else
#ifdef TinkerkitLCDmodule
ISR( PCINT0_vect )
#else
ISR( PCINT1_vect )
#endif
#endif
{

	unsigned long cycleLength;

#ifdef TinkerkitLCDmodule
	cycleLength = timer2_overflow_count + TCNT0; // read current TCNT0
	if (TIFR0 & (1 << TOV0)) cycleLength = timer2_overflow_count + 256 + TCNT0; // if overflow occurred, re-read TCNT0 and adjust for overflow
#else
	cycleLength = timer2_overflow_count + TCNT2; // read current TCNT2
	if (TIFR2 & (1 << TOV2)) cycleLength = timer2_overflow_count + 256 + TCNT2; // if overflow occurred, re-read TCNT2 and adjust for overflow
#endif

	static uint8_t p;
	static uint8_t q;

#ifdef ArduinoMega2560
	p = PINK; // read current pin K state
	q = p ^ lastPINKstate; // detect any changes from the last time this ISR is called
#else
#ifdef TinkerkitLCDmodule
	p = PINB; // read current pin B state
	q = p ^ lastPINBstate; // detect any changes from the last time this ISR is called
#else
	p = PINC; // read current pin C state
	q = p ^ lastPINCstate; // detect any changes from the last time this ISR is called
#endif
#endif

	if (q & vssBit) // if a VSS pulse is received
	{

		if (vssPause == 0) updateVSS(cycleLength); // if there is no VSS pulse delay defined
		else VSSCount = vssPause; // otherwise, set VSS debounce count and let system timer handle the debouncing

	}

#ifdef useLegacyButtons
	if (q & buttonsUp) buttonCount = keyDelay; // set keypress debounce count, and let system timer handle the debouncing
#endif

#ifdef ArduinoMega2560
	lastPINKstate = p; // remember the current pin K state for the next time this ISR gets called
#else
#ifdef TinkerkitLCDmodule
	lastPINBstate = p; // remember the current pin B state for the next time this ISR gets called
#else
	lastPINCstate = p; // remember the current pin C state for the next time this ISR gets called
#endif
#endif

}

#ifdef useParallax5PositionSwitch
const unsigned int analogButtonThreshold[] PROGMEM = {
	0,
	559,
	580,
	586,
	618,
	651,
	664,
	693,
	717,
	728,
	748,
	766,
	786,
	814,
	834,
	858,
	897,
	927,
	980,
};

const uint8_t analogButtonCount = (sizeof(analogButtonThreshold) / sizeof(unsigned int));

const uint8_t analogTranslate[(unsigned int)(analogButtonCount)] PROGMEM = {
	buttonsUp,
	btnShortPress1CL,
	btnShortPress2CL,
	btnShortPressCL,
	btnShortPress1L,
	btnShortPress2L,
	btnShortPressL,
	btnShortPress1RC,
	btnShortPress2RC,
	btnShortPressRC,
	btnShortPress1C,
	btnShortPress2C,
	btnShortPressC,
	btnShortPress1R,
	btnShortPress2R,
	btnShortPressR,
	btnShortPress1,
	btnShortPress2,
	buttonsUp,
};
#endif

#ifdef useAnalogMuxButtons
const unsigned int analogButtonThreshold[] PROGMEM = {
	0,		// 00
	556,	// 01
	560,	// 02
	567,	// 03
	574,	// 04
	583,	// 05
	593,	// 06
	601,	// 07
	609,	// 08
	621,	// 09
	634,	// 0A
	644,	// 0B
	653,	// 0C
	665,	// 0D
	677,	// 0E
	687,	// 0F
	698,	// 10
	722,	// 11
	747,	// 12
	759,	// 13
	772,	// 14
	789,	// 15
	806,	// 16
	820,	// 17
	835,	// 18
	859,	// 19
	884,	// 1A
	902,	// 1B
	921,	// 1C
	944,	// 1D
	968,	// 1E
	989,	// 1F
	1012,	// 20
};

const uint8_t analogButtonCount = (sizeof(analogButtonThreshold) / sizeof(unsigned int));

const uint8_t analogTranslate[(unsigned int)(analogButtonCount)] PROGMEM = {
	buttonsUp,
	btnShortPress21RCL,
	btnShortPress1RCL,
	btnShortPress2RCL,
	btnShortPressRCL,
	btnShortPress21CL,
	btnShortPress1CL,
	btnShortPress2CL,
	btnShortPressCL,
	btnShortPress21RL,
	btnShortPress1RL,
	btnShortPress2RL,
	btnShortPressRL,
	btnShortPress21L,
	btnShortPress1L,
	btnShortPress2L,
	btnShortPressL,
	btnShortPress21RC,
	btnShortPress1RC,
	btnShortPress2RC,
	btnShortPressRC,
	btnShortPress21C,
	btnShortPress1C,
	btnShortPress2C,
	btnShortPressC,
	btnShortPress21R,
	btnShortPress1R,
	btnShortPress2R,
	btnShortPressR,
	btnShortPress21,
	btnShortPress1,
	btnShortPress2,
	buttonsUp,
};
#endif

#ifdef useAnalogInterrupt
ISR( ADC_vect )
{

#ifdef useAnalogButtons
	static uint8_t lastAnalogKeyPressed = buttonsUp;
#endif
#ifdef useAnalogRead
	static unsigned int rawRead;
	union union_16 * rawValue = (union union_16 *) &rawRead;
	static uint8_t ADCstate = 1;

	rawValue->u8[0] = ADCL; // (locks ADC sample result register from AtMega hardware)
	rawValue->u8[1] = ADCH; // (releases ADC sample result register to AtMega hardware)

	if (ADCstate)
	{

		ADCstate--;
		ADMUX = analogChannelValue[(unsigned int)(analogChannelIdx)]; // select next analog channel to read (this has to be done quickly!)

	}
	else
	{

#ifdef TinkerkitLCDmodule
		ADMUX = (1 << REFS0) | (1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0); // ground ADC sample/hold capacitor to reset it
#else
		ADMUX = (1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0); // ground ADC sample/hold capacitor to reset it
#endif

		analogValue[(unsigned int)(analogChannelIdx)] = rawRead;

		analogChannelIdx++;
		if (analogChannelIdx == ADCchannelCount) analogChannelIdx = 0;

		ADCstate = 3;

#ifdef useAnalogButtons
		if (analogChannelIdx == 2)
		{

			for (uint8_t x = analogButtonCount - 1; x < analogButtonCount; x--)
			{

				if (analogValue[(unsigned int)(analogChannelIdx)] >= pgm_read_word(&analogButtonThreshold[(unsigned int)(x)]))
				{

					thisAnalogKeyPressed = pgm_read_byte(&analogTranslate[(unsigned int)(x)]);
					break;

				}

			}

			if (thisAnalogKeyPressed != lastAnalogKeyPressed) buttonCount = keyDelay;

			lastAnalogKeyPressed = thisAnalogKeyPressed;

		}
#endif

	}

#endif

#ifdef useLegacyLCD
	if (timerCommand & tcLCDdelay) // if main program has requested a delay
	{

		if (lcdDelayCount == 0)
		{

#ifdef useLegacyLCDbuffered
			lcdBuffer.pull(); // pull a buffered LCD byte and output it
#else
			timerCommand &= ~tcLCDdelay; // signal to main program that delay timer has completed main program request
#endif

		}
		else lcdDelayCount--; // bump timer delay value down by one tick

	}
#endif

}
#endif

#ifdef useBufferedSerialPort
#ifdef ArduinoMega2560
ISR( USART0_UDRE_vect )
#else
ISR( USART_UDRE_vect )
#endif
{

	serialBuffer.pull(); // send a buffered character to the serial hardware

}
#endif

#ifdef useChryslerMAPCorrection
void readMAP(void)
{

	static unsigned int sample[2] = { 0, 0 };
	unsigned long wp;
	uint8_t analogToggle = 1;

	sampleCount = sampleTickLength - 1; // reset sample timer counter

	for (uint8_t x = 0; x < 2; x++)
	{

		// perform 2nd stage IIR filter operation
		sample[(unsigned int)(x)] = sample[(unsigned int)(x)] + 7 * analogValue[(unsigned int)(analogToggle)]; // first order IIR filter - filt = filt + 7/8 * (reading - filt)
		sample[(unsigned int)(x)] >>= 3;

		// calculate MAP and barometric pressures from readings
		wp = (unsigned long)sample[(unsigned int)(x)];
		if (wp < analogFloor[(unsigned int)(x)]) wp = 0;
		else wp -= analogFloor[(unsigned int)(x)];
		wp *= analogSlope[(unsigned int)(x)];
		wp >>= 10;
		pressure[(unsigned int)(MAPpressureIdx + x)] = wp + analogOffset[(unsigned int)(x)];

		analogToggle ^= 1;

	}

	// calculate differential pressure seen across the fuel injector
	wp = pressure[(unsigned int)(fuelPressureIdx)] + pressure[(unsigned int)baroPressureIdx] - pressure[(unsigned int)MAPpressureIdx];
	pressure[(unsigned int)(injPressureIdx)] = wp;

	// to get fuel pressure ratio, multiply differential pressure by denominator factor (1 << 12), then divide by fuel system pressure
	wp <<= 12;
	wp /= pressure[(unsigned int)(fuelPressureIdx)];

	// calculate square root of fuel pressure ratio
	pressure[(unsigned int)(injCorrectionIdx)] = (unsigned long)iSqrt((unsigned int)wp);

}
#endif

#ifdef useIsqrt
unsigned int iSqrt(unsigned int n)
{

	unsigned long w = 4096; // square factor guess
	unsigned int t = 4096; // proposed square root
	int d; // difference between guess and proposed
	int od = 0;

	for (uint8_t x = 0; x < 5; x++)
	{

		od = d;
		d = n - (unsigned int)w;
		d >>= 1;
		t += d;

		od += d;

		if ((d == 0) || (od == 0)) break;

		w = (unsigned long)t * (unsigned long)t;
		w >>= 12;

	}

	return t;

}
#endif

unsigned long lastVSScycle;

void updateVSS(unsigned long cycle)
{

	unsigned long cycleLength;

	uint8_t x = 1;
	uint8_t i = rawIdx;

	if (dirty & dirtyGoodVSS)
	{

		cycleLength = findCycleLength(lastVSScycle, cycle);

#ifdef trackIdleEOCdata
		if (!(dirty & dirtyGoodInj)) x++; // if no valid fuel injector event has been read, vehicle is in EOC mode
#endif

		for (uint8_t y = 0; y < x; y++)
		{

			tripArray[(unsigned int)(i)].collectedData[(unsigned int)(rvVSSpulseIdx)]++; // update the VSS pulse count

			tripArray[(unsigned int)(i)].add64s(rvVSScycleIdx, cycleLength); // add to VSS cycle accumulator

#ifdef trackIdleEOCdata
			i ^= (rawIdx ^ rawIdleIdx);
#endif

		}

		timerCommand |= tcWakeUp; // tell system timer to wake up the main program

	}

	vssResetCount = vssResetDelay;
	dirty |= dirtyGoodVSS; // annotate that a valid VSS pulse has been read

	lastVSScycle = cycle;

}

void initStatusLine(void)
{

	gotoXY(0, 0);
	clrEOL();
#ifdef blankScreenOnMessage
	gotoXY(0, 1);
	clrEOL();
#endif
	gotoXY(0, 0);

}

void execStatusLine(void)
{

	clrEOL();
	timerCommand |= tcDisplayDelay;
	while (timerCommand & tcDisplayDelay);

}

void clrEOL(void)
{

	while (hPos < 16) charOut(' ');

}

void gotoXY(uint8_t x, uint8_t y) // x = 0..16, y = 0..1
{

	LCD::gotoXY(x, y);

	hPos = x;
	vPos = y;

}

void blinkFlash(const char * str, uint8_t condition)
{

	uint8_t chr;
	uint8_t f = ((condition) && (timerHeartBeat & 0b01010101));

	hPos &= 0x7F;
	while (0 != (chr = pgm_read_byte(str++)))
	{

		if (f) charOut(' ');
		else charOut(chr);

	}

}

const char * findStr(const char * str, uint8_t strIdx)
{

	while (strIdx)
	{

		while (pgm_read_byte(str++));
		strIdx--;

	}

	return str;

}

void printStr(const char * str, uint8_t strIdx)
{

	printFlash(findStr(str, strIdx));

}

void printFlash(const char * str)
{

	uint8_t chr;

	hPos &= 0x7F;
	while (0 != (chr = pgm_read_byte(str++))) charOut(chr);

}

void print(char * str)
{

	hPos &= 0x7F;
	while (*str) charOut(*str++);

}

void charOut(uint8_t chr)
{

	if (chr == ignoreChar) hPos |= 0x80;
	else if ((chr == printChar) || (chr == '}')) hPos &= 0x7F;
	else if (hPos < 0x80)
	{

#ifdef blankScreenOnMessage
		if (!(timerStatus & tsDisplayDelay))
#else
		if ((!(timerStatus & tsDisplayDelay)) || (vPos > 0))
#endif
		{

			if ((chr > 0x07) && (chr < 0x10)) chr &= 0x07;
			LCD::writeData(chr);

		}

		hPos++;

	}

}

void loadCGRAM(const char * c)
{

	uint8_t s = pgm_read_byte(c++);

	if (cgramMode != s)
	{

		cgramMode = s;
		s = pgm_read_byte(c++);

		for (uint8_t x = 0; x < s; x++)
		{

			LCD::loadCGRAMcharacter(x, c, 1); //write the character data to the character generator ram
			c += 8;

		}

	}

}

#ifdef useBigTimeDisplay // Big time output section
void displayBigTime(char * val, uint8_t b)
{

	val[4] = val[0];
	val[5] = val[1];
	val[7] = val[2];
	val[8] = val[3];
	val[9] = 0;
	val[6] = ':';

	if (timerHeartBeat & 0b01010101) // if it's time to blink something
	{

		if (b == 4) val[6] = ';'; // if hh:mm separator is selected, blink it
		else if (b < 2) val[(unsigned int)(b + 4)] = ' '; // if digit 0 or 1 is selected, blink it
		else if (b < 4) val[(unsigned int)(b + 5)] = ' '; // otherwise, if digit 2 or 3 is selected, blink it

	}

	displayBigNumber(&val[4]);

}

#endif
#ifdef useBigNumberDisplay // Big number output section
void displayBigNumber(char * str)
{

	uint8_t c;
	uint8_t d;
	uint8_t e;
	uint8_t x = hPos;

 	while (*str)
	{

		c = *str++;
		d = *str;
		e = ' ';

		if ((d == '.') || (d == ':') || (d == ';'))
		{
  
			if (d == ':') e = decimalPtChar;
			if (d == ';') d = ' ';
			else d = decimalPtChar;
			str++;
  
  		}
  		else d = ' ';

		c -= '0';

		if (c == 240) c = 10;
		else if (c > 9) c = 11;

		loadCGRAM(bigNumFont);

		gotoXY(x, 0);
		printStr(bigNumChars1, c);
		charOut(e);
		gotoXY(x, 1);
		printStr(bigNumChars2, c);
		charOut(d);
		x += 4;

	}

	gotoXY(x, 0);

}

const uint8_t fedSelectList[] PROGMEM = {
	instantIdx,
	currentIdx,
	tankIdx,
};

void displayBigStatus(uint8_t dIdx, const char * str)
{

	initStatusLine();
	printStr(bigFEDispChars, fedSelect(dIdx));
	printFlash(str); // briefly display screen name
	execStatusLine();

}

uint8_t fedSelect(uint8_t dIdx)
{

	return pgm_read_byte(&fedSelectList[(unsigned int)(screenCursor[(unsigned int)(dIdx)])]);

}

#endif
#ifdef useBarGraph // Bar Graph Output support section

uint8_t bgPlotArea[16];
unsigned long barGraphData[(unsigned int)(bgDataSize)];

const uint8_t bgLabels[] PROGMEM = {
	'Q',	// fuel used
	'R',	// fuel rate
	'T',	// engine run time
	'T',	// time to empty
	'D',	// distance travelled
	'S',	// average speed
	'T',	// time in motion
	'E',	// fuel economy
	'Q',	// remaining fuel
	'D',	// remaining distance
	't',	// engine speed
	'u',	// fuel used, in microseconds
	'u',	// engine run time, in microseconds
	'u',	// time in motion, in microseconds
	'p',	// injector pulses
	'p',	// VSS pulses
#ifdef useFuelCost
	'C',	// fuel cost
	'C',	// fuel rate cost
	'C',	// fuel cost per distance
	'D',	// distance per fuel cost
	'C',	// remaining fuel cost
#endif
#ifdef useAnalogRead
#ifdef TinkerkitLCDmodule
	'V',	// voltage
	'V',	// voltage
	'V',	// voltage
#else
	'V',	// voltage
	'V',	// voltage
#ifdef useAnalogButtons
	'V',	// voltage
	'V',	// voltage
	'V',	// voltage
#endif
#endif
#endif
#ifdef useChryslerMAPCorrection
	'P',	// pressure
	'P',	// pressure
	'P',	// pressure
	'P',	// pressure
	'F',	// correction factor
#endif
};

const uint8_t tripIDchars[tripSlotCount] PROGMEM = {
	'r',
	'i',
	'c',
	't',
#ifdef trackIdleEOCdata
	'R',
	'I',
	'C',
	'T',
#endif
#ifdef useBarFuelEconVsTime
	'p',
#endif
#ifdef useBarFuelEconVsSpeed
	'0',
	'1',
	'2',
	'3',
	'4',
	'5',
	'6',
	'7',
	'8',
	'9',
	'A',
	'B',
	'C',	// *facepalm*
	'D',
	'E',
#endif
#ifdef useCoastDownCalculator
	'T',
	'L',
#endif
};

void clearBGplot(uint8_t yIdx)
{

	for (uint8_t x = 0; x < 16; x++) bgPlotArea[(unsigned int)(x)] = 0;
	if (yIdx < 16) bgPlotArea[(unsigned int)(15 - yIdx)] = 31;

}

uint8_t bgPlotConvert(uint8_t coord)
{

	if (coord == 254) coord = 15;
	else if (coord > 15) coord = 0;
	else coord = 15 - coord;

	return coord;

}

void bgPlot(uint8_t idx, uint8_t lowerPoint, uint8_t upperPoint, uint8_t mode)
{

	uint8_t i = idx + 3;
	uint8_t k = i % 5;

	lowerPoint = bgPlotConvert(lowerPoint);
	upperPoint = bgPlotConvert(upperPoint);

	if (lowerPoint < upperPoint)
	{

		lowerPoint ^= upperPoint;
		upperPoint ^= lowerPoint;
		lowerPoint ^= upperPoint;

	}

	uint8_t bitMask = (1 << (4 - k));

	while ((lowerPoint >= upperPoint) && (lowerPoint < 16))
	{

		if ((mode) && (timerHeartBeat & 0b01010101)) bgPlotArea[(unsigned int)(lowerPoint)] ^= bitMask;
		else bgPlotArea[(unsigned int)(lowerPoint)] |= bitMask;
		lowerPoint--;

	}

}

void bgOutputPlot(uint8_t idx, uint8_t yIdx)
{

	uint8_t i = idx + 3;
	uint8_t j = i / 5;
	uint8_t k = i % 5;

	if ((i == 3) || (k == 0))
	{

		if (i == 3)
		{

			for (uint8_t x = 0; x < 16; x++) bgPlotArea[(unsigned int)(x)] |= 16;

			for (uint8_t x = ((yIdx < 16) ? ((15 - yIdx) & 0x03): 3); x < 16; x += 4) bgPlotArea[(unsigned int)(x)] |= 8;

		}

		cgramMode = 0;

		LCD::loadCGRAMcharacter(j, (const char *)(&bgPlotArea[0]), 0);
		j |= 0x04;
		LCD::loadCGRAMcharacter(j, (const char *)(&bgPlotArea[8]), 0);

		clearBGplot(yIdx);

	}

}

uint8_t bgConvert(unsigned long v, unsigned long ll, unsigned long d)
{

	uint8_t b;

	v *= 15;
	if (v < ll) b = 254;
	else
	{

		v -= ll;
		if (d == 0) b = 7;
		else
		{

			v /= d;
			if (v > 15) b = 255;
			else b = (uint8_t)(v);

		}

	}

	return b;

}

void formatBarGraph(uint8_t bgSize, uint8_t slotIdx, unsigned long centerVal, unsigned long topLimit)
{

	uint8_t i;
	uint8_t k;
	uint8_t t;
	uint8_t y = 0;

	unsigned long v;
	unsigned long v1 = centerVal;
	unsigned long v2 = centerVal;
	unsigned long v3;
	unsigned long v4 = topLimit / 4;

	if (centerVal) v3 = topLimit - v4;
	else v3 = 0;
	v4 += topLimit;

	if (v1 < v3)
	{

		v = v1;
		v1 = v3;
		v3 = v;

	}

	i = 0;
	while (i < bgSize)
	{

		v = barGraphData[(unsigned int)(i)];
		if (v > v2)
		{

			if (v < v4) v2 = v;
			else v2 = v4;

		}
		if (v < v1)
		{

			if (v > v3) v1 = v;
			else v1 = v3;

		}
		i++;

	}

	if (v2 - v1)
	{

		v2 -= v1;

		v1 *= 15;

		y = bgConvert(centerVal, v1, v2);

		i = 0;
		while (i < bgSize)
		{

			mBuff2[(unsigned int)(i)] = bgConvert(barGraphData[(unsigned int)(i)], v1, v2);
			i++;

		}

	}
	else
	{

		y = 7;

		i = 0;
		while (i < bgSize) mBuff2[(unsigned int)(i++)] = y;

	}

	clearBGplot(y);
	k = bgDataSize;

	i = 0;

	while (i < bgSize)
	{

		k--;
		t = mBuff2[(unsigned int)(i)];

		if ((k == slotIdx) && (timerHeartBeat & 0b01010101))
		{

			if (t > 253) bgPlot(k, y, t, 1);
			else bgPlot(k, t, t, 1);

		}
		else
		{

			if (t > 253) bgPlot(k, y, t, 1);
			else bgPlot(k, y, t, 0);

		}

		bgOutputPlot(k, y);

		i++;

	}

	while ((--k) < bgDataSize) bgOutputPlot(k, y);

}

void displayBarGraphLine(uint8_t lineNum, uint8_t tripIdx, uint8_t tripCalcIdx)
{

	gotoXY(0, lineNum);

	for (uint8_t x = 0; x < 2; x++) charOut(' ');
	for (uint8_t x = 0; x < 4; x++) charOut(x + lineNum * 4 + 8);
	for (uint8_t x = 0; x < 2; x++) charOut(' ');

	if (tripIdx < 255)
	{

		charOut(pgm_read_byte(&tripIDchars[(unsigned int)(tripIdx)]));
		charOut(pgm_read_byte(&bgLabels[(unsigned int)(tripCalcIdx)]));

		print(doFormat(tripIdx, tripCalcIdx, 0));
	}
	else
	{

		clrEOL();

	}

}

void displayBarGraph(uint8_t trip1idx, uint8_t trip1CalcIdx, uint8_t trip2idx, uint8_t trip2CalcIdx)
{

	displayBarGraphLine(0, trip1idx, trip1CalcIdx);
	displayBarGraphLine(1, trip2idx, trip2CalcIdx);

}
#endif
#ifdef useLegacyButtons
const uint8_t errorButtonConflict = 1; // can  have only legacy buttons, Analog MUX buttons, or Parallax 5-button joystick selected
#endif

#ifdef useAnalogMuxButtons
const uint8_t errorButtonConflict = 1; // can  have only legacy buttons, Analog MUX buttons, or Parallax 5-button joystick selected
#endif

#ifdef useParallax5PositionSwitch
const uint8_t errorButtonConflict = 1; // can  have only legacy buttons, Analog MUX buttons, or Parallax 5-button joystick selected
#endif

#ifdef useParallaxLCD
const uint8_t errorSerialConflict = 1; // cannot have both Parallax LCD and serial data logging enabled

void LCD::init(void)
{

	delay2(delay0005ms);
	writeData(12);
	delay2(delay0005ms);
	writeData(22);
	writeData(232);
	setBright(brightnessIdx);

}

void LCD::gotoXY(uint8_t x, uint8_t y) // x = 0..16, y = 0..1
{

	uint8_t dr = 128 + 20 * y + x;

	writeData(dr);

}

void LCD::loadCGRAMcharacter(uint8_t chr, const char * chrData, uint8_t mode)
{

	uint8_t b = chr & 0x07;

	writeData(248 + b);

	for (uint8_t x = 0; x < 8; x++) writeData(((mode == 1) ? pgm_read_byte(chrData++) : *chrData++)); //write the character data to the character generator ram

}

void LCD::setBright(uint8_t idx)
{

	if (idx) writeData(17);
	else writeData(18);

}

void LCD::writeData(uint8_t value)
{

	pushSerialCharacter(value);

}
#endif
#ifdef useLegacyLCD

const uint8_t lcdNullValue = 				0b00000000;

const uint8_t lcdClearDisplay = 			0b00000001;

const uint8_t lcdReturnHome = 				0b00000010;

const uint8_t lcdEntryModeSet = 			0b00000100;
const uint8_t lcdEMSincrement = 			0b00000010;		// 1 = increment, 0 = decrement
const uint8_t lcdEMSsetDisplayShift = 		0b00000001;		// 1 = display shift, 0 = no display shift

const uint8_t lcdDisplayControl = 			0b00001000;
const uint8_t lcdDCdisplayShow = 			0b00000100;		// 1 = enable display, 0 = disable display
const uint8_t lcdDCcursorControl =			0b00000010;		// 1 = cursor on, 0 = cursor off
const uint8_t lcdDCcursorBlinkControl = 	0b00000001;		// 1 = cursor blink, 0 = cursor steady

const uint8_t lcdShift = 					0b00010000;
const uint8_t lcdSdisplayShift = 			0b00001000;		// 1 = shift display, 0 = cursor move
const uint8_t lcdSdirection = 				0b00000100;		// 1 = move right, 0 = move left

const uint8_t lcdFunctionSet = 				0b00100000;
const uint8_t lcdFSdataLength = 			0b00010000;		// 1 = 8 bit data, 0 = 4 bit data
const uint8_t lcdFSnumberOfLines = 			0b00001000;		// 1 = 2 lines, 0 = 1 line
const uint8_t lcdFScharacterFont = 			0b00000100;		// 1 = 5x10 dot character font, 0 = 5x8 dot character font

const uint8_t lcdSetCGRAMaddress = 			0b01000000;

const uint8_t lcdSetDDRAMaddress = 			0b10000000;

void LCD::init(void)
{
#ifndef TinkerkitLCDmodule

	TCCR0A &= ~((1 << COM0A0) | (1 << COM0B1) | (1 << COM0B0));  // put timer 0 in 8-bit fast pwm mode
	TCCR0A |= ((1 << COM0A1) | (1 << WGM01) | (1 << WGM00));
	TCCR0B &= ~((1 << FOC0A) | (1 << FOC0B) | (1 << WGM02) | (1 << CS02)); // set timer 0 prescale factor to 64
	TCCR0B |= ((1 << CS01) | (1 << CS00));
	TIMSK0 &= ~((1 << OCIE0B) | (1 << OCIE0A) | (1 << TOIE0)); // disable timer 0 interrupts
	TIFR0 |= ((1 << OCF0B) | (1 << OCF0A) | (1 << TOV0)); // clear timer 0 interrupt flags
#endif

#ifdef TinkerkitLCDmodule
	TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0) | (1 << COM1B0) | (1 << WGM11)); // put timer 1 in 8-bit phase correct pwm mode
	TCCR1A |= ((1 << COM1A1) | (1 << COM1B1) | (1 << WGM10));
#else
	TCCR1A &= ~((1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM11)); // put timer 1 in 8-bit phase correct pwm mode
	TCCR1A |= ((1 << COM1A1) | (1 << WGM10));
#endif
	TCCR1B &= ~((1 << ICNC1) | (1 << ICES1) | (1 << WGM13)  | (1 << WGM12) | (1 << CS12)); // set timer 1 prescale factor to 64
	TCCR1B |= ((1 << CS11) | (1 << CS10));
	TCCR1C &= ~((1 << FOC1A) | (1 << FOC1B));
	TIMSK1 &= ~((1 << ICIE1) | (1 << OCIE1B) | (1 << OCIE1A) | (1 << TOIE1)); // disable timer 1 interrupts
	TIFR1 |= ((1 << ICF1) | (1 << OCF1B) | (1 << OCF1A) | (1 << TOV1)); // clear timer 1 interrupt flags

#ifdef ArduinoMega2560
	DDRA = lcdBit3 | lcdBit2 | lcdBit1 | lcdBit0 | lcdEnable | lcdData; // set direction to output on selected port A pins
	DDRB = lcdBrightness | lcdContrast; // set direction to output on selected port B pins
#else
#ifdef TinkerkitLCDmodule
	DDRB = lcdBit3 | lcdContrast | lcdBrightness;
	DDRD = lcdBit2 | lcdBit1;
	DDRE = lcdEnable;
	DDRF = lcdData | lcdBit0 | lcdDirection;
	PORTF &= ~lcdDirection;
#else
	DDRB = lcdBit3 | lcdBit2 | lcdBit1 | lcdBrightness; // set direction to output on selected port B pins
	DDRD = lcdBit0 | lcdContrast | lcdEnable | lcdData; // set direction to output on selected port D pins
#endif
#endif

	setBright(brightnessIdx);
	setContrast(eepromReadVal((unsigned int)(pContrastIdx)));

	cgramMode = 0; // clear CGRAM font status

#ifdef useLegacyLCDbuffered
	lcdBuffer.init();
	lcdBuffer.process = LCD::outputNybble;
	lcdBuffer.onNoLongerEmpty = LCD::startOutput;
#endif
	writeNybble(lcdNullValue, lcdDelay0015ms); // wait for more than 15 msec
	writeNybble(lcdFunctionSet | lcdFSdataLength, lcdCommandByte | lcdSendByte | lcdDelay4100us); // send (B0011) to DB7-4, then wait for more than 4.1 ms
	writeNybble(lcdFunctionSet | lcdFSdataLength, lcdCommandByte | lcdSendByte | lcdDelay0100us); // send (B0011) to DB7-4, then wait for more than 100 us
	writeNybble(lcdFunctionSet | lcdFSdataLength, lcdCommandByte | lcdSendByte | lcdDelay0100us); // send (B0011) to DB7-4, then wait for more than 100 us
	writeNybble(lcdFunctionSet, lcdCommandByte | lcdSendByte | lcdDelay0100us); // send (B0010) to DB7-4 for 4 bit mode, then wait for more than 100 us

	// ready to use normal writeCommand() function now!
	writeCommand(lcdFunctionSet | lcdFSnumberOfLines); // 4-bit interface, 2 display lines, 5x8 font
	writeCommand(lcdDisplayControl | lcdDCdisplayShow); // display control:

	writeCommand(lcdClearDisplay); // clear display, set cursor position to zero
	writeNybble(lcdNullValue, lcdDelay0015ms); // wait for more than 15 msec for display clear

	gotoXY(0, 0);

}

void LCD::gotoXY(uint8_t x, uint8_t y) // x = 0..16, y = 0..1
{

	uint8_t dr = lcdSetDDRAMaddress | x;

	if (y == 1) dr += 0x40;
	writeCommand(dr);

}

void LCD::loadCGRAMcharacter(uint8_t chr, const char * chrData, uint8_t mode)
{

	uint8_t b = chr & 0x07;

	writeCommand(lcdEntryModeSet | lcdEMSincrement); // entry mode set: increment automatically, no display shift
	writeCommand(lcdSetCGRAMaddress + (b << 3)); // set CGRAM

	for (uint8_t x = 0; x < 8; x++) writeData(((mode == 1) ? pgm_read_byte(chrData++) : *chrData++)); //write the character data to the character generator ram

	writeCommand(lcdSetDDRAMaddress); // set DDRAM to zero

}

void LCD::setBright(uint8_t idx)
{

#ifdef TinkerkitLCDmodule
	OCR1B = pgm_read_byte(&brightness[(unsigned int)(idx)]);
#else
	OCR1A = pgm_read_byte(&brightness[(unsigned int)(idx)]);
#endif

}

void LCD::setContrast(uint8_t idx)
{

#ifdef TinkerkitLCDmodule
	OCR1A = idx;
#else
	OCR0A = idx;
#endif

}

void LCD::writeCommand(uint8_t value)
{

	value = writeNybble(value, lcdCommandByte | lcdSendByte | lcdDelay0080us);
	writeNybble(value, lcdCommandByte | lcdSendByte | lcdDelay0080us);

}

void LCD::writeData(uint8_t value)
{

	value = writeNybble(value, lcdDataByte | lcdSendByte | lcdDelay0080us);
	writeNybble(value, lcdDataByte | lcdSendByte | lcdDelay0080us);

}

uint8_t LCD::writeNybble(uint8_t value, uint8_t flags)
{

#ifdef useLegacyLCDbuffered
	lcdBuffer.push((value & 0xF0) | (flags & 0x0F));
#else
	while (timerCommand & tcLCDdelay);

	outputNybble((value & 0xF0) | (flags & 0x0F));
#endif

	return value << 4;

}

void LCD::startOutput(void)
{

	timerCommand |= tcLCDdelay;

}

void LCD::outputNybble(uint8_t LCDchar)
{

	lcdDelayCount = pgm_read_byte(&lcdDelayTable[(unsigned int)(LCDchar & 0x03)]);

	if (LCDchar & lcdSendByte)
	{

#ifdef ArduinoMega2560
		PORTA &= ~(lcdData | lcdBit3 | lcdBit2 | lcdBit1 | lcdBit0);
		if (LCDchar & lcdDataByte) PORTA |= lcdData; // set nybble type
		if (LCDchar & 0b10000000) PORTA |= lcdBit3; // set bit 3
		if (LCDchar & 0b01000000) PORTA |= lcdBit2; // set bit 2
		if (LCDchar & 0b00100000) PORTA |= lcdBit1; // set bit 1
		if (LCDchar & 0b00010000) PORTA |= lcdBit0; // set bit 0

		PORTA |= lcdEnable; // set enable high
		PORTA &= ~lcdEnable; // set enable low
#else
#ifdef TinkerkitLCDmodule
		PORTF &= ~(lcdData | lcdBit0);
		if (LCDchar & lcdDataByte) PORTF |= lcdData; // set nybble type
		if (LCDchar & 0b00010000) PORTF |= lcdBit0; // set bit 0

		PORTB &= ~(lcdBit3);
		if (LCDchar & 0b10000000) PORTB |= lcdBit3; // set bit 3

		PORTD &= ~(lcdBit2 | lcdBit1);
		if (LCDchar & 0b01000000) PORTD |= lcdBit2; // set bit 2
		if (LCDchar & 0b00100000) PORTD |= lcdBit1; // set bit 1

		PORTE |= lcdEnable; // set enable high
		PORTE &= ~lcdEnable; // set enable low
#else
		PORTD &= ~(lcdData | lcdBit0);
		if (LCDchar & lcdDataByte) PORTD |= lcdData; // set nybble type
		if (LCDchar & 0b00010000) PORTD |= lcdBit0; // set bit 0

		PORTB &= ~(lcdBit3 | lcdBit2 | lcdBit1);
		if (LCDchar & 0b10000000) PORTB |= lcdBit3; // set bit 3
		if (LCDchar & 0b01000000) PORTB |= lcdBit2; // set bit 2
		if (LCDchar & 0b00100000) PORTB |= lcdBit1; // set bit 1

		PORTD |= lcdEnable; // set enable high
		PORTD &= ~lcdEnable; // set enable low
#endif
#endif

	}

	timerCommand |= tcLCDdelay;

}
#endif
#ifdef useBuffering

void Buffer::init(void)
{

	bufferStart = 0;
	bufferEnd = 0;
	bufferStatus = bufferIsEmpty;

	onEmpty = doNothing;
	onNoLongerEmpty = doNothing;
	process = doNothing2;
	onNoLongerFull = doNothing;
	onFull = doNothing;

}

uint8_t Buffer::updatePointer(volatile uint8_t * pointer, uint8_t clearFlag, uint8_t setFlag)
{

	uint8_t i = * pointer;

	(* pointer)++;
	if ((* pointer) == bufferSize) (* pointer) = 0;
	bufferStatus &= ~clearFlag;
	if (bufferStart == bufferEnd) bufferStatus |= setFlag;

	return i;

}

void Buffer::push(uint8_t value)
{

	while (bufferStatus & bufferIsFull);

	uint8_t oldSREG = SREG; // save interrupt flag status
	cli(); // disable interrupts

	if (bufferStatus & bufferIsFull) onFull();
	else
	{

		if (bufferStatus & bufferIsEmpty) onNoLongerEmpty();
		storage[(unsigned int)(updatePointer(&bufferStart, bufferIsEmpty, bufferIsFull))] = value; // save a buffered character

	}

	SREG = oldSREG; // restore interrupt flag status

}

void Buffer::pull(void)
{

	uint8_t s = 0;

	uint8_t oldSREG = SREG; // save interrupt flag status
	cli(); // disable interrupts

	if (bufferStatus & bufferIsEmpty) onEmpty();
	else
	{

		if (bufferStatus & bufferIsFull) onNoLongerFull();
		process(storage[(unsigned int)(updatePointer(&bufferEnd, bufferIsFull, bufferIsEmpty))]); // get a buffered character

	}

	SREG = oldSREG; // restore interrupt flag status

}
#endif

void Trip::reset(void)
{

	for (uint8_t x = 0; x < rvLength; x++) collectedData[(unsigned int)(x)] = 0;

}

void Trip::transfer(Trip t)
{

	for (uint8_t x = 0; x < rvLength; x++) collectedData[(unsigned int)(x)] = t.collectedData[(unsigned int)(x)];

}

void Trip::update(Trip src)
{

	add32(rvVSSpulseIdx, src.collectedData[(unsigned int)(rvVSSpulseIdx)]);
	add32(rvInjPulseIdx, src.collectedData[(unsigned int)(rvInjPulseIdx)]);

	for (uint8_t x = rvVSScycleIdx; x < rvLength; x += 2)
	{

		add64s(x, src.collectedData[(unsigned int)(x)]);
		add32(x + 1, src.collectedData[(unsigned int)(x + 1)]);

	}

}

void Trip::add64s(uint8_t calcIdx, unsigned long v)
{

	add32(calcIdx, v); // add to accumulator
	if (collectedData[(unsigned int)(calcIdx)] < v) collectedData[(unsigned int)(calcIdx + 1)]++; // handle any possible overflow

}

void Trip::add32(uint8_t calcIdx, unsigned long v)
{

	collectedData[(unsigned int)(calcIdx)] += v;

}

#ifdef useWindowFilter
void Trip::subtract(Trip t)
{

	sub32(rvVSSpulseIdx, t.collectedData[(unsigned int)(rvVSSpulseIdx)]);
	sub32(rvInjPulseIdx, t.collectedData[(unsigned int)(rvInjPulseIdx)]);

	for (uint8_t x = 2; x < rvLength; x += 2)
	{

		if (collectedData[(unsigned int)(x)] < t.collectedData[(unsigned int)(x)]) collectedData[(unsigned int)(x + 1)]--;
		sub32(x, t.collectedData[(unsigned int)(x)]);
		sub32(x + 1, t.collectedData[(unsigned int)(x + 1)]);

	}

}

void Trip::sub32(uint8_t calcIdx, unsigned long v)
{

	collectedData[(unsigned int)(calcIdx)] -= v;

}

#endif
#ifdef useSavedTrips
unsigned int getBaseTripPointer(uint8_t tripPos)
{

	return (unsigned int)(tripPos) * (unsigned int)(tripListSize) + eePtrSavedTripsStart;

}

uint8_t Trip::load(uint8_t tripPos)
{

	unsigned int t = getBaseTripPointer(tripPos);
	uint8_t b = (uint8_t)(eepromReadVal((unsigned int)(t + tripListSigPointer)));

	reset();

	if (b == guinosig)
	{

		for (uint8_t x = 0; x < tripListLength; x++) collectedData[(unsigned int)(x)] = eepromReadVal(++t);

		b = 1;

	}
	else b = 0;

	return b;

}

uint8_t Trip::save(uint8_t tripPos)
{

	unsigned int t = getBaseTripPointer(tripPos);

#ifndef useClock
	unsigned long outputCycles[2];

	cli(); // perform atomic transfer of clock to main program

	outputCycles[0] = systemCycles[0]; // perform atomic transfer of system time to main program
	outputCycles[1] = systemCycles[1];

	sei();
#endif

	eepromWriteVal(t++, convertTime(outputCycles));

	for (uint8_t x = 0; x < tripListLength; x++) eepromWriteVal(t++, collectedData[(unsigned int)(x)]);

	eepromWriteVal(t++, guinosig);

	return 1;

}
#endif

unsigned long tmp1[2] = { 0, 0 };
unsigned long tmp2[2] = { 0, 0 };
unsigned long tmp3[2] = { 0, 0 };
unsigned long tmp4[2] = { 0, 0 };
unsigned long tmp5[2] = { 0, 0 };

union union_64 * tempPtr[5] = {
	(union union_64 *)&tmp1,
	(union union_64 *)&tmp2,
	(union union_64 *)&tmp3,
	(union union_64 *)&tmp4,
	(union union_64 *)&tmp5,
};

union union_64 * tu1 = tempPtr[0];
union union_64 * tu2 = tempPtr[1];

const uint8_t * const S64programList[] PROGMEM = {
	prgmFuelUsed,
	prgmFuelRate,
	prgmEngineRunTime,
	prgmTimeToEmpty,
	prgmDistance,
	prgmSpeed,
	prgmMotionTime,
	prgmFuelEcon,
	prgmRemainingFuel,
	prgmDistanceToEmpty,
	prgmEngineSpeed,
	prgmInjectorOpenTime,
	prgmInjectorTotalTime,
	prgmVSStotalTime,
	prgmInjectorPulseCount,
	prgmVSSpulseCount,
#ifdef useFuelCost
	prgmFuelCost,
	prgmFuelRateCost,
	prgmFuelCostPerDistance,
	prgmDistancePerFuelCost,
	prgmRemainingFuelCost,
#endif
#ifdef useAnalogRead
#ifdef TinkerkitLCDmodule
	prgmVoltage,
	prgmVoltage,
	prgmVoltage,
#else
	prgmVoltage,
	prgmVoltage,
#ifdef useAnalogButtons
	prgmVoltage,
	prgmVoltage,
	prgmVoltage,
#endif
#endif
#endif
#ifdef useChryslerMAPCorrection
	prgmPressure,
	prgmPressure,
	prgmPressure,
	prgmPressure,
	prgmCorrF,
#endif
	prgmFindRemainingFuel,
	prgmDoMultiply,
	prgmDoDivide,
	prgmFindCyclesPerQuantity,
	prgmConvertToMicroSeconds,
	prgmDoAdjust,
	prgmFormatToNumber,
};

unsigned long SWEET64(const uint8_t * sched, uint8_t tripIdx)
{

	uint8_t spnt = 0;
	uint8_t instr;
	uint8_t b;
	uint8_t f;
	const uint8_t * prgmStack[16];
	uint8_t tf = 0;

	while (true)
	{

#ifdef useSWEET64trace
		if (tf)
		{

			pushSerialCharacter(13);
			pushHexWord((unsigned int)(sched));

		}

#endif
		instr = pgm_read_byte(sched++);

#ifdef useSWEET64trace
		if (tf)
		{

			pushSerialCharacter(32);
			pushHexByte(tripIdx);
			pushSerialCharacter(32);
			pushHexByte(spnt);
			pushSerialCharacter(32);
			pushHexByte(instr);

		}
#endif

		if (instr & 0x40)
		{

			b = pgm_read_byte(sched++) - 0x11;
#ifdef useSWEET64trace

			if (tf)
			{

				pushSerialCharacter(32);
				pushHexByte(b);

			}
#endif

			tu1 = tempPtr[(unsigned int)((b >> 4) & 0x07)];
			tu2 = tempPtr[(unsigned int)(b & 0x07)];

		}

		if (instr & 0x80)
		{

			b = pgm_read_byte(sched++);

#ifdef useSWEET64trace
			if (tf)
			{

				pushSerialCharacter(32);
				pushHexByte(b);

			}

#endif
		}

#ifdef useSWEET64trace
		if (tf) pushSerialCharacter(13);
#endif
		f = 0;

		if ((instr == instrLdNumer) || (instr == instrLdDenom)) b = pgm_read_byte(&convNumerIdx[(unsigned int)(tripIdx)]);
		if (instr == instrLdDenom) b ^= 1;
		if ((instr == instrLdEEPROMindirect) || (instr == instrStEEPROMindirect)) b = pgm_read_byte(&convIdx[(unsigned int)(tripIdx)]);
		if (instr == instrLdEEPROMindexed) b += tripIdx;

		if ((instr == instrLdNumer) || (instr == instrLdDenom)) instr = instrLdConst;
		else if ((instr == instrLdEEPROMindexed) || (instr == instrLdEEPROMindirect)) instr = instrLdEEPROM;
		else if (instr == instrStEEPROMindirect) instr = instrStEEPROM;

		if (instr == instrDone)
		{

			if (spnt--) sched = prgmStack[(unsigned int)(spnt)];
			else break;

		}
		else if (instr == instrTraceOn) tf = 1;
		else if (instr == instrTraceOff) tf = 0;
		else if (instr == instrSkip) f = 1;
		else if (instr == instrSkipIfMetricMode) f = metricFlag;
		else if (instr == instrSkipIfZero) f = zeroTest64(tu2);
		else if (instr == instrSkipIfLTorE) f = ltOrEtest64(tu1, tu2);
		else if (instr == instrSkipIfLSBset) f = lsbTest64(tu2);
		else if (instr == instrSkipIfMSBset) f = msbTest64(tu2);
		else if (instr == instrSkipIfIndexBelow) f = (tripIdx < pgm_read_byte(sched++));
		else if (instr == instrLd) copy64(tu1, tu2);
		else if (instr == instrLdByte) init64(tu2, b);
		else if (instr == instrLdByteFromYindexed) init64(tu1, tu2->u8[(unsigned int)(tripIdx)]);
		else if (instr == instrLdTripVar) tripVarLoad64(tu2, tripIdx, b);
		else if (instr == instrLdTtlFuelUsed) tripVarLoad64(tu2, tankIdx, rvInjOpenCycleIdx);
		else if (instr == instrLdConst) init64(tu2, pgm_read_dword(&convNumbers[(unsigned int)(b)]));
		else if (instr == instrLdEEPROM) init64(tu2, eepromReadVal((unsigned int)(b)));
		else if (instr == instrStEEPROM) EEPROMsave64(tu2, b);
		else if (instr == instrStByteToYindexed) tu2->u8[(unsigned int)(tripIdx)] = tu1->u8[0];
		else if (instr == instrLdIndex) tripIdx = b;
		else if (instr == instrCall)
		{
			prgmStack[(unsigned int)(spnt++)] = sched;
			if (spnt > 15) break;
			else sched = (const uint8_t *)pgm_read_word(&S64programList[(unsigned int)(b)]);
		}
		else if (instr == instrJump) sched = (const uint8_t *)pgm_read_word(&S64programList[(unsigned int)(b)]);
		else if (instr == instrSwap) swap64(tu1, tu2);
		else if (instr == instrSubYfromX) add64(tu1, tu2, 1);
		else if (instr == instrAddYtoX) add64(tu1, tu2, 0);
#ifndef useSWEET64multDiv
		else if (instr == instrMulXbyY) mul64(tu1, tu2);
		else if (instr == instrDivXbyY) div64(tu1, tu2);
#endif
		else if (instr == instrShiftLeft) shl64(tu2);
		else if (instr == instrShiftRight) shr64(tu2);
		else if (instr == instrAddToIndex) tripIdx += b;
#ifdef useAnalogRead
		else if (instr == instrLdVoltage) init64(tu2, analogValue[(unsigned int)(tripIdx)]);
#endif
#ifdef useChryslerMAPCorrection
		else if (instr == instrLdPressure) init64(tu2, pressure[(unsigned int)(tripIdx)]);
#endif
#ifdef useIsqrt
		else if (instr == instrIsqrt) tu2->ui[0] = iSqrt(tu2->ui[0]);
#endif
		else break; // just found an unsupported opcode

#ifdef useSWEET64trace
		if (tf)
		{

			for (uint8_t x = 0;x < 5; x++)
			{

				pushSerialCharacter(9);
				pushHexDWord(tempPtr[(unsigned int)(x)]->ul[1]);
				pushSerialCharacter(32);
				pushHexDWord(tempPtr[(unsigned int)(x)]->ul[0]);
				pushSerialCharacter(13);

			}

		}
#endif

		if (f)
		{

			if (b < 128) sched += b;
			else sched -= (256 - b);
#ifdef useSWEET64trace

			if (tf)
			{

				pushSerialCharacter(9);
				pushHexWord((unsigned int)(sched));
				pushSerialCharacter(13);

			}
#endif

		}

#ifdef useSWEET64trace
		if (tf) pushSerialCharacter(13);

#endif
	}

	return tempPtr[1]->ul[0];

}

#ifdef useSerialDebugOutput
void pushHexNybble(uint8_t val)
{

	val &= 0x0F;
	if (val < 0x0A) pushSerialCharacter(val + 0x30);
	else pushSerialCharacter(val + 0x37);

}

void pushHexByte(uint8_t val)
{

	pushHexNybble(val >> 4);
	pushHexNybble(val);

}

void pushHexWord(unsigned int val)
{
	pushHexByte(val >> 8);
	pushHexByte(val);
}

void pushHexDWord(unsigned long val)
{
	pushHexWord(val >> 16);
	pushHexWord(val);
}
#endif

void copy64(union union_64 * an, union union_64 * ann)
{

	for (uint8_t x = 0; x < 8; x++) an->u8[(unsigned int)(x)] = ann->u8[(unsigned int)(x)];

}

void tripVarLoad64(union union_64 * an, uint8_t tripIdx, uint8_t dataIdx)
{

	if (dataIdx < rvVSScycleIdx) init64(an, tripArray[(unsigned int)(tripIdx)].collectedData[(unsigned int)(dataIdx)]);
	else copy64(an, (union union_64 *)&tripArray[(unsigned int)(tripIdx)].collectedData[(unsigned int)(dataIdx)]);

}

void EEPROMsave64(union union_64 * an, uint8_t dataIdx)
{

	eepromWriteVal((unsigned int)(dataIdx), an->ul[0]);

}

void init64(union union_64 * an, unsigned long dWordL)
{

	an->ull = 0;
	an->ul[0] = dWordL;

}

void swap64(union union_64 * an, union union_64 * ann) // swap ann and an
{

	uint8_t b = 0;

	for (uint8_t x = 0; x < 8; x++)
	{

		b = ann->u8[(unsigned int)(x)];
		ann->u8[(unsigned int)(x)] = an->u8[(unsigned int)(x)];
		an->u8[(unsigned int)(x)] = b;

	}

}

void shr64(union union_64 * an)
{

	uint8_t b = 0;
	uint8_t c;

	for (uint8_t x = 7; x < 8; x--)
	{

		c = b;
		b = ((an->u8[(unsigned int)(x)] & 0x01) ? 0x80 : 0x00);
		an->u8[(unsigned int)(x)] >>= 1;
		an->u8[(unsigned int)(x)] += c;

	}

}

void shl64(union union_64 * an)
{

	uint8_t b = 0;
	uint8_t c;

	for (uint8_t x = 0; x < 8; x++)
	{

		c = b;
		b = ((an->u8[(unsigned int)(x)] & 0x80) ? 0x01 : 0x00);
		an->u8[(unsigned int)(x)] <<= 1;
		an->u8[(unsigned int)(x)] += c;

	}

}

void add64(union union_64 * an, union union_64 * ann, uint8_t mode)
{

	uint8_t d;
	int enn;
	union union_16 * n = (union union_16 *)&enn;

	n->u8[1] = ((mode) ? 0x01 : 0x00);
	for (uint8_t x = 0; x < 8; x++)
	{

		d = ((mode) ? 0xFF : 0x00);
		d ^= ann->u8[(unsigned int)(x)];
		n->u8[0] = n->u8[1];
		n->u8[1] = 0;
		n->ui += (unsigned int)an->u8[(unsigned int)(x)];
		n->ui += d;
		an->u8[(unsigned int)(x)] = n->u8[0];

	}

}

#ifndef useSWEET64multDiv
void mul64(union union_64 * an, union union_64 * ann)
{

	union union_64 * multiplier = tempPtr[3];
	union union_64 * multiplicand = tempPtr[4];

	copy64(multiplier, an);
	copy64(multiplicand, ann);
	an->ull = 0;

	while (!(zeroTest64(multiplier)))
	{

		if (lsbTest64(multiplier)) add64(an, multiplicand, 0);

		shl64(multiplicand);
		shr64(multiplier);

	}

}

void div64(union union_64 * an, union union_64 * ann) // dividend in an, divisor in ann
{

	union union_64 * quotientBit = tempPtr[3];
	union union_64 * divisor = tempPtr[4];

	copy64(divisor, ann); // copy ann value to divisor
	copy64(ann, an); // copy an value (dividend) to ann (this will become remainder)
	an->ull = 0; // zero out result
	init64(quotientBit, 1); // initialize quotient mark bit

	if (zeroTest64(divisor))
	{ // if divisor is zero, mark as overflow, then exit

		add64(an, quotientBit, 1); // subtract 1 from zeroed-out result to generate overflow value
		copy64(ann, an); // copy overflow value to remainder

	}
	else if (!(zeroTest64(ann)))
	{ // if dividend is not zero,

		while (!(msbTest64(divisor)))
		{ // ensure that divisor MSB is set

			shl64(divisor); // shift divisor left one bit
			shl64(quotientBit); // shift quotient mark bit left by one

		}

		while (!(zeroTest64(quotientBit)))
		{ // continue while there is a quotient mark bit

			if (ltOrEtest64(divisor, ann))
			{ // if divisor is less than or equal to dividend,
				add64(ann, divisor, 1); // subtract divisor value from dividend
				add64(an, quotientBit, 0); // mark corresponding bit in quotient
			}

			shr64(divisor); // shift divisor right by one bit
			shr64(quotientBit); // shift quotient mark bit right by one bit

		}

	}

}
#endif

uint8_t zeroTest64(union union_64 * an)
{

	uint8_t b = 0;

	for (uint8_t x = 0; x < 8; x++) b |= an->u8[(unsigned int)(x)];

	return (b == 0);

}

uint8_t ltOrEtest64(union union_64 * an, union union_64 * ann)
{

	uint8_t b = 1;

	for (uint8_t x = 7; x < 8; x--)
	{

		if (an->u8[(unsigned int)(x)] < ann->u8[(unsigned int)(x)]) break;
		else if (an->u8[(unsigned int)(x)] > ann->u8[(unsigned int)(x)])
		{

			b = 0;
			break;

		}

	}

	return (b == 1);

}

uint8_t lsbTest64(union union_64 * an)
{

	return ((an->u8[0] & 0x01) != 0);

}

uint8_t msbTest64(union union_64 * an)
{

	return ((an->u8[7] & 0x80) != 0);

}

char * doFormat(uint8_t tripIdx, uint8_t dispPos)
{

	uint8_t r = (tripIdx & dfTripMask) >> dfBitShift;
	uint8_t f = tripIdx & dfValMask;

	return doFormat(r, f, dispPos);

}

#if useFuelCost
const uint8_t lblCGRAMextendedBase = 40;
#else
const uint8_t lblCGRAMextendedBase = 28;
#endif

const uint8_t calcLabelIdx[] PROGMEM = { // +128 is metric flag - metric label has offset of 2 bytes from index
	20 + 128,			// fuel used
	24 + 128,			// fuel rate
	0,				// engine run time
	0,				// time to empty
	12 + 128,			// distance travelled
	16 + 128,			// speed
	0,				// time in motion
	8 + 128,			// fuel economy
	20 + 128,			// remaining fuel
	12 + 128,			// remaining distance
	6,				// engine speed
	2,				// fuel used, in microseconds
	2,				// engine run time, in microseconds
	2,				// time in motion, in microseconds
	4,				// injector pulses
	4,				// injector pulses
#ifdef useFuelCost
	28,				// fuel cost
	30,				// fuel rate cost
	32 + 128,			// fuel cost per unit distance
	36 + 128,			// distance per unit fuel cost
	28,				// fuel cost remaining
#endif
#ifdef useAnalogRead
#ifdef TinkerkitLCDmodule
	lblCGRAMextendedBase,		// voltage
	lblCGRAMextendedBase,		// voltage
	lblCGRAMextendedBase,		// voltage
#else
	lblCGRAMextendedBase,		// voltage
	lblCGRAMextendedBase,		// voltage
#ifdef useAnalogButtons
	lblCGRAMextendedBase,		// voltage
	lblCGRAMextendedBase,		// voltage
	lblCGRAMextendedBase,		// voltage
#endif
#endif
#endif
#ifdef useChryslerMAPCorrection
	lblCGRAMextendedBase + 2 + 128,	// voltage
	lblCGRAMextendedBase + 2 + 128,	// voltage
	lblCGRAMextendedBase + 2 + 128,	// voltage
	lblCGRAMextendedBase + 2 + 128,	// voltage
	lblCGRAMextendedBase + 6,	// correction factor
#endif
};

unsigned long doCalculate(uint8_t calcIdx, uint8_t tripIdx)
{

	uint8_t i = tripIdx;
#ifdef useAnalogRead
	if ((calcIdx >= dfMaxValCount) && (calcIdx < dfMaxValAnalogCount)) i = calcIdx - dfMaxValCount;
#endif
#ifdef useChryslerMAPCorrection
	if ((calcIdx >= dfMaxValAnalogCount) && (calcIdx < dfMaxValMAPCount)) i = calcIdx - dfMaxValAnalogCount;
#endif

	return SWEET64((const uint8_t *)pgm_read_word(&S64programList[(unsigned int)(calcIdx)]), i);

}

char * format64(const uint8_t * prgmPtr, unsigned long num, char * str, uint8_t ndp)
{

	uint8_t b;
	uint8_t c;

	init64(tempPtr[1], num);
	SWEET64(prgmPtr, ndp);

	uint8_t l = tempPtr[2]->u8[6];	// load total length

	if (l == 255) strcpy_P(str, overFlowStr);
	else
	{

		uint8_t z = tempPtr[2]->u8[7];	// load leading zero character

		for (uint8_t x = 0; x < l; x++)
		{

			uint8_t y = x * 2;
			b = tempPtr[2]->u8[(unsigned int)(x)];
			c = b / 10;
			b -= c * 10;
			c = ((c) ? c + 48 : z);
			if (c > 48) z = 48;
			if ((x + 1) == l) z = 48;
			b = ((b) ? b + 48 : z);
			if (b > 48) z = 48;
			str[(unsigned int)(y)] = c;
			str[(unsigned int)(y + 1)] = b;

		}

		str[l * 2] = 0;

	}

	return str;

}

char * format(unsigned long num, uint8_t ndp)
{

	uint8_t x = 9;
	uint8_t y = 10;
	uint8_t c;

	format64(prgmRoundOffNumber, num, mBuff1, ndp);

	if (mBuff1[2] != '-')
	{

		while (x > 5)
		{

			if (y != 7)
			{

				c = mBuff1[(unsigned int)(x)];
				if (c == ' ') c = '0';
				x--;

			}
			else c = '.';

			mBuff1[(unsigned int)(y)] = c;
			y--;

		}

		x = 1;

		if (ndp)
		{

			y = 2;

			while ((y < 2 + ndp) && (mBuff1[(unsigned int)(y)] == ' '))
			{

				y++;
				x = y;

			}

		}

		for (uint8_t z = 0; z < 6; z++)
		{

			mBuff1[(unsigned int)(z)] = mBuff1[(unsigned int)(x)];
			x++;

		}

		mBuff1[6] = 0;

	}

	return mBuff1;

}

char * doFormat(uint8_t tripIdx, uint8_t calcIdx, uint8_t dispPos)
{

	uint8_t numDecPt = pgm_read_byte(&calcDecimalPoints[(unsigned int)(calcIdx)]);
	uint8_t calcWord = pgm_read_byte(&calcLabelIdx[(unsigned int)(calcIdx)]);

	uint8_t p;
	uint8_t c;
	unsigned long an;

	if ((calcIdx < dfMaxValDisplayCount) && (tripIdx < tripSlotCount))
	{

		an = doCalculate(calcIdx, tripIdx);

		if ((dispPos & dispRaw) || (dispPos & dispFE) || (dispPos & dispDTE))
		{

			if (numDecPt) format(an, 3);
			else format64(prgmFormatToNumber, an, mBuff1, 3);

			if (dispPos & dispFE)
			{

 				c = 3;

			}

			if (dispPos & dispDTE)
			{

 				c = 4;

			}

			if ((dispPos & dispFE) || (dispPos & dispDTE))
			{

				p = 0;
				if (mBuff1[2] != '-') // if number did not overflow
				{
  
					
					if ((mBuff1[(unsigned int)(2)] == '.') || ((mBuff1[(unsigned int)(3)] == '.') && (dispPos & dispDTE)))
					{

						if (mBuff1[0] == ' ') p++; // if number is less than 10, point to start of number
						c++; // update end of number

					}
					else if (mBuff1[(unsigned int)(c)] != '.') strcpy_P(mBuff1, overFlowStr); // if number is greater than 999(9), mark as overflow

				}

				if (mBuff1[2] == '-') p++; // if number overflowed, point to start of overflow dashes

				if (p > 0) for (uint8_t x = 0; x < (c + 1); x++) mBuff1[(unsigned int)(x)] = mBuff1[(unsigned int)(p++)];

				mBuff1[(unsigned int)(c)] = 0;

			}

		}
		else
		{

			if (calcWord == 0) format64(prgmFormatToTime, an, mBuff1, 0);
			else format(an, numDecPt);

		}

	}
	else
	{

		strcpy_P(mBuff1, overFlowStr);

	}

	return mBuff1;

}

unsigned long rformat(void)
{

	unsigned long v = 0ul;
	uint8_t c;

	for (uint8_t p = 0; p < 10; p++)
	{

		c = pBuff[(unsigned int)(p)];
		if (c == 32) c = 0;
		else c -= '0';
		v *= 10;
		v += c;

	}

	return v;

}

const uint8_t prgmConvertToTime[] PROGMEM = {
	instrLdConst, 0x01, idxCyclesPerSecond,
	instrJump, idxS64doDivide
};

unsigned long convertTime(unsigned long * an)
{

	copy64(tempPtr[1], (union union_64 *)(an));
	return SWEET64(prgmConvertToTime, 0);

}

#ifdef useChryslerMAPCorrection
const uint8_t prgmGenerateVoltageSlope[] PROGMEM = {
	instrLdEEPROMindexed, 0x02, pMAPsensorCeilingIdx,
	instrLdEEPROMindexed, 0x01, pMAPsensorFloorIdx,
	instrSubYfromX, 0x21,
	instrSwap, 0x23,

	instrLdConst, 0x02, idxDenomVoltage,
	instrLdEEPROMindexed, 0x01, pMAPsensorRangeIdx,
	instrCall, idxS64doMultiply,

	instrSwap, 0x13,
	instrJump, idxS64doDivide,
};

const uint8_t prgmConvertVolts[] PROGMEM = {
	instrLdEEPROMindexed, 0x02, pMAPsensorFloorIdx,
	instrLdConst, 0x01, idxNumerVoltage,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxDenomVoltage,
	instrJump, idxS64doDivide,
};
#endif

const uint8_t prgmConvertInjSettleTime[] PROGMEM = {
	instrLdConst, 0x01, idxCyclesPerSecond,
	instrLdEEPROM, 0x02, pInjectorSettleTimeIdx,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxMicroSecondsPerSecond,
	instrJump, idxS64doDivide,
};

const uint8_t prgmFindSleepTicks[] PROGMEM = {
	instrLdConst, 0x01, idxCyclesPerSecond,
	instrLdEEPROM, 0x02, pActivityTimeoutIdx,
	instrCall, idxS64doMultiply,
	instrLdIndex, 0,
	instrShiftRight, 0x02,
	instrAddToIndex, 1,
	instrSkipIfIndexBelow, 249, 8,
	instrDone
};

const uint8_t prgmFindMinGoodRPM[] PROGMEM = {
	instrLdByte, 0x01, 60,						// load seconds per minute into register 1
	instrLdEEPROM, 0x02, pCrankRevPerInjIdx,			// load crank revolutions per injector event into register 2
	instrCall, idxS64doMultiply,					// perform multiply
	instrLdConst, 0x01, idxCyclesPerSecond,				// load cycles per second into register 1
	instrCall, idxS64doMultiply,					// perform conversion
	instrLdEEPROM, 0x01, pMinGoodRPMidx,				// get minimum good RPM figure from EEPROM
	instrCall, idxS64doDivide,					// convert figure into cycles
	instrLd, 0x32,							// move result into register 3 (minGoodRPMcycles)
	instrDone
};

const uint8_t prgmFindInjResetDelay[] PROGMEM = {
	instrLdIndex, 0,						// divide by 256 to generate timer2 cycles
	instrShiftRight, 0x02,
	instrAddToIndex, 1,
	instrSkipIfIndexBelow, 249, 8,
	instrLdByte, 0x01, 2,						// add 2 to result
	instrAddYtoX, 0x21,						// register 2 (injResetDelay)
	instrDone
};

const uint8_t prgmFindMaxGoodInjCycles[] PROGMEM = {
	instrLd, 0x23,							// load register 2 with contents of register 3
	instrLdByte, 0x01, 80,						// multiply minGoodRPMcycles figure by 0.8
	instrCall, idxS64doMultiply,
	instrLdByte, 0x01, 100,
	instrJump, idxS64doDivide,					// (maxGoodInjCycles)
};

#ifdef useBarFuelEconVsTime
const uint8_t prgmFindFEvsTimePeriod[] PROGMEM = {
	instrLdByte, 0x01, loopsPerSecond,
	instrLdEEPROM, 0x02, pFEvsTimeIdx,
	instrJump, idxS64doMultiply,
};

#endif
#ifdef useWindowFilter
uint8_t windowFilterIdx = 0;
uint8_t windowFilterCount = 0;

void resetWindowFilter(void)
{

	tripArray[(unsigned int)(windowFilterSumIdx)].reset();
	windowFilterCount = 0;
	windowFilterIdx = 0;

}

#endif
void initGuino(void) // initialize all the parameters
{

	uint8_t injDirection;

	vssPause = (uint8_t)eepromReadVal((unsigned int)(pVSSpauseIdx));
	metricFlag = (uint8_t)eepromReadVal((unsigned int)(pMetricFlagIdx));
	ignoreChar = (metricFlag ? '{' : '\\');
	printChar = ignoreChar ^ ('{' ^ '\\');

#ifdef useWindowFilter
	resetWindowFilter();

#endif
	cli(); // disable interrupts while messing with fuel injector settings

#ifdef useChryslerMAPCorrection

	for (uint8_t x = 0; x < 2; x++)
	{

		analogFloor[(unsigned int)(x)] = SWEET64(prgmConvertVolts, 0);
		analogSlope[(unsigned int)(x)] = SWEET64(prgmGenerateVoltageSlope, x);
		analogOffset[(unsigned int)(x)] = eepromReadVal((unsigned int)(pMAPsensorOffsetIdx + x));

	}

	pressure[(unsigned int)fuelPressureIdx] = eepromReadVal((unsigned int)(pSysFuelPressureIdx)); // this is in psig * 1000
	pressure[(unsigned int)injCorrectionIdx] = 4096;

#endif

	dirty &= ~(dirtyGoodInj | dirtyInjOpenRead); // reset fuel injector capture mechanism

#ifdef ArduinoMega2560
	EIMSK &= ~((1 << INT5) | (1 << INT4)); // disable fuel injector sense interrupts

	EICRB |= ((1 << ISC51) | (1 << ISC50) | (1 << ISC41) | (1 << ISC40)); // set injector sense pin control
	EICRB &= ~(1 << (eepromReadVal((unsigned int)(pInjEdgeTriggerIdx)) ? ISC50 : ISC40));

	EIFR |= ((1 << INTF5) | (1 << INTF4)); // clear fuel injector sense flag
	EIMSK |= ((1 << INT5) | (1 << INT4)); // enable fuel injector sense interrupts
#else
	EIMSK &= ~((1 << INT1) | (1 << INT0)); // disable fuel injector sense interrupts

	EICRA |= ((1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00)); // set injector sense pin control
	EICRA &= ~(1 << (eepromReadVal((unsigned int)(pInjEdgeTriggerIdx)) ? ISC10 : ISC00));

	EIFR |= ((1 << INTF1) | (1 << INTF0)); // clear fuel injector sense flag
	EIMSK |= ((1 << INT1) | (1 << INT0)); // enable fuel injector sense interrupts
#endif

	// convert seconds into cycles
	sleepTicks = SWEET64(prgmFindSleepTicks, 0);
	// convert microseconds into timer2 clock cycles
	injSettleCycles =  SWEET64(prgmConvertInjSettleTime, 0);
	// minimum time that consecutive injector open pulses must be received
	minGoodRPMcycles = SWEET64(prgmFindMinGoodRPM, 0);
	// used by main timer to timeout any long pending injector reads
	injResetDelay = SWEET64(prgmFindInjResetDelay, 0);
	// maximum time that injector may be open (should be 0.8 times the minimum good RPM time)
	maxGoodInjCycles = SWEET64(prgmFindMaxGoodInjCycles, 0);

	sei(); // re-enable interrupts

#ifdef useBarFuelEconVsTime
	bFEvTperiod = (unsigned int)SWEET64(prgmFindFEvsTimePeriod, 0);
	doResetBarFEvT();
#endif

}

void delay2(unsigned int ms)
{

	timerDelayCount = ms; // request a set number of timer tick delays per millisecond
	timerCommand |= tcDoDelay; // signal request to timer
	while (timerCommand & tcDoDelay);

}

#ifdef useSerialPortDataLogging
const uint8_t errorSerialConflict = 1; // cannot have both Parallax LCD and serial data logging enabled

const uint8_t dataLogInstr[] PROGMEM = {
(instantIdx << dfBitShift) | tFuelEcon,				// average fuel economy  for the past loop
(instantIdx << dfBitShift) | tSpeed,				// average vehicle speed for the past loop
(instantIdx << dfBitShift) | tInjectorOpenTime,		// fuel injector raw open time for the past loop
(instantIdx << dfBitShift) | tInjectorPulseCount,	// fuel injector pulse count for the past loop
(instantIdx << dfBitShift) | tVSSpulseCount,		// vss pulse count for the past loop
};

const uint8_t dLIcount = (sizeof(dataLogInstr) / sizeof(uint8_t));

void doOutputDataLog(void)
{

	uint8_t c = ',';

	for (uint8_t x = 0; x < dLIcount; x++)
	{

		if ((x + 1) == dLIcount) c = '\n';

		simpletx(doFormat(pgm_read_byte(&dataLogInstr[(unsigned int)(x)]), dispRaw));
		pushSerialCharacter(c);

	}

}

void simpletx(char * str)
{

	while (*str) pushSerialCharacter(*str++);

}
#endif

#ifdef useSerialPort
void pushSerialCharacter(uint8_t chr)
{

#ifdef useBufferedSerialPort

	serialBuffer.push(chr);

#else
	if (UCSR0B != (1 << TXEN0)) UCSR0B = (1 << TXEN0); // if serial output is not yet enabled, enable it

	while (!(UCSR0A & (1 << UDRE0))); // wait until transmit buffer is empty

	UDR0 = chr; //send the data
#endif

}

#ifdef useBufferedSerialPort
void serialTransmitEnable(void)
{

	UCSR0B = ((1 << TXEN0) | (1 << UDRIE0)); // Enable transmitter and interrupt

}

void serialTransmitDisable(void)
{

	UCSR0B = 0; // Disable transmitter and interrupt

}

void serialTransmitByte(uint8_t s)
{

	UDR0 = s; // Transmit a byte

}

#endif
#endif

/* Display cursor update section */

void doCursorMoveAbsolute(uint8_t i, uint8_t j)
{
#ifdef useScreenEditor

	if (menuLevel == screenEditIdx) doSaveScreen();
#endif

	menuLevel = i;
	if (pgm_read_byte(&screenParameters[(unsigned int)(menuLevel)][2]) > j) screenCursor[(unsigned int)(menuLevel)] = j;

	callFuncPointer(&screenParameters[(unsigned int)(menuLevel)][4]);

}

void doCursorMoveRelative(uint8_t i, uint8_t j)
{

	uint8_t k = 0;
	uint8_t v;
	uint8_t w = pgm_read_byte(&screenParameters[(unsigned int)(menuLevel)][1]);
	uint8_t x = pgm_read_byte(&screenParameters[(unsigned int)(menuLevel)][0]);
	uint8_t y = menuLevel - x;
	uint8_t z = pgm_read_byte(&screenParameters[(unsigned int)(menuLevel)][2]);
#ifdef useScreenEditor

	if (menuLevel == screenEditIdx) doSaveScreen();
#endif

	if (j)
	{

		v = screenCursor[(unsigned int)(menuLevel)] + j;

		if (v == z)
		{

			v = 0;
			i = j;
			k = 1;

		}
		else if (v > z)
		{

			v = z - 1;
			i = j;
			k = 1;

		}

		screenCursor[(unsigned int)(menuLevel)] = v;

	}

	if (i)
	{

		y += i;

		if (y == w) y = 0;
		if (y > w) y = w - 1;
		menuLevel = y + x;

		if (k)
		{

			if (i == 1) v = 0;
			else v = pgm_read_byte(&screenParameters[(unsigned int)(menuLevel)][2]) - 1;

			screenCursor[(unsigned int)(menuLevel)] = v;

		}

	}

	doRefreshDisplay(); // call the appropriate display routine
	callFuncPointer(&screenParameters[(unsigned int)(menuLevel)][4]);

}

void doRefreshDisplay(void)
{

	gotoXY(0, 0);
	callFuncPointer(&screenParameters[(unsigned int)(menuLevel)][3]);

}

void doNothing(void)
{
}

void doNothing2(uint8_t s)
{
}

void noSupport(void)
{

	initStatusLine();
	printFlash(PSTR("Btn "));
	print(itoa((unsigned int)(buttonState), mBuff1, 10));
	printFlash(PSTR(" Pressed"));
	execStatusLine();

}

const uint8_t calcLabelCGRAM[] PROGMEM = {
	0b00000000,		// clock
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000011,
	0b00000100,
	0b00000100,
	0b00000011,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00010100,
	0b00010101,
	0b00010110,
	0b00010101,

	0b00010010,		// microseconds
	0b00010010,
	0b00010010,
	0b00011110,
	0b00010000,
	0b00110000,
	0b01000000,
	0b01100000,

	0b00001110,
	0b00010000,
	0b00001100,
	0b00000010,
	0b00011100,
	0b00000000,
	0b00000000,
	0b00000000,

	0b00000000,		// pulse count
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000011,
	0b00000100,
	0b00000100,
	0b00000011,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00001000,
	0b00011100,
	0b00001000,
	0b00001000,

	0b00001100,		// revolutions per minute
	0b00010000,
	0b00010001,
	0b00010010,
	0b00000100,
	0b00001000,
	0b00000000,
	0b00000000,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00000000,
	0b00011011,
	0b00010101,
	0b00010101,

	0b00011011,		// miles per gallon
	0b00010101,
	0b00010101,
	0b00000000,
	0b00000001,
	0b00000010,
	0b00000100,
	0b00001000,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00001100,
	0b00010000,
	0b00010100,
	0b00001100,

	0b00010000,		// liters per 100 km
	0b00010000,
	0b00011001,
	0b00000010,
	0b00000100,
	0b00001001,
	0b00000001,
	0b00000001,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00000000,
	0b00011111,
	0b00010101,
	0b00011111,

	0b00000000,		// miles
	0b00000000,
	0b00000000,
	0b00000000,
	0b00010001,
	0b00011011,
	0b00010101,
	0b00010101,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00010000,
	0b00000000,
	0b00010000,
	0b00010000,

	0b00000000,		// kilometers
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000100,
	0b00000101,
	0b00000110,
	0b00000101,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00000000,
	0b00001010,
	0b00010101,
	0b00010101,

	0b00011011,		// miles per hour
	0b00010101,
	0b00010101,
	0b00000000,
	0b00000110,
	0b00000101,
	0b00000110,
	0b00000100,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00010100,
	0b00011100,
	0b00010100,
	0b00010100,

	0b00010000,		// kilometers per hour
	0b00010100,
	0b00011000,
	0b00010100,
	0b00000001,
	0b00000010,
	0b00000100,
	0b00001000,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00010000,
	0b00010000,
	0b00011100,
	0b00010100,

	0b00000000,		// gallons
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000011,
	0b00000100,
	0b00000101,
	0b00000011,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00000001,
	0b00000001,
	0b00011001,
	0b00011101,

	0b00000000,		// liters
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000100,
	0b00000100,
	0b00000100,
	0b00000111,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,

	0b00001100,		// gallons per hour
	0b00010000,
	0b00010100,
	0b00001100,
	0b00000001,
	0b00000010,
	0b00000100,
	0b00001000,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00010000,
	0b00010000,
	0b00011100,
	0b00010100,

	0b00010000,		// liters per hour
	0b00010000,
	0b00010000,
	0b00011100,
	0b00000001,
	0b00000010,
	0b00000100,
	0b00001000,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00010000,
	0b00010000,
	0b00011100,
	0b00010100,
#ifdef useFuelCost

	0b00100000,		// fuel cost
	0b01000000,
	0b01100000,
	0b00000000,
	0b00001100,
	0b00010000,
	0b00010000,
	0b00001100,

	0b00000010,
	0b00000101,
	0b00000100,
	0b00001110,
	0b00000100,
	0b00000100,
	0b00000100,
	0b00000100,

	0b00001100,		// fuel cost rate
	0b00010000,
	0b00010000,
	0b00001100,
	0b00000001,
	0b00000010,
	0b00000100,
	0b00001000,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00010000,
	0b00010000,
	0b00011100,
	0b00010100,

	0b00001100,		// fuel cost per mile
	0b00010000,
	0b00010000,
	0b00001100,
	0b00000001,
	0b00000010,
	0b00000100,
	0b00001000,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00010001,
	0b00011011,
	0b00010101,
	0b00010101,

	0b00001100,		// fuel cost per kilometer
	0b00010000,
	0b00010000,
	0b00001100,
	0b00000001,
	0b00000010,
	0b00000100,
	0b00001000,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00010000,
	0b00010100,
	0b00011000,
	0b00010100,

	0b00011011,		// mile per unit fuel cost
	0b00010101,
	0b00010101,
	0b00000000,
	0b00000001,
	0b00000010,
	0b00000100,
	0b00001000,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00001100,
	0b00010000,
	0b00010000,
	0b00001100,

	0b00010000,		// kilometer per unit fuel cost
	0b00010100,
	0b00011000,
	0b00010100,
	0b00000001,
	0b00000010,
	0b00000100,
	0b00001000,

	0b00100000,
	0b01000000,
	0b01100000,
	0b00000000,
	0b00001100,
	0b00010000,
	0b00010000,
	0b00001100,
#endif
#ifdef useAnalogRead

	0b00010001,		// voltage
	0b00010001,
	0b00010001,
	0b00001010,
	0b00000100,
	0b00000000,
	0b00000000,
	0b00000000,

	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00010001,
	0b00011010,
	0b00011010,
	0b00010001,
#endif
#ifdef useChryslerMAPCorrection

	0b00001110,		// psi
	0b00001001,
	0b00001001,
	0b00001110,
	0b00001000,
	0b00001000,
	0b00001000,
	0b00000000,

	0b00001001,
	0b00010101,
	0b00010001,
	0b00001001,
	0b00000101,
	0b00010101,
	0b00001001,
	0b00000000,

	0b00001000,		// kPa
	0b00001000,
	0b00001001,
	0b00001010,
	0b00001100,
	0b00001010,
	0b00001001,
	0b00000000,

	0b00011000,
	0b00010100,
	0b00010100,
	0b00011000,
	0b00010010,
	0b00010101,
	0b00010011,
	0b00000000,

	0b00000000,		// correction factor
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,

	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
#endif
};

const uint8_t calcLabelTrip[] PROGMEM = {
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,

	0b00000011,
	0b00000111,
	0b00000011,
	0b00000111,

	0b00000100,
	0b00000010,
	0b00000100,
	0b00000010,

	0b00000100,
	0b00000111,
	0b00000011,
	0b00000010,
};

void displayMainScreenFunction(uint8_t readingIdx, uint8_t k, uint8_t functBlink, uint8_t tripBlink)
{

	readingIdx &= 3;
	uint8_t x = (readingIdx & 1) << 3;
	uint8_t y = (readingIdx & 2) >> 1;
	uint8_t z = readingIdx << 1;

	uint8_t r = (k & dfTripMask) >> dfBitShift;
	uint8_t f = k & dfValMask;

	uint8_t j = pgm_read_byte(&calcLabelIdx[(unsigned int)(f)]);
	if ((j & 128) && (metricFlag)) j += 2;
	j &= 127;

	writeCGRAMlabelChar(z, j, r, functBlink, tripBlink);
	writeCGRAMlabelChar(z + 1, j + 1, r, functBlink, tripBlink);

	gotoXY(x, y);
	print(doFormat(k, 0));
	charOut(8 + z);
	charOut(9 + z);

}

void writeCGRAMlabelChar(uint8_t cgChar, uint8_t functIdx, uint8_t tripIdx, uint8_t functBlink, uint8_t tripBlink)
{

	uint8_t i = 0x1F;
	uint8_t j = 0x1F;
	unsigned int k = (unsigned int)(functIdx << 3);
	if (timerHeartBeat & tripBlink) j = 0; // determine if trip label component should blink or not
	if (timerHeartBeat & functBlink) i = 0; // determine if function label component should blink or not
	tripIdx &= 3; // strip off unnecessary bits of trip index

	for (uint8_t x = 0; x < 8; x++)
	{

		 uint8_t l = pgm_read_byte(&calcLabelCGRAM[(unsigned int)(k++)]); // read a byte of function label bit pattern
		 uint8_t m = l >> 3; // fetch partial address of trip label component
		 m &= 0b00001100; // strip off un-needed bits
		 m |= tripIdx; // combine with trip index to form full address of trip label component
		 m = pgm_read_byte(&calcLabelTrip[(unsigned int)(m)]); // read a byte of trip label bit pattern
		 l &= i; // provide for blinking function label component
		 m &= j; // provide for blinking trip label component
		 mBuff1[(unsigned int)(x)] = l | m; // combine trip label and function label components

	}

	cgramMode = 0; // reset CGRAM mode
	LCD::loadCGRAMcharacter(cgChar, (const char *)(mBuff1), 0); // write out generated CGRAM character

}

/* Main screen section */

const char mainScreenFuncNames[] PROGMEM = {
	"Instrument\0"
	"Custom\0"
#ifdef useChryslerMAPCorrection
	"Pressures\0"
#endif
#ifdef useAnalogRead
	"Voltages\0"
#endif
	"Instant/Current\0"
	"Instant/Tank\0"
	"Current\0"
	"Tank\0"
#ifdef trackIdleEOCdata
	"EOC/Idle\0"
#endif
	"Tank Data\0"
	"Current Data\0"
#ifdef trackIdleEOCdata
	"Tank EOC/Idle\0"
	"Current EOC/Idle\0"
#endif
	"Remaining\0"
};

void doCursorUpdateMain(void)
{

	printStatusMessage(findStr(mainScreenFuncNames, screenCursor[(unsigned int)(mainScreenIdx)])); // briefly display screen name

}

void doMainScreenDisplay(void)
{

	uint8_t i = screenCursor[(unsigned int)(mainScreenIdx)];

	i <<= 2;

	for (uint8_t x = 0; x < 4; x++)
	{

#ifdef useScreenEditor
		uint8_t k = displayFormats[(unsigned int)(i++)];
#else
		uint8_t k = pgm_read_byte(&displayFormats[(unsigned int)(i++)]);
#endif
		displayMainScreenFunction(x, k, 0, 136);

	}

}

void doNextBright(void)
{

	brightnessIdx++;
	if (brightnessIdx >= brightnessLength) brightnessIdx = 0;
	LCD::setBright(brightnessIdx);

	initStatusLine();
	printFlash(PSTR("Backlight = "));
	printStr(brightString, brightnessIdx);
	execStatusLine();

}

void doLongGoLeft(void)
{

	doCursorMoveRelative(255, 0);

}

void doLongGoRight(void)
{

	doCursorMoveRelative(1, 0);

}

void doTripResetTank(void)
{

	tripArray[(unsigned int)(tankIdx)].reset();
#ifdef trackIdleEOCdata
	tripArray[(unsigned int)(eocIdleTankIdx)].reset();
#endif
#ifdef useBarFuelEconVsSpeed
	doResetBarFEvS();
#endif
	printStatusMessage(PSTR("Tank Reset"));

}

void doTripResetCurrent(void)
{

	tripArray[(unsigned int)(currentIdx)].reset();
#ifdef trackIdleEOCdata
	tripArray[(unsigned int)(eocIdleCurrentIdx)].reset();
#endif
	printStatusMessage(PSTR("Current Reset"));

}

/* Setting selector section */

void doCursorUpdateSetting(void)
{

	paramPtr = screenCursor[(unsigned int)(settingScreenIdx)] + (uint8_t)(eePtrSettingsStart);
	doParamRevert();

}

void doSettingEditDisplay(void)
{

	printStr(parmLabels, screenCursor[(unsigned int)(settingScreenIdx)]); // print parameter name at top left
	clrEOL();
	gotoXY(0, 1); // go to next line
	print(pBuff);
	clrEOL();

}

void doGoSettingsEdit(void)
{

	prevMenuLevel = menuLevel;
	doCursorMoveAbsolute(settingScreenIdx, 0);

}

void doReturnToMain(void)
{

	menuLevel = prevMenuLevel;

}

/* Individual parameter editor section */

void doParamEditDisplay(void)
{

	printStr(parmLabels, screenCursor[(unsigned int)(settingScreenIdx)]); // print parameter name at top left
	clrEOL();
	gotoXY(0, 1); // go to next line

	uint8_t c = pBuff[(unsigned int)(screenCursor[(unsigned int)(paramScreenIdx)])]; // save existing character
	if ((timerHeartBeat & 0b01010101) && (screenCursor[(unsigned int)(paramScreenIdx)] < 10)) pBuff[(unsigned int)(screenCursor[(unsigned int)(paramScreenIdx)])] = '_'; // replace character with an underscore
	print(pBuff); // print number
	pBuff[(unsigned int)(screenCursor[(unsigned int)(paramScreenIdx)])] = c;

	blinkFlash(&paramButtonChars[0], (screenCursor[(unsigned int)(paramScreenIdx)] == 10));
	blinkFlash(&paramButtonChars[4], (screenCursor[(unsigned int)(paramScreenIdx)] == 11));

}

void doGoParamEdit(void)
{

	paramLength = pgm_read_byte(&paramsLength[(unsigned int)(screenCursor[(unsigned int)(settingScreenIdx)])]);
	paramMaxValue = (1 << paramLength);
	paramMaxValue -= 1;

	menuLevel = paramScreenIdx;
	format64(prgmFormatToNumber, paramMaxValue, mBuff2, 3);
	doParamFindLeft();

}

void doParamExit(void)
{

	doParamRevert();
	generalMenuLevelReturn(PSTR("Param Reverted"), settingScreenIdx);

}

#ifdef useCalculatedFuelFactor
const uint8_t prgmCalculateFuelFactor[] PROGMEM = {
	instrLdConst, 0x02, idxCorrFactor,
	instrLdEEPROM, 0x01, pSysFuelPressureIdx,
	instrCall, idxS64doMultiply,
	instrLdEEPROM, 0x01, pRefFuelPressureIdx,
	instrCall, idxS64doDivide,
	instrIsqrt, 0x02,
	instrLdEEPROM, 0x01, pInjectorCountIdx,
	instrCall, idxS64doMultiply,
	instrLdEEPROM, 0x01, pInjectorSizeIdx,
	instrCall, idxS64doMultiply,
	instrSkipIfMetricMode, 10,

	instrLdConst, 0x01, idxNumerVolume,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxDenomVolume,
	instrCall, idxS64doDivide,

	instrSwap, 0x23,
	instrLdByte, 0x02, 60,								// load seconds per minute into register 2
	instrLdConst, 0x01, idxMicroSecondsPerSecond,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxDecimalPoint,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxCorrFactor,
	instrCall, idxS64doMultiply,
	instrSwap, 0x13,
	instrCall, idxS64doDivide,
	instrStEEPROM, 0x02, pMicroSecondsPerQuantityIdx,
	instrDone
};
#endif

const uint8_t prgmDoEEPROMmetricConversion[] PROGMEM = {
	instrTraceOn,
	instrLdIndex, 0,

	instrLdEEPROMindirect, 0x02,

	instrSkipIfMetricMode, 6,
	instrLdNumer, 0x01,
	instrLdDenom, 0x03,
	instrSkip, 4,

	instrLdNumer, 0x03,
	instrLdDenom, 0x01,

	instrCall, idxS64doMultiply,
	instrSwap, 0x31,
	instrCall, idxS64doDivide,
	instrCall, idxS64doAdjust,
	instrStEEPROMindirect, 0x02,
	instrAddToIndex, 1,
	instrSkipIfIndexBelow, 227, convSize,

	instrDone
};

void doParamSave(void)
{

	uint8_t t;

	if (eepromWriteVal((unsigned int)(paramPtr), rformat())) // if the setting has changed
	{

#ifdef useBarFuelEconVsSpeed
		if ((paramPtr == pBarLowSpeedCutoffIdx) || (paramPtr == pBarSpeedQuantumIdx)) doResetBarFEvS();
#endif

		if (paramPtr == pMetricFlagIdx) SWEET64(prgmDoEEPROMmetricConversion, 0); // if metric flag has changed

#ifdef useCalculatedFuelFactor
		// if fuel pressure, reference pressure, injector count, or injector size changed
		if ((paramPtr == pSysFuelPressureIdx) || (paramPtr == pRefFuelPressureIdx) || (paramPtr == pInjectorCountIdx) || (paramPtr == pInjectorSizeIdx))
			SWEET64(prgmCalculateFuelFactor, 0); // calculate and store microseconds per gallon factor
#endif

		initGuino(); // reconfigure system based on changed settings
		generalMenuLevelReturn(PSTR("Param Changed"), settingScreenIdx);

	}
	else generalMenuLevelReturn(PSTR("Param Unchanged"), settingScreenIdx);

}

void generalMenuLevelReturn(const char * s, uint8_t newMenuLevel)
{

	menuLevel = newMenuLevel;
	printStatusMessage(s);

}

void printStatusMessage(const char * s)
{

	initStatusLine();
	printFlash(s);
	execStatusLine();

}

void doParamFindLeft(void)
{

	screenCursor[(unsigned int)(paramScreenIdx)] = 9;

	// do a nice thing and put the edit cursor at the first non zero number
	for (uint8_t x = 9; x < 10; x--) if (pBuff[(unsigned int)(x)] != ' ') screenCursor[(unsigned int)(paramScreenIdx)] = x;

}

void doParamFindRight(void)
{

	screenCursor[(unsigned int)(paramScreenIdx)] = 9;

}

void doParamStoreMax(void)
{

	doParamStoreNumber(paramMaxValue);

}

void doParamStoreMin(void)
{

	doParamStoreNumber(0);

}

void doParamRevert(void)
{

	doParamStoreNumber(eepromReadVal((unsigned int)(paramPtr)));

}

void doParamStoreNumber(unsigned long v)
{

	format64(prgmFormatToNumber, v, pBuff, 3);
#ifdef useLegacyLCD
	if (paramPtr == pContrastIdx) LCD::setContrast((uint8_t)(v)); // adjust contrast dynamically
#endif
	doParamFindLeft();

}

void doParamReformat(void)
{

	uint8_t c = '0';
	uint8_t d = ' ';

	for (uint8_t x = 0; x < 9; x++)
	{

		if (pBuff[(unsigned int)(x)] == c) pBuff[(unsigned int)(x)] = d;
		else if ((c == '0') && (pBuff[(unsigned int)(x)] != ' '))
		{

			c = ' ';
			d = '0';

		}

	}

	if (pBuff[9] == ' ') pBuff[9] = '0';

}

void doParamChangeDigit(void)
{

	uint8_t w;

	if (screenCursor[(unsigned int)(paramScreenIdx)] == 10) doParamSave();
	else if (screenCursor[(unsigned int)(paramScreenIdx)] == 11) doParamExit();
	else
	{

		if (paramLength == 1) pBuff[(unsigned int)(screenCursor[(unsigned int)(paramScreenIdx)])] ^= 1;
		else
		{

			w = pBuff[(unsigned int)(screenCursor[(unsigned int)(paramScreenIdx)])]; // fetch digit from stored numeric string representing parameter to be changed
			if (w == ' ') w = '0'; // if this is a leading space, use 0 as working digit
			w++; // adjust working digit
			if (w > '9') w = '0'; // handle working digit rollover

			pBuff[(unsigned int)(screenCursor[(unsigned int)(paramScreenIdx)])] = w;
			doParamReformat();

			for (uint8_t x = 0; x < 10; x++)
			{

				if (pBuff[(unsigned int)(x)] < mBuff2[(unsigned int)(x)]) x = 10;
				else if (pBuff[(unsigned int)(x)] > mBuff2[(unsigned int)(x)])
				{

					x = 10;
					pBuff[(unsigned int)(screenCursor[(unsigned int)(paramScreenIdx)])] = '0';
					doParamReformat();

				}

			}

#ifdef useLegacyLCD
			if (paramPtr == pContrastIdx) LCD::setContrast((uint8_t)(rformat())); // adjust contrast dynamically
#endif

		}

	}

}

#ifdef useBigFE // large Fuel Economy display support section
void doCursorUpdateBigFEscreen(void)
{

	displayBigStatus(bigFEscreenIdx, PSTR(" Fuel Econ"));

}

void doBigFEdisplay(void)
{

	uint8_t dIdx = fedSelect(bigFEscreenIdx);

       	displayBigNumber(doFormat(dIdx, tFuelEcon, dispFE));

	printStr(bigFEDispChars, dIdx);
	gotoXY(12, 1);
	printFlash(PSTR("{MPG \\L100}"));

}

#endif
#ifdef useBigDTE // large Distance-To-Empty display support section
void doCursorUpdateBigDTEscreen(void)
{

	displayBigStatus(bigDTEscreenIdx, PSTR(" DistToEmpty"));

}

void doBigDTEdisplay(void)
{

       	displayBigNumber(doFormat(fedSelect(bigDTEscreenIdx), tDistanceToEmpty, dispDTE));

}

#endif
#ifdef useBigTTE // large Time-To-Empty display support section
void doCursorUpdateBigTTEscreen(void)
{

  	displayBigStatus(bigTTEscreenIdx, PSTR(" TimeToEmpty"));

}

void doBigTTEdisplay(void)
{

	displayBigTime(format64(prgmFormatToTime, SWEET64(prgmTimeToEmpty, fedSelect(bigTTEscreenIdx)), mBuff1, 3), 4);

}

#endif
#ifdef useClock // Clock support section
void doCursorUpdateSystemTimeScreen(void)
{

	printStatusMessage(PSTR("System Time"));

}

void doDisplaySystemTime(void) // display system time
{

	displayBigTime(format64(prgmFormatToTime, convertTime(outputCycles), mBuff1, 3), 4);

}

void doGoEditSystemTime(void)
{

	format64(prgmFormatToTime, convertTime(outputCycles), pBuff, 3); // convert system time from ticks into seconds, and format for output
	doCursorMoveAbsolute(systemTimeEditScreenIdx, 0);

}

void doEditSystemTimeDisplay(void)
{

	displayBigTime(pBuff, screenCursor[(unsigned int)(systemTimeEditScreenIdx)]);

}

void doEditSystemTimeChangeDigit(void)
{

	pBuff[(unsigned int)(screenCursor[(unsigned int)(systemTimeEditScreenIdx)])]++;
	if (pBuff[(unsigned int)(screenCursor[(unsigned int)(systemTimeEditScreenIdx)])] > '9') pBuff[(unsigned int)(screenCursor[(unsigned int)(systemTimeEditScreenIdx)])] = '0';

	if (pBuff[2] > '5') pBuff[2] = '0'; // this will only happen if systemTimeEditScreenIdx == 2
	if ((pBuff[0] == '2') && (pBuff[1] > '3')) pBuff[1] = '0'; // this will only happen if systemTimeEditScreenIdx == 0 or 1
	if (pBuff[0] > '2') pBuff[0] = '0'; // this will only happen if systemTimeEditScreenIdx == 0

}

const uint8_t prgmConvertToCycles[] PROGMEM = {
	instrLdConst, 0x01, idxCyclesPerSecond,
	instrCall, idxS64doDivide,
	instrLdConst, 0x01, idxSecondsPerDay,
	instrCall, idxS64doDivide,

	instrLdIndex, 0,
	instrLdByte, 0x01, 24,
	instrCall, idxS64doMultiply,
	instrLdByteFromYindexed, 0x13,
	instrAddYtoX, 0x21,

	instrLdIndex, 2,
	instrLdByte, 0x01, 60,
	instrCall, idxS64doMultiply,
	instrLdByteFromYindexed, 0x13,
	instrAddYtoX, 0x21,

	instrLdIndex, 4,
	instrLdByte, 0x01, 60,
	instrCall, idxS64doMultiply,
	instrLdByteFromYindexed, 0x13,
	instrAddYtoX, 0x21,

	instrLdConst, 0x01, idxCyclesPerSecond,
	instrJump, idxS64doMultiply,
};

void doEditSystemTimeSave(void)
{

	uint8_t b;

	pBuff[4] = '0';
	pBuff[5] = '0';

	copy64(tempPtr[1], (union union_64 *)&outputCycles);

	for (uint8_t x = 4; x < 6; x -= 2)
	{

		b = pBuff[(unsigned int)(x)] - '0';
		b *= 10;
		b += pBuff[(unsigned int)(x + 1)] - '0';
		tempPtr[2]->u8[(unsigned int)(x)] = b;

	}

	SWEET64(prgmConvertToCycles, 0); // convert time into timer2 clock cycles

	cli();
	copy64((union union_64 *)&clockCycles, tempPtr[1]);
	sei();

	generalMenuLevelReturn(PSTR("Time Set"), systemTimeDisplayScreenIdx);

}

void doEditSystemTimeCancel(void)
{

	generalMenuLevelReturn(PSTR("Time NOT Set"), systemTimeDisplayScreenIdx);

}

#endif
#ifdef useBarFuelEconVsSpeed // (parameter) vs. Speed Bar Graph display section
uint8_t FEvSpdTripIdx;

void doCursorUpdateBarFEvS(void)
{

	uint8_t b = pgm_read_byte(&barFEvSdisplayFuncs[(unsigned int)(screenCursor[(unsigned int)(barFEvSscreenIdx)])]);

	for (uint8_t x = 0; x < bgDataSize; x++) barGraphData[(unsigned int)(bgDataSize - x - 1)] = doCalculate(b, (x + FEvsSpeedIdx));

	printStatusMessage(findStr(barFEvSfuncNames, screenCursor[(unsigned int)(barFEvSscreenIdx)])); // briefly display screen name

}

void doBarFEvSdisplay(void)
{

	uint8_t b = pgm_read_byte(&barFEvSdisplayFuncs[(unsigned int)(screenCursor[(unsigned int)(barFEvSscreenIdx)])]);

	if (FEvSpdTripIdx < 255) barGraphData[(unsigned int)(bgDataSize + FEvsSpeedIdx - FEvSpdTripIdx - 1)] = doCalculate(b, FEvSpdTripIdx);

	formatBarGraph(bgDataSize, (FEvSpdTripIdx - FEvsSpeedIdx), 0, doCalculate(b, tankIdx));

	displayBarGraph(FEvSpdTripIdx, b, ((timerHeartBeat & 0b00110011) ? tankIdx : instantIdx), ((timerHeartBeat & 0b00110011) ? b : tSpeed));

}

void doResetBarFEvS(void)
{

	for (uint8_t x = 0; x < bgDataSize; x++) tripArray[(unsigned int)(x + FEvsSpeedIdx)].reset();

}

#endif
#ifdef useBarFuelEconVsTime // Differential/Absolute Fuel Economy vs. Time display section
void doResetBarFEvT(void)
{

	tripArray[(unsigned int)(periodIdx)].reset();
	bFEvTcount = 0;
	bFEvTstartIDx = 0;
	bFEvTsize = 0;

}

void doCursorUpdateBarFEvT(void)
{

	printStatusMessage(findStr(barFEvTfuncNames, screenCursor[(unsigned int)(barFEvTscreenIdx)])); // briefly display screen name

}

void doBarFEvTdisplay(void)
{

	uint8_t i = 0;
	uint8_t j = bFEvTstartIDx;
	unsigned long v = doCalculate(tFuelEcon, currentIdx);

	while (i < bFEvTsize)
	{

		if (j == 0) j = bgDataSize;
		j--;

		barGraphData[(unsigned int)(i)] = barFEvsTimeData[(unsigned int)(j)];
		i++;

	}

	formatBarGraph(bFEvTsize, (bgDataSize - 1), ((screenCursor[(unsigned int)(barFEvTscreenIdx)]) ? 0 : v), v);

	displayBarGraph(currentIdx, tFuelEcon, periodIdx, tFuelEcon);

}

#endif
#ifdef useCPUreading // System utilization display section
void doDisplaySystemInfo(void) // display max cpu utilization and RAM
{

	unsigned int i = (unsigned int)(&i);
	unsigned long t[2];

	if ((unsigned int) __brkval == 0) i -= (unsigned int)(&__bss_end);
	else i -= (unsigned int)(__brkval);

	unsigned long mem = (unsigned long)i;
	mem *= 1000;

	cli(); // perform atomic transfer of clock to main program

	t[0] = systemCycles[0]; // perform atomic transfer of system time to main program
	t[1] = systemCycles[1];

	sei();

	displayCPUutil();
	printFlash(PSTR(" T"));
	print(format64(prgmFormatToTime, convertTime(t), mBuff1, 3));
	gotoXY(0, 1);
	printFlash(PSTR(" FREE MEM:"));
	print(format(mem, 0));

}

const uint8_t prgmFindCPUutilPercent[] PROGMEM = {
	instrLdConst, 0x01, idxNumerCPUutil,
	instrCall, idxS64doMultiply,
	instrLdConst, 0x01, idxDenomCPUutil,
	instrJump, idxS64doDivide,
};

void displayCPUutil(void)
{

	printFlash(PSTR("C%"));
	init64(tempPtr[1], timerLoopLength);
	print(format(SWEET64(prgmFindCPUutilPercent, 0), 2));

}

void doShowCPU(void)
{

	initStatusLine();
	displayCPUutil();
	execStatusLine();

}

#ifdef useBenchMark
const uint8_t prgmBenchMarkTime[] PROGMEM = {
	instrJump, idxS64doConvertToMicroSeconds,
};

void doBenchMark(void)
{

	unsigned long t = 0;
	unsigned long w;
	unsigned long s;
	unsigned long e;

	unsigned long c;

	cli(); // disable interrupts

	while (!(TIFR2 & (1 << TOV2))); // wait for timer2 to overflow
	TIFR2 |= (1 << TOV2); // reset timer2 overflow flag

#ifdef TinkerkitLCDmodule
	s = TCNT0; // do a microSeconds() - like read to determine loop length in cycles
#else
	s = TCNT2; // do a microSeconds() - like read to determine loop length in cycles
#endif

	for (unsigned int x = 0; x < 1000; x++)
	{

		if (TIFR2 & (1 << TOV2))
		{

			t += 256ul;
			TIFR2 |= (1 << TOV2);

		}

		c = (unsigned long)iSqrt((unsigned int)(s));
//		readMAP();
//		c = s * pressure[(unsigned int)(injCorrectionIdx)];
//		c >>= 12;

	}

#ifdef TinkerkitLCDmodule
	e = TCNT0; // do a microSeconds() - like read to determine loop length in cycles
	if (TIFR0 & (1 << TOV0))
#else
	e = TCNT2; // do a microSeconds() - like read to determine loop length in cycles
	if (TIFR2 & (1 << TOV2))
#endif
	{

#ifdef TinkerkitLCDmodule
		e = TCNT0;
		TIFR0 |= (1 << TOV0);
#else
		e = TCNT2;
		TIFR2 |= (1 << TOV2);
#endif
		t += 256;

	}

	e += t;

	sei();

	w = findCycleLength(s, e) - 156;

	init64(tempPtr[1], w);

	initStatusLine();
	print(format(SWEET64(prgmBenchMarkTime, 0), 3));
	printFlash(PSTR(" usec"));
	execStatusLine();

}

#endif
#endif
#ifdef useEEPROMviewer
void doEEPROMviewDisplay(void)
{

	print(format64(prgmFormatToNumber, (unsigned long)(screenCursor[(unsigned int)(eepromViewIdx)]), mBuff1, 3));
	clrEOL();
	gotoXY(0, 1);
	print(format64(prgmFormatToNumber, eepromReadVal((unsigned int)(screenCursor[(unsigned int)(eepromViewIdx)])), mBuff1, 3));
	clrEOL();

}

void goEEPROMview(void)
{

	prevMenuLevel = menuLevel;
	doCursorMoveAbsolute(eepromViewIdx, 255);

}

#endif
#ifdef useSavedTrips // Trip save/restore/raw data view support section
void doCursorUpdateTripShow(void)
{

	paramPtr = (uint8_t)(getBaseTripPointer(tripShowSlot)) + screenCursor[(unsigned int)(tripShowScreenIdx)];
	doParamRevert();

}

void doTripSaveDisplay(void)
{

	unsigned int t = getBaseTripPointer(tripShowSlot);
	uint8_t b = (uint8_t)(eepromReadVal((unsigned int)(t + tripListSigPointer)));
	uint8_t i = screenCursor[(unsigned int)(tripSaveScreenIdx)];
	uint8_t j;

	if (i == tslCount) j = tslSubSize;
	else
	{

		j = i;
		i /= tslSubSize;
		j -= i * tslSubSize;

		i = pgm_read_byte(&tripSelectList[(unsigned int)(i)]);

	}

	printStr(tripNames, j); // print trip function name at top left
	if (j < tslSubSize) printStr(bigFEDispChars, i);
	clrEOL();
	gotoXY(0, 1); // go to next line

	charOut('0' + tripShowSlot);
	charOut(':');

	if (b == guinosig) print(format64(prgmFormatToTime, eepromReadVal(t), mBuff1, 3));
	else printFlash(PSTR("Empty"));

	clrEOL();

}

void doTripShowDisplay(void)
{

	charOut('0' + tripShowSlot);
	charOut(':');

	uint8_t b = screenCursor[(unsigned int)(tripShowScreenIdx)];

	if (b > 16) b -= 1;
	else if (b > 3) b = b / 2 + 2;

	printStr(ertvNames, b);

	charOut(' ');
	charOut(76 - 4 * (screenCursor[(unsigned int)(tripShowScreenIdx)] & 1));

	clrEOL();
	gotoXY(0, 1); // go to next line
	print(pBuff);
	clrEOL();

}

void doGoTripTank(void)
{

	goSavedTrip(0);

}

void doGoTripCurrent(void)
{

	goSavedTrip(1);

}

void goSavedTrip(uint8_t tripSlot)
{

	tripShowSlot = tripSlot;
	prevMenuLevel = menuLevel;
	doCursorMoveAbsolute(tripSaveScreenIdx, tripSlot * tslSubSize);

}

void doTripSelect(void)
{

	goTripSelect(0);

}

void doTripLongSelect(void)
{

	goTripSelect(1);

}

void goTripSelect(uint8_t pressFlag)
{

	uint8_t i = screenCursor[(unsigned int)tripSaveScreenIdx];
	uint8_t j;

	if (i == tslCount) doCursorMoveAbsolute(tripShowScreenIdx, 0);
	else
	{

		j = i;
		i /= tslSubSize;
		j -= i * tslSubSize;

		if ((j == 0) && (pressFlag == 0)) doCursorMoveAbsolute(prevMenuLevel, i + tripScreenIdxBase);
		else
		{

			i = pgm_read_byte(&tripSelectList[(unsigned int)(i)]);

			if ((j == 1) && (pressFlag == 1)) doTripSave(i);
			else if ((j == 2) && (pressFlag == 1)) doTripLoad(i);
			else if ((j == 3) && (pressFlag == 0)) doTripReset(i);
			else doTripBumpSlot();

		}

	}

}

void doTripSave(uint8_t tripIdx)
{

	tripArray[(unsigned int)(tripIdx)].save(tripShowSlot);
	doTripPrintType(tripIdx);
	printFlash(PSTR(" Save"));

}

void doTripLoad(uint8_t tripIdx)
{

	tripArray[(unsigned int)(tripIdx)].load(tripShowSlot);
	doMainScreenDisplay();
	gotoXY(0, 0);
	doTripPrintType(tripIdx);
	printFlash(PSTR(" Load"));
	menuLevel = mainScreenIdx;

}

const uint8_t autoSaveInstr[] PROGMEM = {
	pAutoSaveActiveIdx,
	pAutoSaveActiveIdx,
#ifdef trackIdleEOCdata
	pAutoSaveIdleIdx,
	pAutoSaveIdleIdx,
#endif
};

uint8_t doTripAutoAction(uint8_t taaMode)
{

	uint8_t b;
	uint8_t c = 0;

	for (uint8_t x = 0; x < tslSize; x++)
	{

		if (eepromReadVal((unsigned int)(pgm_read_byte(&autoSaveInstr[(unsigned int)(x)]))))
		{

			b = pgm_read_byte(&tripSelectList[(unsigned int)(x)]);

			if (taaMode) c += tripArray[(unsigned int)(b)].load(x);
			else c += tripArray[(unsigned int)(b)].save(x);

		}

	}

	return c;

}

void doTripReset(uint8_t tripIdx)
{

	tripArray[(unsigned int)tripIdx].reset();
	doTripPrintType(tripIdx);
	printFlash(PSTR(" Reset"));

}

void doTripPrintType(uint8_t tripIdx)
{

	printStr(bigFEDispChars, tripIdx);
	printFlash(PSTR(" Trip "));
	charOut('0' + tripShowSlot);

}

void doTripBumpSlot(void)
{

	tripShowSlot++;
	if (tripShowSlot == eeAdrSavedTripsTemp3) tripShowSlot = 0;

}

void doTripShowCancel(void)
{

	menuLevel = tripSaveScreenIdx;

}

#endif
#ifdef useScreenEditor // Programmable main display screen edit support section

uint8_t screenEditValue = 0;

void doCursorUpdateScreenEdit(void)
{

	uint8_t b = screenCursor[(unsigned int)(screenEditIdx)] >> 1;

	screenEditValue = displayFormats[(unsigned int)(b)] & dfValMask;
	paramLength = (displayFormats[(unsigned int)(b)] & dfTripMask) >> dfBitShift;

}

void doScreenEditDisplay(void)
{
  
	uint8_t i = screenCursor[(unsigned int)(screenEditIdx)];
	uint8_t j = i;
	i >>= 1;
	uint8_t k = i;
	uint8_t l;
	uint8_t m;
	uint8_t n;

	i &= 0xFC;
	j &= 0x01;
	k &= 0x03;

	for (uint8_t x = 0; x < 4; x++)
	{

		l = displayFormats[(unsigned int)(i++)];
		m = 0;
		n = 0;

		if (x == k)
		{

  			if (j == 1) m = 170;
			else n = 170;

  		}

		displayMainScreenFunction(x, l, m, n);

	}

}

void doGoScreenEdit(void)
{

	prevMenuLevel = menuLevel;
	doCursorMoveAbsolute(screenEditIdx, screenCursor[(unsigned int)(mainScreenIdx)] * displayPageCount);

}

void doScreenEditReturnToMain(void)
{

	doSaveScreen();
	doReturnToMain();

}

void doScreenEditRevert(void)
{

	uint8_t b = screenCursor[(unsigned int)(screenEditIdx)] >> 1;

	paramPtr = (uint8_t)(eePtrScreensStart) + b;
	displayFormats[(unsigned int)(b)] = (uint8_t)(eepromReadVal((unsigned int)(paramPtr)));

}

void doScreenEditBump(void)
{

	uint8_t b = screenCursor[(unsigned int)(screenEditIdx)];
	uint8_t c = b;
	b &= 0x01;
	c >>= 1;

	if (b)
	{

		screenEditValue++;
		if (screenEditValue == dfMaxValDisplayCount) screenEditValue = 0;

	}
	else
	{

		paramLength++;
		if (paramLength == dfMaxTripCount) paramLength = 0;

	}

	displayFormats[(unsigned int)(c)] = (paramLength << dfBitShift) | screenEditValue;

}

void doSaveScreen(void)
{

	uint8_t b = screenCursor[(unsigned int)(screenEditIdx)] >> 1;

	paramPtr = (uint8_t)(eePtrScreensStart) + b;
	eepromWriteVal((unsigned int)(paramPtr), displayFormats[(unsigned int)(b)]);

}

#endif

uint8_t loadParams(void)
{

	uint8_t b = 1;
	uint8_t t;

#ifdef forceEEPROMsettingsInit
	if (true)
#else
	if (eepromReadVal((unsigned int)(eePtrSignature)) != newEEPROMsignature)
#endif
	{

		b = 0;

		eepromWriteVal((unsigned int)(eePtrSignature), newEEPROMsignature);

		t = eePtrSettingsStart;
		for (uint8_t x = 0; x < settingsSize; x++) eepromWriteVal((unsigned int)(t++), pgm_read_dword(&params[(unsigned int)(x)]));

#ifdef useScreenEditor
		t = eePtrScreensStart;
		for (uint8_t x = 0; x < displayFormatSize; x++) eepromWriteVal((unsigned int)(t++), (unsigned long)(displayFormats[(unsigned int)(x)]));

#endif
#ifdef useScreenEditor

	}
	else
	{

		t = eePtrScreensStart;
		for (uint8_t x = 0; x < displayFormatSize; x++) displayFormats[(unsigned int)(x)] = (uint8_t)(eepromReadVal((unsigned int)(t++)));
#endif

	}

	initGuino();
	return b;

}

uint8_t eepromWriteVal(unsigned int eePtr, unsigned long val)
{

	unsigned int t = eepromGetAddress(eePtr);
	uint8_t l;
	uint8_t w;
	uint8_t s = 0;

	l = (uint8_t)(t & 0x07);
	l++;
	t >>= 3;
	t += (unsigned int)(l);

	while (l > 0)
	{

		w = (uint8_t)(val & 0xFF);
		if (w != eeprom_read_byte((uint8_t *)(--t)))
		{

			eeprom_write_byte((uint8_t *)(t), w);
			s = 1;

		}
		val >>= 8;
		l--;

	}

	return s;

}

unsigned long eepromReadVal(unsigned int eePtr)
{

	unsigned int t = eepromGetAddress(eePtr);

	uint8_t l;
	unsigned long val = 0;

	l = (uint8_t)(t & 0x07);
	l++;
	t >>= 3;

	while (l > 0)
	{

		val <<= 8;
		val += (unsigned long)(eeprom_read_byte((uint8_t *)(t)));
		t++;
		l--;

	}

	return val;

}

unsigned int eepromGetAddress(unsigned int eePtr)
{

	unsigned int t;
	uint8_t l;

	if (eePtr == eePtrSignature)
	{

		t = (unsigned int)(eeAdrSignature);
		l = 3;

	}
	else if ((eePtr >= eePtrSettingsStart) && (eePtr < eePtrSettingsEnd))
	{

		eePtr -= eePtrSettingsStart;
		t = (unsigned int)(pgm_read_byte(&paramAddrs[eePtr]));
		l = pgm_read_byte(&paramAddrs[eePtr + 1]);
		l -= (uint8_t)(t);

#ifdef useScreenEditor
	}
	else if ((eePtr >= eePtrScreensStart) && (eePtr < eePtrScreensEnd))
	{

		eePtr -= eePtrScreensStart;
		l = 1;
		t = eePtr + eeAdrScreensStart;

#endif
#ifdef useSavedTrips
	}
	else if ((eePtr >= eePtrSavedTripsStart) && (eePtr < eePtrSavedTripsEnd))
	{

		eePtr -= eePtrSavedTripsStart;
		l = (uint8_t)(eePtr / tripListSize);
		eePtr -= (unsigned int)(l) * (unsigned int)(tripListSize);

		t = eeAdrSavedTripsStart + (unsigned int)(l) * (unsigned int)(eepromTripListSize);

		if ((eePtr > 0) && (eePtr < tripListSigPointer))
		{

			l = 4;
			t += 4 * (unsigned int)(eePtr - 1);
			t++;

		}
		else
		{

			if (eePtr > 0) t += (unsigned int)(eepromTripListSize - 1);
			l = 1;

		}
#endif

	}
	else
	{

		l = 0;
		t = 0;

	}

	t <<= 3;
	l--;
	t += (unsigned int)(l);

	return t;

}

void callFuncPointer(const uint8_t * funcIdx)
{

	pFunc mainFunc = (pFunc)pgm_read_word(&funcPointers[(unsigned int)(pgm_read_byte(funcIdx))]); // go perform action
	mainFunc();

}

unsigned long cycles2(void)
{

	unsigned long t;

	uint8_t oldSREG = SREG; // save state of interrupt flag

	cli(); // disable interrupts
#ifdef TinkerkitLCDmodule
	t = timer2_overflow_count + TCNT0; // do a microSeconds() - like read to determine loop length in cycles
	if (TIFR0 & (1 << TOV0)) t = timer2_overflow_count + 256 + TCNT0; // if overflow occurred, re-read with overflow flag taken into account
#else
	t = timer2_overflow_count + TCNT2; // do a microSeconds() - like read to determine loop length in cycles
	if (TIFR2 & (1 << TOV2)) t = timer2_overflow_count + 256 + TCNT2; // if overflow occurred, re-read with overflow flag taken into account
#endif
	SREG = oldSREG; // restore state of interrupt flag

	return t;

}

unsigned long findCycleLength(unsigned long lastCycle, unsigned long thisCycle)
{

	unsigned long t;

	if (thisCycle < lastCycle) t = 4294967295ul - lastCycle + thisCycle + 1;
	else t = thisCycle - lastCycle;

	return t;

}

int main(void)
{

	uint8_t i;
	uint8_t j;

	const uint8_t * bpPtr;

	cli(); // disable interrupts while interrupts are being fiddled with

#ifdef TinkerkitLCDmodule
	TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0) | (1 << COM0B1) | (1 << COM0B0)); // put timer 0 in 8-bit fast pwm mode
	TCCR0A |= ((1 << WGM01) | (1 << WGM00));
	TCCR0B &= ~((1 << FOC0A) | (1 << FOC0B) | (1 << WGM02) | (1 << CS02)); // set timer 0 prescale factor to 64
	TCCR0B |= ((1 << CS01) | (1 << CS00));
	TIMSK0 &= ~((1 << OCIE0B) | (1 << OCIE0A)); // disable timer 0 output compare interrupts
	TIMSK0 |= (1 << TOIE0); // enable timer 0 overflow interrupt
	TIFR0 |= ((1 << OCF0B) | (1 << OCF0A) | (1 << TOV0)); // clear timer 0 interrupt flags
#else
	TCCR2A &= ~((1 << COM2A1) | (1 << COM2A0) | (1 << COM2B1) | (1 << COM2B0)); // put timer 2 in 8-bit fast pwm mode
	TCCR2A |= ((1 << WGM21) | (1 << WGM20));
	TCCR2B &= ~((1 << FOC2A) | (1 << FOC2B) | (1 << WGM22) | (1 << CS21) | (1 << CS20)); // set timer 2 prescale factor to 64
	TCCR2B |= (1 << CS22);
	TIMSK2 &= ~((1 << OCIE2B) | (1 << OCIE2A)); // disable timer 2 output compare interrupts
	TIMSK2 |= (1 << TOIE2); // enable timer 2 overflow interrupt
	TIFR2 |= ((1 << OCF2B) | (1 << OCF2A) | (1 << TOV2)); // clear timer 2 interrupt flags
#endif

#ifdef useAnalogInterrupt
#ifndef useAnalogRead
	ADMUX = (1 << REFS0); // set ADC voltage reference to AVCC, and right-adjust the ADC reading
#endif
	 // enable ADC, enable ADC interrupt, clear any pending ADC interrupt, and set frequency to 1/128 of system timer
	ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
	ADCSRB = 0; // disable analog comparator multiplexer, and set ADC auto trigger source to free-running mode
#ifdef useLegacyButtons
	DIDR0 = ((1 << ADC2D) | (1 << ADC1D)); // only enable digital input on VSS and button pins
#else
#ifdef TinkerkitLCDmodule
	DIDR0 = ((1 << ADC7D) | (1 << ADC6D) | (1 << ADC5D));
	DIDR2 = (1 << ADC10D);
#else
	DIDR0 = ((1 << ADC5D) | (1 << ADC4D) | (1 << ADC3D) | (1 << ADC2D) | (1 << ADC1D)); // only enable digital input on VSS pin
#endif
#endif
#endif

#ifdef useSerialPort
	UBRR0H = (uint8_t)(myubbr >> 8); // set serial uart baud rate
	UBRR0L = (uint8_t)(myubbr);
	UCSR0A &= ~(1 << U2X0);
	UCSR0B = 0; // disable serial uart pins
	UCSR0C = (1 << UCSZ01)| (1 << UCSZ00); // set for 8 data bits, no parity, and 1 stop bit
#ifdef useBufferedSerialPort
	serialBuffer.init();
	serialBuffer.process = serialTransmitByte;
	serialBuffer.onEmpty = serialTransmitDisable;
	serialBuffer.onNoLongerEmpty = serialTransmitEnable;
#endif
#endif

	timer2_overflow_count = 0; // initialize timer 2 overflow counter

#ifdef useLegacyButtons
#ifdef ArduinoMega2560
	PORTK |= ((1 << PORTK5) | (1 << PORTK4) | (1 << PORTK3)); // enable port K button pullup resistors
	PCMSK2 |= ((1 << PCINT21) | (1 << PCINT20) | (1 << PCINT19) | (1 << PCINT16)); // enable port K button and VSS pin interrupts
#else
	PORTC |= ((1 << PORTC5) | (1 << PORTC4) | (1 << PORTC3)); // enable port C button pullup resistors
	PCMSK1 |= ((1 << PCINT13) | (1 << PCINT12) | (1 << PCINT11) | (1 << PCINT8)); // enable port C button and VSS pin interrupts
#endif
#else
#ifdef ArduinoMega2560
	PCMSK2 |= (1 << PCINT16); // enable port K VSS pin interrupt
#else
#ifdef TinkerkitLCDmodule
	PCMSK0 |= (1 << PCINT1); // enable port B VSS pin interrupt
#else
	PCMSK1 |= (1 << PCINT8); // enable port C VSS pin interrupt
#endif
#endif
#endif
#ifdef ArduinoMega2560
	PCICR |= (1 << PCIE2); // enable selected interrupts on port K
#else
#ifdef TinkerkitLCDmodule
	PCICR |= (1 << PCIE0); // enable selected interrupts on port ?
#else
	PCICR |= (1 << PCIE1); // enable selected interrupts on port C
#endif
#endif

#ifdef ArduinoMega2560
	lastPINKstate = PINK; // initialize last PINK state value so as to not erroneously detect a keypress on start
#else
#ifdef TinkerkitLCDmodule
	lastPINBstate = PINB; // initialize last PINB state value so as to not erroneously detect a keypress on start
#else
	lastPINCstate = PINC; // initialize last PINC state value so as to not erroneously detect a keypress on start
#endif
#endif

	for (uint8_t x = 0; x < tUDcount; x++) tripArray[(unsigned int)(pgm_read_byte(&tripUpdateDestList[(unsigned int)(x)]) & 0x7F)].reset();

	if (loadParams() != 1) doGoSettingsEdit(); // go through the initialization screen

#ifdef useAnalogRead
	timerCommand = tcWakeUp | tcResetADC;
#else
	timerCommand = tcWakeUp;
#endif
	timerStatus = tsButtonsUp;
	timerHeartBeat = 1;
	injResetCount = 0;
	vssResetCount = 0;
	buttonCount = 0;
	timerDelayCount = 0;
	dirty = 0;

	sei();

	LCD::init();
	gotoXY(0, 0);
	printFlash(PSTR("MPGuino v1.92tav"));
	gotoXY(0, 1);
	printFlash(PSTR("2014-MAY-12     "));

	delay2(delay1500ms); // show splash screen for 1.5 seconds

#ifdef useSavedTrips
	if (doTripAutoAction(1)) printStatusMessage(PSTR("AutoRestore Done"));
#endif

	while (true)
	{

		if (!(timerStatus & tsLoopExec)) // if not currently executing a cycle
		{

			timerCommand |= tcStartLoop; // start a new cycle
			while (timerCommand & tcStartLoop);

			timerLoopStart = cycles2();

#ifdef useClock
			cli(); // perform atomic transfer of clock to main program

			copy64((union union_64 *)&outputCycles, (union union_64 *)&clockCycles); // perform atomic transfer of system time to main program

			sei();
#endif

			if (timerStatus & tsAwake)
			{

				if (timerStatus & tsFellAsleep)
				{

					LCD::setBright(brightnessIdx); // restore backlight brightness setting
					if (eepromReadVal((unsigned int)(pWakupResetCurrentIdx))) doTripResetCurrent();
					timerStatus &= ~tsFellAsleep;

				}

#ifdef useDebugReadings
				tripArray[(unsigned int)(rawIdx)].collectedData[(unsigned int)(rvInjCycleIdx)] = (t2CyclesPerSecond / loopsPerSecond);
				tripArray[(unsigned int)(rawIdx)].collectedData[(unsigned int)(rvInjOpenCycleIdx)] = ((16391ul * processorSpeed) / (loopsPerSecond * 10));
				tripArray[(unsigned int)(rawIdx)].collectedData[(unsigned int)(rvVSScycleIdx)] = (t2CyclesPerSecond / loopsPerSecond);
				tripArray[(unsigned int)(rawIdx)].collectedData[(unsigned int)(rvInjPulseIdx)] = (20ul / loopsPerSecond);
				tripArray[(unsigned int)(rawIdx)].collectedData[(unsigned int)(rvVSSpulseIdx)] = (208ul / loopsPerSecond);

				timerCommand |= tcWakeUp; // tell system timer to wake up the main program
#endif
#ifdef useBarFuelEconVsTime
				bFEvTcount++;

				if (bFEvTcount >= bFEvTperiod)
				{

					if (bFEvTsize < bgDataSize) bFEvTsize++;

					barFEvsTimeData[(unsigned int)(bFEvTstartIDx)] = SWEET64(prgmFuelEcon, periodIdx);

					bFEvTstartIDx++;
					if (bFEvTstartIDx == bgDataSize) bFEvTstartIDx = 0;

					tripArray[(unsigned int)(periodIdx)].reset();
					bFEvTcount = 0;

				}
#endif

				for (uint8_t x = 0; x < tUScount; x++)
				{

					i = pgm_read_byte(&tripUpdateDestList[(unsigned int)x]);
					j = pgm_read_byte(&tripUpdateSrcList[(unsigned int)x]);

					if (j & 0x80) cli(); // perform atomic transfer of raw measurements to main program

					if (i & 0x80)
					{

						tripArray[(unsigned int)(i & 0x7F)].transfer(tripArray[(unsigned int)(j & 0x7F)]);
						tripArray[(unsigned int)(j & 0x7F)].reset();

					}
					else tripArray[(unsigned int)(i & 0x7F)].update(tripArray[(unsigned int)(j & 0x7F)]);

					if (j & 0x80) sei();

				}

#ifdef useBarFuelEconVsSpeed
				FEvSpdTripIdx = (uint8_t)(SWEET64(prgmFEvsSpeed, instantIdx));
				if (FEvSpdTripIdx < 255) tripArray[(unsigned int)(FEvSpdTripIdx)].update(tripArray[(unsigned int)(instantIdx)]);

#endif
#ifdef useSerialPortDataLogging
				if (eepromReadVal((unsigned int)(pSerialDataLoggingIdx))) doOutputDataLog();

#endif
#ifdef useWindowFilter
				if (eepromReadVal((unsigned int)(pWindowFilterIdx)))
				{

					if (tripArray[(unsigned int)(instantIdx)].collectedData[(unsigned int)(rvInjOpenCycleIdx)] == 0)

						resetWindowFilter(); // if no fuel is being consumed, reset filter

					else
					{ // update the CIC filter

						if (windowFilterCount < windowFilterSize) windowFilterCount++;
						else tripArray[(unsigned int)(windowFilterSumIdx)].subtract(tripArray[(unsigned int)(windowFilterElemIdx + windowFilterIdx)]);

						tripArray[(unsigned int)(windowFilterSumIdx)].update(tripArray[(unsigned int)(instantIdx)]);
						tripArray[(unsigned int)(windowFilterElemIdx + windowFilterIdx)].transfer(tripArray[(unsigned int)(instantIdx)]);
						tripArray[(unsigned int)(instantIdx)].transfer(tripArray[(unsigned int)(windowFilterSumIdx)]);

						windowFilterIdx++;
						if (windowFilterIdx == windowFilterSize) windowFilterIdx = 0;

					}

				}

#endif
			}

		}

		if (timerStatus & tsAwake) doRefreshDisplay();
		else
		{

			if (!(timerStatus & tsFellAsleep))
			{

#ifdef useSavedTrips
				if (doTripAutoAction(0)) printStatusMessage(PSTR("AutoSave Done"));
#endif
				LCD::setBright(0); // set backlight brightness to zero
				timerStatus |= tsFellAsleep;

			}

#ifdef useClock
			gotoXY(0, 0);
			doDisplaySystemTime();
#endif

		}

		if (timerStatus & tsMarkLoop) timerLoopLength = findCycleLength(timerLoopStart, cycles2());
		timerStatus &= ~tsMarkLoop;

		// wait for cycle to end, or for a keypress
		// while we're waiting anyway, let's do a few useful things
		while ((timerStatus & tsLoopExec) && (timerStatus & tsButtonsUp));

		if (!(timerStatus & tsButtonsUp)) // see if any buttons were pressed, display a brief message if so
		{

			j = buttonState;
			timerStatus |= tsButtonsUp; // reset keypress flag

			if (j == btnShortPressR) doCursorMoveRelative(0, 1);
			else if (j == btnShortPressL) doCursorMoveRelative(0, 255);
			else
			{

				bpPtr = (const uint8_t *)(pgm_read_word(&buttonPressAdrList[(unsigned int)(pgm_read_byte(&screenParameters[(unsigned int)(menuLevel)][5]))]));

				while (true)
				{

					i = pgm_read_byte(bpPtr++);

					if ((i == buttonsUp) || (j == i)) break;

					bpPtr++;

				}

				gotoXY(0, 0);
				if (j != buttonsUp) callFuncPointer(bpPtr); // go perform action

			}

		}

	}

}
