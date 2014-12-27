/*
 * =====================================================================================
 *
 *       Filename:  configure.h
 *
 *    Description:  Configuration variables for mpguino
 *
 *        Created:  12/27/2014 09:48:33 PM
 *
 * =====================================================================================
 */

#ifndef __configure_h__
#define __configure_h__

/* 
 * if the below "#define"s are commented out, 
 * code will compile for an AtMega328-series processor 
 */
//#define ArduinoMega2560 true
//#define TinkerkitLCDmodule true

/* 
 * if the below #define is commented out, 16 MHz system clock will be assumed
 */
#define use20MHz true // force 20 MHz system clock values

/* 
 * only one of the below LCD options may be chosen - choosing more than one will 
 * cause a compilation error to occur
 * if TinkerkitLCDmodule is used, useLegacyLCD will automatically be used, 
 * and the below options will be ignored
 */
#define useLegacyLCD true
//#define useParallaxLCD true

/*
 * only one of the below button options may be chosen - choosing more than one 
 * will cause a compilation error to occur
 */
#define useLegacyButtons true
//#define useAnalogMuxButtons true
//#define useParallax5PositionSwitch true

/* 
 * The below options only work if useLegacyLCD is selected. If useLegacyLCD 
 * is not selected, the below options will not be inserted at all
 */

//#define useLegacyLCDinvertedBrightness true	/* For alternate LCD backlight connections */
#define useLegacyLCDbuffered true		/* Speed up LCD output */

/* 
 * selectable options - all may be chosen independently of one another, 
 * save for serial data logging.
 * the serial data logging option will conflict with the Parallax LCD output 
 * option, if both are selected at the same time
 */
//#define blankScreenOnMessage true		/* Completely blank display screen upon display of message */
//#define trackIdleEOCdata true			/* Ability to track engine idling and EOC modes */
//#define useSerialPortDataLogging true		/* Ability to output 5 basic parameters to a data logger or SD card */
//#define useBufferedSerialPort true		/* Speed up serial output */
//#define useCalculatedFuelFactor true		/* Ability to calculate that pesky us/gal (or L) factor from easily available published fuel injector data */
#define useWindowFilter true			/* Smooths out "jumpy" instant FE figures that are caused by modern OBDII engine computers */
#define useBigFE true				/* Show big fuel economy displays */
#define useBigDTE true				/* Show big distance-to-empty displays */
//#define useBigTTE true			/* Show big time-to-empty displays */
//#define useClock true				/* Show system clock, and provide means to set it */
#define useSavedTrips true			/* Ability to save current or tank trips to any one of 10 different trip slots in EEPROM */
#define useScreenEditor true			/* Ability to change any of 8 existing trip data screens, with 4 configurable figures on each screen */
#define useBarFuelEconVsTime true		/* Show Fuel Economy over Time bar graph */
#define useBarFuelEconVsSpeed true		/* Show Fuel Economy vs Speed, Fuel Used vs Speed bar graphs */
#define useSpiffyBigChars true
//#define useFuelCost true			/* Show fuel cost */
//#define useChryslerMAPCorrection true		/* Ability to perform on-the-fly fuel injector data correction for late-model Chrysler vehicles */
#define useABresultViewer true			/* Ability to graphically show current (B) versus stored (A) fuel consumption rates */
//#define useCoastDownCalculator true		/* Ability to calculate C(rr) and C(d) from coastdown */

/* 
 * program measurement and debugging tools
 */
//#define useCPUreading true			/* Show CPU loading and available RAM usage */
//#define useDebugReadings true
//#define forceEEPROMsettingsInit true
//#define useEEPROMviewer true			/* Ability to directly examine EEPROM */
//#define useBenchMark true			/* this is probably broken - last time I used it was in August 2013 */
//#define useSerialDebugOutput true

/* 
 * SWEET64 configuration/debugging
 */
//#define useSWEET64trace true			/* Ability to view real-time 64-bit calculations from SWEET64 kernel */
//#define useSWEET64multDiv true		/* shift mul64 and div64 from native C++ to SWEET64 bytecode */

/*
 * these #defines are used to select various features to support the above 
 * choices do not mess with them, or compilation errors will occur
 */
#ifdef TinkerkitLCDmodule
#undef useParallaxLCD
#define useLegacyLCD true
#ifdef useLegacyButtons
#undef useLegacyButtons
#define useAnalogMuxButtons true
#endif
#endif

#ifdef useClock
#define useBigTimeDisplay true
#endif

#ifdef useBigTTE
#define useBigTimeDisplay true
#endif

#ifdef useBigTimeDisplay
#define useBigNumberDisplay true
#endif

#ifdef useBigDTE
#define useBigNumberDisplay true
#endif

#ifdef useBigFE
#define useBigNumberDisplay true
#endif

#ifdef useCalculatedFuelFactor
#define useIsqrt true
#endif

#ifdef useChryslerMAPCorrection
#define useIsqrt true
#define useAnalogRead true
#endif

#ifdef useSerialPortDataLogging
#define useSerialPort true
#endif

#ifdef useParallaxLCD
#define useSerialPort true
#endif

#ifdef useAnalogMuxButtons
#define useAnalogButtons true
#endif

#ifdef useParallax5PositionSwitch
#define useAnalogButtons true
#endif

#ifdef useAnalogButtons
#define useAnalogRead true
#endif

#ifdef useLegacyLCD
#define useAnalogInterrupt true
#endif

#ifdef useAnalogRead
#define useAnalogInterrupt true
#endif

#ifdef useSWEET64trace
#define useSerialDebugOutput true
#endif

#ifdef useSerialDebugOutput
#define useSerialPort true
#endif

#ifdef useBarFuelEconVsTime
#define useBarGraph true
#endif

#ifdef useBarFuelEconVsSpeed
#define useBarGraph true
#endif

#ifdef useCoastDownCalculator
#define useVehicleMass true
#endif

#ifdef useBufferedSerialPort
#define useBuffering true
#endif

#ifdef useLegacyLCDbuffered
#define useBuffering true
#endif

#endif /* __configure_h__ */
