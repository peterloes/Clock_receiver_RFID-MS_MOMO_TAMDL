/***************************************************************************//**
 * @file
 * @brief	Header file of module WWVB.c
 * @author	Peter Loes
 * @version	2018-03-20
 ****************************************************************************//*
Revision History:
2018-03-20,rage DCF77 into WWVB
2016-04-13,rage	Removed DCF_TRIG_MASK (no more required by EXTI module).
2016-04-05,rage	Reverted DCF77_ENABLE_PIN to 1 and DCF77_SIGNAL_PIN to 2.
2015-07-28,rage	Changed DCF77_ENABLE_PIN to 2 and DCF77_SIGNAL_PIN to 1.
2014-11-21,rage	Added DCF77_DISPLAY_PROGRESS and DCF77_INDICATOR.
2014-05-10,rage	Initial version.
*/

#ifndef __INC_WWVB_h
#define __INC_WWVB_h

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_gpio.h"
#include "config.h"		// include project configuration parameters

/*=============================== Definitions ================================*/

#ifndef WWVB_HARDWARE_ENABLE
    /*!@brief Set to 1 if WWVB hardware module needs enable via pin. */
    #define WWVB_HARDWARE_ENABLE      	1
#endif

#ifndef WWVB_ONCE_PER_DAY
    /*!@brief Set 1 to activate WWVB only once per day. */
    #define WWVB_ONCE_PER_DAY		1
#endif

#ifndef WWVB_DISPLAY_PROGRESS
    /*!@brief Set 1 to display receive progress on segment LCD.
     * If 1, functions SegmentLCD_ARing(), SegmentLCD_ARingSetAll(),
     * SegmentLCD_Symbol(), and SegmentLCD_Update() are called to display
     * the progress of receiving WWVB information.
     */
    #define WWVB_DISPLAY_PROGRESS	0
#endif

    /*!@brief Set 1 to use a WWVB signal indicator, e.g. an LED.
     * If 1, function ShowWWVBIndicator() will be called during the
     * synchronization, whenever the WWVB signal changes to high or low.
     */
#define WWVB_INDICATOR		1

/*!@brief Here follows the definition of GPIO ports and pins used to connect
 * to the external WWVB hardware module.
 */
#if WWVB_HARDWARE_ENABLE
    #define WWVB_ENABLE_PORT	gpioPortD
    #define WWVB_ENABLE_PIN	1
#endif

#define WWVB_SIGNAL_PORT	gpioPortD
#define WWVB_SIGNAL_PIN	2


/*!@brief Bit mask of the affected external interrupt (EXTI). */
#define WWV_EXTI_MASK		(1 << WWVB_SIGNAL_PIN)

/*================================ Prototypes ================================*/

/* Initialize WWVB hardware */
void	WWVBInit (void);

/* Enable the WWVB decoder */
void	WWVBEnable (void);

/* Disable the WWVB decoder */
void	WWVBDisable (void);

/* Signal handler, called from interrupt service routine */
void	WWVBHandler	(int extiNum, bool extiLvl, uint32_t timeStamp);


#endif /* __INC_WWVB_h */
