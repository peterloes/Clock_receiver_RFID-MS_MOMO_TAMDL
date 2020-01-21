/***************************************************************************//**
 * @file
 * @brief	WWVB Atomic Clock Decoder
 * @author	Peter Loes
 * @version	2018-03-20
 *
 * This module implements an Atomic Clock Decoder for the signal of the
 * German-based WWVB long wave transmitter.
 *
 * In detail, it includes:
 * - Initialization of the hardware (GPIOs that are connected to the WWVB
 *   receiver).
 * - A signal handler which is called whenever the logic level of the WWVB
 *   receiver changes.  It decodes the 59 bits of the WWVB signal to ones
 *   and zeros, and stores them into a <b>tm</b> structure.  When the data is
 *   complete, the system clock is synchronized with this value.
 * - A signal supervisor that detects when the WWVB signal does not change
 *   its level for more than 5 seconds.
 * - Automatic switching between winter time (DST) and summer time (daylight
 *   saving time, DSTS).  All alarm times will be changed plus/minus one hour
 *   to effectively occur at the time, regardless whether DST or DSTS.
 *
 * When initialized, the WWVB hardware module is enabled to receive the time
 * information and set the system clock.  If a segment LCD is available, the
 * <b>Antenna</b> symbol is blinking during this first time.  The further
 * behavior depends on the settings in the <i>config.h</i> file.  If the
 * define @ref WWVB_ONCE_PER_DAY is 1, the receiver (and also the Antenna
 * symbol) will be switched off after time has been synchronized.  As the name
 * of the define suggests, it will be switched on once per day.  To properly
 * detect a change between winter and summer time, i.e. <i>normal time</i>
 * (DST) and <i>daylight saving time</i> (DSTS), this happens at 01:55 for DST,
 * and 02:55 for DSTS.<br> If the define is 0, the receiver remains switched on
 * and the Antenna symbol is permanently shown.  However, when no WWVB signal
 * could be received for more than 5 seconds, the Antenna symbol will not be
 * shown to identify this kind of error.<br>
 * The <b>ARing</b> symbol is used to visualize the progress of receiving one
 * complete time frame.
 * For hardware platforms without a segment LCD, an LED can be used as
 * indicator for receiving a WWVB signal, please refer to the configuration
 * parameters @ref WWVB_DISPLAY_PROGRESS and @ref WWVB_INDICATOR.
 *
 * @see
 * https://de.wikipedia.org/wiki/WWVB for a description of the WWVB signal,
 * and the FBM10030R_EM2S 60kHz_DD.pdf of the WWVB 
 * hardware module.
 *
 * @note
 * To make the @ref WWVB_ONCE_PER_DAY feature work, an enum @ref
 * ALARM_WWVB_WAKE_UP of type @ref ALARM_ID must be defined in <i>config.h</i>.
 *
 * @warning
 * Be aware that the WWVB hardware module is very sensitive against high
 * frequency signals.  Cases have been observed where an SPI access to the
 * SD-Card disturbs the WWVB signal in a way, that a re-synchronization is
 * necessary.
 *
 ****************************************************************************//*
Revision History:
2018-03-20,rage DCF77 into WWVB
2018-03-13,rage DCF77Disable(WWVBDisable): set l_flgFeederOn = true 
2016-04-06,rage	Made local variables of type "volatile".
		BugFix: implicit DST to MESZ change during daylight saving time
		was misinterpreted during initial time synchronisation.
2016-02-15,rage	Implemented additional check for consecutive received DCF77
		time frames, see FRAME_SEQ_CNT.  Consider change of timezone.
2016-02-10,rage	Changed ShowWWVBIndicator() to ON for STATE_NO_SIGNAL to show
		loss of signal.
2015-05-08,rage	BugFixes: In DCF77Handler() STATE_UP_TO_DATE must be set before
		calling WWVBDisable().  WWVBDisable() must also disable the
		Signal Supervision timer.
2015-02-26,rage	Define WWVB_INDICATOR to call external ShowWWVBIndicator()
		routine, e.g. for using an LED as indicator for DCF77 signal.
2014-05-16,rage	DCF77Handler(): Changed switch() statement into separate case
		values because the IAR compiler cannot handle ranges.
2014-04-07,rage	Initial version.
*/

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include "em_device.h"
#include "em_assert.h"
#include "em_cmu.h"
#include "Logging.h"
#include "WWVB.h"
#include "ExtInt.h"
#include "AlarmClock.h"
#if WWVB_DISPLAY_PROGRESS
  #include "SegmentLCD.h"
#endif

/*=============================== Definitions ================================*/

    // Module Debugging
#define MOD_DEBUG	0	// set 1 to enable debugging of this module
#if ! MOD_DEBUG
    #undef  DBG_PUTC
    #undef  DBG_PUTS
    #define DBG_PUTC(ch)	// define as empty
    #define DBG_PUTS(str)
#endif

/*!@brief Frame sequence count (number of consecutive valid frames) */
#define FRAME_SEQ_CNT		2

/*!@brief Time when WWVB will be activated for switching from DST to DSTS */
#define ALARM_DST_TO_DSTS   1, 55
/*!@brief Time when WWVB will be activated for switching from DSTS to DST */
#define ALARM_DSTS_TO_DST   2, 55

/*=========================== Typedefs and Structs ===========================*/

    /*!@brief Local states of the WWVB signal */
typedef enum
{
    STATE_OFF,			//!< 0: [A] Atomic Clock Decoder is OFF
    STATE_NO_SIGNAL,		//!< 1: [B] No signal detected
    STATE_SYNC_WAIT,		//!< 2: [C] Waiting for 2s SYNC pause
    STATE_RECV_DATA,		//!< 3: [D] Got SYNC pulse, receive data
    STATE_UP_TO_DATE,		//!< 4: [E] Received complete time information
} WWV_STATE;

/*======================== External Data and Routines ========================*/

#if WWVB_INDICATOR
    /* external routine to switch WWVB signal indicator on or off */
    extern void ShowWWVBIndicator (bool enable);
    
      /* external routine to call when <l_State> reaches STATE_UP_TO_DATE */
    extern void DisplayUpdEnable (void);
    
#endif

/*========================= Global Data and Routines =========================*/

    /*!@brief WWVB date and time structure. */
struct tm    wwvb;	// not intended to be used by other modules

/*================================ Local Data ================================*/

    /*!@brief Current WWVB state */
static volatile WWV_STATE  l_State = STATE_OFF;

    /*!@brief sTimer handle for signal supervision. */
static volatile TIM_HDL	 l_TimHdl = NONE;

    /*!@brief Frame sequence counter (1..FRAME_SEQ_CNT) */
static volatile uint8_t	 l_FrameSeqCnt = 1;


/*=========================== Forward Declarations ===========================*/

static void	TimeSynchronize (struct tm *pTime);
static void	SignalSuperVisor (TIM_HDL hdl);
static void	StateChange (WWV_STATE newState);


/***************************************************************************//**
 *
 * @brief	Initialize WWVB hardware
 *
 * This routine initializes the GPIO pins which are connected to the external
 * WWVB module.  The GPIO ports and pins have been defined in the header file.
 * You additionally have to call WWVBEnable() after module ExtInt and the
 * LCD have been initialized to start the decoder.
 *
 ******************************************************************************/
void	WWVBInit (void)
{
    /* Be sure to enable clock to GPIO (should already be done) */
    CMU_ClockEnable (cmuClock_GPIO, true);

#if WWVB_HARDWARE_ENABLE
    /* The WWVB module provides a low-active enable pin, set to 1 for OFF */
    GPIO_PinModeSet (WWVB_ENABLE_PORT, WWVB_ENABLE_PIN, gpioModePushPull, 1);
#endif

    /*
     * Configure WWVB signal input pin and connect it to the external
     * interrupt (EXTI) facility.  At this stage, the interrupt is not
     * enabled, this is done later by calling ExtIntInit().
     */
    GPIO_PinModeSet (WWVB_SIGNAL_PORT, WWVB_SIGNAL_PIN, gpioModeInput, 0);
    GPIO_IntConfig  (WWVB_SIGNAL_PORT, WWVB_SIGNAL_PIN, false, false, false);

    /* When called for the first time, allocate timer handle */
    if (l_TimHdl == NONE)
	l_TimHdl = sTimerCreate (SignalSuperVisor);

#if WWVB_ONCE_PER_DAY
    /*
     * Set up wake-up time for WWVB receiver and decoder. The initial time zone
     * is DST.  Alarm times will be converted during first time synchronization
     * if daylight saving time (DSTS) is active.
     */
    AlarmAction (ALARM_WWVB_WAKE_UP, (ALARM_FCT)WWVBEnable);
    AlarmSet (ALARM_WWVB_WAKE_UP, ALARM_DST_TO_DSTS);
    AlarmEnable (ALARM_WWVB_WAKE_UP);
#endif
}

/***************************************************************************//**
 *
 * @brief	Enable the WWVB decoder
 *
 * This routine switches the WWVB decoder ON, i.e. the respective external
 * interrupt will be enabled.  If the define @ref WWVB_HARDWARE_ENABLE is 1,
 * additionally the WWVB receiver will be activated via a dedicated pin.
 *
 * @warning
 * Call this routine only after software module ExtInt and the LCD already
 * have been initialized.
 *
 ******************************************************************************/
void	WWVBEnable (void)
{

#ifdef LOGGING
    Log ("WWVB: Enabled");
#endif

#if WWVB_HARDWARE_ENABLE
    /* Set low-active enable pin of WWVB module to 0 */
    GPIO->P[WWVB_ENABLE_PORT].DOUTCLR = (1 << WWVB_ENABLE_PIN);
#endif

    /* Reset frame counter */
    l_FrameSeqCnt = 1;

    /* Interrupt enable */
    ExtIntEnable (WWVB_SIGNAL_PIN);

    /* Change WWVB state */
    StateChange (STATE_NO_SIGNAL);
}

/***************************************************************************//**
 *
 * @brief	Disable the WWVB decoder
 *
 * This routine switches the WWVB decoder OFF, i.e. the respective external
 * interrupt will be disabled.  If the define @ref WWVB_HARDWARE_ENABLE is 1,
 * additionally the WWVB receiver will be shut off via a dedicated pin.
 *
 ******************************************************************************/
void	WWVBDisable (void)
{
    /* Disable Signal Supervision */
    if (l_TimHdl != NONE)
	sTimerCancel (l_TimHdl);

    /* Interrupt disable */
    ExtIntDisable (WWVB_SIGNAL_PIN);

#if WWVB_HARDWARE_ENABLE
    /* Set low-active enable pin of WWVB module to 1 */
    GPIO->P[WWVB_ENABLE_PORT].DOUTSET = (1 << WWVB_ENABLE_PIN);
#endif

#if WWVB_INDICATOR
    /* be sure to switch indicator OFF */
    ShowWWVBIndicator (false);
#endif

    /* Change WWVB state */
    StateChange (STATE_OFF);

#ifdef LOGGING
    Log ("WWVB: Disabled");
#endif
}


//#define WWVB_TEST_CODE	// UNCOMMENT TO INCLUDE TEST CODE
#ifdef	WWVB_TEST_CODE
/* test code for "change time zone" - MODIFY THESE VARIABLES VIA DEBUGGER! */
int testMode = 0;	// 0: no test, 1: test DST->DSTS, 2: test DSTS->DST
int testCountDown;	// count down in [min] when change should occur
void testChangeTZ (void)
{
    if (testMode == 1)			// DST -> DSTS
    {
	wwvb.tm_isdst = 0;		// DST
	if (--testCountDown > 0)
	    return;
	wwvb.tm_isdst = 1;		// now DSTS
	if (++wwvb.tm_hour > 23)
	    wwvb.tm_hour = 0;
    }
    else if (testMode == 2)		// DSTS -> DST
    {
	wwvb.tm_isdst = 1;		// DSTS
	if (--testCountDown > 0)
	    return;
	wwvb.tm_isdst = 0;		// now DST
	if (--wwvb.tm_hour < 0)
	    wwvb.tm_hour = 23;
    }
}
#endif

/***************************************************************************//**
 *
 * @brief	Signal handler
 *
 * This handler is called by the EXTI interrupt service routine whenever the
 * logical level of the WWVB signal changes.
 *
 * @param[in] extiNum
 *	EXTernal Interrupt number of the WWVB signal.  This is identical
 *	with the pin number, i.e. @ref WWVB_SIGNAL_PIN.
 *
 * @param[in] extiLvl
 *	EXTernal Interrupt level: 0 means falling edge, logic level is now 0,
 *	1 means rising edge, logic level is now 1.  The signal from the WWVB
 *	module uses positive logic, i.e. pulses of about 100ms or 200ms are
 *	received.
 *
 * @param[in] timeStamp
 *	Time stamp (24bit) when the signal has changed its level.  This is
 *	used to decode a one or zero bit, and the synchronization pause.
 *
 * @note
 * The time stamp is read from the Real Time Counter (RTC), so its resolution
 * depends on the RTC.  Use the define @ref RTC_COUNTS_PER_SEC to convert the
 * RTC value into a duration.
 *
 ******************************************************************************/
void	WWVBHandler (int extiNum, bool extiLvl, uint32_t timeStamp)
{
static int8_t	bitNum = NONE;	// bit number, or NONE if waiting for SYNC
static uint32_t	pulseLength=0;	// length of high-pulse in number of RTC tics
static uint32_t	tsRising;	// time-stamp of previous rising edge
static uint32_t	value;		// general purpose variable
static uint32_t	value_1;		// general purpose variable
static uint32_t	value_2;		// general purpose variable
static uint32_t	abc_4;		// general purpose variable
static uint32_t	abc_5;		// general purpose variable
static uint32_t a = 0;
static uint32_t isLeapyear = 0;
static int8_t	flgDSTS;	// true for DSTS (daylight saving time)

#if WWVB_ONCE_PER_DAY
static bool	flgAwaitChange;	// true if awaiting DST/DSTS change
#endif

static time_t	prevTime;	// previous time in seconds (UNIX time)
static int8_t	bit;		// current data bit

int eomYear[14][2] = {
  {0,0},      // Begin
  {31,31},    // Jan
  {59,60},    // Feb
  {90,91},    // Mar
  {120,121},  // Apr
  {151,152},  // May
  {181,182},  // Jun
  {212,213},  // Jul
  {243,244},  // Aug
  {273,274},  // Sep
  {304,305},  // Oct
  {334,335},  // Nov
  {365,366},  // Dec
  {366,367}   // overflow
};




    (void) extiNum;	// suppress compiler warning "unused parameter"

    /* If interrupt has just been "replayed", we have to ignore it */
    if (timeStamp == 0)
	return;

    /* Check state to see if decoder is enabled */
    if (l_State == STATE_OFF)
    {
	/* Be sure to disable WWVB */
	WWVBDisable();
	return;
    }

    /* Received signal, verify current state */
    if (l_State == STATE_NO_SIGNAL)
    {
	/* We've got a signal now - wait for SYNC */
	StateChange (STATE_SYNC_WAIT);
	bitNum = NONE;		// be sure to activate SYNC mode
    }

    /* Start timer to detect future signal inactivity after 5s */
    if (l_TimHdl != NONE)
	sTimerStart (l_TimHdl, 5);

    /* See if rising or falling edge */
    if (extiLvl)
    {
	/*========== Rising edge ==========*/

	DBG_PUTC('#');

#if WWVB_INDICATOR
  #if WWVB_ONCE_PER_DAY
	/* switch indicator always ON */
	DBG_PUTC('*');
	ShowWWVBIndicator (true);
  #else
	/* switch indicator ON - except if time is already up-to-date */
	if (l_State != STATE_UP_TO_DATE)
	{
	    DBG_PUTC('*');
	    ShowWWVBIndicator (true);
	}
  #endif
#endif
	/* see if new WWVB time was prepared for setting system clock */
	if (bitNum == 58)
	{
#ifdef	WWVB_TEST_CODE
	    testChangeTZ();	// test routine for "change time zone"
#endif

	    /*
	     * Since WWVB data may be faulty due to signal weakness, we have
	     * to receive a sequence of valid frames.  These must show a time
	     * data difference of exactly 60s (except the timezone has changed).
	     * The number of frames to be received ist defined by FRAME_SEQ_CNT.
	     */
	    /* Convert <tm> structure to <time_t> */
	    struct tm	currTimeTM = wwvb;
	    time_t	currTime;
	    long	expDiff;	// expected time difference

	    currTimeTM.tm_isdst = 0;		// always 0 for mktime()
	    currTime = mktime (&currTimeTM);	// current time in seconds

	    /*
	     * Flag to detect whether DST<=>DSTS change occurred.  The system
	     * always starts up in DST (g_isdst is 0), so, when initial time
	     * has been received during daylight saving time (DSTS), this flag
	     * will always be <true> and the difference to the previous time
	     * frame is expected to be +3660 seconds - which is not correct!
	     */
	    bool changeOccurred = (g_isdst != (bool)wwvb.tm_isdst);

	    /* Consider timezone change only when really expected */
	    expDiff = (flgAwaitChange && changeOccurred ?
			(g_isdst ? -3540 : +3660) : +60);

	    if (currTime == prevTime + expDiff)
		l_FrameSeqCnt++;	// count valid frame
	    else
		l_FrameSeqCnt = 1;	// reset frame counter

	    prevTime = currTime;	// save time for next compare

#ifdef LOGGING
	    Log ("WWVB: Time Frame %d is 20%02d%02d%02d-%02d%02d%02d",
		 l_FrameSeqCnt,
                 wwvb.tm_year, wwvb.tm_mon + 1, wwvb.tm_mday,
		 wwvb.tm_hour, wwvb.tm_min, wwvb.tm_sec);
#endif

	    /* see if enough valid frames have been received */
	    if (l_FrameSeqCnt >= FRAME_SEQ_CNT)
	    {
		/*
		 * A sequence of enough consecutive frames has been
		 * received, we can trust the information and synchronize
		 * the local clock.
		 */
		if (l_FrameSeqCnt > 250)
		    l_FrameSeqCnt = 250;	// prevent counter from overflow

		/* set local time, show time on display */
		TimeSynchronize (&wwvb);

		/* RTC is 0, correct timeStamp and tsRising values */
		tsRising -= timeStamp;
		timeStamp = 0;

#if WWVB_ONCE_PER_DAY
		if (changeOccurred)
		{
		    /* change already occurred - clear flag */
		    flgAwaitChange = false;
		}
#endif

		/* received complete and valid data, set STATE_UP_TO_DATE */
		StateChange (STATE_UP_TO_DATE);

#if WWVB_ONCE_PER_DAY
		/* disable the WWVB to save power, except if waiting for change */
		if (! flgAwaitChange  ||  wwvb.tm_min < 50)
		{
		    WWVBDisable();
		    return;
		}
#endif
	    }

	    /* enter SYNC mode again (without blinking antenna) */
	    bitNum = NONE;
	}	// if (bitNum == 58)

	/* check for SYNC mode */
	if (bitNum == NONE)
	{
	    /*
	     * Waiting for SYNC, verify that previous high-pulse was valid and
	     * calculate distance of rising edges.  In case of SYNC this should
	     * be about 2 two back-back position markers with 800ms
	     */
            
          
            if (MS2TICS(750) < pulseLength  &&  pulseLength < MS2TICS(850))
	    {
	       uint32_t pauseLen;
               
               /* Previous pulse was valid - check distance of rising edges */
	       pauseLen = (timeStamp - tsRising) & 0x00FFFFFF;
               if (MS2TICS(900) < pauseLen  &&  pauseLen < MS2TICS(1000))
	       {
           
               /* SYNC detected - bit 0 will follow, receive data */
	       if (l_State != STATE_UP_TO_DATE)
	       StateChange (STATE_RECV_DATA);
               bitNum = 0;
               }
            }
        }
        else
	{
	   /* Regular mode, increase bit number */
	   bitNum++;
        }

	/* Save time stamp for rising edge */
	tsRising = timeStamp;

	return;	
        	// DONE - return from interrupt
    }

    /*========== Falling edge ==========*/

    DBG_PUTC('_');
    DBG_PUTC('A' + l_State);	// A=OFF B=NoSig C=SYNC D=Data E=UpToDate

#if WWVB_INDICATOR
  #if WWVB_ONCE_PER_DAY
	/* switch indicator always OFF */
	DBG_PUTC('.');
	ShowWWVBIndicator (false);
  #else
	/* switch indicator OFF - except if time is already up-to-date */
	if (l_State != STATE_UP_TO_DATE)
	{
	    DBG_PUTC('.');
	    ShowWWVBIndicator (false);
	}
  #endif
#endif

    /* Measure pulse length (24bit) - consider wrap-around */
    pulseLength = (timeStamp - tsRising) & 0x00FFFFFF;

    /* Ignore pulse if still seeking for SYNC */
    if (bitNum == NONE)
	return;		// DONE - return from interrupt

    /* Decode value for received bit */
    if (MS2TICS(160) < pulseLength  &&  pulseLength < MS2TICS(230))
    {
	bit = 0;	// 160~230ms means 0
	DBG_PUTC('0');
    }
    else if (MS2TICS(440) < pulseLength  &&  pulseLength < MS2TICS(530))
    {
	bit = 1;	// 440~530ms means 1
	DBG_PUTC('1');
    }
    else if (MS2TICS(700) < pulseLength  &&  pulseLength < MS2TICS(900))
    {
	bit = 2;	// 700~900ms means Marker
	DBG_PUTC('M');
    }
    else
    {
	/* Invalid pulse width - return to SYNC mode */
	StateChange (STATE_SYNC_WAIT);
	bitNum = NONE;
	DBG_PUTC('X');
	return;
    }

    /* Perform data processing according to the bit number */
    switch (bitNum)
    {
       case 0:
#if WWVB_DISPLAY_PROGRESS
	    SegmentLCD_ARing (7, SYM_ON);	// display first ARing symbol
	    SegmentLCD_Update();		// Update display
#endif        
         
          if (bit != 2)
	     break;		// bit not 2 means no Frame reference marker
	  return;
         
       case 1:  /* Bit 1~8: Minutes (00-59) */
       case 2:
       case 3:
       case 4:
       case 5:
       case 6:
       case 7:
       case 8:
          if (bit)
	     value |= (1 << (bitNum - 1));
	  return;
      
       case 9:        /* Bit 9: Marker P1 */
          if (bit != 2)
	     break;		// bit not 2 means no Marker
	  
             /* Reverse bits in byte */
             value_1 = (value & 0xF0) >> 4 | (value & 0x0F) << 4;
             value_1 = (value_1 & 0xCC) >> 2 | (value_1 & 0x33) << 2;
             value_1 = (value_1 & 0xAA) >> 1 | (value_1 & 0x55) << 1;
                 
             /* calculate and store minutes value */
	     wwvb.tm_min = ((((value_1 >> 4) & 0x0F) >> 1) * 10)+ (value_1 & 0x0F);
            
	     /* prepare variables for Hours value */
	     value  = 0;         // value means "Hours" in this case
             value_1 = 0;        // reverse value 
	  return;           
          
	case 10:         /* Bit 10~11: Unused, always 0 */
           if (bit != 0)   // bit always 0
	      break;	           
	   return;
           
        case 11:
           if (bit != 0)   // bit always 0
	      break;
           return;            	  
          
	case 12:         /* Bit 12~18: Hours (00-23) */  
	case 13:
	case 14:
        case 15:	
	case 16:	
	case 17:	
	case 18:
           if (bit)
              value |= (1 << (bitNum - 12));
           return;
      
        case 19:	/* Bit 19: Marker P2 */
           if (bit != 2)
	      break;		// bit not 2 means no Marker
              
              /* Reverse bits in byte */
              value_1 = (value & 0xF0) >> 4 | (value & 0x0F) << 4;
              value_1 = (value_1 & 0xCC) >> 2 | (value_1 & 0x33) << 2;
              value_1 = (value_1 & 0xAA) >> 1 | (value_1 & 0x55) << 1;

              /* calculate and store Hours value */
              wwvb.tm_hour =  ((((value_1 >> 4) & 0x0F) >> 2) * 10) + ((value_1 >> 1) & 0x0F);

	      /* prepare variable for Day of year */
	      value  = 0;    	// value means "Day of year" in this case
              value_1 = 0;     // reverse value 
	    return;
          
	case 20:	/* Bit 20~21: Unused, always 0 */
           if (bit != 0)        // bit always 0
	      break;
           return;            	  
          
	case 21:
           if (bit != 0)        // bit always 0
	      break;
           return;            	  
          
	case 22:        /* Bit 22~33: Day of year (100 and 10)*/
	case 23:
	case 24:
	case 25:
	case 26:
	case 27:
	case 28:
           if (bit)
              value |= (1 << (bitNum - 22));
           return;
      
        case 29:	/* Bit 19: Marker P3 */
           if (bit != 2)     // bit not 2 means no Marker
	      break;		
          
              /* Reverse bits in byte */
              value_1 = (value & 0xF0) >> 4 | (value & 0x0F) << 4;
              value_1 = (value_1 & 0xCC) >> 2 | (value_1 & 0x33) << 2;
              value_1 = (value_1 & 0xAA) >> 1 | (value_1 & 0x55) << 1;
            
              value_1 = (((value_1 >> 6 )& 0x0F) * 100) + (((value_1 & 0x0F) >> 1) * 10);
       
              /* prepare variable for date value */
	      value  = 0;		// value means "Day of year (1)" in this case
           return;                   
            
        case 30:      /* Bit 22~33: Day of year(1) */
	case 31:
	case 32:
	case 33:
            if (bit)
               value |= (1 << (bitNum - 30));
            return;
                 
	case 34:        /* Bit 34~35: Unused, always 0 */
           if (bit != 0)       // bit always 0
	      break;		
            
              value_2 = (value & 0xF0) >> 4 | (value & 0x0F) << 4;
              value_2 = (value_2 & 0xCC) >> 2 | (value_2 & 0x33) << 2;
              value_2 = (value_2 & 0xAA) >> 1 | (value_2 & 0x55) << 1;
       
              /* calculate and store minutes value */
	      abc_4 = ((value_2 >> 4) & 0x0F);
              abc_5 = value_1 + abc_4;
           
              /* prepare variables for DUT1 sign */
              abc_4 = 0;
	      value  = 0;        // value means "DUT1 sign" in this case
              value_1 = 0;       // reverse value
              value_2 = 0;       // value Variable 
	   return;
          
        case 35:
           if (bit != 0)        // bit always 0
	      break;
           return;
            
	case 36:	/* Bit 36~38: DUT1 Sign */
	case 37:
        case 38:
            return;

        case 39:	/* Bit 39: Marker P4 */
            if (bit != 2)   // bit not 2 means no Marker
	       break;
               /* The time correction DUT1 (sometimes also written DUT) is the difference
               between Universal Time (UT1), which is defined by Earth's rotation,
               and Coordinated Universal Time (UTC), which is defined by a
               network of precision atomic clocks. DUT1 = UT1 - UTC 
               DUT1 is unaccounted for WWVB receiver.*/   
            return;
            
	case 40:        /* Bit 40~43:  DUT1 Value */
	case 41:
	case 42:
	case 43:
            return;
            
	case 44:       /* Bit 44: Unused, always 0 */
           if (bit != 0)        // bit always 0
	      break;
              /* The time correction DUT1 (sometimes also written DUT) is the difference
              between Universal Time (UT1), which is defined by Earth's rotation,
              and Coordinated Universal Time (UTC), which is defined by a
              network of precision atomic clocks. DUT1 = UT1 - UTC 
              DUT1 is unaccounted for WWVB receiver.*/   
           return;
            
        case 45:       /* Bit 45~53: Year ten */
	case 46:
	case 47:
        case 48:
           if (bit)
              value |= (1 << (bitNum - 45));
           return;
           
	case 49:       	/* Bit 49: Marker P5 */
           if (bit != 2)   // bit not 2 means no Marker
	      break;		
              /* Reverse bits in byte */
              value_1 = (value & 0xF0) >> 4 | (value & 0x0F) << 4;
              value_1 = (value_1 & 0xCC) >> 2 | (value_1 & 0x33) << 2;
              value_1 = (value_1 & 0xAA) >> 1 | (value_1 & 0x55) << 1;
              
              /* prepare parity and variable for date value */
	      value  = 0;  // value means "Year" in this case
	   return;
          
	case 50:         /* Bit 53~50: Year one */
	case 51:
	case 52:
	case 53:
           if (bit)
              value |= (1 << (bitNum - 50));
           return;
             
        case 54:          /* Bit 54: Unused, always 0 */
           if (bit != 0)        // bit always 0
	      break;
              /* Reverse bits in byte */
              value_2 = (value & 0xF0) >> 4 | (value & 0x0F) << 4;
              value_2 = (value_2 & 0xCC) >> 2 | (value_2 & 0x33) << 2;
              value_2 = (value_2 & 0xAA) >> 1 | (value_2 & 0x55) << 1;
                 
              wwvb.tm_year = ((value_2 >> 4) & 0x0F) + (((value_1 >> 4) & 0x0F) * 10);
                             
              /* prepare parity and variable for date value */
	      value  = 0;      // value means "Leap year indicator" in this case
              value_1 = 0;     // reverse value
              value_2 = 0;     // value variable
           return;
           
        case 55:         /* Bit 55: Leap year indicator LYI */
           isLeapyear = bit; 
	  
           while( (a < 14) && (eomYear[a][isLeapyear] < abc_5) )
           {
              a++;
           }
           if (a > 0)
           {
              // calculate and store date values 
              wwvb.tm_mday = (abc_5 - eomYear[a-1][isLeapyear]);
              wwvb.tm_mon  = ((a > 12)?1:a) - 1;
           }
           
           /* prepare parity and variable for date value */
           abc_5 = 0;
	   isLeapyear = 0;     // reverse value
           return;
                    
        case 56: 	/* Bit 56: Leap second at end of month LSW */
	   return;
           
        case 57:        /* Bit 57: 0=DST begins, 1=DST ends - set DSTS flag */
           flgDSTS = bit; 
	   return;
            
        case 58:       /* Bit 58: 0=DST ends, 1=DST begins - set DSTS flag */       
           if (flgDSTS == bit)
	      break;		// if both bits are 0 or 1, this means error
	      /* store flag for daylight saving time */
	      wwvb.tm_isdst = flgDSTS;
	  return;
          
      case 59:	        /* Bit 59: Marker P0 bit for Year sign */
          if (bit != 2)
	     break;		// bit not 2 means no Marker
          
#if WWVB_DISPLAY_PROGRESS
	    /* clear all ARing segments */
	    SegmentLCD_ARingSetAll (SYM_OFF);
	    SegmentLCD_Update();		// Update display
#endif        

         return;
      
       default:		// invalid bit number
	    EFM_ASSERT(0);	// stall if DEBUG_EFM is set
	    break;
    }

    /* When the code arrives here, invalid data has been received */
    StateChange (STATE_SYNC_WAIT);
    bitNum = NONE;
}

/***************************************************************************//**
 *
 * @brief	Time Synchronization
 *
 * This function is called from the WWVB interrupt handler after a sequence
 * of consecutive valid frames has been received, so we can be sure the time
 * information is valid.  It sets the system clock via ClockSet() and updates
 * the display with the new time.
 * It also checks for a change of DST to DSTS and vice versa.  If this happens,
 * all configured alarm times will be adjusted accordingly.
 *
 * @note
 * Be aware, this function is called in interrupt context!
 *
 * @param[in] pTime
 *	Pointer to a <b>tm</b> structure that holds the current time.
 *
 ******************************************************************************/
static void	TimeSynchronize (struct tm *pTime)
{
    /* flag to detect whether DST<=>DSTS change occurred */
    bool changeOccurred = (g_isdst != (bool)pTime->tm_isdst);

    /* set system clock to WWVB time */
    g_CurrDateTime = *pTime;
    g_isdst = pTime->tm_isdst;		// flag for daylight saving time
    ClockSet (&g_CurrDateTime, true);	// set milliseconds to zero

    /* show time on display */
    ClockUpdate (false);	// g_CurrDateTime is already up to date

#if WWVB_ONCE_PER_DAY  &&  defined(LOGGING)
    /* log current WWVB time */
    Log("WWVB: Time Synchronization %02d:%02d:%02d (%s)",
	pTime->tm_hour, pTime->tm_min, pTime->tm_sec, g_isdst ? "DST" : "DSTS");
#endif

    /*
     * WWVB may be activated once per day only.  To detect a change
     * between DST and DSTS properly, the WWVB will be switched-on at
     * the right time.  When such a change is detected, all alarm times
     * must be corrected to still occur at the same effective time.
     * This includes the ALARM_WWVB_WAKE_UP, which is switched between
     * 01:55 (DST) and 02:55 (DSTS) properly.
     */
    if (changeOccurred)
    {
    int	    alarm;
    int8_t  hour, minute;

	if (g_isdst)
	    Log ("WWVB: Changing time zone from DST to DSTS");
	else
	    Log ("WWVB: Changing time zone from DSTS to DST");

	/* DST <-> DSTS change detected */
	for (alarm = 0;  alarm < MAX_ALARMS;  alarm++)
	{
	    AlarmGet (alarm, &hour, &minute);

	    hour += (g_isdst ? +1 : -1);
	    if (hour < 0)
		hour = 23;
	    else if (hour > 23)
		hour = 0;

	    AlarmSet (alarm, hour, minute);
	}
    }
}

/***************************************************************************//**
 *
 * @brief	Signal Supervision
 *
 * This function is called if the WWVB signal is inactive (does not change
 * its state) for more than 5 seconds.  It clears the <i>Antenna</i> symbol
 * from the LC-Display (or the indicator LED) and sets the WWVB state to
 * STATE_NO_SIGNAL.
 *
 * @note
 * Be aware, this function is called in interrupt context!
 *
 * @param[in] hdl
 *	Timer handle (not used here).
 *
 ******************************************************************************/
static void	SignalSuperVisor (TIM_HDL hdl)
{
    (void) hdl;

    /* No signal for more than 5 seconds - set state to "no signal" */
    StateChange (STATE_NO_SIGNAL);
}

/***************************************************************************//**
 *
 * @brief	State Change
 *
 * This routine must be called whenever the state of the WWVB receiver has
 * changed.  It updates the <i>Antenna</i> symbol on the LCD (or the indicator
 * LED).
 *
 * @note
 * Be aware, this function can be called in interrupt context!
 *
 * @param[in] newState
 *	New state to establish.  The value is of type @ref WWV_STATE.
 *
 ******************************************************************************/
static void	StateChange (WWV_STATE newState)
{
    /* Check new state */
    if (newState == l_State)
	return;			// no change at all

    /* Handle antenna symbol according to the new state */
    switch (newState)
    {
	case STATE_OFF:		// decoder switched off
	case STATE_NO_SIGNAL:	// no WWVB signal detected
#if WWVB_DISPLAY_PROGRESS
	    /* Clear antenna symbol */
	    SegmentLCD_Symbol (LCD_SYMBOL_ANT, SYM_OFF);

	    /* Clear all ARing symbols */
	    SegmentLCD_ARingSetAll (SYM_OFF);
#endif
#if WWVB_INDICATOR
	    if (newState == STATE_NO_SIGNAL)
	    {
		/* switch indicator ON for lost signal */
		ShowWWVBIndicator (true);
	    }
	    else
	    {
		/* switch indicator OFF when WWVB disabled */
		ShowWWVBIndicator (false);
	    }
#endif
	    break;

	case STATE_SYNC_WAIT:	// waiting for SYNC
#if WWVB_DISPLAY_PROGRESS
	    /* Let antenna symbol blink */
	    SegmentLCD_Symbol (LCD_SYMBOL_ANT, SYM_BLINK);

	    /* Clear all ARing symbols */
	    SegmentLCD_ARingSetAll (SYM_OFF);
#endif
	    break;

	case STATE_RECV_DATA:	// got SYNC pulse, receive data
	    break;		// no changes

	case STATE_UP_TO_DATE:	// time completely received
#if WWVB_DISPLAY_PROGRESS
	    /* Switch antenna symbol on */
	    SegmentLCD_Symbol (LCD_SYMBOL_ANT, SYM_ON);
#endif
	    /*
	     * If WWVB is enabled once per day, the indicator shall be active
	     * as long as the receiver is enabled.
	     * If the receiver is enabled permanently, the indicator shall be
	     * switched off as soon as time is up-to-date, otherwise it would
	     * consume too much power all the day.
	     */
#if WWVB_INDICATOR  &&  ! WWVB_ONCE_PER_DAY
	    /* switch indicator OFF */
	    ShowWWVBIndicator (false);
#endif
            
            /* callback to allow LCD updates */
	    DisplayUpdEnable();
	    break;

	default:		// unhandled state
	    EFM_ASSERT(0);	// stall if DEBUG_EFM is set
	    break;
    }

    /* Establish new state */
    l_State = newState;

#if WWVB_DISPLAY_PROGRESS
    /* Update display */
    SegmentLCD_Update();
#endif
}
