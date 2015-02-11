/****************************************************************************
 * arch/arm/src/stm32/stm32_oneshot.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   		 David Sidrane <david_s5@nscdg.com>
 *
 * References:
 *
 *   STM32Series Data Sheet
 *   Atmel NoOS sample code.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <arch/irq.h>
#include <nuttx/clock.h>

#include "stm32_tim.h"
#include "stm32_oneshot.h"

#ifdef CONFIG_STM32_ONESHOT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing the watchdog
 * timer
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_TIMER
#endif

#ifdef CONFIG_DEBUG_TIMER
#  define tcdbg                 dbg
#  define tclldbg               lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define tcvdbg              vdbg
#    define tcllvdbg            llvdbg
#  else
#    define tcvdbg(x...)
#    define tcllvdbg(x...)
#  endif
#else
#  define tcdbg(x...)
#  define tclldbg(x...)
#  define tcvdbg(x...)
#  define tcllvdbg(x...)
#endif

#define EXPIRED 0
#define ISR_OVERHEAD_IN_TICKS 1

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
static struct stm32_oneshot_s *g_oneshot;
/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_oneshot_handler
 *
 * Description:
 *   Timer interrupt callback.  When the oneshot timer interrupt expires,
 *   this function will be called.  On the final expiration it will
 *   forward the call to the next level up.
 *
 * Input Parameters:
 *   tch - The handle that represents the timer state
 *   arg - An opaque argument provided when the interrupt was registered
 *   sr  - The value of the timer interrupt status register at the time
 *         that the interrupt occurred.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/


static int stm32_oneshot_handler(int irq, void *context)
{
  struct stm32_oneshot_s *oneshot = g_oneshot;
  FAR struct stm32_tim_dev_s * dev = (FAR struct stm32_tim_dev_s *) oneshot->tch;
  oneshot_handler_t oneshot_handler;
  void *oneshot_arg;

  tcllvdbg("Expired...\n");
  DEBUGASSERT(oneshot && oneshot->handler);
  PROBE(4,true);
  PROBE(5,true);

  switch(--oneshot->loops)
     {
	   case 0:
	 		  (void)STM32_TIM_SETMODE(dev, STM32_TIM_MODE_PULSE);
		 	  if (oneshot->remainder)
		 	    {
			   	   PROBE_MARK(5);
		 		  (void)STM32_TIM_SETPERIOD(dev, oneshot->remainder);
		 		  (void)STM32_TIM_SETCLOCK(dev,oneshot->frequency);
		 		  break;
		 	    }
		 	 /* no break */
  	   case -1:
		 	  if (oneshot->frac)
		 	    {
			   	   PROBE_MARK(5);
			   	   PROBE_MARK(5);
		 		  (void)STM32_TIM_SETPERIOD(dev, oneshot->frac);
		 		  (void)STM32_TIM_SETCLOCK(dev,1000000);
		 		  break;
		 	    }
		 	 /* no break */

  	   case -2:

	   	   PROBE_MARK(5);
	   	   PROBE_MARK(5);
	   	   PROBE_MARK(5);
 		  (void)STM32_TIM_SETMODE(dev, STM32_TIM_MODE_DISABLED);

  		   /* The clock was stopped, and disabled when the over flow occurred.   */

  		  /* The timer is no longer running */

  		  oneshot->running = false;

  		  /* Forward the event, clearing out any vestiges */

  		  oneshot_handler  = (oneshot_handler_t)oneshot->handler;
  		  oneshot->handler = NULL;
  		  oneshot_arg      = (void *)oneshot->arg;
  		  oneshot->arg     = NULL;

  		  oneshot_handler(oneshot_arg);
  		  break;

  	   default:
   		  break;
     }
   STM32_TIM_ACKINT(dev,0);
   PROBE(4,false);
   PROBE(5,false);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer wrapper
 *
 * Input Parameters:
 *   oneshot    Caller allocated instance of the oneshot state structure
 *   chan       Timer counter channel to be used.  See the TC_CHAN*
 *              definitions in arch/arm/src/stm32/stm32_tc.h.
 *   resolution The required resolution of the timer in units of
 *              microseconds.  NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int stm32_oneshot_initialize(struct stm32_oneshot_s *oneshot, int chan,
                           uint16_t resolution)
{
  FAR struct stm32_tim_dev_s * dev;
  int ret = OK;

  g_oneshot = oneshot;
  tcvdbg("chan=%d resolution=%d usec\n", chan, resolution);
  DEBUGASSERT(oneshot && resolution > 0);

  PROBE_INIT(PROBE_N(1)|PROBE_N(2)|PROBE_N(3)|PROBE_N(4)|PROBE_N(5)|PROBE_N(6));
  PROBE_MARK(1);PROBE(1,false);
  PROBE_MARK(2);PROBE(2,false);
  PROBE_MARK(3);PROBE(3,false);
  PROBE_MARK(4);PROBE(4,false);
  PROBE_MARK(5);PROBE(5,false);
  PROBE_MARK(6);PROBE(6,false);

  dev = stm32_tim_init(chan);

  if (!dev)
  	{
	  tcdbg("ERROR: Failed to allocate timer %d\n", chan);
	  ret = -EBUSY;
  	}
  else
    {
	  /* Initialize the remaining fields in the state structure */

	  oneshot->tch 		  = dev;
	  oneshot->chan       = chan;
	  oneshot->running	  = false;
	  oneshot->handler    = NULL;
	  oneshot->arg        = NULL;
	  oneshot->resolution = resolution;
	  oneshot->frequency 	  = USEC_PER_SEC / resolution;
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_oneshot_start
 *
 * Description:
 *   Start the oneshot timer
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           stm32_oneshot_initialize();
 *   handler The function to call when when the oneshot timer expires.
 *   arg     An opaque argument that will accompany the callback.
 *   ts      Provides the duration of the one shot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/
int stm32_oneshot_start(struct stm32_oneshot_s *oneshot, oneshot_handler_t handler,
                      void *arg, const struct timespec *ts)
{
  FAR struct stm32_tim_dev_s * dev;
  uint64_t usec;
  uint16_t period;
  stm32_tim_mode_t mode;
  irqstate_t flags;

  dev = (FAR struct stm32_tim_dev_s *) oneshot->tch;

  tcvdbg("handler=%p arg=%p, ts=(%lu, %lu)\n",
         handler, arg, (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);
  DEBUGASSERT(oneshot && handler && ts);

  /* Was the oneshot already running? */

  flags = irqsave();
  if (oneshot->running)
    {
      /* Yes.. then cancel it */

      tcvdbg("Already running... cancelling\n");
      (void)stm32_oneshot_cancel(oneshot, NULL);
    }

  /* Save the new handler and its argument */

  oneshot->handler = handler;
  oneshot->arg     = arg;

  /* Express the delay in microseconds */

  usec = (uint64_t)ts->tv_sec * USEC_PER_SEC + (uint64_t)(ts->tv_nsec / NSEC_PER_USEC);
  /* Because the timer is a 16 bit timer, we have to handle values
   * greater then MAX_TIMER_CNT.
   * resolutionticks is the number of ticks of resolution size
   *
   * loops is a pre decrement count of the number of times the ISR will run with a
   * period of MAX_TIMER_CNT and a frequency of USEC_PER_SEC / resolution
   *
   * remainder is the number of whole ticks of resolution size. The ISR
   * will run with a period of remainder and a frequency of USEC_PER_SEC / resolution
   *
   * frac is the odd amount of us left. The ISR will run one list time with a period
   * of frac and a frequency of 1 mHz
   */
  oneshot->resolutionticks = usec / oneshot->resolution;
  oneshot->loops 		   = (oneshot->resolutionticks / STM32_MAX_TIMER_CNT) + 1;
  oneshot->remainder 	   = oneshot->resolutionticks % STM32_MAX_TIMER_CNT;
  oneshot->frac  		   = usec % oneshot->resolution;

  if (oneshot->loops > 1)
    {
	  period = STM32_MAX_TIMER_CNT-1;
	  mode = STM32_TIM_MODE_UP;
    }
  else
  	{
	  period = oneshot->remainder;
	  mode = STM32_TIM_MODE_PULSE;
    }

  /* Set up to receive the callback when the interrupt occurs */

  (void)STM32_TIM_SETISR(dev, stm32_oneshot_handler, 0);

  /* Enable device interrupts. */

  STM32_TIM_ENABLEINT(dev,0);

  (void)STM32_TIM_SETMODE(dev, mode);
  (void)STM32_TIM_SETPERIOD(dev, period);

  /* Start the counter */

  (void)STM32_TIM_SETCLOCK(dev,oneshot->frequency);

  /* Enable interrupts.  We should get the callback when the interrupt
   * occurs.
   */

  oneshot->running = true;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: stm32_oneshot_cancel
 *
 * Description:
 *   Cancel the oneshot timer and return the time remaining on the timer.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           stm32_oneshot_initialize();
 *   ts      The location in which to return the time remaining on the
 *           oneshot timer.  A time of zero is returned if the timer is
 *           not running.  ts may be zero in which case the time remaining
 *           is not returned.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

int stm32_oneshot_cancel(struct stm32_oneshot_s *oneshot, struct timespec *ts)
{
  irqstate_t flags;
  uint64_t usec;
  uint64_t sec;
  uint64_t nsec;
  uint32_t count;
  int sr;

  /* Was the timer running? */

  flags = irqsave();
  if (!oneshot->running)
    {
      /* No.. Just return zero timer remaining and successful cancellation.
       * This function may execute at a high rate with no timer running
       * (as when pre-emption is enabled and disabled).
       */

      ts->tv_sec  = 0;
      ts->tv_nsec = 0;
      irqrestore(flags);
      return OK;
    }

  /* Yes.. Get the timer counter and stop the counter.  If
   * the counter expires while we are doing this, the counter clock will be
   * stopped, and the clock will be disabled.
   */

  tcvdbg("Cancelling...\n");

  FAR struct stm32_tim_dev_s * dev = (FAR struct stm32_tim_dev_s *) oneshot->tch;

  /* Read remaining */

  count = STM32_TIME_GETREMAING(dev,&sr);


  /* Now we can disable the interrupt and stop the timer. */

  (void)STM32_TIM_DISABLEINT(dev,0);
  (void)STM32_TIM_SETISR(dev, NULL, 0);
  (void)STM32_TIM_SETCLOCK(dev,0);

  oneshot->running = false;
  oneshot->handler = NULL;
  oneshot->arg     = NULL;
  irqrestore(flags);

  /* Did the caller provide us with a location to return the time
   * remaining?
   */

  if (ts)
    {
      /* Yes.. then calculate and return the time remaining on the
       * oneshot timer.
       */

      tcvdbg("rc=%lu count=%lu usec=%lu\n",
             (unsigned long)rc, (unsigned long)count, (unsigned long)usec);

      if (count <= 0)
        {
          /* No time remaining (?) */

          ts->tv_sec  = 0;
          ts->tv_nsec = 0;
        }
      else
        {
          /* The total time remaining is the difference.  Convert the that
           * to units of microseconds. */

          usec        = (((uint64_t)(count)) * CONFIG_USEC_PER_TICK);

          /* Return the time remaining in the correct form */

          sec         = usec / USEC_PER_SEC;
          nsec        = ((usec) - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

          ts->tv_sec  = (time_t)sec;
          ts->tv_nsec = (unsigned long)nsec;
        }

      tcvdbg("remaining (%lu, %lu)\n",
             (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);
    }

  return OK;
}

#endif /* CONFIG_STM32_ONESHOT */
