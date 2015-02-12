/****************************************************************************
 * arch/arm/src/stm32/stm32_oneshot.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           David Sidrane <david_s5@nscdg.com>
 *
 * References:
 *
 *   STM32Series Data Sheet
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
#include <arch/board/board.h> // delete me with PROBES

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

#define MHZ 1000000

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
  oneshot_handler_t oneshot_handler;
  void *oneshot_arg;

  tcllvdbg("Expired...\n");
  DEBUGASSERT(oneshot && oneshot->handler);
  PROBE(4,true);
  PROBE(5,true);

  switch(--oneshot->loops)
  {

  case 0:

    /* State: loops started above 0 and has transitioned from 1 to 0
     * which occurs because the count needed is greater then the timer
     * can hold in one go. So we must transition from continuous
     * mode to pulse
     */

    if (oneshot->remainder)
      {
        PROBE_MARK(5);
        /* Process the remander with */
        (void) STM32_TIME_STARTTIMER(oneshot->dev, oneshot->frequency, STM32_TIM_MODE_PULSE,
            oneshot->remainder);

        /* mark it done */

        oneshot->remainder = 0;
        break;
      }

    /* no break */

  case -1:

    /* State: a) loops started above 0 and has transitioned from 0 to -1
     * OR b) started at 0 transitioned from 0 to -1
     * This case handles a count that is less than or equal to what  the timer
     * can hold in one go.
     * Since b) can happen with out a) we must transition from continuous
     * mode to pulse here as well
     */

    if (oneshot->fraction)
      {
        PROBE_MARK(5);
        PROBE_MARK(5);

        /* Process the fractions with 1 us periods */

        (void) STM32_TIME_STARTTIMER(oneshot->dev, (1*MHZ), STM32_TIM_MODE_PULSE,
            oneshot->fraction);

        /* mark it done */

        oneshot->fraction = 0;
        break;
      }

    /* no break */

  case -2:

    /* State: We have either processed both a remainder and a fraction
     * or none either way we are are done.
     */

    PROBE_MARK(5);
    PROBE_MARK(5);
    PROBE_MARK(5);

    /* If we had a remainder or a fract the pulse mode was used and the clock
     * was stopped, and disabled when the over flow occurred.
     * But in case we did not have a remainder or fract shut it down
     */

    (void)STM32_TIM_DISABLEINT(oneshot->dev,0);
    (void)STM32_TIM_SETMODE(oneshot->dev, STM32_TIM_MODE_DISABLED);

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

    /* State: loops started above 1  which occurs because the count
     * needed is greater then the timer can hold in one go.
     * keep looping until 0 are left. Then process any remainder
     * and fraction.
     */

    break;
  }
  STM32_TIM_ACKINT(oneshot->dev,0);
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

  oneshot->dev = stm32_tim_init(chan);

  if (!oneshot->dev)
    {
      tcdbg("ERROR: Failed to allocate timer %d\n", chan);
      ret = -EBUSY;
    }
  else
    {
      /* Initialize the remaining fields in the state structure */

      oneshot->running    = false;
      oneshot->handler    = NULL;
      oneshot->arg        = NULL;
      oneshot->resolution = resolution;
      oneshot->frequency  = USEC_PER_SEC / resolution;

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
  uint64_t usec;
  uint64_t resolutionticks;
  uint16_t period;
  stm32_tim_mode_t mode;
  uint32_t frequency;
  irqstate_t flags;

  PROBE(1,true);

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

    frequency  = oneshot->frequency;
    resolutionticks    = usec / oneshot->resolution;
    oneshot->loops     = resolutionticks / STM32_MAX_TIMER_CNT;
    oneshot->remainder = resolutionticks % STM32_MAX_TIMER_CNT;
    oneshot->fraction  = usec % oneshot->resolution;

    if (oneshot->loops >= 1)
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

  (void)STM32_TIM_SETISR(oneshot->dev, stm32_oneshot_handler, 0);

  /* Ack any interrupts. */

  STM32_TIM_ACKINT(oneshot->dev,0);


  (void) STM32_TIME_STARTTIMER(oneshot->dev, frequency, mode, period);

  /* Enable device interrupts. */

  STM32_TIM_ENABLEINT(oneshot->dev,0);

  /* Enable interrupts.  We should get the callback when the interrupt
   * occurs.
   */

  oneshot->running = true;
  irqrestore(flags);
  PROBE(1,false);
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
  uint64_t count;
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

  /* Read remaining */

  count = (uint64_t)STM32_TIME_GETREMAINING(oneshot->dev, &sr);

  /* Now we can disable the interrupt and stop the timer. */

  (void)STM32_TIM_SETCLOCK(oneshot->dev,0);

  /* Set up to receive the callback when the interrupt occurs */

  (void)STM32_TIM_SETISR(oneshot->dev, NULL, 0);

  /* Enable device interrupts. */

  STM32_TIM_DISABLEINT(oneshot->dev,0);

  /* Ack any interrupts. */

  STM32_TIM_ACKINT(oneshot->dev,0);


  count += oneshot->remainder;
  if (oneshot->loops > 0)
    {
      count += oneshot->loops * STM32_MAX_TIMER_CNT;
    }

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

      if (count == 0 && oneshot->fraction == 0)
        {
          /* No time remaining (?) */

          ts->tv_sec  = 0;
          ts->tv_nsec = 0;
        }
      else
        {
          /* The total time remaining is the difference.  Convert the that
           * to units of microseconds. */

          usec        = count * CONFIG_USEC_PER_TICK;
          usec        += oneshot->fraction;


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
