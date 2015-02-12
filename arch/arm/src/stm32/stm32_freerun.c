/****************************************************************************
 * arch/arm/src/stm32/stm32_freerun.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           David Sidrane <david_s5@nscdg.com>
 *
 * References:
 *
 *   STM32 Series Data Sheet
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
* 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
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

#include <arch/irq.h>
#include <nuttx/clock.h>

#include "stm32_freerun.h"

#ifdef CONFIG_STM32_FREERUN

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

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
static struct stm32_freerun_s *g_freerun;
/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: stm32_freerun_handler
 *
 * Description:
 *   Timer interrupt callback.  When the freerun timer counter overflows,
 *   this interrupt will occur.  We will just increment an overflow count.
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

static int  stm32_freerun_handler(int irq, void *context)
{
  DEBUGASSERT(freerun && g_freerun->overflow < UINT32_MAX);
  g_freerun->overflow++;
  STM32_TIM_ACKINT(g_freerun->dev,0);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_freerun_initialize
 *
 * Description:
 *   Initialize the freerun timer wrapper
 *
 * Input Parameters:
 *   freerun    Caller allocated instance of the freerun state structure
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

int stm32_freerun_initialize(struct stm32_freerun_s *freerun, int chan,
    uint16_t resolution)
{
  int ret = OK;

  tcvdbg("chan=%d resolution=%d usec\n", chan, resolution);
  DEBUGASSERT(freerun && resolution > 0);

  g_freerun = freerun;

  /* Get the Timer Counter frequency the corresponds to the requested resolution */
  freerun->dev = stm32_tim_init(chan);

  if (!freerun->dev)
    {
      tcdbg("ERROR: Failed to allocate timer %d\n", chan);
      ret = -EBUSY;
    }
  else
    {
      /* Initialize the remaining fields in the state structure and return
       * success.
       */

      freerun->overflow = 0;


      /* Set up to receive the callback when the interrupt occurs */

      (void)STM32_TIM_SETISR(freerun->dev, stm32_freerun_handler,0);
      (void)STM32_TIM_SETPERIOD(freerun->dev, STM32_MAX_TIMER_CNT-1);

      /* Enable device interrupts. */

      STM32_TIM_ENABLEINT(freerun->dev,0);

      /* Start the counter */

      (void)STM32_TIM_SETCLOCK(freerun->dev, USEC_PER_SEC / resolution);
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_freerun_counter
 *
 * Description:
 *   Read the counter register of the free-running timer.
 *
 * Input Parameters:
 *   freerun Caller allocated instance of the freerun state structure.  This
 *           structure must have been previously initialized via a call to
 *           stm32_freerun_initialize();
 *   ts      The location in which to return the time from the free-running
 *           timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int stm32_freerun_counter(struct stm32_freerun_s *freerun, struct timespec *ts)
{
  uint64_t usec;
  uint32_t counter;
  uint32_t verify;
  int sr;
  uint32_t overflow;
  uint32_t sec;
  irqstate_t flags;

  DEBUGASSERT(freerun && freerun->tch && ts);


  /* Temporarily disable the overflow counter */

  flags = irqsave();

  counter  = STM32_TIME_GETCOUNTER(freerun->dev, &sr);
  overflow = freerun->overflow;
  verify  = STM32_TIME_GETCOUNTER(freerun->dev, &sr);

  irqrestore(flags);

  tcvdbg("counter=%lu (%lu) overflow=%lu  sr=%08lx\n",
      (unsigned long)counter,(unsigned long)verify,
      (unsigned long)overflow,(unsigned long) sr);

  /* If an interrupt was pending before we re-enabled interrupts,
   * then our value of overflow needs to be incremented.
   */

  if ((sr & ATIM_SR_UIF) != 0)
    {
      /* Increment the overflow count and use the value of the
       * guaranteed to be AFTER the overflow occurred.
       */
      overflow++;
      counter = verify;
      tcvdbg("counter=%lu overflow=%lu\n",
          (unsigned long)counter, (unsigned long)overflow);
    }

  /* Convert the whole thing to units of microseconds. */

  usec = ((uint64_t)overflow * STM32_MAX_TIMER_CNT * CONFIG_USEC_PER_TICK) + ((uint64_t)counter * CONFIG_USEC_PER_TICK);

  /* And return the value of the timer */

  sec         = (uint32_t)(usec / USEC_PER_SEC);
  ts->tv_sec  = sec;
  ts->tv_nsec = (usec - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

  tcvdbg("usec=%llu ts=(%lu, %lu)\n",
      usec, (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);

  return OK;
}

/****************************************************************************
 * Name: stm32_freerun_uninitialize
 *
 * Description:
 *   Stop the free-running timer and release all resources that it uses.
 *
 * Input Parameters:
 *   freerun Caller allocated instance of the freerun state structure.  This
 *           structure must have been previously initialized via a call to
 *           stm32_freerun_initialize();
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int stm32_freerun_uninitialize(struct stm32_freerun_s *freerun)
{
  DEBUGASSERT(freerun && freerun->tch);

  /* Now we can disable the timer interrupt and disable the timer. */

  STM32_TIM_SETMODE(freerun->dev,STM32_TIM_MODE_DISABLED);
  STM32_TIM_DISABLEINT(freerun->dev,0);
  (void)STM32_TIM_SETISR(freerun->dev, NULL,0);

  /* Free the timer */

  stm32_tim_deinit(freerun->dev);
  freerun->dev = NULL;
  g_freerun = NULL;
  return OK;
}

#endif /* CONFIG_STM32_FREERUN */
