/* ULP Example: using ADC in deep sleep

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   This file contains assembly code which runs on the ULP.

   ULP wakes up to run this code at a certain period, determined by the values
   in SENS_ULP_CP_SLEEP_CYCx_REG registers. On each wake up, the program
   measures input voltage on the given ADC channel 'adc_oversampling_factor'
   times. Measurements are accumulated and average value is calculated.
   Average value is compared to the two thresholds: 'low_thr' and 'high_thr'.
   If the value is less than 'low_thr' or more than 'high_thr', ULP wakes up
   the chip from deep sleep.
*/

/* ULP assembly files are passed through C preprocessor first, so include directives
   and C macros may be used in these files 
 */
#include "soc/rtc_cntl_reg.h"
#include "soc/soc_ulp.h"

  /* Configure the number of ADC samples to average on each measurement.
     For convenience, make it a power of 2. */
  .set adc_oversampling_factor_log, 2
  .set adc_oversampling_factor, (1 << adc_oversampling_factor_log)

  .set soil_threshold, 300

  .data   /* .data section */

  /* Define variables, which go into .bss section (zero-initialized data) */
  .bss

  .global max_diff
max_diff:
  .long 0

  .global soil0
soil0:
  .long 0

soil0_shadow:
  .long 0

  .global soil1
soil1:
  .long 0

soil1_shadow:
  .long 0
  
  .global soil2
soil2:
  .long 0

soil2_shadow:
  .long 0

   .global soil3
soil3:
  .long 0

soil3_shadow:
  .long 0

   .global soil4
soil4:
  .long 0

soil4_shadow:
  .long 0

   .global soil5
soil5:
  .long 0

soil5_shadow:
  .long 0
  
  /* Code goes into .text section */
  .text
  .global entry
entry:
  /* do measurements using ADC */
  /* r0 will be used as accumulator */
  /* initialize the loop counter */

measure_soil0:
  move r2, adc_soil0
  move r3, save_soil0
  jump start_measurement

  .global save_soil0
save_soil0:
  move r2, soil0_shadow
  st r0, r2, 0

measure_soil1:
  move r2, adc_soil1
  move r3, save_soil1
  jump start_measurement

  .global save_soil1
save_soil1:
  move r2, soil1_shadow
  st r0, r2, 0

measure_soil2:
  move r2, adc_soil2
  move r3, save_soil2
  jump start_measurement

  .global save_soil2
save_soil2:
  move r2, soil2_shadow
  st r0, r2, 0

measure_soil3:
  move r2, adc_soil3
  move r3, save_soil3
  jump start_measurement

  .global save_soil3
save_soil3:
  move r2, soil3_shadow
  st r0, r2, 0

measure_soil4:
  move r2, adc_soil4
  move r3, save_soil4
  jump start_measurement

  .global save_soil4
save_soil4:
  move r2, soil4_shadow
  st r0, r2, 0

measure_soil5:
  move r2, adc_soil5
  move r3, save_soil5
  jump start_measurement

  .global save_soil5
save_soil5:
  move r2, soil5_shadow
  st r0, r2, 0

calc_max_diff:
// -- get maximum change in soil value since last wake --
  move r0, max_diff
  move r1, 0
  st r1, r0, 0
  move r0, 0

// -- soil 0
soil0_diff:
  move r1, soil0
  move r2, soil0_shadow
  move r3, soil0_compare_diff

  ld r1, r1, 0
  ld r2, r2, 0

  sub r0, r1, r2
  jump absolute_diff, OV

soil0_compare_diff:
  move r2, max_diff
  ld r1, r2, 0
  
  sub r3, r0, r1
  jump soil1_diff, OV

  st r0, r2, 0

// -- soil 1
soil1_diff:
  move r1, soil1
  move r2, soil1_shadow
  move r3, soil1_compare_diff

  ld r1, r1, 0
  ld r2, r2, 0

  sub r0, r1, r2
  jump absolute_diff, OV

soil1_compare_diff:
  move r2, max_diff
  ld r1, r2, 0

  sub r3, r0, r1
  jump soil2_diff, OV

  st r0, r2, 0

// -- soil 2
soil2_diff:
  move r1, soil2
  move r2, soil2_shadow
  move r3, soil2_compare_diff

  ld r1, r1, 0
  ld r2, r2, 0

  sub r0, r1, r2
  jump absolute_diff, OV

soil2_compare_diff:
  move r2, max_diff
  ld r1, r2, 0

  sub r3, r0, r1
  jump soil3_diff, OV

  st r0, r2, 0

// -- soil 3
soil3_diff:
  move r1, soil3
  move r2, soil3_shadow
  move r3, soil3_compare_diff

  ld r1, r1, 0
  ld r2, r2, 0

  sub r0, r1, r2
  jump absolute_diff, OV

soil3_compare_diff:
  move r2, max_diff
  ld r1, r2, 0
  
  sub r3, r0, r1
  jump soil4_diff, OV

  st r0, r2, 0

// -- soil 4
soil4_diff:
  move r1, soil4
  move r2, soil4_shadow
  move r3, soil4_compare_diff

  ld r1, r1, 0
  ld r2, r2, 0

  sub r0, r1, r2
  jump absolute_diff, OV

soil4_compare_diff:
  move r2, max_diff
  ld r1, r2, 0
  
  sub r3, r0, r1
  jump soil5_diff, OV

  st r0, r2, 0

// -- soil 5
soil5_diff:
  move r1, soil5
  move r2, soil5_shadow
  move r3, soil5_compare_diff

  ld r1, r1, 0
  ld r2, r2, 0

  sub r0, r1, r2
  jump absolute_diff, OV

soil5_compare_diff:
  move r2, max_diff
  ld r1, r2, 0

  sub r3, r0, r1
  jump try_wake, OV

  st r0, r2, 0

// -- wake in case max_diff >= soil_threshold, copy shadow values to output values
try_wake:
  move r0, max_diff
  ld r0, r0, 0
  
  jumpr copy_and_wake, soil_threshold, GE
  
  .global exit
exit:
  halt

start_measurement:
  move r0, 0
  stage_rst

measure:
  /* measure and add value to accumulator */
  jump r2

accumulate_measurement:
  add r0, r0, r1
  /* increment loop counter and check exit condition */
  stage_inc 1
  jumps measure, adc_oversampling_factor, lt

  /* divide accumulator by adc_oversampling_factor.
     Since it is chosen as a power of two, use right shift */
  rsh r0, r0, adc_oversampling_factor_log
  /* averaged value is now in r0; store it into last_result */

  jump r3

absolute_diff:
  sub r0, r2, r1
  jump r3

adc_soil0:
  adc r1, 0, 8  
  jump accumulate_measurement

adc_soil1:
  adc r1, 0, 5
  jump accumulate_measurement

adc_soil2:
  adc r1, 0, 6
  jump accumulate_measurement

adc_soil3:
  adc r1, 0, 7
  jump accumulate_measurement

adc_soil4:
  adc r1, 0, 4
  jump accumulate_measurement

adc_soil5:
  adc r1, 0, 1
  jump accumulate_measurement

copy_and_wake:
  move r0, soil0_shadow
  move r1, soil0
  ld r0, r0, 0
  st r0, r1, 0
  move r0, soil1_shadow
  move r1, soil1
  ld r0, r0, 0
  st r0, r1, 0
  move r0, soil2_shadow
  move r1, soil2
  ld r0, r0, 0
  st r0, r1, 0
  move r0, soil3_shadow
  move r1, soil3
  ld r0, r0, 0
  st r0, r1, 0
  move r0, soil4_shadow
  move r1, soil4
  ld r0, r0, 0
  st r0, r1, 0
  move r0, soil5_shadow
  move r1, soil5
  ld r0, r0, 0
  st r0, r1, 0

  .global wake_up
wake_up:
  /* Check if the system can be woken up */
  READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)

  and r0, r0, 1
  jump exit, eq

  /* Wake up the SoC, end program */
  wake
  WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
  halt
