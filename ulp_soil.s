/* ULP Soil Sensor

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   This file contains assembly code which runs on the ULP.
*/

/* ULP assembly files are passed through C preprocessor first, so include directives
   and C macros may be used in these files 
 */
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

  /* Configure the number of ADC samples to average on each measurement.
     For convenience, make it a power of 2. */
  .set adc_oversampling_factor_log, 2
  .set adc_oversampling_factor, (1 << adc_oversampling_factor_log)

  .set soil_threshold, 300

  .data   /* .data section */

  /* Define variables, which go into .bss section (zero-initialized data) */
  .bss

  /* Stores maximum difference of last sent value vs. last measured value of all soil sensors */
  .global max_diff
max_diff:
  .long 0

  /* soilx value stores last sent value */
  .global soil0
soil0:
  .long 0

  /* soilx_shadow stores last measured value */
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
  
  /* Entry point into code. */
  .text
  .global entry
entry:

  /* set GPIO25/RTC_GPIO06 low to enable moisture sensors */
enable_sensors:
  WRITE_RTC_REG(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_HOLD_S, 1, 0)
  WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + 6, 1, 1)
  WRITE_RTC_REG(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_HOLD_S, 1, 1)

  /* r2: adc_soilx maps soil sensor to adc channel */
  /* r3: after measurement return to store soilx value to its shadow value */
measure_soil0:
  move r2, adc_soil0
  move r3, save_soil0
  jump start_measurement

  .global save_soil0
save_soil0:
  /* stores current measurement as shadow value */
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

  /* set GPIO25/RTC_GPIO06 high to disable moisture sensors */
disable_sensors:
  WRITE_RTC_REG(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_HOLD_S, 1, 0)
  WRITE_RTC_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + 6, 1, 1)
  WRITE_RTC_REG(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_HOLD_S, 1, 1)
  /* get maximum change in soil value compared to last sent */
calc_max_diff:
  /* reset max_diff, r0, r1 to zero */
  move r0, max_diff
  move r1, 0
  st r1, r0, 0
  move r0, 0

soil0_diff:
  move r1, soil0
  move r2, soil0_shadow
  move r3, soil0_compare_diff

  /* load last sent value and shadow value into r1, r2 */
  ld r1, r1, 0
  ld r2, r2, 0

  /* calculate absolute difference between sent and shadow value to r0 */
  sub r0, r1, r2
  /* in case r2 > r1, absolute_diff calcs r0 = r2 - r1 and returnso to r3 */
  jump absolute_diff, OV

soil0_compare_diff:
  /* load current max_diff to r1 */
  move r2, max_diff
  ld r1, r2, 0

  /* in case r1 (max_diff) > r0 (soil0_diff) continue to next soil diff */
  sub r3, r0, r1
  jump soil1_diff, OV

  /* in case r0 (soil0_diff) > r1 (max_diff) store soil0_diff to max_value */
  st r0, r2, 0

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

/* wake in case max_diff >= soil_threshold, copy shadow values to output values else halt */
try_wake:
  move r0, max_diff
  ld r0, r0, 0
  
  jumpr copy_and_wake, soil_threshold, GE
  
  .global exit
exit:
  halt

/* reset accumulator r0 and stage counter */
start_measurement:
  move r0, 0
  stage_rst

measure:
  /* measure adc value to r1, according to soil -> adc channel mapping */
  jump r2

accumulate_measurement:
  /* accumulate adc measurements in r0 */
  add r0, r0, r1

  /* increment loop counter and check exit condition */
  stage_inc 1
  jumps measure, adc_oversampling_factor, lt

  /* divide accumulator by adc_oversampling_factor.
     Since it is chosen as a power of two, use right shift */
  rsh r0, r0, adc_oversampling_factor_log

  /* averaged value is now in r0
     jump back to store value */
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
  /* copy shadow values into sensor value location */
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

  /* wake main processor */
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
