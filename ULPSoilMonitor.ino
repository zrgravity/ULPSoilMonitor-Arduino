/*
 * Must allocate more memory for the ulp in 
 * esp32/tools/sdk/include/sdkconfig.h 
 * -> #define CONFIG_ULP_COPROC_RESERVE_MEM
 * for this sketch to compile. 2048b seems 
 * good.
 */
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "esp32/ulp.h"
#include "ulp_soil.h"
#include "ulptool.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

/* This function is called once after power-on reset, to load ULP program into
   RTC memory and configure the ADC.
*/
static void init_ulp_program();

/* This function is called every time before going into deep sleep.
   It starts the ULP program and resets measurement counter.
*/
static void start_ulp_program();

void setup() {
  Serial.begin(115200);
  delay(100);
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause != ESP_SLEEP_WAKEUP_ULP) {
    Serial.printf("Not ULP wakeup\n");
    init_ulp_program();
  } else {
    Serial.printf("Deep sleep wakeup\n");
    /* Count temperature form -5 ℃ , so ulp_temperature minus 5 */
    Serial.printf("max_diff:%d\n", (uint16_t)ulp_max_diff);
    Serial.printf("Soil0:%d\n", (uint16_t)ulp_soil0);
    Serial.printf("Soil1:%d\n", (uint16_t)ulp_soil1);
    Serial.printf("Soil2:%d\n", (uint16_t)ulp_soil2);
    Serial.printf("Soil3:%d\n", (uint16_t)ulp_soil3);
    Serial.printf("Soil4:%d\n", (uint16_t)ulp_soil4);
    Serial.printf("Soil5:%d\n", (uint16_t)ulp_soil5);
  }
  Serial.printf("Entering deep sleep\n\n");
  start_ulp_program();
  ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
  esp_deep_sleep_start();
}

void loop() {

}

static void init_ulp_program()
{
  esp_err_t err = ulptool_load_binary(0, ulp_main_bin_start,
                                  (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
  ESP_ERROR_CHECK(err);

  /* Configure ADC channel */
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_0);
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_0);
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_0);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_0);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_ulp_enable();

  gpio_deep_sleep_hold_en();

  rtc_gpio_init(GPIO_NUM_25);
  rtc_gpio_set_direction(GPIO_NUM_25, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_pullup_en(GPIO_NUM_25);
  rtc_gpio_set_level(GPIO_NUM_25, 1);
  rtc_gpio_hold_en(GPIO_NUM_25);

  /* Set ULP wake up period to 1000ms */
  ulp_set_wakeup_period(0, 10000 * 1000);

  /* Disable pullup on GPIO15, in case it is connected to ground to suppress
     boot messages.
  */
  rtc_gpio_pullup_dis(GPIO_NUM_15);
  rtc_gpio_hold_en(GPIO_NUM_15);
}

static void start_ulp_program()
{
  /* Start the program */
  esp_err_t err = ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t));
  ESP_ERROR_CHECK(err);
}
