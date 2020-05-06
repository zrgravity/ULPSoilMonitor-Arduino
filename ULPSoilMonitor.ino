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
#include <math.h>

#define ADC_FACTOR (3.5f)
#define ADC_VCC_PIN (ADC2_CHANNEL_9)

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

struct {
  float soil0;
  float soil1;
  float soil2;
  float soil3;
  float soil4;
  float soil5;
} sensor_offset, sensor_gradient;

struct soil_data {
  float vcc;
  float soil0;
  float soil1;
  float soil2;
  float soil3;
  float soil4;
  float soil5;
  bool valid = false;
} soil;

/* This function is called once after power-on reset, to load ULP program into
   RTC memory and configure the ADC.
*/
static void init_ulp_program();

/* This function is called every time before going into deep sleep.
   It starts the ULP program and resets measurement counter.
*/
static void start_ulp_program();

/* measures voltage on GPIO26 */
static void measure_vcc();

static void init_sensor_coefficients();

static float soil_moisture(uint16_t adc_value, float vcc, float offset, float gradient);

/* fills soil_data struct */
static void calculate_soil_data();

/* logs current soil_data */
static void print_soil_data();

void setup() {
  Serial.begin(115200);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_ULP) {

    Serial.printf("Deep sleep wakeup\n");
    /* Count temperature form -5 ℃ , so ulp_temperature minus 5 */
    Serial.printf("max_diff:%d\n", (uint16_t)ulp_max_diff);

    measure_vcc();
    calculate_soil_data();
    print_soil_data();

  } else {

    esp_err_t err = ulptool_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

  }

  Serial.printf("Entering deep sleep\n\n");
  start_ulp_program();
  ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
  esp_deep_sleep_start();
}

void loop() {

}

static void init_ulp_program()
{
  /* Configure ADC channel */
  adc1_ulp_enable();
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_0);
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_0);
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_0);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_0);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);
  adc1_config_width(ADC_WIDTH_BIT_12);

  gpio_deep_sleep_hold_en();

  /* Configure SENSOR_ENABLE */
  rtc_gpio_init(GPIO_NUM_25);
  rtc_gpio_set_direction(GPIO_NUM_25, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_pullup_en(GPIO_NUM_25);
  rtc_gpio_set_level(GPIO_NUM_25, 1);
  rtc_gpio_hold_en(GPIO_NUM_25);

  /* Set ULP wake up period to 1000ms */
  ulp_set_wakeup_period(0, 1000 * 1000);

  /* Disable pullup on GPIO15, in case it is connected to ground to suppress
     boot messages.
  */
  rtc_gpio_pullup_dis(GPIO_NUM_15);
  rtc_gpio_hold_en(GPIO_NUM_15);
}

static void start_ulp_program()
{
  init_ulp_program();

  /* Start the program */
  esp_err_t err = ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t));
  ESP_ERROR_CHECK(err);
}

static void measure_vcc()
{
  int raw;

  gpio_reset_pin(GPIO_NUM_25);
  gpio_pad_select_gpio(GPIO_NUM_25);
  gpio_hold_dis(GPIO_NUM_25);
  gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT_OD);
  gpio_set_level(GPIO_NUM_25, 0);

  vTaskDelay(10);

  adc2_config_channel_atten(ADC_VCC_PIN, ADC_ATTEN_DB_0);
  esp_err_t r = adc2_get_raw(ADC_VCC_PIN, ADC_WIDTH_12Bit, &raw);

  gpio_set_level(GPIO_NUM_25, 1);
  gpio_hold_en(GPIO_NUM_25);

  if (r != ESP_OK) {
    soil.vcc = NAN;
    soil.valid = false;
  } else {
    float vcc = ((float)raw * ADC_FACTOR) / 4095;
    soil.vcc = vcc;
  }
}

static void  init_sensor_coefficients()
{
  sensor_offset.soil0 = 1.746;
  sensor_offset.soil1 = 1.746;
  sensor_offset.soil2 = 1.746;
  sensor_offset.soil3 = 1.746;
  sensor_offset.soil4 = 1.746;
  sensor_offset.soil5 = 1.746;

  sensor_gradient.soil0 = -2.849;
  sensor_gradient.soil1 = -2.849;
  sensor_gradient.soil2 = -2.849;
  sensor_gradient.soil3 = -2.849;
  sensor_gradient.soil4 = -2.849;
  sensor_gradient.soil5 = -2.849;
}

static float soil_moisture(uint16_t adc_value, float vcc, float offset, float gradient)
{
  if (adc_value == 0.0) {
    return NAN;
  }
  float moisture = 100.0 * (gradient * ((adc_value * ADC_FACTOR / 4095) / vcc) + offset);

  if (moisture > 100.0)
    moisture = 100.0;
  else if (moisture < 0.0)
    moisture = 0.0;

  return moisture;
}

static void calculate_soil_data()
{
  if (isnan(soil.valid))
    return;

  init_sensor_coefficients();

  soil.soil0 = soil_moisture(ulp_soil0, soil.vcc, sensor_offset.soil0, sensor_gradient.soil0);
  soil.soil1 = soil_moisture(ulp_soil1, soil.vcc, sensor_offset.soil1, sensor_gradient.soil1);
  soil.soil2 = soil_moisture(ulp_soil2, soil.vcc, sensor_offset.soil2, sensor_gradient.soil2);
  soil.soil3 = soil_moisture(ulp_soil3, soil.vcc, sensor_offset.soil3, sensor_gradient.soil3);
  soil.soil4 = soil_moisture(ulp_soil4, soil.vcc, sensor_offset.soil4, sensor_gradient.soil4);
  soil.soil5 = soil_moisture(ulp_soil5, soil.vcc, sensor_offset.soil5, sensor_gradient.soil5);

  soil.valid = true;
}

static void print_soil_data()
{
    Serial.printf("valid: %d\n", soil.valid);
    Serial.printf("VCC: %f\n", soil.vcc);
    Serial.printf("Soil0:%d -> %f\n", (uint16_t)ulp_soil0, soil.soil0);
    Serial.printf("Soil1:%d -> %f\n", (uint16_t)ulp_soil1, soil.soil1);
    Serial.printf("Soil2:%d -> %f\n", (uint16_t)ulp_soil2, soil.soil2);
    Serial.printf("Soil3:%d -> %f\n", (uint16_t)ulp_soil3, soil.soil3);
    Serial.printf("Soil4:%d -> %f\n", (uint16_t)ulp_soil4, soil.soil4);
    Serial.printf("Soil5:%d -> %f\n", (uint16_t)ulp_soil5, soil.soil5);
}
