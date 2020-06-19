#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "esp32/ulp.h"
#include "ulp_soil.h"
#include "ulptool.h"
#include "config.h"
#include <math.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define ADC_FACTOR (3.5f)
#define ADC_VCC_PIN (ADC2_CHANNEL_9)

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

volatile enum ulpsm_states {
	ULPSM_RESETTING,
	ULPSM_SETUP,
	ULPSM_CONNECTING_WIFI,
	ULPSM_CONNECTING_MQTT,
	ULPSM_PUBLISHING_CONFIG,
	ULPSM_PUBLISHED_CONFIG,
	ULPSM_PUBLISHING_DATA,
	ULPSM_PUBLISHED_DATA,
	ULPSM_DISCONNECTING_MQTT,
	ULPSM_DISCONNECTING_WIFI,
	ULPSM_DONE,
} state;


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

WiFiClient wifi_client;
PubSubClient mqtt(wifi_client);

static void init_ulp_periphery();

/* To be called every time before going into deep sleep.
   It starts the ULP program and resets measurement counter.
 */
static void start_ulp_program();

/* measures voltage on ADC_VCC_PIN(GPIO26) */
static void measure_vcc();

static void init_sensor_coefficients();

static float soil_moisture(uint16_t adc_value, float vcc, float offset, float gradient);

/* fills soil_data struct */
static void calculate_soil_data();

/* logs current soil_data to Serial */
static void print_soil_data();

/* publish sensor config */
static void publish_sensor_config();

/* publish current soil_data via MQTT */
static void publish_soil_data();

void setup() {
	Serial.begin(115200);

	WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
		Serial.printf("WiFi connected & got IP\n");
		mqtt.connect(HOSTNAME);
		state = ULPSM_CONNECTING_MQTT;
	}, WiFiEvent_t::SYSTEM_EVENT_STA_GOT_IP);

	WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
		Serial.printf("WiFi disconnected, ");
		if (state == ULPSM_DISCONNECTING_WIFI) {
			Serial.printf("done.\n");
			state = ULPSM_DONE;
		} else {
			Serial.printf("seems unexpected, resetting.\n");
			state = ULPSM_RESETTING;
		}
	}, WiFiEvent_t::SYSTEM_EVENT_STA_DISCONNECTED);

	esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
	if (cause == ESP_SLEEP_WAKEUP_ULP) {

		Serial.printf("Deep sleep wakeup\n");
		Serial.printf("max_diff:%d\n", (uint16_t)ulp_max_diff);

		if ((uint16_t)ulp_max_diff == 4095) {
			Serial.printf("Skip first measurement after reset\n");
			state = ULPSM_DONE;
			return;
		}

		measure_vcc();

		state = ULPSM_SETUP;
	}

}

void loop() {
	mqtt.loop();

	if (state == ULPSM_RESETTING) {
		Serial.printf("Loading ULP...\n");

		esp_err_t err = ulptool_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
		ESP_ERROR_CHECK(err);

		/* Set ULP wake up period */
		ulp_set_wakeup_period(0, ULP_PERIOD_MS * 1000);

		state = ULPSM_SETUP;
	}

	if (state == ULPSM_SETUP) {
		Serial.printf("Setup WiFi, MQTT, Soil data...\n");

		WiFi.begin(STA_SSID, STA_PASSWD);

		mqtt.setServer(MQTT_HOST, 1883);

		calculate_soil_data();
		print_soil_data();

		state = ULPSM_CONNECTING_WIFI;
	}

	if (state == ULPSM_CONNECTING_MQTT) {
		auto mqtt_state = mqtt.state();
		if (mqtt_state == MQTT_CONNECTED) {
			Serial.printf("MQTT connected.\n");
			esp_reset_reason_t reason = esp_reset_reason();
			if (reason == ESP_RST_DEEPSLEEP)
				state = ULPSM_PUBLISHING_DATA;
			else
				state = ULPSM_PUBLISHING_CONFIG;

		} else if (mqtt_state == MQTT_CONNECTION_TIMEOUT || mqtt_state == MQTT_CONNECT_FAILED) {
			Serial.printf("MQTT connection failed.\n");
			state = ULPSM_DONE;

		} else if (mqtt_state > MQTT_CONNECTED) {
			Serial.printf("MQTT failed after connection: %d\n", mqtt_state);
			state = ULPSM_DONE;

		} else {
			Serial.printf("MQTT unkown error: %d\n", mqtt_state);
			state = ULPSM_DONE;
		}
	}

	if (state == ULPSM_PUBLISHING_CONFIG) {
		Serial.printf("Publishing config...\n");
		publish_sensor_config();
		state = ULPSM_PUBLISHED_CONFIG;
	}

	if (state == ULPSM_PUBLISHING_DATA) {
		Serial.printf("Publishing data...\n");
		publish_soil_data();
		state = ULPSM_PUBLISHED_DATA;
	}

	if (state == ULPSM_PUBLISHED_DATA || state == ULPSM_PUBLISHED_CONFIG) {
		wifi_client.flush();
		mqtt.loop();
		wifi_client.flush();

		mqtt.disconnect();
		state = ULPSM_DISCONNECTING_MQTT;
	}

	if (state == ULPSM_DISCONNECTING_MQTT) {
		Serial.printf("Disconnecting MQTT & WiFi...\n");
		auto mqtt_state = mqtt.state();
		if (mqtt_state != MQTT_CONNECTED) {
			WiFi.disconnect();
			state = ULPSM_DISCONNECTING_WIFI;
		}
	}

	if (state == ULPSM_DONE) {
		Serial.printf("Starting ULP & entering deep sleep\n\n");
		start_ulp_program();

		ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
		esp_deep_sleep_start();
	}
}

static void init_ulp_periphery()
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

	/* Disable pullup on GPIO15, in case it is connected to ground to suppress
	   boot messages.
	 */
	rtc_gpio_pullup_dis(GPIO_NUM_15);
	rtc_gpio_hold_en(GPIO_NUM_15);
}

static void start_ulp_program()
{

	init_ulp_periphery();

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

static void get_device(JsonObject *dev)
{
	(*dev)["name"] = HOSTNAME;
	(*dev)["identifiers"] = HOSTNAME;
	(*dev)["manufacturer"] = "Flobs";
	(*dev)["model"] = "ULPSoilMonitor v2.0";
}

static String get_topic(String object_id)
{
	return String("homeassistant/sensor/") + String(HOSTNAME) + String("/") + String(object_id) + String("/");
}

static String get_topic(const char object_id[])
{
	return get_topic(String(object_id));
}

static String get_unique_id(String object_id)
{
	return String(HOSTNAME) + String("_") + String(object_id);
}

static String get_unique_id(const char object_id[])
{
	return get_unique_id(String(object_id));
}

static void publish_sensor_config_vcc()
{
	String topic = get_topic("vcc");
	String id = get_unique_id("vcc");

	StaticJsonDocument<512> config;
	config["unique_id"] = id;
	config["name"] = id;
	config["state_topic"] = topic + "state";
	config["unit_of_measurement"] = "V";

	JsonObject dev = config.createNestedObject("device");
	get_device(&dev);

	size_t message_size = measureJson(config);
	mqtt.beginPublish((topic + "config").c_str(), message_size, true);
	serializeJson(config, mqtt);
	mqtt.endPublish();

}

static void publish_sensor_config_soil(uint8_t index)
{

	String object_id = String("soil") + String(index);
	String topic = get_topic(object_id);
	String id = get_unique_id(object_id);

	StaticJsonDocument<512> config;
	config["unique_id"] = id;
	config["name"] = id;
	config["state_topic"] = topic + "state";
	config["unit_of_measurement"] = "%";

	JsonObject dev = config.createNestedObject("device");
	get_device(&dev);

	size_t message_size = measureJson(config);
	mqtt.beginPublish((topic + "config").c_str(), message_size, true);
	serializeJson(config, mqtt);
	mqtt.endPublish();
}

static void publish_sensor_config()
{
	publish_sensor_config_vcc();

	for (uint8_t index = 0; index < 6; index++)
		publish_sensor_config_soil(index);
}

static void publish_vcc()
{
	String topic = get_topic("vcc");
	String id = get_unique_id("vcc");

	String vcc = String(soil.vcc);
	mqtt.publish((topic + "state").c_str(), vcc.c_str(), true);
}

static void publish_soil(uint8_t index, float value)
{
	String object_id = String("soil") + String(index);
	String topic = get_topic(object_id);
	String id = get_unique_id(object_id);

	mqtt.publish((topic + "state").c_str(), String(value).c_str(), true);
}

static void publish_soil_data()
{
	publish_vcc();
	publish_soil(0, soil.soil0);
	publish_soil(1, soil.soil1);
	publish_soil(2, soil.soil2);
	publish_soil(3, soil.soil3);
	publish_soil(4, soil.soil4);
	publish_soil(5, soil.soil5);
}
