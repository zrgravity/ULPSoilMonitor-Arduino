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
#include <Preferences.h>

#define ADC_FACTOR (3.5f)
#define ADC_VCC_PIN (ADC2_CHANNEL_9)

#define DEFAULT_SENSOR_OFFSET (1.746f)
#define DEFAULT_SENSOR_GRADIENT (-2.849f)

#define PREFNAME "ULPSM"

static Preferences prefs;

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

volatile enum ulpsm_states {
	ULPSM_RESETTING,
	ULPSM_SETUP,

	ULPSM_CONNECTING_WIFI,
	ULPSM_CONNECTING_MQTT,

	ULPSM_CONNECTED,

	ULPSM_PUBLISHING_CONFIG,
	ULPSM_PUBLISHED_CONFIG,
	ULPSM_PUBLISHING_DATA,
	ULPSM_PUBLISHED_DATA,

	ULPSM_ADJUSTMENT_START,
	ULPSM_ADJUSTMENT_IDLE,
	ULPSM_ADJUSTMENT_MEASURE_LOW_VALUE,
	ULPSM_ADJUSTMENT_MEASURE_HIGH_VALUE,
	ULPSM_ADJUSTMENT_CALCULATING,
	ULPSM_ADJUSTMENT_ABORT,
	ULPSM_ADJUSTMENT_RESET,

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

struct adjustment_data {
	int low_value = 0;
	int high_value = 0;
} adjustments[6];

adc1_channel_t sensor_map[] = {
	ADC1_CHANNEL_7,
	ADC1_CHANNEL_4,
	ADC1_CHANNEL_5,
	ADC1_CHANNEL_6,
	ADC1_CHANNEL_3,
	ADC1_CHANNEL_0,
};

WiFiClient wifi_client;
PubSubClient mqtt(wifi_client);

static void init_ulp_periphery();

/* To be called every time before going into deep sleep.
   It starts the ULP program and resets measurement counter.
 */
static void start_ulp_program();

static void activate_sensors();
static void deactivate_sensors();

/* measures voltage on ADC_VCC_PIN(GPIO26) */
static void measure_vcc();

static void init_sensor_coefficients();
static void store_sensor_coefficients();
static void reset_sensor_coefficients();

static float soil_moisture(uint16_t adc_value, float vcc, float offset, float gradient);

/* fills soil_data struct */
static void calculate_soil_data();

/* logs current soil_data to Serial */
static void print_soil_data();

/* publish sensor config */
static void publish_sensor_config();

static void publish_system_config();

/* publish current soil_data via MQTT */
static void publish_soil_data();

static void publish_system_data();

static void mqtt_callback(const char* topic, byte* payload, unsigned int length);

static void init_adjustment_topic();

static int measure_sensor(int sensor);

static void calculate_adjustment();

static void publish_adjustement_state(const char* state);

void setup() {
	Serial.begin(115200);

	Serial.printf("This is %s\n", HOSTNAME);

	ulp_reboots += 1;

	WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
		Serial.printf("WiFi connected & got IP\n");
		mqtt.setCallback(mqtt_callback);
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

		ulp_reboots = 0;
		ulp_runs = 0;

		/* Set ULP wake up period */
		ulp_set_wakeup_period(0, ULP_PERIOD_MS * 1000);

		state = ULPSM_SETUP;
	}

	if (state == ULPSM_SETUP) {
		Serial.printf("Setup WiFi, MQTT, Soil data...\n");
		measure_vcc();

		WiFi.begin(STA_SSID, STA_PASSWD);

		mqtt.setServer(MQTT_HOST, 1883);

		calculate_soil_data();
		print_soil_data();

		Serial.printf("VCC: %f\n", soil.vcc);

		state = ULPSM_CONNECTING_WIFI;
	}

	if (state == ULPSM_CONNECTING_MQTT) {
		auto mqtt_state = mqtt.state();
		if (mqtt_state == MQTT_CONNECTED) {
			Serial.printf("MQTT connected.\n");

			init_adjustment_topic();

			state = ULPSM_CONNECTED;

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

	if (state == ULPSM_CONNECTED) {
		esp_reset_reason_t reason = esp_reset_reason();
		if (reason == ESP_RST_DEEPSLEEP) {
			state = ULPSM_PUBLISHING_DATA;
		} else {
			static auto connection_time = 0;
			if (!connection_time) {
				connection_time = millis();
			} else if (millis() - connection_time > 750) {
				state = ULPSM_PUBLISHING_CONFIG;
			}
		}
	}

	if (state == ULPSM_PUBLISHING_CONFIG) {
		Serial.printf("Publishing config...\n");
		publish_sensor_config();
		publish_system_config();
		publish_vcc();
		state = ULPSM_PUBLISHED_CONFIG;
	}

	if (state == ULPSM_PUBLISHING_DATA) {
		Serial.printf("Publishing data...\n");
		publish_vcc();
		publish_soil_data();
		publish_system_data();

		mqtt.loop();

		state = ULPSM_PUBLISHED_DATA;
	}

	if (state == ULPSM_PUBLISHED_DATA || state == ULPSM_PUBLISHED_CONFIG) {
		Serial.printf("About to disconnect.\n");
		wifi_client.flush();
		mqtt.loop();
		wifi_client.flush();

		if (!wifi_client.available()) {
			mqtt.disconnect();
			state = ULPSM_DISCONNECTING_MQTT;
		}
	}

	if (state == ULPSM_ADJUSTMENT_START) {
		Serial.printf("Adjustment starting...\n");

		activate_sensors();

		Serial.printf("Activated sensors.\n");

		publish_adjustement_state("started");

		state = ULPSM_ADJUSTMENT_IDLE;
	}

	if (state == ULPSM_ADJUSTMENT_MEASURE_LOW_VALUE) {
		for (auto i = 0; i < 6; i++) {
			adjustments[i].low_value = measure_sensor(i);
			Serial.printf("Got low value %i on sensor %i\n", adjustments[i].low_value, i);
		}

		publish_adjustement_state("idle");

		state = ULPSM_ADJUSTMENT_IDLE;
	}

	if (state == ULPSM_ADJUSTMENT_MEASURE_HIGH_VALUE) {
		for (auto i = 0; i < 6; i++) {
			adjustments[i].high_value = measure_sensor(i);
			Serial.printf("Got high value %i on sensor %i\n", adjustments[i].high_value, i);
		}

		publish_adjustement_state("idle");

		state = ULPSM_ADJUSTMENT_IDLE;
	}

	if (state == ULPSM_ADJUSTMENT_CALCULATING) {
		Serial.printf("Adjustment calculation...\n");

		deactivate_sensors();

		calculate_adjustment();

		store_sensor_coefficients();

		publish_adjustement_state("done");

		state = ULPSM_PUBLISHING_CONFIG;
	}

	if (state == ULPSM_ADJUSTMENT_ABORT) {
		Serial.printf("Adjustment aborting...\n");

		deactivate_sensors();

		publish_adjustement_state("aborted");

		state = ULPSM_RESETTING;
	}

	if (state == ULPSM_ADJUSTMENT_RESET) {
		Serial.printf("Adjustment resetting...\n");

		reset_sensor_coefficients();

		publish_adjustement_state("idle");

		state = ULPSM_ADJUSTMENT_IDLE;
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

		ulp_runs = 0;
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

static void activate_sensors()
{
	gpio_reset_pin(GPIO_NUM_25);
	gpio_pad_select_gpio(GPIO_NUM_25);
	gpio_hold_dis(GPIO_NUM_25);
	gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT_OD);
	gpio_set_level(GPIO_NUM_25, 0);

	vTaskDelay(10);
}

static void deactivate_sensors()
{
	gpio_set_level(GPIO_NUM_25, 1);
	gpio_hold_en(GPIO_NUM_25);
}


static void measure_vcc()
{
	int raw;

	activate_sensors();

	adc2_config_channel_atten(ADC_VCC_PIN, ADC_ATTEN_DB_0);
	esp_err_t r = adc2_get_raw(ADC_VCC_PIN, ADC_WIDTH_12Bit, &raw);

	deactivate_sensors();

	if (r != ESP_OK) {
		soil.vcc = NAN;
		soil.valid = false;
	} else {
		float vcc = ((float)raw * ADC_FACTOR) / 4095;
		soil.vcc = vcc;
	}
}

static void init_sensor_coefficients()
{
	prefs.begin(PREFNAME, true);

	sensor_offset.soil0 = prefs.getFloat("offset_0", DEFAULT_SENSOR_OFFSET);
	sensor_offset.soil1 = prefs.getFloat("offset_1", DEFAULT_SENSOR_OFFSET);
	sensor_offset.soil2 = prefs.getFloat("offset_2", DEFAULT_SENSOR_OFFSET);
	sensor_offset.soil3 = prefs.getFloat("offset_3", DEFAULT_SENSOR_OFFSET);
	sensor_offset.soil4 = prefs.getFloat("offset_4", DEFAULT_SENSOR_OFFSET);
	sensor_offset.soil5 = prefs.getFloat("offset_5", DEFAULT_SENSOR_OFFSET);

	sensor_gradient.soil0 = prefs.getFloat("gradient_0", DEFAULT_SENSOR_GRADIENT);
	sensor_gradient.soil1 = prefs.getFloat("gradient_1", DEFAULT_SENSOR_GRADIENT);
	sensor_gradient.soil2 = prefs.getFloat("gradient_2", DEFAULT_SENSOR_GRADIENT);
	sensor_gradient.soil3 = prefs.getFloat("gradient_3", DEFAULT_SENSOR_GRADIENT);
	sensor_gradient.soil4 = prefs.getFloat("gradient_4", DEFAULT_SENSOR_GRADIENT);
	sensor_gradient.soil5 = prefs.getFloat("gradient_5", DEFAULT_SENSOR_GRADIENT);

	prefs.end();
}

static void store_sensor_coefficients()
{
	prefs.begin(PREFNAME);

	prefs.putFloat("offset_0", sensor_offset.soil0);
	prefs.putFloat("offset_1", sensor_offset.soil1);
	prefs.putFloat("offset_2", sensor_offset.soil2);
	prefs.putFloat("offset_3", sensor_offset.soil3);
	prefs.putFloat("offset_4", sensor_offset.soil4);
	prefs.putFloat("offset_5", sensor_offset.soil5);

	prefs.putFloat("gradient_0", sensor_gradient.soil0);
	prefs.putFloat("gradient_1", sensor_gradient.soil1);
	prefs.putFloat("gradient_2", sensor_gradient.soil2);
	prefs.putFloat("gradient_3", sensor_gradient.soil3);
	prefs.putFloat("gradient_4", sensor_gradient.soil4);
	prefs.putFloat("gradient_5", sensor_gradient.soil5);

	prefs.end();
}

static void reset_sensor_coefficients()
{
	prefs.begin(PREFNAME);

	prefs.putFloat("offset_0", DEFAULT_SENSOR_OFFSET);
	prefs.putFloat("offset_1", DEFAULT_SENSOR_OFFSET);
	prefs.putFloat("offset_2", DEFAULT_SENSOR_OFFSET);
	prefs.putFloat("offset_3", DEFAULT_SENSOR_OFFSET);
	prefs.putFloat("offset_4", DEFAULT_SENSOR_OFFSET);
	prefs.putFloat("offset_5", DEFAULT_SENSOR_OFFSET);

	prefs.putFloat("gradient_0", DEFAULT_SENSOR_GRADIENT);
	prefs.putFloat("gradient_1", DEFAULT_SENSOR_GRADIENT);
	prefs.putFloat("gradient_2", DEFAULT_SENSOR_GRADIENT);
	prefs.putFloat("gradient_3", DEFAULT_SENSOR_GRADIENT);
	prefs.putFloat("gradient_4", DEFAULT_SENSOR_GRADIENT);
	prefs.putFloat("gradient_5", DEFAULT_SENSOR_GRADIENT);

	prefs.end();

	init_sensor_coefficients();
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
	if (isnan(soil.vcc) or soil.vcc == 0.0)
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

static void publish_system_config_entry(String name, String uom)
{
	String topic = get_topic(name);
	String id = get_unique_id(name);

	StaticJsonDocument<512> config;
	config["unique_id"] = id;
	config["name"] = id;
	config["state_topic"] = topic + "state";
	config["unit_of_measurement"] = uom;

	JsonObject dev = config.createNestedObject("device");
	get_device(&dev);

	size_t message_size = measureJson(config);
	mqtt.beginPublish((topic + "config").c_str(), message_size, true);
	serializeJson(config, mqtt);
	mqtt.endPublish();
}

static void publish_system_config()
{
	publish_system_config_entry(String("reboots"), String("#"));
	publish_system_config_entry(String("ulp_runs"), String("#"));
	publish_system_config_entry(String("adjustment"), String(""));
}

static void publish_vcc()
{
	if (isnan(soil.vcc) or soil.vcc == 0.0)
		return;

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
	if (!soil.valid)
		return;

	publish_soil(0, soil.soil0);
	publish_soil(1, soil.soil1);
	publish_soil(2, soil.soil2);
	publish_soil(3, soil.soil3);
	publish_soil(4, soil.soil4);
	publish_soil(5, soil.soil5);
}

static void publish_system_data()
{
	mqtt.publish((get_topic(String("reboots")) + "state").c_str(), String((uint16_t)ulp_reboots).c_str(), true);
	mqtt.publish((get_topic(String("ulp_runs")) + "state").c_str(), String((uint16_t)ulp_runs).c_str(), true);

}

static void mqtt_callback(const char* topic, byte* payload, unsigned int length)
{
	String adjustment_topic = get_topic("adjustment") + "state";

	if (adjustment_topic.equals(String(topic))) {
		char *command_data = (char *)malloc(length + 1);
		memcpy(command_data, payload, length);
		command_data[length] = '\0';

		String command = String((char *)command_data);

		Serial.printf("Adjustment command: '%s'\n", command);

		if (state == ULPSM_CONNECTED) {
			if (command.equals(String("start"))) {
				Serial.printf("Starting adjustment.\n");
				state = ULPSM_ADJUSTMENT_START;
			}

		} else if (state == ULPSM_ADJUSTMENT_IDLE) {
			if (command.equals(String("low"))) {
				Serial.printf("Measuring low value.\n");
				state = ULPSM_ADJUSTMENT_MEASURE_LOW_VALUE;

			} else if (command.equals(String("high"))) {
				Serial.printf("Measuring high value.\n");
				state = ULPSM_ADJUSTMENT_MEASURE_HIGH_VALUE;

			} else if (command.equals(String("calculate"))) {
				Serial.printf("Calculating adjustment.\n");
				state = ULPSM_ADJUSTMENT_CALCULATING;

			} else if (command.equals(String("abort"))) {
				Serial.printf("Aborting adjustment.\n");
				state = ULPSM_ADJUSTMENT_ABORT;

			} else if (command.equals(String("reset"))) {
				Serial.printf("Resetting adjustment.\n");
				state = ULPSM_ADJUSTMENT_RESET;
			}
		}
	}
}

static void init_adjustment_topic()
{
	String topic = get_topic("adjustment") + "state";

	Serial.printf("Subscribing %s\n", topic.c_str());

	mqtt.subscribe(topic.c_str(), 0);

	mqtt.loop();
}

static int measure_sensor(int sensor)
{
	adc1_channel_t channel = sensor_map[sensor];

	adc1_config_channel_atten(channel, ADC_ATTEN_DB_0);
	adc1_config_width(ADC_WIDTH_BIT_12);

	return adc1_get_raw(channel);
}

static void calculate_adjustment_sensor(int sensor, float &offset, float &gradient)
{
	struct adjustment_data adj = adjustments[sensor];

	float low = ((float)adj.low_value * ADC_FACTOR) / 4095;
	float high = ((float)adj.high_value * ADC_FACTOR) / 4095;

	float vcc = soil.vcc;

	if (low == 0.0 || high == 0.0 || vcc == 0.0) {
		Serial.printf("Skipping sensor %i\n", sensor);
	} else {
		gradient = 1 / (high / vcc - low / vcc);
		offset = 1 - gradient * high / vcc;

		Serial.printf("Sensor %i; %.3f, %.3f\n", sensor, offset, gradient);
	}
}

static void calculate_adjustment()
{
	calculate_adjustment_sensor(0, sensor_offset.soil0, sensor_gradient.soil0);
	calculate_adjustment_sensor(1, sensor_offset.soil1, sensor_gradient.soil1);
	calculate_adjustment_sensor(2, sensor_offset.soil2, sensor_gradient.soil2);
	calculate_adjustment_sensor(3, sensor_offset.soil3, sensor_gradient.soil3);
	calculate_adjustment_sensor(4, sensor_offset.soil4, sensor_gradient.soil4);
	calculate_adjustment_sensor(5, sensor_offset.soil5, sensor_gradient.soil5);
}

static void publish_adjustement_state(const char* state)
{
	String adjustment_topic = get_topic("adjustment") + "state";
	mqtt.publish(adjustment_topic.c_str(), state, true);
}
