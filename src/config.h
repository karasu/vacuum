#ifndef _CONFIG_H_
#define _CONFIG_H_

#define ROOMBA_650_SLEEP_FIX

#define D_PROGRAMNAME "roomba-esp8266"
#define D_AUTHOR "karasu"
#define HOSTNAME "roomba"
#define WIFI_SSID "Unitat-Popular"
#define WIFI_PASSWORD ""
#define OTA_PASSWORD ""

#define MQTT_SERVER "10.1.1.178"
#define MQTT_PORT 1883
#define MQTT_USER "mosquitto"
#define MQTT_PASS ""
#define MQTT_COMMAND_TOPIC "vacuum/command"
#define MQTT_STATE_TOPIC "vacuum/state"

// #define ENABLE_ADC_SLEEP
#define ADC_VOLTAGE_DIVIDER 44.551316985

#define LOGGING

#endif
