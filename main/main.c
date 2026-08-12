/*
	Copyright 2022 Benjamin Vedder      benjamin@vedder.se
	Copyright 2023 Rasmus Söderhielm    rasmus.soderhielm@gmail.com

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "driver/uart.h"

#include "conf_general.h"
#include "comm_ble.h"
#include "comm_uart.h"
#include "comm_usb.h"
#include "comm_can.h"
#include "comm_wifi.h"
#include "commands.h"
#include "flash_helper.h"
#include "crc.h"

#ifdef OVR_CONF_XML_H
#include OVR_CONF_XML_H
#else
#include "confxml.h"
#endif

#ifdef OVR_CONF_PARSER_H
#include OVR_CONF_PARSER_H
#else
#include "confparser.h"
#endif

#include "log.h"
#include "adc.h"
#include "ublox.h"
#include "nmea.h"
#include "terminal.h"
#include "main.h"
#include "mempools.h"
#include "lispif.h"
#include "bms.h"
#include "ble/custom_ble.h"
#include "nyx_lighting.h"

#include <string.h>
#include <sys/time.h>

// Global variables
volatile backup_data backup;

/* Layout used by the pre-NyxExpress firmware. Keep this separate from the
 * current config so adding Lighting/Brake fields does not erase networking
 * and CAN settings stored in the old NVS blob. */
typedef struct {
	int controller_id;
	CAN_BAUD can_baud_rate;
	int can_status_rate_hz;
	WIFI_MODE wifi_mode;
	char wifi_sta_ssid[36];
	char wifi_sta_key[26];
	char wifi_ap_ssid[36];
	char wifi_ap_key[26];
	bool use_tcp_local;
	bool use_tcp_hub;
	char tcp_hub_url[36];
	uint16_t tcp_hub_port;
	char tcp_hub_id[26];
	char tcp_hub_pass[26];
	BLE_MODE ble_mode;
	char ble_name[9];
	uint32_t ble_pin;
	uint32_t ble_service_capacity;
	uint32_t ble_chr_descr_capacity;
} main_config_legacy_t;

/* NyxExpress config immediately before the user-selectable palette was
 * added. The field order must stay identical so the raw NVS blob can be
 * upgraded without dropping the user's lighting, CAN, WiFi or BLE settings. */
typedef struct {
	int controller_id;
	CAN_BAUD can_baud_rate;
	int can_status_rate_hz;
	WIFI_MODE wifi_mode;
	char wifi_sta_ssid[36];
	char wifi_sta_key[26];
	char wifi_ap_ssid[36];
	char wifi_ap_key[26];
	bool use_tcp_local;
	bool use_tcp_hub;
	char tcp_hub_url[36];
	uint16_t tcp_hub_port;
	char tcp_hub_id[26];
	char tcp_hub_pass[26];
	BLE_MODE ble_mode;
	char ble_name[9];
	uint32_t ble_pin;
	uint32_t ble_service_capacity;
	uint32_t ble_chr_descr_capacity;
	uint8_t lighting_enabled;
	uint8_t lighting_effect;
	uint16_t lighting_led_count;
	float lighting_brightness;
	float lighting_speed;
	uint8_t lighting_auto_start;
	uint16_t lighting_red;
	uint16_t lighting_green;
	uint16_t lighting_blue;
	uint8_t brake_enabled;
	uint16_t brake_can_id;
	float brake_adc_threshold;
	uint8_t brake_active_high;
	float brake_brightness;
	float brake_blink_hz;
} main_config_palette_legacy_t;

/* NyxExpress config immediately before the preset-color selectors were
 * added. It includes the user-selectable palette field. */
typedef struct {
	int controller_id;
	CAN_BAUD can_baud_rate;
	int can_status_rate_hz;
	WIFI_MODE wifi_mode;
	char wifi_sta_ssid[36];
	char wifi_sta_key[26];
	char wifi_ap_ssid[36];
	char wifi_ap_key[26];
	bool use_tcp_local;
	bool use_tcp_hub;
	char tcp_hub_url[36];
	uint16_t tcp_hub_port;
	char tcp_hub_id[26];
	char tcp_hub_pass[26];
	BLE_MODE ble_mode;
	char ble_name[9];
	uint32_t ble_pin;
	uint32_t ble_service_capacity;
	uint32_t ble_chr_descr_capacity;
	uint8_t lighting_enabled;
	uint8_t lighting_effect;
	uint16_t lighting_led_count;
	float lighting_brightness;
	float lighting_speed;
	uint8_t lighting_auto_start;
	uint16_t lighting_red;
	uint16_t lighting_green;
	uint16_t lighting_blue;
	uint8_t brake_enabled;
	uint16_t brake_can_id;
	float brake_adc_threshold;
	uint8_t brake_active_high;
	float brake_brightness;
	float brake_blink_hz;
	uint8_t lighting_palette;
} main_config_color_legacy_t;

typedef struct {
	uint32_t controller_id_init_flag;
	uint16_t controller_id;
	uint32_t can_baud_rate_init_flag;
	CAN_BAUD can_baud_rate;
	uint32_t config_init_flag;
	main_config_color_legacy_t config;
	volatile uint32_t pad1;
	volatile uint32_t pad2;
} backup_data_color_legacy_t;

typedef struct {
	uint32_t controller_id_init_flag;
	uint16_t controller_id;
	uint32_t can_baud_rate_init_flag;
	CAN_BAUD can_baud_rate;
	uint32_t config_init_flag;
	main_config_legacy_t config;
	volatile uint32_t pad1;
	volatile uint32_t pad2;
} backup_data_legacy_t;

typedef struct {
	uint32_t controller_id_init_flag;
	uint16_t controller_id;
	uint32_t can_baud_rate_init_flag;
	CAN_BAUD can_baud_rate;
	uint32_t config_init_flag;
	main_config_palette_legacy_t config;
	volatile uint32_t pad1;
	volatile uint32_t pad2;
} backup_data_palette_legacy_t;

// Private variables
volatile static bool init_done = false;

// Private functions
static void terminal_nmea(int argc, const char **argv);
static void terminal_ublox_reinit(int argc, const char **argv);

void app_main(void) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	settimeofday(&tv, NULL);

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		nvs_flash_erase();
		ret = nvs_flash_init();
	}

	{
		nvs_handle_t my_handle;
		nvs_open("vesc", NVS_READONLY, &my_handle);
		size_t required_size = 0;
		nvs_get_blob(my_handle, "backup", NULL, &required_size);
		backup_data_legacy_t legacy = {0};
		backup_data_palette_legacy_t palette_legacy = {0};
		backup_data_color_legacy_t color_legacy = {0};
		bool legacy_loaded = false;
		bool palette_legacy_loaded = false;
		bool color_legacy_loaded = false;

		memset((void*)&backup, 0, sizeof(backup));

		if (required_size == sizeof(backup_data)) {
			nvs_get_blob(my_handle, "backup", (void*)&backup, &required_size);
		} else if (required_size == sizeof(backup_data_palette_legacy_t)) {
			size_t old_size = sizeof(palette_legacy);
			palette_legacy_loaded = nvs_get_blob(my_handle, "backup", &palette_legacy, &old_size) == ESP_OK;
			if (palette_legacy_loaded) {
				backup.controller_id_init_flag = palette_legacy.controller_id_init_flag;
				backup.controller_id = palette_legacy.controller_id;
				backup.can_baud_rate_init_flag = palette_legacy.can_baud_rate_init_flag;
				backup.can_baud_rate = palette_legacy.can_baud_rate;
			}
		} else if (required_size == sizeof(backup_data_color_legacy_t)) {
			size_t old_size = sizeof(color_legacy);
			color_legacy_loaded = nvs_get_blob(my_handle, "backup", &color_legacy, &old_size) == ESP_OK;
			if (color_legacy_loaded) {
				backup.controller_id_init_flag = color_legacy.controller_id_init_flag;
				backup.controller_id = color_legacy.controller_id;
				backup.can_baud_rate_init_flag = color_legacy.can_baud_rate_init_flag;
				backup.can_baud_rate = color_legacy.can_baud_rate;
			}
		} else if (required_size == sizeof(backup_data_legacy_t)) {
			size_t legacy_size = sizeof(legacy);
			legacy_loaded = nvs_get_blob(my_handle, "backup", &legacy, &legacy_size) == ESP_OK;
		}

		if (backup.controller_id_init_flag != VAR_INIT_CODE) {
			backup.controller_id = HW_DEFAULT_ID;
			backup.controller_id_init_flag = VAR_INIT_CODE;
		}

		if (backup.can_baud_rate_init_flag != VAR_INIT_CODE) {
			backup.can_baud_rate = CONF_CAN_BAUD_RATE;
			backup.can_baud_rate_init_flag = VAR_INIT_CODE;
		}

		if (backup.config_init_flag != MAIN_CONFIG_T_SIGNATURE) {
#ifdef OVR_CONF_SET_DEFAULTS
			OVR_CONF_SET_DEFAULTS((main_config_t*)(&backup.config));
#else
			confparser_set_defaults_main_config_t((main_config_t*)(&backup.config));
#endif
			backup.config_init_flag = MAIN_CONFIG_T_SIGNATURE;
			if (color_legacy_loaded) {
				memcpy((void *)&backup.config, &color_legacy.config,
						sizeof(color_legacy.config));
				backup.config.lighting_palette = color_legacy.config.lighting_palette;
				backup.config.lighting_color = 15;
				backup.config.brake_color = 0;
				backup.config.brake_red = 255;
				backup.config.brake_green = 255;
				backup.config.brake_blue = 0;
			} else if (palette_legacy_loaded) {
				memcpy((void *)&backup.config, &palette_legacy.config,
						sizeof(palette_legacy.config));
				backup.config.lighting_palette = 18;
			} else if (legacy_loaded) {
				backup.config.controller_id = legacy.config.controller_id;
				backup.config.can_baud_rate = legacy.config.can_baud_rate;
				backup.config.can_status_rate_hz = legacy.config.can_status_rate_hz;
				backup.config.wifi_mode = legacy.config.wifi_mode;
				memcpy((void *)backup.config.wifi_sta_ssid, legacy.config.wifi_sta_ssid,
						sizeof(backup.config.wifi_sta_ssid));
				memcpy((void *)backup.config.wifi_sta_key, legacy.config.wifi_sta_key,
						sizeof(backup.config.wifi_sta_key));
				memcpy((void *)backup.config.wifi_ap_ssid, legacy.config.wifi_ap_ssid,
						sizeof(backup.config.wifi_ap_ssid));
				memcpy((void *)backup.config.wifi_ap_key, legacy.config.wifi_ap_key,
						sizeof(backup.config.wifi_ap_key));
				backup.config.use_tcp_local = legacy.config.use_tcp_local;
				backup.config.use_tcp_hub = legacy.config.use_tcp_hub;
				memcpy((void *)backup.config.tcp_hub_url, legacy.config.tcp_hub_url,
						sizeof(backup.config.tcp_hub_url));
				backup.config.tcp_hub_port = legacy.config.tcp_hub_port;
				memcpy((void *)backup.config.tcp_hub_id, legacy.config.tcp_hub_id,
						sizeof(backup.config.tcp_hub_id));
				memcpy((void *)backup.config.tcp_hub_pass, legacy.config.tcp_hub_pass,
						sizeof(backup.config.tcp_hub_pass));
				backup.config.ble_mode = legacy.config.ble_mode;
				memcpy((void *)backup.config.ble_name, legacy.config.ble_name,
						sizeof(backup.config.ble_name));
				backup.config.ble_pin = legacy.config.ble_pin;
				backup.config.ble_service_capacity = legacy.config.ble_service_capacity;
				backup.config.ble_chr_descr_capacity = legacy.config.ble_chr_descr_capacity;
			}
			backup.config.controller_id = backup.controller_id;
			backup.config.can_baud_rate = backup.can_baud_rate;
		}

		nvs_close(my_handle);
	}

	adc_init();

#ifdef HW_EARLY_LBM_INIT
	HW_INIT_HOOK();
	lispif_init();
	HW_POST_LISPIF_HOOK();
#endif

	mempools_init();
	bms_init();
	commands_init();
#ifdef CAN_TX_GPIO_NUM
	comm_can_start(CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM);
#endif
	comm_usb_init();

	vTaskDelay(1);

	#if CONFIG_BT_BLUEDROID_ENABLED
	switch (backup.config.ble_mode) {
		case BLE_MODE_DISABLED: {
			break;
		}
		case BLE_MODE_OPEN:
		case BLE_MODE_ENCRYPTED: {
			comm_ble_init();
			break;
		}
		case BLE_MODE_SCRIPTING: {
			custom_ble_init();
			break;
		}
	}
	#endif

	#if CONFIG_ESP_WIFI_ENABLED || CONFIG_ESP_WIFI_REMOTE_ENABLED
	if (backup.config.wifi_mode != WIFI_MODE_DISABLED) {
		comm_wifi_init();
	}
	#endif

	nmea_init();
	log_init();
#ifdef SD_PIN_MOSI
	log_mount_card(SD_PIN_MOSI, SD_PIN_MISO, SD_PIN_SCK, SD_PIN_CS, SDMMC_FREQ_DEFAULT);
#endif
#ifdef NAND_PIN_MOSI
	log_mount_nand_flash(NAND_PIN_MOSI, NAND_PIN_MISO, NAND_PIN_SCK, NAND_PIN_CS, FLASH_FREQ_KHZ);
#endif

#ifndef HW_EARLY_LBM_INIT
	HW_INIT_HOOK();
	lispif_init();
	HW_POST_LISPIF_HOOK();
#endif

#ifndef HW_NO_UART
#ifdef HW_UART_COMM
	comm_uart_init(UART_TX, UART_RX, UART_NUM, UART_BAUDRATE);
#else
	ublox_init(false, 500, UART_NUM, UART_RX, UART_TX);
#endif
#endif

	terminal_register_command_callback(
			"nmea_info",
			"Print NMEA message information",
			0,
			terminal_nmea);

	terminal_register_command_callback(
			"ublox_reinit",
			"Re-initialize ublox gnss receiver",
			0,
			terminal_ublox_reinit);

	init_done = true;

	// Exit main to free up heap-space
	vTaskDelete(NULL);
}

uint32_t main_calc_hw_crc(void) {
	uint32_t crc = 0;

	crc = crc32_with_init(
			data_main_config_t_,
			DATA_MAIN_CONFIG_T__SIZE,
			crc);

	if (flash_helper_code_size(CODE_IND_QML) > 0) {
		crc = crc32_with_init(
				flash_helper_code_data_ptr(CODE_IND_QML),
				flash_helper_code_size(CODE_IND_QML),
				crc);
	}

	return crc;
}

void main_store_backup_data(void) {
	nvs_handle_t my_handle;
	backup.controller_id = backup.config.controller_id;
	backup.can_baud_rate = backup.config.can_baud_rate;
	nvs_open("vesc", NVS_READWRITE, &my_handle);
	nvs_set_blob(my_handle, "backup", (void*)&backup, sizeof(backup_data));
	nvs_commit(my_handle);
	nvs_close(my_handle);
}

bool main_init_done(void) {
	return init_done;
}

void main_wait_until_init_done(void) {
	while (!init_done) {
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
}

static void terminal_nmea(int argc, const char **argv) {
	(void)argc;(void)argv;
	nmea_state_t *s = nmea_get_state();

	commands_printf(
			"GGA Cnt   : %d\n"
			"GSV GP cnt: %d\n"
			"GSV GL cnt: %d\n"
			"RMC cnt   : %d\n"
			"Fix Type  : %s\n"
			"Num sats  : %d\n"
			"HDOP      : %.2f\n"
			"Lat       : %.8f\n"
			"Lon       : %.8f\n"
			"Height    : %f\n"
			"Time      : %02d-%02d-%02d %02d:%02d:%02d\n",
			s->gga_cnt,
			s->gsv_gp_cnt,
			s->gsv_gl_cnt,
			s->rmc_cnt,
			nmea_fix_type(),
			s->gga.n_sat,
			s->gga.h_dop,
			s->gga.lat,
			s->gga.lon,
			s->gga.height,
			s->rmc.yy, s->rmc.mo, s->rmc.dd, s->rmc.hh, s->rmc.mm, s->rmc.ss
			);
}

static void terminal_ublox_reinit(int argc, const char **argv) {
	(void)argc;(void)argv;
	commands_printf("Res: %d", ublox_init(true, 500, UART_NUM, UART_RX, UART_TX));
}
