/*
 * VESC Express target for the RadioStack ESP32-C3 Super Mini pinout.
 *
 * External hardware:
 *   - 3.3 V CAN transceiver (for example SN65HVD230)
 *   - WS2812/WS2812B strip on GPIO10
 */

#ifndef MAIN_HWCONF_SUPER_MINI_C3_RGB_H_
#define MAIN_HWCONF_SUPER_MINI_C3_RGB_H_

#define HW_NAME                     "NyxExpress"
#define HW_TARGET                   "esp32c3"
#define OVR_CONF_PARSER_C           "nyx_main_confparser.c"
#define OVR_CONF_PARSER_H           "nyx_main_confparser.h"
#define OVR_CONF_XML_C              "nyx_main_confxml.c"
#define OVR_CONF_XML_H              "nyx_main_confxml.h"
#define OVR_CONF_DEFAULT             "nyx_main_conf_default.h"
#define OVR_CONF_SERIALIZE           nyx_main_confparser_serialize_main_config_t
#define OVR_CONF_DESERIALIZE         nyx_main_confparser_deserialize_main_config_t
#define OVR_CONF_SET_DEFAULTS        nyx_main_confparser_set_defaults_main_config_t
// Express stores the BLE name in char ble_name[9]: max 8 characters.
// Keep the longer board name in HW_NAME, but use a safe BLE name here.
#define CONF_BLE_NAME               "NyxC3"

// Keep the UART available on the board's TX/RX header pins.
#define HW_UART_COMM
#define UART_NUM                    0
#define UART_BAUDRATE               115200
#define UART_TX                     21
#define UART_RX                     20

// ESP32-C3 TWAI controller -> external 3.3 V CAN transceiver.
// TXD of the transceiver is driven by GPIO1; RXD is read on GPIO0.
#define CAN_TX_GPIO_NUM             1
#define CAN_RX_GPIO_NUM             0

// Hardware wiring reference for the Lisp example.
#define WS2812_GPIO                 10

#include "nyx_lighting.h"
/* Start after lispif_init so the shared RGB driver's mutex is initialized
 * before the firmware renderer can submit its first frame. */
#define HW_POST_LISPIF_HOOK()       nyx_lighting_init(WS2812_GPIO, 1)

#endif /* MAIN_HWCONF_SUPER_MINI_C3_RGB_H */
