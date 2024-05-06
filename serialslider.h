#include <windows.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <winioctl.h>
#include <conio.h>
#include <stdbool.h>
#include <ctype.h>

#define BUFSIZE 128
#define CMD_TIMEOUT 3000

typedef enum slider_cmd {
	SLIDER_CMD_NOP = 0,
	SLIDER_CMD_AUTO_SCAN = 0x01,
	SLIDER_CMD_SET_LED = 0x02,
	SLIDER_CMD_AUTO_SCAN_START = 0x03,
	SLIDER_CMD_AUTO_SCAN_STOP = 0x04,
	SLIDER_CMD_AUTO_AIR = 0x05,
	SLIDER_CMD_AUTO_AIR_START = 0x06,
	SLIDER_CMD_SET_AIR_LED_LEFT = 0x07,
	SLIDER_CMD_SET_AIR_LED_RIGHT = 0x08,
	SLIDER_CMD_DIVA_UNK_09 = 0x09,
	SLIDER_CMD_DIVA_UNK_0A = 0x0A,
	SLIDER_CMD_RESET = 0x10,
	SLIDER_CMD_GET_BOARD_INFO = 0xF0
} slider_cmd_t;

typedef union slider_packet {
	struct {
		uint8_t syn;
		uint8_t cmd;
		uint8_t size;
		union {
			struct {
				uint8_t led_unk;
				uint8_t leds[96];
			};
			char version[32];
			uint8_t pressure[32];
			uint8_t air_status;
			uint8_t air_leds[9];
		};
	};
	uint8_t data[BUFSIZE];
} slider_packet_t;

slider_packet_t request;

BOOL open_port();
void sliderserial_writeresp(slider_packet_t *request);
uint8_t sliderserial_readreq(slider_packet_t *reponse);
void package_init(slider_packet_t *request);
void slider_rst();
void slider_start_scan();
void slider_stop_scan();
void slider_send_leds(const uint8_t *rgb);