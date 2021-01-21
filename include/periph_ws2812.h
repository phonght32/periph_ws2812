#ifndef _PERIPH_WS2812_H_
#define _PERIPH_WS2812_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_peripherals.h"

#include "driver/gpio.h"
#include "driver/rmt.h"

typedef struct periph_ws2812 *periph_ws2812_handle_t;
typedef struct  anim_item *periph_ws2812_anim_handle_t;

typedef enum {
	WS2812_ANIM_TYPE_NONE = 0,
	WS2812_ANIM_TYPE_RAINBOW,
} ws2812_anim_type_t;

typedef struct {
	gpio_num_t 				pin;
	rmt_channel_t 			rmt_chnl;
	uint32_t 				num_led;
	uint32_t 				speed;
	uint32_t 				color;
	uint8_t 				brightness;
	ws2812_anim_type_t 		anim_type;
} periph_ws2812_anim_cfg_t;

typedef struct {
	const char 		*tag;
} periph_ws2812_cfg_t;

esp_periph_handle_t periph_ws2812_init(periph_ws2812_cfg_t *config);
periph_ws2812_anim_handle_t periph_ws2812_anim_init(periph_ws2812_anim_cfg_t *config);
esp_err_t periph_ws2812_set_anim_type(periph_ws2812_anim_handle_t anim_item, ws2812_anim_type_t anim_type);
esp_err_t periph_ws2812_set_speed(periph_ws2812_anim_handle_t anim_item, uint32_t speed);
esp_err_t periph_ws2812_set_brightness(periph_ws2812_anim_handle_t anim_item, uint8_t brightness);
esp_err_t periph_ws2812_set_color(periph_ws2812_anim_handle_t anim_item, uint32_t rgb);
esp_err_t periph_ws2812_anim_pause(periph_ws2812_anim_handle_t anim_item);
esp_err_t periph_ws2812_anim_run(periph_ws2812_anim_handle_t anim_item);

#ifdef __cplusplus
}
#endif

#endif /* _PERIPH_WS2812_H_ */