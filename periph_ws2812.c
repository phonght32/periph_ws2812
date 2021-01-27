#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "colors.h"
#include "periph_ws2812.h"

#define RMT_NON_RX_BUFF_SIZE        (0) 												/*!< Non rx buff size */
#define RMT_DEFAULT_FLAG            (0)
#define RMT_APB_CLOCK_SOURCE        (APB_CLK_FREQ / 1e6) 								/*!< Mhz */
#define RMT_TIMER_DEVIDER           (4)
#define RMT_TICK_TIME               (RMT_TIMER_DEVIDER * 1e3 / RMT_APB_CLOCK_SOURCE) 	/*!< in ns */

#define WS2812_BIT0_HIGH_TICK_NUM   (250 / RMT_TICK_TIME) 		/*!< 0.35 us: bit 0 high level time  */
#define WS2812_BIT0_LOW_TICK_NUM    (1000 / RMT_TICK_TIME)  	/*!< 0.8 us: bit 0 low level time */
#define WS2812_BIT1_HIGH_TICK_NUM   (850 / RMT_TICK_TIME) 		/*!< 0.7 us: bit 1 high level time */
#define WS2812_BIT1_LOW_TICK_NUM    (400 / RMT_TICK_TIME)  		/*!< 0.6 us: bit 1 low level time */
#define WS2812_RESET_TICK_NUM       (0xFF / RMT_TICK_TIME)    	/*!< >50 us: bit reset low level time */

#define COLOR_MAX_HUE 				(360)
#define COLOR_MAX_SAT 				(100)
#define COLOR_MAX_VAL 				(100)

#define PERIPH_WS2812_TICK 			10

#define mutex_lock(x)       		while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x)     		xSemaphoreGive(x)
#define mutex_create()      		xSemaphoreCreateMutex()
#define mutex_destroy(x)    		vQueueDelete(x)

#define VALIDATE_WS2812(periph, ret) if(!esp_periph_validate(periph, PERIPH_ID_WS2812)) {		\
	ESP_LOGE(TAG, "Invalid PERIPH_ID_WS2812");													\
	return ret;																					\
}

#define WS2812_CHECK(a, str, action) if(!(a)) {                             \
    ESP_LOGE(TAG, "%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);  \
    action;                                                                 \
}

typedef struct anim_item {
	gpio_num_t 				pin;
	rmt_channel_t 			rmt_chnl;
	rmt_config_t 			rmt_config;
	uint32_t 				num_led;
	uint8_t 				*buf[2];
	uint32_t 				buf_len;
	uint8_t					buf_idx;
	ws2812_anim_type_t		anim_type;
	uint32_t 				hue;
	uint32_t 				sat;
	uint32_t				val;
	uint32_t				hue2;
	uint32_t 				speed;
	uint32_t 				time_lapse;
	uint32_t 				step;
	bool 					pause;
	SemaphoreHandle_t 		lock;
	STAILQ_ENTRY(anim_item) next;
} periph_ws2812_anim_t;

typedef struct periph_ws2812 {
	bool 					is_started;
	STAILQ_HEAD(anim_list, anim_item) list_anim;
} periph_ws2812_t;

static const char *TAG  = "PERIPH_WS2812";
static esp_periph_handle_t g_ws2812;

static const rmt_item32_t WS2812_BIT_0 = {{{WS2812_BIT0_HIGH_TICK_NUM, 1, WS2812_BIT0_LOW_TICK_NUM, 0}}}; /* Bit 0 */
static const rmt_item32_t WS2812_BIT_1 = {{{WS2812_BIT1_HIGH_TICK_NUM, 1, WS2812_BIT1_LOW_TICK_NUM, 0}}}; /* Bit 1 */

static void IRAM_ATTR _ws2812_adapter(const void *src,
                                      rmt_item32_t *dest,
                                      size_t src_size,
                                      size_t wanted_num,
                                      size_t *translated_size,
                                      size_t *item_num)
{
	if (src == NULL || dest == NULL) {
		*translated_size = 0;
		*item_num = 0;
		return;
	}
	size_t size = 0, num = 0;
	uint8_t *psrc = (uint8_t *)src;
	rmt_item32_t *pdest = dest;
	while (size < src_size && num < wanted_num) {
		for (int i = 0; i < 8; i++) {
			// MSB first
			if (*psrc & (1 << (7 - i))) {
				pdest->val = WS2812_BIT_1.val;
			} else {
				pdest->val = WS2812_BIT_0.val;
			}
			num++;
			pdest++;
		}
		size++;
		psrc++;
	}
	*translated_size = size;
	*item_num = num;
}

static esp_err_t IRAM_ATTR _ws2812_scan(void)
{
	periph_ws2812_t *periph_ws2812 = esp_periph_get_data(g_ws2812);

	periph_ws2812_anim_t *anim_item = NULL;
	STAILQ_FOREACH(anim_item, &periph_ws2812->list_anim, next) {
		if (!anim_item->pause) {
			mutex_lock(anim_item->lock);
			WS2812_CHECK(!rmt_write_sample(anim_item->rmt_chnl, anim_item->buf[anim_item->buf_idx], anim_item->buf_len, true),
			"rmt write sample error", {
				mutex_unlock(anim_item->lock);
				return ESP_FAIL;
			});

			mutex_unlock(anim_item->lock);
		}
	}

	return ESP_OK;
}

static void _ws2812_set_pixel(periph_ws2812_anim_t *anim_item, uint8_t buf_idx, uint32_t pixel_idx, uint32_t rgb)
{
	anim_item->buf[buf_idx][pixel_idx * 3 + 0] = (uint8_t)((rgb >>  8) & 0xFF);
	anim_item->buf[buf_idx][pixel_idx * 3 + 1] = (uint8_t)((rgb >> 16) & 0xFF);
	anim_item->buf[buf_idx][pixel_idx * 3 + 2] = (uint8_t)((rgb >>  0) & 0xFF);
}

static void _ws2812_anim_none(periph_ws2812_anim_t *anim_item)
{
	uint8_t buf_idx = (anim_item->buf_idx ^ 1);
	uint32_t num_led = anim_item->num_led;

	color_hsv_t hsv = {
		.hue = anim_item->hue,
		.sat = anim_item->sat,
		.val = anim_item->val,
	};
	uint32_t rgb;
	hsv_2_rgb(hsv, &rgb);

	for (int i = 0; i < num_led; i++) {
		_ws2812_set_pixel(anim_item, buf_idx, i, rgb);
	}
}

static void _ws2812_anim_fade(periph_ws2812_anim_t *anim_item)
{
	uint8_t buf_idx = (anim_item->buf_idx ^ 1);
	uint32_t num_led = anim_item->num_led;

    color_hsv_t hsv = {
		.hue = anim_item->hue,
		.sat = anim_item->sat,
		.val = anim_item->step % COLOR_MAX_VAL,
	};
	uint32_t rgb;
	hsv_2_rgb(hsv, &rgb);

    for (int i = 0; i < num_led; i++) {
		_ws2812_set_pixel(anim_item, buf_idx, i, rgb);
	}

    anim_item->step++;
    if(anim_item->step > COLOR_MAX_VAL) anim_item->step = 0;
}

static void _ws2812_anim_rainbow(periph_ws2812_anim_t *anim_item)
{
	uint8_t buf_idx = (anim_item->buf_idx ^ 1);
	uint32_t step = anim_item->step;
	uint32_t num_led = anim_item->num_led;

	uint32_t sat = anim_item->sat;
	uint32_t val = anim_item->val;
	uint32_t hue = (step * 5) % COLOR_MAX_HUE;
	for (int i = 0; i < num_led; i++) {
		color_hsv_t hsv = {
			.hue = hue + i * 20,
			.sat = sat,
			.val = val,
		};
		uint32_t rgb;
		hsv_2_rgb(hsv, &rgb);
		_ws2812_set_pixel(anim_item, buf_idx, i, rgb);
	}

	anim_item->step++;
}

static void _ws2812_task(void *pv)
{
	esp_periph_handle_t self = (esp_periph_handle_t)pv;
	periph_ws2812_t *periph_ws2812 = esp_periph_get_data(self);

	while (periph_ws2812->is_started) {
		periph_ws2812_anim_t *anim_item = NULL;
		STAILQ_FOREACH(anim_item, &periph_ws2812->list_anim, next) {
			if (anim_item->time_lapse != 0) {
				anim_item->time_lapse--;
			} else {
				anim_item->time_lapse = anim_item->speed / PERIPH_WS2812_TICK;
				switch (anim_item->anim_type) {
				case WS2812_ANIM_TYPE_NONE:
					_ws2812_anim_none(anim_item);
					break;
				case WS2812_ANIM_TYPE_FADE:
					_ws2812_anim_fade(anim_item);
					break;
				case WS2812_ANIM_TYPE_RAINBOW:
					_ws2812_anim_rainbow(anim_item);
					break;
				default :
					_ws2812_anim_none(anim_item);
					break;
				}
				anim_item->buf_idx ^= 1;
			}
		}

		_ws2812_scan();
		vTaskDelay(10 / portTICK_RATE_MS);
	}
	vTaskDelete(NULL);
}

static esp_err_t _ws2812_init(esp_periph_handle_t self)
{
	VALIDATE_WS2812(self, ESP_FAIL);
	periph_ws2812_t *periph_ws2812 = esp_periph_get_data(self);
	if (!periph_ws2812->is_started) {
		periph_ws2812->is_started = true;
		xTaskCreate(_ws2812_task, "ws2812_task", 4 * 1024, self, 4, NULL);
	}
	return ESP_OK;
}

static esp_err_t _ws2812_run(esp_periph_handle_t self, audio_event_iface_msg_t *msg)
{
	return ESP_OK;
}

static esp_err_t _ws2812_destroy(esp_periph_handle_t self)
{
	VALIDATE_WS2812(self, ESP_FAIL);
	periph_ws2812_t *periph_ws2812 = esp_periph_get_data(self);

	periph_ws2812_anim_t *anim_item = NULL;
	STAILQ_FOREACH(anim_item, &periph_ws2812->list_anim, next) {
		STAILQ_REMOVE(&periph_ws2812->list_anim, anim_item, anim_item, next);
		mutex_destroy(anim_item->lock);
		free(anim_item->buf[0]);
		free(anim_item->buf[1]);
		free(anim_item);
	}
	free(periph_ws2812);

	return ESP_OK;
}

esp_periph_handle_t periph_ws2812_init(periph_ws2812_cfg_t *config)
{
	esp_periph_handle_t periph = esp_periph_create(PERIPH_ID_WS2812, config->tag ? config->tag : "periph_ws2812");
	PERIPH_MEM_CHECK(TAG, periph, return NULL);
	periph_ws2812_t *periph_ws2812 = calloc(1, sizeof(periph_ws2812_t));
	PERIPH_MEM_CHECK(TAG, periph_ws2812, {free(periph); return NULL;});

	periph_ws2812->is_started = false;
	STAILQ_INIT(&periph_ws2812->list_anim);

	esp_periph_set_data(periph, periph_ws2812);
	esp_periph_set_function(periph, _ws2812_init, _ws2812_run, _ws2812_destroy);
	g_ws2812 = periph;

	return periph;
}

periph_ws2812_anim_handle_t periph_ws2812_anim_init(periph_ws2812_anim_cfg_t *config)
{
	WS2812_CHECK(g_ws2812, "error periph handle null", return NULL);
	WS2812_CHECK(config, "error config null", return NULL);

	periph_ws2812_t *periph_ws2812 = esp_periph_get_data(g_ws2812);
	periph_ws2812_anim_t *anim_item = calloc(1, sizeof(periph_ws2812_anim_t));
	WS2812_CHECK(anim_item, "error calloc anim item", return NULL);

	color_hsv_t hsv;
	rgb_2_hsv(config->color, &hsv);
	anim_item->hue = hsv.hue;
	anim_item->sat = hsv.sat;
	anim_item->val = config->brightness;
	anim_item->hue2 = 0;

	anim_item->anim_type = config->anim_type;
	anim_item->speed = config->speed;
	anim_item->step = 0;
	anim_item->time_lapse = config->speed / PERIPH_WS2812_TICK;

	anim_item->buf_len = config->num_led * 3;
	anim_item->buf_idx = 0;
	anim_item->buf[0] = (uint8_t *)calloc(config->num_led * 3, sizeof(uint8_t));
	anim_item->buf[1] = (uint8_t *)calloc(config->num_led * 3, sizeof(uint8_t));

	WS2812_CHECK(anim_item->buf[0], "error allocate anim led buffer 0", {
		free(anim_item);
		return NULL;
	});

	WS2812_CHECK(anim_item->buf[1], "error allocate anim led buffer 1", {
		free(anim_item->buf[0]);
		free(anim_item);
		return NULL;
	});

	rmt_config_t rmt_cfg = RMT_DEFAULT_CONFIG_TX(config->pin, config->rmt_chnl);
	rmt_cfg.clk_div = RMT_TIMER_DEVIDER;

	rmt_config(&rmt_cfg);
	rmt_driver_install(rmt_cfg.channel, RMT_NON_RX_BUFF_SIZE, RMT_DEFAULT_FLAG);
	rmt_translator_init(rmt_cfg.channel, _ws2812_adapter);

	anim_item->pin = config->pin;
	anim_item->rmt_chnl = config->rmt_chnl;
	anim_item->num_led = config->num_led;
	anim_item->rmt_config = rmt_cfg;

	anim_item->pause = false;
	anim_item->lock = mutex_create();

	STAILQ_INSERT_TAIL(&periph_ws2812->list_anim, anim_item, next);

	ESP_LOGI(TAG, "insert anim {pin: %d, num_led: %d, rmt_chnl: %d}",
	         config->pin,
	         config->num_led,
	         config->rmt_chnl);

	return anim_item;
}

esp_err_t periph_ws2812_set_anim_type(periph_ws2812_anim_handle_t anim_item, ws2812_anim_type_t anim_type)
{
	WS2812_CHECK(anim_item, "error anim item null", return ESP_ERR_INVALID_ARG);

	mutex_lock(anim_item->lock);
	anim_item->anim_type = anim_type;
	mutex_unlock(anim_item->lock);

	return ESP_OK;
}

esp_err_t periph_ws2812_set_speed(periph_ws2812_anim_handle_t anim_item, uint32_t speed)
{
	WS2812_CHECK(anim_item, "error anim item null", return ESP_ERR_INVALID_ARG);

	mutex_lock(anim_item->lock);
	anim_item->speed = speed;
	mutex_unlock(anim_item->lock);

	return ESP_OK;
}

esp_err_t periph_ws2812_set_brightness(periph_ws2812_anim_handle_t anim_item, uint8_t brightness)
{
	WS2812_CHECK(anim_item, "error anim item null", return ESP_ERR_INVALID_ARG);

	mutex_lock(anim_item->lock);
	anim_item->val = brightness;
	mutex_unlock(anim_item->lock);

	return ESP_OK;
}

esp_err_t periph_ws2812_set_color(periph_ws2812_anim_handle_t anim_item, uint32_t rgb)
{
	WS2812_CHECK(anim_item, "error anim item null", return ESP_ERR_INVALID_ARG);

	mutex_lock(anim_item->lock);
	color_hsv_t hsv;
	rgb_2_hsv(rgb, &hsv);
	anim_item->hue = hsv.hue;
	anim_item->sat = hsv.sat;
	mutex_unlock(anim_item->lock);

	return ESP_OK;
}

esp_err_t periph_ws2812_anim_pause(periph_ws2812_anim_handle_t anim_item)
{
	WS2812_CHECK(anim_item, "error anim item null", return ESP_ERR_INVALID_ARG);

	mutex_lock(anim_item->lock);
	anim_item->pause = true;
	mutex_unlock(anim_item->lock);

	return ESP_OK;
}

esp_err_t periph_ws2812_anim_run(periph_ws2812_anim_handle_t anim_item)
{
	WS2812_CHECK(anim_item, "error anim item null", return ESP_ERR_INVALID_ARG);

	mutex_lock(anim_item->lock);
	anim_item->pause = false;
	mutex_unlock(anim_item->lock);

	return ESP_OK;
}