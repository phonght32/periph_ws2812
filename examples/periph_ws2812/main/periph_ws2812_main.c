#include <stdio.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "periph_ws2812.h"

esp_periph_handle_t ws2812_handle = NULL;
periph_ws2812_anim_handle_t anim_handle = NULL;

static esp_err_t periph_event_handle(audio_event_iface_msg_t *event, void *context)
{
    return ESP_OK;
}

void app_main(void)
{
    esp_periph_config_t config = {
        .event_handle = periph_event_handle,
        .max_parallel_connections = 9,
    };
    esp_periph_init(&config);

    periph_ws2812_cfg_t periph_ws2812_cfg = {
        .tag = NULL,
    };
    ws2812_handle = periph_ws2812_init(&periph_ws2812_cfg);

    periph_ws2812_anim_cfg_t anim_cfg = {
        .pin = 32,
        .rmt_chnl = RMT_CHANNEL_0,
        .num_led = 20,
        .speed = 10,
        .color = 0x00FFFF,
        .brightness = 10,
        .anim_type = WS2812_ANIM_TYPE_RAINBOW,
    };
    anim_handle = periph_ws2812_anim_init(&anim_cfg);
    esp_periph_start(ws2812_handle);

    while (1)
    {
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}
