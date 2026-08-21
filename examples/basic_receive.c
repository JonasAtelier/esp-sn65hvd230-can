/* API demo: poll the bus and log every frame received.
 *
 * Not a buildable ESP-IDF example project, just the call sequence.
 */

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hvd230can.h"

#define CAN_TX_PIN  5
#define CAN_RX_PIN  4
#define CAN_BITRATE CAN_BITRATE_500K

static const char *TAG = "basic_receive";

static void log_frame(const can_frame_t *frame)
{
    char payload[3 * 8 + 1] = {0};
    size_t offset = 0;
    uint8_t index;

    for (index = 0; index < frame->len; index++) {
        offset += snprintf(payload + offset, sizeof(payload) - offset,
                           "%s%02" PRIX8, index == 0 ? "" : " ",
                           frame->data[index]);
    }

    ESP_LOGI(TAG, "%s ID 0x%" PRIX32 " data: %s",
             frame->extended ? "Extended" : "Standard", frame->id, payload);
}

void app_main(void)
{
    hvd230_t can;

    hvd230_init(&can);

    if (hvd230_begin(&can, CAN_TX_PIN, CAN_RX_PIN, CAN_BITRATE) != CAN_OK) {
        ESP_LOGE(TAG, "CAN startup failed");
        return;
    }

    while (true) {
        can_frame_t frame;
        const can_sta result = hvd230_rcv(&can, &frame, 100);

        /* CAN_ETIMEOUT just means nothing was waiting */
        if (result == CAN_ETIMEOUT)
            continue;

        if (result != CAN_OK) {
            ESP_LOGE(TAG, "CAN receive failed: %u", (unsigned)result);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        log_frame(&frame);
    }
}
