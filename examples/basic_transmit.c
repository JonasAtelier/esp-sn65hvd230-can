/* API demo: queue one standard CAN frame every second.
 *
 * Not a buildable ESP-IDF example project, just the call sequence.
 */

#include <inttypes.h>
#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hvd230can.h"

#define CAN_TX_PIN     5
#define CAN_RX_PIN     4
#define CAN_BITRATE    CAN_BITRATE_500K
#define CAN_IDENTIFIER 0x123

static const char *TAG = "basic_transmit";

void app_main(void)
{
    static const uint8_t data[] = {0x11, 0x22, 0x33};
    hvd230_t can;

    hvd230_init(&can);   /* required first: a zeroed struct is not a valid one */

    if (hvd230_begin(&can, CAN_TX_PIN, CAN_RX_PIN, CAN_BITRATE) != CAN_OK) {
        ESP_LOGE(TAG, "CAN startup failed");
        return;
    }

    while (true) {
        /* No default arguments in C: `false, 0` is a standard frame that does
         * not block if the transmit queue is full.
         */
        const can_sta result = hvd230_tx(&can, CAN_IDENTIFIER, data,
                                         sizeof(data), false, 0);

        if (result == CAN_OK)
            ESP_LOGI(TAG, "Queued CAN frame 0x%03X", CAN_IDENTIFIER);
        else
            ESP_LOGE(TAG, "CAN send failed: %u", (unsigned)result);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* Unreachable here, but a task that does return owes the controller a
     * hvd230_end(&can): C has no destructor to call it for you.
     */
}
