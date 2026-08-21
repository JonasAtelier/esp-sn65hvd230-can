/* Bus stress test, sending half. Run throughput_rx.c on a second board.
 *
 * Floods the bus for RUN_SECONDS, then prints what the channel carried.
 *
 * Four things that decide whether the result means anything:
 *
 *   1. YOU NEED THE SECOND BOARD. A transmitter needs someone to ACK, or it
 *      goes bus-off and measures zero. A silent listener is enough.
 *   2. TWO 120 ohm terminators, one at each end. Wrong termination shows up
 *      under load, on a bus that looked fine idle.
 *   3. 8 data bytes cost 111 bits on the wire, not 64. So 1 Mbps is ~9k
 *      frames/s at best.
 *   4. Both boards must agree on the bitrate.
 *
 * Not a buildable ESP-IDF example project, just the call sequence.
 */

#include <inttypes.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hvd230can.h"

#define CAN_TX_PIN  5
#define CAN_RX_PIN  4
#define CAN_BITRATE CAN_BITRATE_1M

#define STRESS_ID 0x100

/* Deep queues: the point is to find the bus's limit, not the queue's. */
#define TX_QUEUE 32
#define RX_QUEUE 8      /* this node barely receives */

#define RUN_SECONDS     30
#define REPORT_EVERY_US 1000000

/* One 8-byte standard frame before stuffing:
 * SOF 1 + arb 12 + control 6 + data 64 + CRC 16 + ACK 2 + EOF 7 + IFS 3.
 */
#define BITS_MIN 111

/* With worst-case stuffing: 98 stuffable bits add at most (98-1)/4 = 24.
 * Real payloads land between, so the summary measures rather than assumes.
 */
#define BITS_MAX 135

static const char *TAG = "tp-tx";

static void put_seq(uint8_t *data, uint32_t seq)
{
    data[0] = (uint8_t)(seq >> 24);
    data[1] = (uint8_t)(seq >> 16);
    data[2] = (uint8_t)(seq >> 8);
    data[3] = (uint8_t)seq;
}

static void log_health(hvd230_t *can)
{
    can_health_t h;

    if (hvd230_health(can, &h) != CAN_OK)
        return;

    ESP_LOGI(TAG, "  tec=%" PRIu32 " rec=%" PRIu32 " bus_err=%" PRIu32
                  " arb_lost=%" PRIu32 " tx_fail=%" PRIu32,
             h.tx_err_cnt, h.rx_err_cnt, h.bus_err_cnt,
             h.arb_lost_cnt, h.tx_fail_cnt);

    /* Errors climbing while frames move: alive but degrading, so wiring. */
    if (h.tx_err_cnt > 96 || h.rx_err_cnt > 96)
        ESP_LOGW(TAG, "  error counters high -- check termination, wiring, "
                      "and that both ends agree on the bitrate");

    if (h.state == CAN_STATE_BUS_OFF)
        ESP_LOGE(TAG, "  BUS-OFF. Usually means nothing else is on the bus "
                      "to ACK -- is throughput_rx running?");
}

/* What the channel carried.
 *
 * Once the queue has gone full even once, the bus was busy essentially all
 * the time -- and then bits/frame = bitrate / fps is a measurement of the
 * real stuffed length, not an estimate. Outside BITS_MIN..BITS_MAX means
 * something else is on the bus.
 */
static void summarise(uint64_t frames, uint64_t us, uint32_t saturated)
{
    uint32_t fps, goodput;

    /* Integer division by zero is UB, so refuse before the first divide. */
    if (us == 0) {
        ESP_LOGE(TAG, "no elapsed time to measure over -- is esp_timer "
                      "running?");
        return;
    }

    fps = (uint32_t)(frames * 1000000u / us);
    goodput = fps * 8u * 8u;                  /* payload bits a second */

    ESP_LOGI(TAG, "----- %" PRIu32 " s, %" PRIu32 " frames -----",
             (uint32_t)(us / 1000000u), (uint32_t)frames);
    ESP_LOGI(TAG, "rate      %" PRIu32 " frames/s", fps);
    ESP_LOGI(TAG, "goodput   %" PRIu32 " kbit/s of payload", goodput / 1000u);

    if (fps == 0) {
        ESP_LOGE(TAG, "nothing got out. Check the second board, the "
                      "termination, and the bitrate");
        return;
    }

    if (saturated > 0) {
        const uint32_t bits = (uint32_t)(CAN_BITRATE / fps);

        ESP_LOGI(TAG, "frame     %" PRIu32 " bits on the wire, measured "
                      "(%d..%d expected)", bits, BITS_MIN, BITS_MAX);
        ESP_LOGI(TAG, "bandwidth %" PRIu32 " kbit/s, the whole channel",
                 (uint32_t)(CAN_BITRATE / 1000u));
        ESP_LOGI(TAG, "efficiency %" PRIu32 "%% of the wire is payload",
                 goodput * 100u / (uint32_t)CAN_BITRATE);

        if (bits < BITS_MIN || bits > BITS_MAX)
            ESP_LOGW(TAG, "  measured frame length is outside the possible "
                          "range -- something else is sharing this bus");
    } else {
        /* Queue never filled: the CPU was the bottleneck, not the bus. */
        const uint32_t lo = fps * BITS_MIN;
        const uint32_t hi = fps * BITS_MAX;

        ESP_LOGW(TAG, "queue never filled -- this is a CPU limit, not the "
                      "bus. Channel was only %" PRIu32 "-%" PRIu32
                      "%% busy", lo * 100u / (uint32_t)CAN_BITRATE,
                 hi * 100u / (uint32_t)CAN_BITRATE);
    }
}

void app_main(void)
{
    uint8_t data[CAN_MAX_DLC] = {0};
    uint32_t seq = 0;
    uint64_t total = 0;
    uint32_t window = 0, saturated = 0, failed = 0;
    int64_t started, window_start;
    hvd230_t can;

    hvd230_init(&can);

    if (hvd230_begin_ex(&can, CAN_TX_PIN, CAN_RX_PIN, CAN_BITRATE,
                        CAN_NO_PIN, TX_QUEUE, RX_QUEUE) != CAN_OK) {
        ESP_LOGE(TAG, "CAN startup failed");
        return;
    }

    /* A stress test is where bus-off happens. */
    hvd230_set_auto_recover(&can, 100);

    ESP_LOGI(TAG, "flooding id 0x%03X for %d s at %" PRIu32 " bit/s",
             STRESS_ID, RUN_SECONDS, (uint32_t)CAN_BITRATE);
    ESP_LOGI(TAG, "ceiling %" PRIu32 "-%" PRIu32 " frames/s",
             (uint32_t)(CAN_BITRATE / BITS_MAX),
             (uint32_t)(CAN_BITRATE / BITS_MIN));

    started = window_start = esp_timer_get_time();

    while (esp_timer_get_time() - started < (int64_t)RUN_SECONDS * 1000000) {
        int64_t now;
        can_sta res;

        put_seq(data, seq);

        /* timeout 0 on purpose: CAN_EQFULL is the measurement -- it means
         * we outran the wire, which is what the summary needs.
         */
        res = hvd230_tx(&can, STRESS_ID, data, sizeof(data), false, 0);

        if (res == CAN_OK) {
            window++;
            total++;
            seq++;
        } else if (res == CAN_EQFULL) {
            saturated++;

            /* Let the driver drain, and feed the watchdog. */
            vTaskDelay(1);
        } else {
            failed++;
            vTaskDelay(1);
        }

        /* Steps bus-off recovery along; does nothing while the bus is fine */
        hvd230_service(&can);

        now = esp_timer_get_time();
        if (now - window_start < REPORT_EVERY_US)
            continue;

        ESP_LOGI(TAG, "%" PRIu32 " frames/s, queue-full %" PRIu32
                      ", failed %" PRIu32,
                 (uint32_t)((uint64_t)window * 1000000u /
                            (uint64_t)(now - window_start)),
                 saturated, failed);
        log_health(&can);

        window = failed = 0;
        window_start = now;
    }

    summarise(total, (uint64_t)(esp_timer_get_time() - started), saturated);
    log_health(&can);

    hvd230_end(&can);
}
