/* Bus stress test, receiving half. Run throughput_tx.c on a second board.
 *
 * Counts what arrives for RUN_SECONDS, then prints the delivered bandwidth
 * and how much went missing.
 *
 * This node also does the sender a favour by existing: a CAN transmitter
 * needs someone else to ACK every frame, so without a second node on the bus
 * the sender goes bus-off and measures nothing. Listening is enough -- the
 * ACK is generated in hardware, whether or not this code reads the frame.
 *
 * Two counters, and the difference between them is the whole point:
 *
 *   lost         a gap in the sequence number. The frame never reached this
 *                node at all -- the wire dropped it, or the sender never got
 *                it out.
 *   rx_miss_cnt  the driver had the frame and threw it away because the
 *                receive queue was full. That one is this node being too
 *                slow, and RX_QUEUE or the work per frame is the fix.
 *
 * Both boards must agree on the bitrate, and the bus wants exactly two
 * 120 ohm terminators, one at each end.
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

#define TX_QUEUE 8      /* this node barely sends */
#define RX_QUEUE 64

#define RUN_SECONDS     30
#define REPORT_EVERY_US 1000000

/* One standard frame with eight data bytes, before and after worst-case
 * stuffing. See throughput_tx.c for where the numbers come from.
 */
#define BITS_MIN 111
#define BITS_MAX 135

static const char *TAG = "tp-rx";

static uint32_t get_seq(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) | (uint32_t)data[3];
}

static uint32_t queue_drops(hvd230_t *can)
{
    can_health_t h;

    return hvd230_health(can, &h) == CAN_OK ? h.rx_miss_cnt : 0;
}

static void log_health(hvd230_t *can)
{
    can_health_t h;

    if (hvd230_health(can, &h) != CAN_OK)
        return;

    ESP_LOGI(TAG, "  tec=%" PRIu32 " rec=%" PRIu32 " bus_err=%" PRIu32
                  " rx_miss=%" PRIu32 " rx_pend=%" PRIu32,
             h.tx_err_cnt, h.rx_err_cnt, h.bus_err_cnt,
             h.rx_miss_cnt, h.rx_pending);

    if (h.rx_err_cnt > 96)
        ESP_LOGW(TAG, "  receive errors high -- check termination, wiring, "
                      "and that both ends agree on the bitrate");
}

/** What actually got delivered.
 *
 *  goodput is exact: every frame carried eight bytes, so it is frames a
 *  second times 64 bits. Wire usage can only be bounded, because bit
 *  stuffing depends on the payload -- the sending side measures the real
 *  figure, since it knows when the bus was saturated.
 */
static void summarise(hvd230_t *can, uint64_t frames, uint64_t us,
                      uint64_t lost)
{
    uint32_t fps, goodput;
    uint64_t offered;

    /* us divides everything below. A frozen clock would make that an integer
     * division by zero, which is undefined behaviour, so refuse first.
     */
    if (us == 0) {
        ESP_LOGE(TAG, "no elapsed time to measure over -- is esp_timer "
                      "running?");
        return;
    }

    fps = (uint32_t)(frames * 1000000u / us);
    goodput = fps * 8u * 8u;
    offered = frames + lost;

    ESP_LOGI(TAG, "----- %" PRIu32 " s, %" PRIu32 " frames -----",
             (uint32_t)(us / 1000000u), (uint32_t)frames);

    if (fps == 0) {
        ESP_LOGE(TAG, "nothing arrived. Is throughput_tx running, and are "
                      "both ends on the same bitrate?");
        return;
    }

    ESP_LOGI(TAG, "rate      %" PRIu32 " frames/s", fps);
    ESP_LOGI(TAG, "goodput   %" PRIu32 " kbit/s of payload delivered",
             goodput / 1000u);
    ESP_LOGI(TAG, "bandwidth %" PRIu32 "-%" PRIu32 "%% of the %" PRIu32
                  " kbit/s channel used",
             fps * BITS_MIN * 100u / (uint32_t)CAN_BITRATE,
             fps * BITS_MAX * 100u / (uint32_t)CAN_BITRATE,
             (uint32_t)(CAN_BITRATE / 1000u));

    if (offered > 0)
        ESP_LOGI(TAG, "loss      %" PRIu32 " of %" PRIu32 " frames",
                 (uint32_t)lost, (uint32_t)offered);

    if (lost == 0) {
        ESP_LOGI(TAG, "clean run, nothing missing");
        return;
    }

    /* Which of the two losses it was decides what to change */
    if (queue_drops(can) > 0)
        ESP_LOGW(TAG, "rx_miss_cnt is non-zero: this node could not keep up. "
                      "Raise RX_QUEUE or do less per frame");
    else
        ESP_LOGW(TAG, "rx_miss_cnt is clean, so the frames never arrived. "
                      "Look at the wire: termination, stub length, ground");
}

void app_main(void)
{
    can_frame_t frame;
    uint64_t total = 0, lost = 0;
    uint32_t window = 0, restarts = 0;
    uint32_t next_seq = 0;
    bool seq_known = false;
    int64_t started, window_start;
    hvd230_t can;

    hvd230_init(&can);

    /* Take only the stress identifier, in hardware, so frames from anything
     * else sharing the bus never cost this node an interrupt.
     */
    hvd230_set_filter(&can, STRESS_ID, 0x7FF, false);

    if (hvd230_begin_ex(&can, CAN_TX_PIN, CAN_RX_PIN, CAN_BITRATE,
                        CAN_NO_PIN, TX_QUEUE, RX_QUEUE) != CAN_OK) {
        ESP_LOGE(TAG, "CAN startup failed");
        return;
    }

    hvd230_set_auto_recover(&can, 100);

    ESP_LOGI(TAG, "listening on id 0x%03X for %d s at %" PRIu32 " bit/s",
             STRESS_ID, RUN_SECONDS, (uint32_t)CAN_BITRATE);

    started = window_start = esp_timer_get_time();

    while (esp_timer_get_time() - started < (int64_t)RUN_SECONDS * 1000000) {
        int64_t now;
        /* A short blocking wait, so this task yields between bursts rather
         * than spinning on an empty queue.
         */
        const can_sta res = hvd230_rcv(&can, &frame, 10);

        if (res == CAN_OK && frame.len == CAN_MAX_DLC) {
            const uint32_t seq = get_seq(frame.data);

            window++;
            total++;

            if (!seq_known) {
                seq_known = true;
            } else if ((int32_t)(seq - next_seq) > 0) {
                lost += seq - next_seq;   /* frames that never arrived */
            } else if (seq != next_seq) {
                restarts++;               /* the sender started over */
            }

            next_seq = seq + 1;
        }

        hvd230_service(&can);

        now = esp_timer_get_time();
        if (now - window_start < REPORT_EVERY_US)
            continue;

        ESP_LOGI(TAG, "%" PRIu32 " frames/s, lost %" PRIu32 " so far",
                 (uint32_t)((uint64_t)window * 1000000u /
                            (uint64_t)(now - window_start)),
                 (uint32_t)lost);
        log_health(&can);

        window = 0;
        window_start = now;
    }

    summarise(&can, total, (uint64_t)(esp_timer_get_time() - started), lost);
    log_health(&can);

    if (restarts > 0)
        ESP_LOGI(TAG, "sender restarted %" PRIu32 " time(s)", restarts);

    hvd230_end(&can);
}
