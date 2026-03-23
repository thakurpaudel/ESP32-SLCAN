#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

#define TAG "SLCAN"

#define CAN_TX_GPIO 14
#define CAN_RX_GPIO 13

#define CDC_RX_BUF_SIZE 256

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static volatile bool slcan_opened = false;
static volatile bool slcan_listen_only = false;
static volatile bool slcan_timestamp = false;
static bool usb_ready = false;
static volatile bool twai_stop_req = false; // signal RX task to stop

static twai_timing_config_t current_timing;

static SemaphoreHandle_t twai_mutex = NULL;       // protects open/close
static SemaphoreHandle_t twai_stopped_sem = NULL; // RX task signals it has exited twai_receive

static TaskHandle_t can_rx_task_handle = NULL;

static uint8_t cdc_rx_buf[CDC_RX_BUF_SIZE];
static char cmd_buffer[128];
static int cmd_index = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static uint8_t hex_to_byte(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return 0;
}

static char byte_to_hex(uint8_t b)
{
    return b < 10 ? '0' + b : 'A' + b - 10;
}

static bool get_timing_config(char code, twai_timing_config_t *config)
{
    switch (code)
    {
    case '0':
        *config = (twai_timing_config_t){.clk_src = TWAI_CLK_SRC_DEFAULT,
                                         .quanta_resolution_hz = 1000000,
                                         .brp = 0,
                                         .tseg_1 = 63,
                                         .tseg_2 = 16,
                                         .sjw = 16,
                                         .triple_sampling = false};
        break;
    case '1':
        *config = (twai_timing_config_t){.clk_src = TWAI_CLK_SRC_DEFAULT,
                                         .quanta_resolution_hz = 2000000,
                                         .brp = 0,
                                         .tseg_1 = 63,
                                         .tseg_2 = 16,
                                         .sjw = 16,
                                         .triple_sampling = false};
        break;
    case '2':
        *config = (twai_timing_config_t)TWAI_TIMING_CONFIG_50KBITS();
        break;
    case '3':
        *config = (twai_timing_config_t)TWAI_TIMING_CONFIG_100KBITS();
        break;
    case '4':
        *config = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
        break;
    case '5':
        *config = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
        break;
    case '6':
        *config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
        break;
    case '7':
        *config = (twai_timing_config_t)TWAI_TIMING_CONFIG_800KBITS();
        break;
    case '8':
        *config = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
        break;
    default:
        ESP_LOGW(TAG, "Invalid bitrate code: %c", code);
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// USB CDC send — no usb_ready gate, TinyUSB handles backpressure
// ---------------------------------------------------------------------------

static void send_response(const char *resp)
{
    if (!usb_ready)
    {
        ESP_LOGE(TAG, "USB not ready");
        return;
    }

    size_t len = strlen(resp);
    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (uint8_t *)resp, len);
    tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
}

// ---------------------------------------------------------------------------
// CAN open / close helpers  (caller must hold twai_mutex)
// ---------------------------------------------------------------------------

static bool can_open_locked(twai_mode_t mode)
{
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, mode);
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    g_config.tx_queue_len = 10;
    g_config.rx_queue_len = 20;

    if (twai_driver_install(&g_config, &current_timing, &f_config) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install TWAI driver");
        return false;
    }
    if (twai_start() != ESP_OK)
    {
        twai_driver_uninstall();
        ESP_LOGE(TAG, "Failed to start TWAI");
        return false;
    }
    return true;
}

// Safe close: signal the RX task out of twai_receive FIRST,
// wait for it to acknowledge, THEN tear down the driver.
static void can_close_safe(void)
{
    // 1. Signal the RX task to stop calling twai_receive
    twai_stop_req = true;

    // 2. Wait for RX task to confirm it is no longer inside twai_receive
    //    (it posts twai_stopped_sem after each twai_receive returns)
    xSemaphoreTake(twai_stopped_sem, pdMS_TO_TICKS(500));

    // 3. Now it is safe to stop and uninstall the driver
    xSemaphoreTake(twai_mutex, portMAX_DELAY);
    twai_stop();
    twai_driver_uninstall();
    slcan_opened = false;
    slcan_listen_only = false;
    twai_stop_req = false;
    xSemaphoreGive(twai_mutex);
}

// ---------------------------------------------------------------------------
// SLCAN command processor
// ---------------------------------------------------------------------------

static void process_slcan_command(const char *cmd, size_t len)
{
    if (len < 1)
        return;
    ESP_LOGI(TAG, "Command: %c (len=%d)", cmd[0], (int)len);

    switch (cmd[0])
    {

    // -- Bitrate -----------------------------------------------------------
    case 'S':
        if (len >= 2 && !slcan_opened)
        {
            if (get_timing_config(cmd[1], &current_timing))
            {
                ESP_LOGI(TAG, "Bitrate set to code %c", cmd[1]);
                send_response("\r");
            }
            else
            {
                send_response("\x07");
            }
        }
        else
        {
            ESP_LOGW(TAG, "Cannot set bitrate (open=%d len=%d)", slcan_opened, (int)len);
            send_response("\x07");
        }
        break;

    // -- Open normal -------------------------------------------------------
    case 'O':
        xSemaphoreTake(twai_mutex, portMAX_DELAY);
        if (slcan_opened)
        {
            xSemaphoreGive(twai_mutex);
            send_response("\x07");
            ESP_LOGW(TAG, "CAN already opened");
        }
        else if (can_open_locked(TWAI_MODE_NORMAL))
        {
            slcan_opened = true;
            slcan_listen_only = false;
            xSemaphoreGive(twai_mutex);
            send_response("\r");
            ESP_LOGI(TAG, "CAN opened (normal mode)");
        }
        else
        {
            xSemaphoreGive(twai_mutex);
            send_response("\x07");
        }
        break;

    // -- Open listen-only --------------------------------------------------
    case 'L':
        xSemaphoreTake(twai_mutex, portMAX_DELAY);
        if (slcan_opened)
        {
            xSemaphoreGive(twai_mutex);
            send_response("\x07");
            ESP_LOGW(TAG, "CAN already opened");
        }
        else if (can_open_locked(TWAI_MODE_LISTEN_ONLY))
        {
            slcan_opened = true;
            slcan_listen_only = true;
            xSemaphoreGive(twai_mutex);
            send_response("\r");
            ESP_LOGI(TAG, "CAN opened (listen-only)");
        }
        else
        {
            xSemaphoreGive(twai_mutex);
            send_response("\x07");
        }
        break;

    // -- Close -------------------------------------------------------------
    case 'C':
        if (slcan_opened)
        {
            can_close_safe(); // signals RX task, waits, then tears down
            ESP_LOGI(TAG, "CAN closed");
        }
        else
        {
            ESP_LOGD(TAG, "CAN already closed");
        }
        send_response("\r"); // always ACK
        break;

    // -- Transmit frames ---------------------------------------------------
    case 't':
    case 'T':
    case 'r':
    case 'R':
    {
        if (!slcan_opened || slcan_listen_only)
        {
            ESP_LOGW(TAG, "TX: CAN not open or listen-only");
            send_response("\x07");
            break;
        }

        twai_message_t msg = {0};
        int idx = 1;
        int id_len = (cmd[0] == 't' || cmd[0] == 'r') ? 3 : 8;

        if ((int)len < idx + id_len + 1)
        {
            send_response("\x07");
            break;
        }

        uint32_t id = 0;
        for (int i = 0; i < id_len; i++)
            id = (id << 4) | hex_to_byte(cmd[idx++]);

        msg.identifier = id;
        msg.extd = (cmd[0] == 'T' || cmd[0] == 'R') ? 1 : 0;
        msg.rtr = (cmd[0] == 'r' || cmd[0] == 'R') ? 1 : 0;

        if (idx >= (int)len)
        {
            send_response("\x07");
            break;
        }
        msg.data_length_code = hex_to_byte(cmd[idx++]);
        if (msg.data_length_code > 8)
        {
            send_response("\x07");
            break;
        }

        if (!msg.rtr)
        {
            if ((int)len < idx + (int)msg.data_length_code * 2)
            {
                send_response("\x07");
                break;
            }
            for (int i = 0; i < msg.data_length_code; i++)
            {
                msg.data[i] = (hex_to_byte(cmd[idx]) << 4) | hex_to_byte(cmd[idx + 1]);
                idx += 2;
            }
        }

        if (twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK)
        {
            send_response(msg.extd ? "Z\r" : "z\r");
        }
        else
        {
            send_response("\x07");
            ESP_LOGW(TAG, "TX failed");
        }
        break;
    }

    // -- Misc --------------------------------------------------------------
    case 'Z':
        if (len >= 2)
        {
            slcan_timestamp = (cmd[1] == '1');
            send_response("\r");
            ESP_LOGI(TAG, "Timestamps %s", slcan_timestamp ? "enabled" : "disabled");
        }
        else
        {
            send_response("\x07");
        }
        break;

    case 'V':
        send_response("V1234\r");
        break;
    case 'v':
        send_response("v0100\r");
        break;
    case 'N':
        send_response("NESP32\r");
        break;
    case 'F':
        send_response("F00\r");
        break;
    case 'W': /* fall-through */
    case 'M': /* fall-through */
    case 'm':
        send_response("\r");
        break;

    default:
        ESP_LOGW(TAG, "Unknown command: %c (0x%02X)", cmd[0], (uint8_t)cmd[0]);
        send_response("\x07");
        break;
    }
}

// ---------------------------------------------------------------------------
// CAN RX task
// ---------------------------------------------------------------------------

static void can_rx_task(void *arg)
{
    twai_message_t msg;
    char buf[32];

    ESP_LOGI(TAG, "CAN RX task started");

    while (1)
    {
        if (!slcan_opened)
        {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // Short timeout so we can respond quickly to twai_stop_req
        esp_err_t ret = twai_receive(&msg, pdMS_TO_TICKS(50));

        // If a close was requested, signal that we are out of twai_receive
        if (twai_stop_req)
        {
            xSemaphoreGive(twai_stopped_sem);
            // Wait here until close is complete (slcan_opened goes false)
            while (twai_stop_req)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            continue;
        }

        if (ret != ESP_OK)
            continue; // timeout or error, loop

        // Format SLCAN frame
        int idx = 0;

        if (msg.extd)
            buf[idx++] = msg.rtr ? 'R' : 'T';
        else
            buf[idx++] = msg.rtr ? 'r' : 't';

        int id_len = msg.extd ? 8 : 3;
        for (int i = id_len - 1; i >= 0; i--)
            buf[idx++] = byte_to_hex((msg.identifier >> (i * 4)) & 0xF);

        buf[idx++] = byte_to_hex(msg.data_length_code);

        if (!msg.rtr)
        {
            for (int i = 0; i < msg.data_length_code; i++)
            {
                buf[idx++] = byte_to_hex((msg.data[i] >> 4) & 0xF);
                buf[idx++] = byte_to_hex(msg.data[i] & 0xF);
            }
        }

        // Timestamp: 4 hex nibbles, milliseconds wrapping at 0xFFFF (SLCAN spec)
        if (slcan_timestamp)
        {
            uint32_t ts = (xTaskGetTickCount() * portTICK_PERIOD_MS) & 0xFFFF;
            buf[idx++] = byte_to_hex((ts >> 12) & 0xF);
            buf[idx++] = byte_to_hex((ts >> 8) & 0xF);
            buf[idx++] = byte_to_hex((ts >> 4) & 0xF);
            buf[idx++] = byte_to_hex(ts & 0xF);
        }

        buf[idx++] = '\r';

        tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (uint8_t *)buf, idx);
        tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
    }
}

// ---------------------------------------------------------------------------
// USB CDC callbacks
// ---------------------------------------------------------------------------

void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    size_t rx_size = 0;
    if (tinyusb_cdcacm_read(itf, cdc_rx_buf, CDC_RX_BUF_SIZE, &rx_size) != ESP_OK)
        return;

    for (size_t i = 0; i < rx_size; i++)
    {
        char c = cdc_rx_buf[i];
        if (c == '\r' || c == '\n')
        {
            if (cmd_index > 0)
            {
                cmd_buffer[cmd_index] = '\0';
                process_slcan_command(cmd_buffer, cmd_index);
                cmd_index = 0;
            }
        }
        else if (cmd_index < (int)sizeof(cmd_buffer) - 1)
        {
            cmd_buffer[cmd_index++] = c;
        }
        else
        {
            cmd_index = 0;
            send_response("\x07");
            ESP_LOGW(TAG, "Command buffer overflow");
        }
    }
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    usb_ready = event->line_state_changed_data.dtr;

    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed: DTR=%d RTS=%d", dtr, rts);
}

// ---------------------------------------------------------------------------
// app_main
// ---------------------------------------------------------------------------

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32 SLCAN Adapter ===");
    ESP_LOGI(TAG, "CAN TX: GPIO%d, RX: GPIO%d", CAN_TX_GPIO, CAN_RX_GPIO);

    // Default timing: 250 kbit/s (overridden by 'S' command before 'O')
    current_timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();

    // Synchronisation primitives
    twai_mutex = xSemaphoreCreateMutex();
    twai_stopped_sem = xSemaphoreCreateBinary();
    configASSERT(twai_mutex);
    configASSERT(twai_stopped_sem);

    // USB CDC
    ESP_LOGI(TAG, "Initializing USB CDC...");
    tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 256,
        .callback_rx = &tinyusb_cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = &tinyusb_cdc_line_state_changed_callback,
        .callback_line_coding_changed = NULL,
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    ESP_LOGI(TAG, "USB CDC configured");

    // CAN RX task — created ONCE
    xTaskCreate(can_rx_task, "can_rx", 4096, NULL, 6, &can_rx_task_handle);
    ESP_LOGI(TAG, "CAN RX task started");

    ESP_LOGI(TAG, "SLCAN ready!");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "SLCAN ESP32 Ready");
    ESP_LOGI(TAG, "Use 'candump' or 'cansend' tools");
}