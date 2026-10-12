/* Source file: implements UART-based AT-command communication with an external LoRaWAN
 * module, used to configure it (join mode, keys, data rate, class, etc.) and to send
 * uplink payloads over the LoRaWAN network. See lorawan.h for the related definitions. */
#include <string.h>
#include <esp_task_wdt.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "lorawan.h"

/* Configurations for UART communication with the LoRaWAN module. RTS/CTS are not used
 * (hardware flow control disabled), so they are set to UART_PIN_NO_CHANGE */
#define LORAWAN_TEST_RTS       (UART_PIN_NO_CHANGE)
#define LORAWAN_TEST_CTS       (UART_PIN_NO_CHANGE)
#define LORAWAN_UART_BAUD_RATE 9600
#define LORAWAN_UART_PORT_NUM  1

/* UART RX buffer size (in bytes) */
#define BUF_SIZE (512)

/* Size of the UART driver event queue, filled by the UART RX interrupt handler */
#define UART_EVENT_QUEUE_SIZE  20

/* Maximum size (in bytes, including the null terminator) of a single response line
 * received from the LoRaWAN module. Longer lines are truncated */
#define AT_RESPONSE_LINE_MAX_SIZE  128

/* Number of complete response lines that can wait to be consumed by send_command_over_UART() */
#define AT_RESPONSE_QUEUE_SIZE     4

/* Set to 1 to log (as a hex dump) every chunk of raw bytes received from the LoRaWAN module.
 * Useful to debug the module's responses (terminators, spurious bytes, etc.) */
#define LORAWAN_RX_RAW_DUMP        0

/* UART RX task settings */
#define UART_RX_TASK_STACK_SIZE    3072
#define UART_RX_TASK_PRIORITY      10

/* Buffer used to hold the LoRaWAN module's response received over UART
 * ("buffer_recepcao" is Portuguese for "reception buffer") */
static char buffer_recepcao[AT_RESPONSE_LINE_MAX_SIZE] = {0};

/* UART driver event queue: the UART interrupt handler posts events here (e.g. UART_DATA
 * when bytes are received), waking up the UART RX task */
static QueueHandle_t uart_event_queue = NULL;

/* Queue of complete response lines (already stripped of the '\r' / '\n' terminator), filled by
 * the UART RX task and consumed by send_command_over_UART() */
static QueueHandle_t at_response_queue = NULL;

/* Debug tag, used to prefix this module's log messages */
static const char *TAG_LOGS_LORAWAN = "LORAWAN";

/* Line being assembled by the UART RX task (only accessed by that task) */
static char rx_line[AT_RESPONSE_LINE_MAX_SIZE] = {0};
static size_t rx_line_len = 0;

/* Function: feed received bytes into the line being assembled. Every time the end of a line
 *           ('\r' or '\n') is found, the complete line (without the terminator) is posted to
 *           at_response_queue, signaling send_command_over_UART() that a response has arrived.
 *           A single chunk may hold more than one line, so all of them are handled here
 * Parameters: - pt_bytes: received bytes
 *             - len: number of bytes in pt_bytes
 * Return: none
 */
static void process_rx_bytes(const uint8_t *pt_bytes, int len)
{
    for (int i = 0; i < len; i++)
    {
        char c = (char)pt_bytes[i];

        if (c == '\0')
        {
            /* Spurious null bytes (e.g. glitches on the module's TX line when it wakes up)
             * are discarded, otherwise they would truncate the line when it is printed */
            continue;
        }

        if ((c == '\r') || (c == '\n'))
        {
            /* End of line detected: '\r', '\n' and "\r\n" are all accepted as terminators, since
             * the module's manual does not specify it. A "\r\n" terminator produces an empty line
             * right after the real one; empty lines are ignored, since they carry no response */
            if (rx_line_len > 0)
            {
                if (xQueueSend(at_response_queue, rx_line, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG_LOGS_LORAWAN, "Response queue full, line discarded: %s", rx_line);
                }
            }

            rx_line_len = 0;
            rx_line[0] = '\0';
        }
        else if (rx_line_len < (AT_RESPONSE_LINE_MAX_SIZE - 1))
        {
            rx_line[rx_line_len++] = c;
            rx_line[rx_line_len] = '\0';
        }
        else
        {
            /* Line too long: extra bytes are dropped until the terminator arrives, so the
             * line is published (truncated) instead of being lost */
        }
    }
}

/* Function: UART RX task. Blocks on the UART driver event queue until the UART interrupt
 *           reports that bytes were received (UART_DATA). Then it reads every byte available
 *           in the RX buffer and hands them to process_rx_bytes(), which assembles the lines.
 *           Line terminators are found in software, so no received byte is left behind in the
 *           driver's RX buffer, whatever terminator the module uses
 * Parameters: pvParameters - not used
 * Return: none
 */
static void uart_rx_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t rx_bytes[64];

    while (1)
    {
        /* Sleep here (no CPU usage) until the UART interrupt posts a new event */
        if (xQueueReceive(uart_event_queue, &event, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        switch (event.type)
        {
            case UART_DATA:
            {
                /* Read everything currently available in the RX buffer (it may hold more
                 * than event.size bytes, if previous events were already handled) */
                size_t available = 0;
                uart_get_buffered_data_len(LORAWAN_UART_PORT_NUM, &available);

                while (available > 0)
                {
                    int chunk = (available > sizeof(rx_bytes)) ? (int)sizeof(rx_bytes) : (int)available;
                    int read = uart_read_bytes(LORAWAN_UART_PORT_NUM, rx_bytes, chunk, 0);

                    if (read <= 0)
                    {
                        break;
                    }

#if LORAWAN_RX_RAW_DUMP
                    ESP_LOG_BUFFER_HEXDUMP(TAG_LOGS_LORAWAN, rx_bytes, read, ESP_LOG_INFO);
#endif
                    available -= read;
                    process_rx_bytes(rx_bytes, read);
                }
                break;
            }

            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                /* Data was lost: discard everything received so far and start over */
                ESP_LOGW(TAG_LOGS_LORAWAN, "UART RX overflow, flushing input");
                uart_flush_input(LORAWAN_UART_PORT_NUM);
                xQueueReset(uart_event_queue);
                rx_line_len = 0;
                rx_line[0] = '\0';
                break;

            default:
                break;
        }
    }
}

/* Function: wait for a response line from the LoRaWAN module. Only lines starting with "AT_"
 *           (AT_OK, AT_ERROR, AT_BUSY_ERROR, AT_TX_OK, etc.) are taken as responses; any other
 *           line (e.g. the module's boot banner or noise) is logged and ignored
 * Parameters: timeout_ms - maximum time to wait, in milliseconds
 * Return: true if a response was received (it is left in buffer_recepcao), false on timeout
 */
static bool wait_at_response(uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    TickType_t elapsed;

    while ((elapsed = xTaskGetTickCount() - start) < timeout)
    {
        memset(buffer_recepcao, 0x00, sizeof(buffer_recepcao));

        if (xQueueReceive(at_response_queue, buffer_recepcao, timeout - elapsed) != pdTRUE)
        {
            break;
        }

        if (strncmp(buffer_recepcao, "AT_", 3) == 0)
        {
            return true;
        }

        ESP_LOGI(TAG_LOGS_LORAWAN, "Line ignored (not an AT response): %s", buffer_recepcao);
    }

    return false;
}

/* Function: send an AT command to the LoRaWAN module over the UART interface and wait for its
 *           response. If no response arrives within timeout_ms, the command is re-sent (up to
 *           AT_CMD_MAX_ATTEMPTS attempts). If the module replies "BUSY" (i.e. it is still
 *           processing a previous operation, such as joining the network), the command is
 *           re-sent 5 seconds later (BUSY replies do not count as failed attempts).
 *           Instead of waiting a fixed amount of time, it blocks until the UART RX task
 *           signals that a complete response line has been received, or until timeout_ms expires
 * Parameters: - pt_cmd: buffer containing the AT command string to send
 *             - tamanho: number of bytes to write from pt_cmd ("tamanho" is Portuguese for "size")
 *             - timeout_ms: maximum time to wait for the response to each attempt, in milliseconds
 * Return: none
 */
static void send_command_over_UART(char *pt_cmd, int tamanho, uint32_t timeout_ms)
{
    bool response_received = false;
    int attempt = 0;

    while (attempt < AT_CMD_MAX_ATTEMPTS)
    {
        attempt++;

        /* Discard any stale line (e.g. a late response to a previous command or an
         * unsolicited message) so that only the response to this command is analyzed */
        xQueueReset(at_response_queue);

        /* Send the AT command and wait until its response line arrives (or timeout) */
        uart_write_bytes(LORAWAN_UART_PORT_NUM, pt_cmd, tamanho);
        response_received = wait_at_response(timeout_ms);

        if (!response_received)
        {
            ESP_LOGW(TAG_LOGS_LORAWAN, "No response from LoRaWAN module within %lu ms (attempt %d of %d)",
                     (unsigned long)timeout_ms, attempt, AT_CMD_MAX_ATTEMPTS);
        }
        else if (strstr(buffer_recepcao, "BUSY") != NULL)
        {
            /* If the module reports it is BUSY, wait 5 seconds and resend the same command */
            ESP_LOGE(TAG_LOGS_LORAWAN, "LoRaWAN module is BUSY: %s. Command will be re-sent in 5 seconds from now...", buffer_recepcao);
            vTaskDelay(pdMS_TO_TICKS(5000));
            attempt--;
        }
        else
        {
            break;
        }
    }

    if (response_received)
    {
        ESP_LOGI(TAG_LOGS_LORAWAN, "String received from LoRaWAN module: %s", buffer_recepcao);
    }
    else
    {
        ESP_LOGE(TAG_LOGS_LORAWAN, "LoRaWAN module did not answer the command after %d attempts", AT_CMD_MAX_ATTEMPTS);
    }
}

/* Function: wait for the LoRaWAN module to finish booting, which is signaled by its boot banner
 *           (e.g. "Radioenge AT 1.3.35B0"). Commands sent before that are lost, since the module
 *           is powered up (by the peripherals power rail) right before its UART is initialized
 * Parameters: none
 * Return: none
 */
static void wait_lorawan_module_boot(void)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(LORAWAN_BOOT_TIMEOUT_MS);
    TickType_t elapsed;

    while ((elapsed = xTaskGetTickCount() - start) < timeout)
    {
        memset(buffer_recepcao, 0x00, sizeof(buffer_recepcao));

        if (xQueueReceive(at_response_queue, buffer_recepcao, timeout - elapsed) != pdTRUE)
        {
            break;
        }

        if (strstr(buffer_recepcao, LORAWAN_BOOT_BANNER) != NULL)
        {
            ESP_LOGI(TAG_LOGS_LORAWAN, "LoRaWAN module booted: %s", strstr(buffer_recepcao, LORAWAN_BOOT_BANNER));
            return;
        }
    }

    ESP_LOGW(TAG_LOGS_LORAWAN, "LoRaWAN module boot banner not received within %d ms", LORAWAN_BOOT_TIMEOUT_MS);
}

/* Function: initialize the UART interface used to communicate with the LoRaWAN module
 * Parameters: none
 * Return: none
 */
void init_uart_lorawan(void)
{
    /* Configure UART: 9600 baud, 8N1, no hardware flow control */
    uart_config_t uart_config = {
        .baud_rate = LORAWAN_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    /* Place the UART interrupt handler in IRAM when configured to do so (needed in setups
     * where flash cache may be disabled, e.g. during flash operations) */
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    /* Install the UART driver (with an RX buffer twice BUF_SIZE, no TX buffer) and an event
     * queue, which the driver's RX interrupt handler uses to notify that data has arrived.
     * Then apply the baud rate / frame configuration above and assign the TX/RX/RTS/CTS pins */
    ESP_ERROR_CHECK(uart_driver_install(LORAWAN_UART_PORT_NUM, BUF_SIZE * 2, 0,
                                        UART_EVENT_QUEUE_SIZE, &uart_event_queue, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(LORAWAN_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LORAWAN_UART_PORT_NUM, LORAWAN_TEST_TXD, LORAWAN_TEST_RXD, LORAWAN_TEST_RTS, LORAWAN_TEST_CTS));

    /* Create the queue of complete response lines and the task that assembles them */
    at_response_queue = xQueueCreate(AT_RESPONSE_QUEUE_SIZE, AT_RESPONSE_LINE_MAX_SIZE);
    configASSERT(at_response_queue != NULL);

    BaseType_t task_created = xTaskCreate(uart_rx_task, "lorawan_uart_rx", UART_RX_TASK_STACK_SIZE,
                                          NULL, UART_RX_TASK_PRIORITY, NULL);
    configASSERT(task_created == pdPASS);

    /* The module has just been powered up: wait until it is ready to accept commands */
    wait_lorawan_module_boot();
}

/* Function: configure the LoRaWAN module by sending it, one by one, all the AT commands
 *           needed to set the channel mask, join mode, device address, application EUI,
 *           session keys, ADR, data rate, class and message confirmation mode
 * Parameters: pt_lorawan - pointer to the struct holding the desired LoRaWAN configuration
 * Return: none
 */
void configure_lorawan(TConfig_LoRaWAN *pt_lorawan)
{
    char cmd_at[TAM_MAX_CMD_AT] = {0};

    /* Configure channel mask */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure channel mask: %s ...", pt_lorawan->CHMASK);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+CHMASK=%s\r\n", pt_lorawan->CHMASK);
    send_command_over_UART(cmd_at, strlen(cmd_at), AT_CONFIG_RESPONSE_TIMEOUT_MS);   

    /* Configure Join Mode */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure Join Mode ...");
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+NJM=%c\r\n", pt_lorawan->join_mode);
    send_command_over_UART(cmd_at, strlen(cmd_at), AT_CONFIG_RESPONSE_TIMEOUT_MS);    

    /* Configure LoRaWAN Device Address */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure LoRaWAN Device Address: %s ...", pt_lorawan->DEVADDR);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+DADDR=%s\r\n", pt_lorawan->DEVADDR);
    send_command_over_UART(cmd_at, strlen(cmd_at), AT_CONFIG_RESPONSE_TIMEOUT_MS);    

    /* Configure LoRaWAN Application EUI */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure LoRaWAN Application EUI: %s ...", pt_lorawan->APPEUI);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+APPEUI=%s\r\n", pt_lorawan->APPEUI);
    send_command_over_UART(cmd_at, strlen(cmd_at), AT_CONFIG_RESPONSE_TIMEOUT_MS);    

    /* Configure LoRaWAN Application Session Key */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure LoRaWAN Application Session Key: %s ...", pt_lorawan->APPSKEY);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+APPSKEY=%s\r\n", pt_lorawan->APPSKEY);
    send_command_over_UART(cmd_at, strlen(cmd_at), AT_CONFIG_RESPONSE_TIMEOUT_MS);    

    /* Configure LoRaWAN Network Session Key */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure LoRaWAN Network Session Key: %s ...", pt_lorawan->NWSKEY);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+NWKSKEY=%s\r\n", pt_lorawan->NWSKEY);
    send_command_over_UART(cmd_at, strlen(cmd_at), AT_CONFIG_RESPONSE_TIMEOUT_MS);    

    /* Configure LoRaWAN ADR */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure LoRaWAN ADR: %c ...", pt_lorawan->adr);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+ADR=%c\r\n", pt_lorawan->adr);
    send_command_over_UART(cmd_at, strlen(cmd_at), AT_CONFIG_RESPONSE_TIMEOUT_MS);    

    /* Configure LoRaWAN Data Rate */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure LoRaWAN Data Rate: %c ...", pt_lorawan->dr);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+DR=%c\r\n", pt_lorawan->dr);
    send_command_over_UART(cmd_at, strlen(cmd_at), AT_CONFIG_RESPONSE_TIMEOUT_MS);    

    /* Configure LoRaWAN Class */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure LoRaWAN Class: %c ...", pt_lorawan->class);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+CLASS=%c\r\n", pt_lorawan->class);
    send_command_over_UART(cmd_at, strlen(cmd_at), AT_CONFIG_RESPONSE_TIMEOUT_MS);    

    /* Configure LoRaWAN message confirmation */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure LoRaWAN message confirmation: %c ...", pt_lorawan->msg_sent_confirmation);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+CFM=%c\r\n", pt_lorawan->msg_sent_confirmation);
    send_command_over_UART(cmd_at, strlen(cmd_at), AT_CONFIG_RESPONSE_TIMEOUT_MS);    

    ESP_LOGI(TAG_LOGS_LORAWAN, "all set!");
}

/* Function: send an application payload as a LoRaWAN uplink message, using the AT+SENDB
 *           command (send binary data, provided here as a HEX-ASCII string) on LoRaWAN port 5
 * Parameters: pt_payload - payload to be sent, already encoded as a HEX-ASCII string
 * Return: none
 */
void send_LoRaWAN_payload(char *pt_payload)
{
    char cmd_envio_payload[TAM_MAX_CMD_AT] = {0};  // "cmd_envio_payload" = "payload send command"

    snprintf(cmd_envio_payload, sizeof(cmd_envio_payload), "AT+SENDB=5:%s\r\n", pt_payload);
    ESP_LOGI(TAG_LOGS_LORAWAN, "AT command: AT+SENDB=5:%s", pt_payload);
    send_command_over_UART(cmd_envio_payload, strlen(cmd_envio_payload), AT_RESPONSE_TIMEOUT_MS);

}